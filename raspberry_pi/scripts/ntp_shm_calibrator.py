#!/usr/bin/env python3
"""NTP offset calibration daemon (WAN-only, no SHM).

Reads per-server offset data from chronyc, maintains rolling hourly averages,
selects the best WAN NTP server, and writes calibrated offset terms directly
into chrony.conf.  No SHM, no refclock WANC, no GNSS dependency.

Algorithm:
  Warmup (< MIN_STABILITY_SAMPLES hourly cycles per server):
    All servers equal, no offsets, no 'prefer'.  chrony picks its own source.
  Active (post-warmup):
    Best server selected by stability gate (CV), then stratum, then stdev.
    That server gets 'prefer' in chrony.conf.  Other servers get 'offset'
    terms calibrated from their historical mean sourcestats offsets.
    Offsets are frozen (conf not rewritten) unless server ranking changes.

Reference outage:
    If the preferred server is absent from sourcestats for > REFERENCE_TIMEOUT_S,
    an email alert is sent to NOTIFY_EMAIL.  Alert clears when server returns.

Install as: /usr/local/sbin/ntp_shm_calibrator.py
Service:    ntp-shm-calibrator.service (Type=simple, User=root)
"""

from __future__ import annotations

import argparse
import csv
import enum
import ipaddress
import json
import logging
import os
import re
import signal
import smtplib
import statistics
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from email.message import EmailMessage
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_POLL_INTERVAL_S = 60
DEFAULT_HOURLY_WINDOW_S = 3600
DEFAULT_LOG_DIR = Path("/var/log/ntp-calibrator")
DEFAULT_STATE_PATH = Path("/var/lib/ntp-calibrator/state.json")
DEFAULT_STATUS_PATH = Path("/var/lib/ntp-calibrator/status.json")
DEFAULT_CHRONY_CONF = Path("/etc/chrony/chrony.conf")
DEFAULT_CHRONYC = "chronyc"
DEFAULT_CHRONYC_TIMEOUT_S = 5.0
DEFAULT_RAW_LOG_RETENTION_H = 72
DEFAULT_HOURLY_LOG_RETENTION_H = 72
DEFAULT_MIN_STABILITY_SAMPLES = 6
DEFAULT_CV_THRESHOLD = 3.0
DEFAULT_OVERRIDE_THRESHOLD = 0.20
DEFAULT_ACTIVE_POOL_SIZE = 3
DEFAULT_REFERENCE_TIMEOUT_S = 1800   # 30 minutes before sending outage alert
DEFAULT_NOTIFY_EMAIL: Optional[str] = None
DEFAULT_SMTP_HOST = "localhost"
DEFAULT_SMTP_PORT = 25

ACTIVE_MINPOLL = 6
ACTIVE_MAXPOLL = 6
NOSELECT_MINPOLL = 7
NOSELECT_MAXPOLL = 7

MANAGED_BEGIN = "# BEGIN ntp-calibrator-managed"
MANAGED_END = "# END ntp-calibrator-managed"

RAW_LOG_HEADER = [
    "timestamp_utc", "server", "offset_s", "std_dev_s",
    "stratum", "state_char", "np", "nr",
]
HOURLY_LOG_HEADER = [
    "timestamp_utc", "server", "mean_offset_s", "hourly_stdev_s",
    "n_samples", "stratum", "tier", "role", "applied_offset_s",
]

DEFAULT_INITIAL_SERVERS = [
    "time.apple.com",
    "time.google.com",
    "time.cloudflare.com",
    "time.aws.com",
]
POOL_DISCOVERY_LINE = (
    f"pool pool.ntp.org iburst"
    f" minpoll {ACTIVE_MINPOLL} maxpoll {ACTIVE_MAXPOLL} noselect maxsources 4"
)

# Minimum raw samples per window to compute a meaningful hourly summary
_MIN_HOURLY_SAMPLES = 3

logger = logging.getLogger("ntp_shm_calibrator")


# ---------------------------------------------------------------------------
# Chrony data parsing
# ---------------------------------------------------------------------------

_DURATION_RE = re.compile(
    r"^([+-]?)\s*([\d.]+)\s*(ns|us|ms|s)$",
    re.IGNORECASE,
)


def _parse_chrony_duration(s: str) -> Optional[float]:
    """Parse a chrony timing string like '+49us', '-1.2ms', '312ns' → seconds."""
    m = _DURATION_RE.match(s.strip())
    if not m:
        return None
    sign_str, value_str, unit = m.groups()
    try:
        value = float(value_str)
    except ValueError:
        return None
    if sign_str == "-":
        value = -value
    unit = unit.lower()
    multipliers = {"ns": 1e-9, "us": 1e-6, "ms": 1e-3, "s": 1.0}
    return value * multipliers[unit]


def parse_sourcestats_full(
    sources_output: str,
    stats_output: str,
) -> Dict[str, Dict[str, Any]]:
    """Parse chronyc sources and sourcestats into a per-server dict.

    Returns: {ip/name: {name, state_char, stratum, offset_s, std_dev_s, np, nr}}
    Refclock sources (mode char '#') are excluded — they are not NTP peers.
    """
    # Parse sources: state_char and stratum by IP
    source_info: Dict[str, Dict] = {}
    for line in sources_output.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("MS") or stripped.startswith("="):
            continue
        parts = stripped.split()
        if len(parts) < 3:
            continue
        state_field = parts[0]   # e.g., "^*" for NTP, "#?" for refclock
        if len(state_field) < 2:
            continue
        if state_field[0] == "#":
            continue   # skip local refclocks (SHM, PPS, etc.)
        ip = parts[1]
        state_char = state_field[1]
        try:
            stratum = int(parts[2])
        except (ValueError, IndexError):
            stratum = None
        source_info[ip] = {"state_char": state_char, "stratum": stratum}

    # Parse sourcestats: offset_s and std_dev_s by IP
    result: Dict[str, Dict] = {}
    for line in stats_output.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("Name") or stripped.startswith("="):
            continue
        parts = stripped.split()
        if len(parts) < 8:
            continue
        ip = parts[0]
        if ip not in source_info:
            continue   # not an NTP peer
        try:
            np_val = int(parts[1])
            nr_val = int(parts[2])
        except (ValueError, IndexError):
            np_val = 0
            nr_val = 0
        offset_s = _parse_chrony_duration(parts[6])
        std_dev_s = _parse_chrony_duration(parts[7])
        if offset_s is None or std_dev_s is None:
            continue
        info = source_info.get(ip, {})
        result[ip] = {
            "name": ip,
            "state_char": info.get("state_char", "?"),
            "stratum": info.get("stratum"),
            "offset_s": offset_s,
            "std_dev_s": abs(std_dev_s),
            "np": np_val,
            "nr": nr_val,
        }
    return result


def run_sourcestats_full(
    *,
    chronyc_binary: str = DEFAULT_CHRONYC,
    timeout_s: float = DEFAULT_CHRONYC_TIMEOUT_S,
) -> Dict[str, Dict[str, Any]]:
    """Run chronyc sources + sourcestats and return parsed per-server data."""
    try:
        src = subprocess.run(
            [chronyc_binary, "-n", "sources"],
            capture_output=True, text=True, timeout=timeout_s,
        )
        stats = subprocess.run(
            [chronyc_binary, "-n", "sourcestats"],
            capture_output=True, text=True, timeout=timeout_s,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
        raise RuntimeError(f"chronyc subprocess failed: {exc}") from exc
    if src.returncode != 0 or stats.returncode != 0:
        raise RuntimeError("chronyc returned non-zero exit code")
    return parse_sourcestats_full(src.stdout, stats.stdout)


# ---------------------------------------------------------------------------
# Raw sample logging
# ---------------------------------------------------------------------------

def _raw_log_path(log_dir: Path) -> Path:
    """Return today's raw log path."""
    date_str = datetime.now(timezone.utc).strftime("%Y-%m-%d")
    return log_dir / f"raw_{date_str}.tsv"


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def append_raw_sample(
    log_dir: Path,
    server: str,
    offset_s: float,
    std_dev_s: float,
    stratum: Optional[int],
    state_char: str,
    np_val: int,
    nr_val: int,
) -> None:
    """Append one raw sample row to today's TSV log."""
    _ensure_dir(log_dir)
    path = _raw_log_path(log_dir)
    write_header = not path.exists()
    with path.open("a", newline="") as f:
        writer = csv.writer(f, delimiter="\t")
        if write_header:
            writer.writerow(RAW_LOG_HEADER)
        writer.writerow([
            _utc_now_iso(), server,
            f"{offset_s:.9f}", f"{std_dev_s:.9f}",
            stratum if stratum is not None else "",
            state_char, np_val, nr_val,
        ])


def rotate_raw_logs(log_dir: Path, retention_h: float) -> None:
    """Delete raw log files older than retention_h hours."""
    cutoff = time.time() - retention_h * 3600
    for p in log_dir.glob("raw_*.tsv"):
        try:
            if p.stat().st_mtime < cutoff:
                p.unlink()
        except OSError:
            pass


def load_raw_samples(
    log_dir: Path,
    server: str,
    window_s: float,
) -> List[Dict[str, Any]]:
    """Load raw samples for server from the last window_s seconds."""
    cutoff = time.time() - window_s
    rows = []
    for p in sorted(log_dir.glob("raw_*.tsv")):
        try:
            with p.open(newline="") as f:
                reader = csv.DictReader(f, delimiter="\t")
                for row in reader:
                    if row.get("server") != server:
                        continue
                    try:
                        ts = datetime.fromisoformat(
                            row["timestamp_utc"].replace("Z", "+00:00")
                        ).timestamp()
                    except (ValueError, KeyError):
                        continue
                    if ts < cutoff:
                        continue
                    try:
                        rows.append({
                            "timestamp_s": ts,
                            "offset_s": float(row["offset_s"]),
                            "std_dev_s": float(row["std_dev_s"]),
                            "stratum": int(row["stratum"]) if row["stratum"] else None,
                            "state_char": row.get("state_char", "?"),
                            "np": int(row.get("np", 0)),
                            "nr": int(row.get("nr", 0)),
                        })
                    except (ValueError, KeyError):
                        continue
        except OSError:
            continue
    return rows


# ---------------------------------------------------------------------------
# Hourly aggregation
# ---------------------------------------------------------------------------

def compute_hourly_summary(
    samples: List[Dict[str, Any]],
    window_s: float = DEFAULT_HOURLY_WINDOW_S,
) -> Optional[Dict[str, Any]]:
    """Compute mean and stdev of offset from samples within window_s.

    Returns None if fewer than _MIN_HOURLY_SAMPLES valid samples exist.
    """
    cutoff = time.time() - window_s
    valid = [s for s in samples if s.get("timestamp_s", 0) >= cutoff]
    if len(valid) < _MIN_HOURLY_SAMPLES:
        return None
    offsets = [s["offset_s"] for s in valid]
    mean_off = statistics.mean(offsets)
    stdev_off = statistics.stdev(offsets) if len(offsets) >= 2 else 0.0
    stratum = next(
        (s["stratum"] for s in reversed(valid) if s.get("stratum") is not None),
        None,
    )
    return {
        "mean_offset_s": mean_off,
        "hourly_stdev_s": stdev_off,
        "n_samples": len(valid),
        "stratum": stratum,
    }


def append_hourly_summary(
    log_dir: Path,
    server: str,
    summary: Dict[str, Any],
    tier: int,
    role: str = "warmup",
    applied_offset_s: Optional[float] = None,
) -> None:
    """Append one hourly summary row.

    role: 'preferred', 'active', 'noselect', or 'warmup'
    applied_offset_s: offset term written to chrony.conf (None for noselect/warmup)
    """
    _ensure_dir(log_dir)
    path = log_dir / "hourly.tsv"
    write_header = not path.exists()
    with path.open("a", newline="") as f:
        writer = csv.writer(f, delimiter="\t")
        if write_header:
            writer.writerow(HOURLY_LOG_HEADER)
        writer.writerow([
            _utc_now_iso(), server,
            f"{summary['mean_offset_s']:.9f}",
            f"{summary['hourly_stdev_s']:.9f}",
            summary["n_samples"],
            summary.get("stratum", ""),
            tier, role,
            f"{applied_offset_s:.9f}" if applied_offset_s is not None else "",
        ])


def load_hourly_window(
    log_dir: Path,
    retention_h: float = DEFAULT_HOURLY_LOG_RETENTION_H,
) -> Dict[str, List[Dict[str, Any]]]:
    """Load hourly summaries for all servers within retention window.

    Returns: {server: [{timestamp_s, mean_offset_s, hourly_stdev_s, ...}]}
    """
    path = log_dir / "hourly.tsv"
    if not path.exists():
        return {}
    cutoff = time.time() - retention_h * 3600
    result: Dict[str, List] = {}
    try:
        with path.open(newline="") as f:
            reader = csv.DictReader(f, delimiter="\t")
            for row in reader:
                server = row.get("server", "")
                if not server:
                    continue
                try:
                    ts = datetime.fromisoformat(
                        row["timestamp_utc"].replace("Z", "+00:00")
                    ).timestamp()
                except (ValueError, KeyError):
                    continue
                if ts < cutoff:
                    continue
                try:
                    entry = {
                        "timestamp_s": ts,
                        "mean_offset_s": float(row["mean_offset_s"]),
                        "hourly_stdev_s": float(row["hourly_stdev_s"]),
                        "n_samples": int(row["n_samples"]),
                        "stratum": int(row["stratum"]) if row.get("stratum") else None,
                        "tier": int(row.get("tier", 3)),
                    }
                except (ValueError, KeyError):
                    continue
                result.setdefault(server, []).append(entry)
    except OSError:
        pass
    return result


def rotate_hourly_log(log_dir: Path, retention_h: float) -> None:
    """Rewrite hourly.tsv keeping only rows within retention_h hours."""
    path = log_dir / "hourly.tsv"
    if not path.exists():
        return
    cutoff = time.time() - retention_h * 3600
    kept_rows = []
    try:
        with path.open(newline="") as f:
            reader = csv.DictReader(f, delimiter="\t")
            for row in reader:
                try:
                    ts = datetime.fromisoformat(
                        row["timestamp_utc"].replace("Z", "+00:00")
                    ).timestamp()
                    if ts >= cutoff:
                        kept_rows.append(row)
                except (ValueError, KeyError):
                    continue
    except OSError:
        return
    tmp = path.with_suffix(".tmp")
    try:
        with tmp.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=HOURLY_LOG_HEADER, delimiter="\t")
            writer.writeheader()
            writer.writerows(kept_rows)
        os.replace(tmp, path)
    except OSError:
        try:
            tmp.unlink()
        except OSError:
            pass


# ---------------------------------------------------------------------------
# Server selection
# ---------------------------------------------------------------------------

def compute_cv_stdev(
    stdev_values: List[float],
    min_samples: int = DEFAULT_MIN_STABILITY_SAMPLES,
) -> Optional[float]:
    """Compute CV = stdev(stdev_values) / mean(stdev_values).

    Returns None if fewer than min_samples values or mean is zero.
    """
    if len(stdev_values) < min_samples:
        return None
    mean_val = statistics.mean(stdev_values)
    if mean_val == 0.0:
        return 0.0
    return statistics.stdev(stdev_values) / mean_val


def _build_server_summaries(
    hourly_window: Dict[str, List[Dict]],
    current_sourcestats: Dict[str, Dict],
    privileged_ips: frozenset,
    min_stability_samples: int,
) -> List[Dict[str, Any]]:
    """Build per-server summary dicts for the selection algorithm."""
    summaries = []
    for server, entries in hourly_window.items():
        if not entries:
            continue
        # Exclude WANC and private/loopback IPs — they must never be candidates.
        if not _is_valid_ntp_server(server):
            continue
        stdev_values = [e["hourly_stdev_s"] for e in entries]
        mean_stdev = statistics.mean(stdev_values) if stdev_values else None
        mean_offset = (
            statistics.mean(e["mean_offset_s"] for e in entries)
            if entries else 0.0
        )
        cv = compute_cv_stdev(stdev_values, min_samples=min_stability_samples)
        stratum = next(
            (e["stratum"] for e in reversed(entries) if e.get("stratum") is not None),
            99,
        )
        tier = 2 if server in privileged_ips else 3
        state_char = current_sourcestats.get(server, {}).get("state_char", "?")
        summaries.append({
            "name": server,
            "stratum": stratum if stratum is not None else 99,
            "mean_stdev_s": mean_stdev or 0.0,
            "cv_stdev": cv,
            "mean_offset_s": mean_offset,
            "n_hourly_samples": len(entries),
            "state_char": state_char,
            "tier": tier,
        })
    return summaries


def select_servers(
    server_summaries: List[Dict[str, Any]],
    active_pool_size: int = DEFAULT_ACTIVE_POOL_SIZE,
    cv_threshold: float = DEFAULT_CV_THRESHOLD,
    override_threshold: float = DEFAULT_OVERRIDE_THRESHOLD,
    min_stability_samples: int = DEFAULT_MIN_STABILITY_SAMPLES,
) -> Tuple[Optional[Dict], List[Dict], List[Dict]]:
    """Select preferred + active servers using stability-gated stratum ranking.

    Returns: (preferred, active_list, noselect_list)
    preferred occupies one of the active_pool_size slots.
    """
    stable = [
        s for s in server_summaries
        if s.get("cv_stdev") is not None
        and s["cv_stdev"] < cv_threshold
        and s.get("n_hourly_samples", 0) >= min_stability_samples
    ]
    unstable = [s for s in server_summaries if s not in stable]

    if not stable:
        return None, [], list(server_summaries)

    # Sort: tier asc, stratum asc, mean_stdev_s asc
    stable_sorted = sorted(stable, key=lambda s: (s["tier"], s["stratum"], s["mean_stdev_s"]))

    # Only the second-ranked server may challenge the winner via stdev override.
    winner = stable_sorted[0]
    if len(stable_sorted) >= 2:
        candidate = stable_sorted[1]
        if (candidate["mean_stdev_s"] < winner["mean_stdev_s"] * (1 - override_threshold)
                and candidate.get("cv_stdev") is not None
                and candidate["cv_stdev"] < cv_threshold):
            winner = candidate

    preferred = winner
    rest = [s for s in stable_sorted if s is not winner]
    active = rest[: active_pool_size - 1]
    noselect = rest[active_pool_size - 1 :] + unstable

    return preferred, active, noselect


# ---------------------------------------------------------------------------
# Server name validation
# ---------------------------------------------------------------------------

def _is_valid_ntp_server(name: str) -> bool:
    """Return False for names that must never appear as managed server lines.

    Rejects the legacy WANC refid and non-global IPs (RFC1918, loopback,
    link-local). Hostnames are accepted; chrony resolves them.
    """
    if name.upper() == "WANC":
        return False
    try:
        ip = ipaddress.ip_address(name)
        return ip.is_global
    except ValueError:
        return True  # hostname


def _read_managed_server_set(conf_path: Path) -> frozenset:
    """Return the validated set of server names in the current managed block.

    Called at startup to establish which servers the daemon may manage
    individually. Re-derived on every start so manual edits are respected.
    """
    try:
        content = conf_path.read_text()
    except OSError:
        return frozenset()
    servers: set = set()
    for line in read_managed_block(content):
        parts = line.strip().split()
        if len(parts) >= 2 and parts[0] == "server":
            name = parts[1]
            if _is_valid_ntp_server(name):
                servers.add(name)
    return frozenset(servers)


# ---------------------------------------------------------------------------
# chrony.conf management
# ---------------------------------------------------------------------------

def read_managed_block(conf_content: str) -> List[str]:
    """Extract lines from within the ntp-calibrator managed block."""
    lines = []
    inside = False
    for line in conf_content.splitlines():
        if line.strip() == MANAGED_BEGIN:
            inside = True
            continue
        if line.strip() == MANAGED_END:
            break
        if inside:
            lines.append(line)
    return lines


def build_server_line(
    server: str,
    is_preferred: bool,
    is_noselect: bool,
    offset_s: Optional[float],
    is_pool: bool = False,
) -> str:
    """Format a single chrony.conf server/pool line."""
    keyword = "pool" if is_pool else "server"
    if is_noselect:
        minpoll, maxpoll = NOSELECT_MINPOLL, NOSELECT_MAXPOLL
    else:
        minpoll, maxpoll = ACTIVE_MINPOLL, ACTIVE_MAXPOLL

    parts = [
        f"{keyword} {server} iburst",
        f"minpoll {minpoll}",
        f"maxpoll {maxpoll}",
    ]
    if is_pool:
        parts.append("noselect maxsources 4")
    elif is_noselect:
        parts.append("noselect")
    elif is_preferred:
        parts.append("prefer")

    if not is_noselect and not is_pool and offset_s is not None and offset_s != 0.0:
        parts.append(f"offset {offset_s:+.6f}")

    return " ".join(parts)


def write_chrony_conf(
    conf_path: Path,
    new_block_lines: List[str],
) -> None:
    """Replace the managed block in chrony.conf with new_block_lines (atomic)."""
    try:
        original = conf_path.read_text()
    except OSError:
        original = ""

    before: List[str] = []
    after: List[str] = []
    inside = False
    found = False
    for line in original.splitlines(keepends=True):
        stripped = line.strip()
        if stripped == MANAGED_BEGIN:
            inside = True
            found = True
            continue
        if stripped == MANAGED_END:
            inside = False
            continue
        if not inside:
            if not found:
                before.append(line)
            else:
                after.append(line)

    new_block = (
        [MANAGED_BEGIN + "\n"]
        + [l + "\n" for l in new_block_lines]
        + [MANAGED_END + "\n"]
    )

    if found:
        content = "".join(before) + "".join(new_block) + "".join(after)
    else:
        content = original
        if original and not original.endswith("\n"):
            content += "\n"
        content += "\n" + "".join(new_block)

    tmp_path = conf_path.with_suffix(conf_path.suffix + ".tmp")
    try:
        tmp_path.write_text(content)
        os.replace(tmp_path, conf_path)
    except OSError:
        try:
            tmp_path.unlink()
        except OSError:
            pass
        raise


def _reload_chrony(reload_chrony: bool) -> None:
    """Reload chrony config, falling back to restart if reload is unsupported.

    Prefers `systemctl reload` (SIGHUP). Falls back to `systemctl restart`
    on systems where chrony's service unit has no ExecReload defined (e.g.
    Debian chrony 4.3 on this Pi), because systemctl reload returns non-zero
    in that case and a full restart is needed to apply the new conf.
    """
    if not reload_chrony:
        return
    try:
        result = subprocess.run(
            ["systemctl", "reload", "chrony"],
            capture_output=True, timeout=10,
        )
        if result.returncode != 0:
            logger.warning(
                "systemctl reload chrony failed (rc=%d) — falling back to restart",
                result.returncode,
            )
            subprocess.run(
                ["systemctl", "restart", "chrony"],
                capture_output=True, timeout=15,
            )
            # Restart takes longer; give chrony time to initialize.
            time.sleep(5)
        else:
            # reload returns when SIGHUP is delivered; chrony processes it async.
            time.sleep(2)
    except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
        logger.warning("chrony reload/restart failed: %s", exc)


# ---------------------------------------------------------------------------
# Reference server outage notification
# ---------------------------------------------------------------------------

def send_reference_alert(
    server: str,
    since_utc: str,
    recipient: str,
    smtp_host: str = DEFAULT_SMTP_HOST,
    smtp_port: int = DEFAULT_SMTP_PORT,
) -> bool:
    """Send an email when the preferred NTP server has been unreachable.

    Returns True on success, False on failure (caller logs the error).
    Uses localhost:25 by default; configure an msmtp/ssmtp relay on the Pi.
    """
    msg = EmailMessage()
    msg["Subject"] = f"NTP alert: preferred server {server!r} unreachable"
    msg["From"] = "ntp-calibrator@localhost"
    msg["To"] = recipient
    msg.set_content(
        f"The preferred NTP reference server {server!r} has been unreachable\n"
        f"since {since_utc} (UTC).\n\n"
        "The calibration daemon is still running and chrony will fall back to\n"
        "backup servers.  Calibrated offsets are frozen until the server returns.\n"
    )
    try:
        with smtplib.SMTP(smtp_host, smtp_port, timeout=10) as s:
            s.send_message(msg)
        return True
    except Exception as exc:
        logger.warning("Failed to send reference alert to %s: %s", recipient, exc)
        return False


# ---------------------------------------------------------------------------
# Status JSON (web UI contract)
# ---------------------------------------------------------------------------

def write_status_json(
    path: Path,
    in_warmup: bool,
    preferred_server: Optional[Dict[str, Any]],
    active_servers: List[Dict[str, Any]],
    noselect_servers: List[Dict[str, Any]],
    reference_alert_since_utc: Optional[str],
) -> None:
    """Write status.json for the web UI."""
    payload = {
        "last_update_utc": _utc_now_iso(),
        "mode": "warmup" if in_warmup else "active",
        "preferred_server": preferred_server,
        "active_servers": active_servers,
        "noselect_servers": noselect_servers,
        "reference_alert_since_utc": reference_alert_since_utc,
    }
    _ensure_dir(path.parent)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2))
    os.replace(tmp, path)


# ---------------------------------------------------------------------------
# Daemon state persistence
# ---------------------------------------------------------------------------

def _default_state() -> Dict[str, Any]:
    return {
        "preferred_server": None,
        "preferred_server_last_seen_utc": None,
        "last_update_utc": None,
        "last_conf_servers": None,   # None sentinel so first run writes the block
        "reference_alert_sent_utc": None,
    }


def load_state(path: Path) -> Dict[str, Any]:
    """Load daemon state from JSON, returning defaults on any error."""
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return _default_state()


def save_state(state: Dict[str, Any], path: Path) -> None:
    """Atomically write daemon state to JSON."""
    # Strip runtime-only keys (prefixed with '_') — they are not serialisable
    serialisable = {k: v for k, v in state.items() if not k.startswith("_")}
    _ensure_dir(path.parent)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(serialisable, indent=2))
    os.replace(tmp, path)


# ---------------------------------------------------------------------------
# Main loop helpers
# ---------------------------------------------------------------------------

def _interruptible_sleep(seconds: float, stop_flag: threading.Event) -> None:
    """Sleep for `seconds`, waking early if stop_flag is set."""
    deadline = time.monotonic() + seconds
    while not stop_flag.is_set():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        stop_flag.wait(min(1.0, remaining))


def _server_info_for_ui(
    s: Dict[str, Any],
    cv_threshold: float = DEFAULT_CV_THRESHOLD,
) -> Dict[str, Any]:
    cv = s.get("cv_stdev")
    return {
        "name": s["name"],
        "ip": s["name"],
        "tier": s.get("tier", 3),
        "stratum": s.get("stratum"),
        "mean_offset_ms": round(s.get("mean_offset_s", 0.0) * 1000, 4),
        "hourly_stdev_ms": round(s.get("mean_stdev_s", 0.0) * 1000, 4),
        "cv_stdev": round(cv or 0.0, 4),
        "stable": cv is not None and cv < cv_threshold,
        "state_char": s.get("state_char", "?"),
    }


def _servers_in_raw_logs(log_dir: Path, window_s: float) -> set:
    """Return server names seen in raw logs within the last window_s seconds."""
    cutoff = time.time() - window_s
    servers: set = set()
    for p in sorted(log_dir.glob("raw_*.tsv")):
        try:
            with p.open(newline="") as f:
                reader = csv.DictReader(f, delimiter="\t")
                for row in reader:
                    try:
                        ts = datetime.fromisoformat(
                            row["timestamp_utc"].replace("Z", "+00:00")
                        ).timestamp()
                        if ts >= cutoff:
                            servers.add(row["server"])
                    except (ValueError, KeyError):
                        continue
        except OSError:
            continue
    return servers


def _run_raw_cycle(cfg: Any, state: Dict[str, Any]) -> None:
    """Single 60-second raw logging cycle.

    Collects chronyc sourcestats, appends one row per server to the daily
    raw log, and updates preferred_server_last_seen_utc if the preferred
    server is present in the current sourcestats.
    """
    try:
        sourcestats = run_sourcestats_full(
            chronyc_binary=cfg.chronyc_binary,
            timeout_s=cfg.chronyc_timeout_s,
        )
    except RuntimeError as exc:
        logger.warning("sourcestats failed: %s", exc)
        return

    for ip, srv in sourcestats.items():
        append_raw_sample(
            cfg.log_dir, ip,
            srv["offset_s"], srv["std_dev_s"],
            srv.get("stratum"), srv["state_char"],
            srv["np"], srv["nr"],
        )

    state["_last_sourcestats"] = sourcestats

    # Track when we last saw the preferred server in active sourcestats.
    preferred = state.get("preferred_server")
    if preferred and preferred in sourcestats:
        state["preferred_server_last_seen_utc"] = _utc_now_iso()


def _run_hourly_cycle(cfg: Any, state: Dict[str, Any]) -> None:
    """Hourly aggregation cycle.

    Computes summaries from raw logs, selects servers, updates chrony.conf
    with per-server offset terms, checks reference server outage, and writes
    the status JSON.
    """
    # Use injected hourly window if present (for testing without raw log files)
    if "_hourly_window_override" in state:
        hourly_window = {
            server: [
                {
                    "timestamp_s": time.time(),
                    "mean_offset_s": s["mean_offset_s"],
                    "hourly_stdev_s": s["hourly_stdev_s"],
                    "n_samples": s.get("n_samples", 60),
                    "stratum": s.get("stratum"),
                    "tier": 3,
                }
            ] * cfg.min_stability_samples  # enough to pass stability gate
            for server, s in state["_hourly_window_override"].items()
        }
        new_summaries: Dict[str, Any] = {}
    else:
        # Step 1: compute new hourly summaries from raw logs
        new_summaries = {}
        for server in _servers_in_raw_logs(cfg.log_dir, cfg.hourly_window_s):
            samples = load_raw_samples(cfg.log_dir, server, cfg.hourly_window_s)
            summary = compute_hourly_summary(samples, cfg.hourly_window_s)
            if summary is not None:
                new_summaries[server] = summary

        # Step 2: merge into rolling 72h window
        existing = load_hourly_window(cfg.log_dir, cfg.hourly_log_retention_h)
        hourly_window = dict(existing)
        for server, summary in new_summaries.items():
            entry = {
                "timestamp_s": time.time(),
                "mean_offset_s": summary["mean_offset_s"],
                "hourly_stdev_s": summary["hourly_stdev_s"],
                "n_samples": summary["n_samples"],
                "stratum": summary.get("stratum"),
                "tier": 3,
            }
            hourly_window.setdefault(server, []).append(entry)

        rotate_raw_logs(cfg.log_dir, cfg.raw_log_retention_h)

    current_sourcestats = state.get("_last_sourcestats", {})

    server_summaries = _build_server_summaries(
        hourly_window,
        current_sourcestats,
        frozenset(),   # no privileged IPs in WAN-only mode
        cfg.min_stability_samples,
    )

    preferred, active, noselect = select_servers(
        server_summaries,
        active_pool_size=cfg.active_pool_size,
        cv_threshold=cfg.cv_threshold,
        override_threshold=cfg.override_threshold,
        min_stability_samples=cfg.min_stability_samples,
    )

    # --- Write new hourly rows now that roles are known ---
    if new_summaries:
        in_warmup_for_log = all(
            len(hourly_window.get(srv, [])) < cfg.min_stability_samples
            for srv in new_summaries
        )
        role_map: Dict[str, tuple] = {}
        if not in_warmup_for_log:
            if preferred:
                role_map[preferred["name"]] = ("preferred", preferred.get("mean_offset_s"))
            for s in active:
                role_map[s["name"]] = ("active", s.get("mean_offset_s"))
            for s in noselect:
                role_map[s["name"]] = ("noselect", None)

        for server, summary in new_summaries.items():
            if in_warmup_for_log:
                role, applied = "warmup", None
            else:
                role, applied = role_map.get(server, ("noselect", None))
            append_hourly_summary(cfg.log_dir, server, summary, 3, role, applied)

        rotate_hourly_log(cfg.log_dir, cfg.hourly_log_retention_h)

    in_warmup = (
        all(s.get("n_hourly_samples", 0) < cfg.min_stability_samples
            for s in server_summaries)
        if server_summaries else True
    )

    logger.info(
        "hourly cycle: warmup=%s servers=%d preferred=%s",
        in_warmup, len(server_summaries),
        preferred["name"] if preferred else None,
    )

    # --- Update chrony.conf ---
    managed_set = state.get("_managed_server_set", frozenset())

    def _manageable(name: str) -> bool:
        return _is_valid_ntp_server(name) and (not managed_set or name in managed_set)

    new_conf_servers = [
        n for n in (
            [preferred["name"] if preferred else None]
            + [s["name"] for s in active]
            + [s["name"] for s in noselect]
        )
        if n is not None and _manageable(n)
    ]
    try:
        conf_text = cfg.chrony_conf.read_text()
    except OSError:
        conf_text = ""
    managed_block_missing = MANAGED_BEGIN not in conf_text

    if new_conf_servers != state.get("last_conf_servers") or managed_block_missing:
        if in_warmup:
            initial = getattr(cfg, "initial_servers", DEFAULT_INITIAL_SERVERS)
            block_lines = [
                build_server_line(srv, False, False, None)
                for srv in initial
                if srv != "pool.ntp.org" and _is_valid_ntp_server(srv)
            ] + [POOL_DISCOVERY_LINE]
        else:
            block_lines = []
            if preferred and _manageable(preferred["name"]):
                block_lines.append(
                    build_server_line(
                        preferred["name"], True, False, preferred.get("mean_offset_s")
                    )
                )
            for srv in active:
                if _manageable(srv["name"]):
                    block_lines.append(
                        build_server_line(srv["name"], False, False, srv.get("mean_offset_s"))
                    )
            for srv in noselect:
                if _manageable(srv["name"]):
                    block_lines.append(build_server_line(srv["name"], False, True, None))
            block_lines.append(
                build_server_line("pool.ntp.org", False, True, None, is_pool=True)
            )

        try:
            write_chrony_conf(cfg.chrony_conf, block_lines)
            _reload_chrony(getattr(cfg, "reload_chrony", True))
            state["last_conf_servers"] = new_conf_servers
        except OSError as exc:
            logger.error("chrony.conf write failed: %s", exc)

    # --- Reference server outage detection ---
    preferred_name = preferred["name"] if preferred else state.get("preferred_server")
    state["preferred_server"] = preferred_name

    notify_email = getattr(cfg, "notify_email", None)
    if preferred_name and notify_email:
        last_seen_str = state.get("preferred_server_last_seen_utc")
        absent = False
        if last_seen_str:
            try:
                last_seen_ts = datetime.fromisoformat(
                    last_seen_str.replace("Z", "+00:00")
                ).timestamp()
                absent = (time.time() - last_seen_ts > cfg.reference_timeout_s)
            except (ValueError, TypeError):
                pass

        if absent and not state.get("reference_alert_sent_utc"):
            sent = send_reference_alert(
                preferred_name,
                last_seen_str or "unknown",
                notify_email,
                smtp_host=getattr(cfg, "smtp_host", DEFAULT_SMTP_HOST),
                smtp_port=getattr(cfg, "smtp_port", DEFAULT_SMTP_PORT),
            )
            if sent:
                state["reference_alert_sent_utc"] = _utc_now_iso()
                logger.warning("Reference alert sent for %s", preferred_name)
        elif not absent and state.get("reference_alert_sent_utc"):
            # Server returned — clear the alert
            state["reference_alert_sent_utc"] = None
            logger.info("Reference server %s returned; alert cleared", preferred_name)

    # --- Write status JSON ---
    state["last_update_utc"] = _utc_now_iso()

    try:
        write_status_json(
            path=cfg.status_path,
            in_warmup=in_warmup,
            preferred_server=_server_info_for_ui(preferred, cfg.cv_threshold) if preferred else None,
            active_servers=[_server_info_for_ui(s, cfg.cv_threshold) for s in active],
            noselect_servers=[_server_info_for_ui(s, cfg.cv_threshold) for s in noselect],
            reference_alert_since_utc=state.get("preferred_server_last_seen_utc")
                if state.get("reference_alert_sent_utc") else None,
        )
    except OSError as exc:
        logger.warning("status JSON write failed: %s", exc)


def run_daemon(cfg: Any) -> None:
    """Main daemon loop. Runs until SIGTERM/SIGINT."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    state = load_state(cfg.state_path)
    state.setdefault("last_conf_servers", None)
    state["_managed_server_set"] = _read_managed_server_set(cfg.chrony_conf)

    stop_flag = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop_flag.set())
    signal.signal(signal.SIGINT, lambda *_: stop_flag.set())

    last_hourly_s = 0.0
    last_raw_s = 0.0
    logger.info("NTP calibration daemon started")

    while not stop_flag.is_set():
        now = time.time()

        if now - last_raw_s >= cfg.poll_interval_s:
            try:
                _run_raw_cycle(cfg, state)
            except Exception as exc:
                logger.error("raw cycle error: %s", exc)
            last_raw_s = now

        if now - last_hourly_s >= cfg.hourly_window_s:
            try:
                _run_hourly_cycle(cfg, state)
            except Exception as exc:
                logger.error("hourly cycle error: %s", exc)
            try:
                save_state(state, cfg.state_path)
            except OSError as exc:
                logger.warning("state save failed: %s", exc)
            last_hourly_s = now

        _interruptible_sleep(cfg.poll_interval_s, stop_flag)

    logger.info("Shutting down")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="NTP offset calibration daemon (WAN-only, no SHM)"
    )
    p.add_argument("--poll-interval", dest="poll_interval_s", type=float,
                   default=DEFAULT_POLL_INTERVAL_S)
    p.add_argument("--hourly-window", dest="hourly_window_s", type=float,
                   default=DEFAULT_HOURLY_WINDOW_S)
    p.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    p.add_argument("--state-path", type=Path, default=DEFAULT_STATE_PATH)
    p.add_argument("--status-path", type=Path, default=DEFAULT_STATUS_PATH)
    p.add_argument("--chrony-conf", type=Path, default=DEFAULT_CHRONY_CONF)
    p.add_argument("--chronyc-binary", default=DEFAULT_CHRONYC)
    p.add_argument("--chronyc-timeout", dest="chronyc_timeout_s", type=float,
                   default=DEFAULT_CHRONYC_TIMEOUT_S)
    p.add_argument("--raw-log-retention-h", type=float,
                   default=DEFAULT_RAW_LOG_RETENTION_H)
    p.add_argument("--hourly-log-retention-h", type=float,
                   default=DEFAULT_HOURLY_LOG_RETENTION_H)
    p.add_argument("--min-stability-samples", type=int,
                   default=DEFAULT_MIN_STABILITY_SAMPLES)
    p.add_argument("--cv-threshold", type=float, default=DEFAULT_CV_THRESHOLD)
    p.add_argument("--override-threshold", type=float,
                   default=DEFAULT_OVERRIDE_THRESHOLD)
    p.add_argument("--active-pool-size", type=int, default=DEFAULT_ACTIVE_POOL_SIZE)
    p.add_argument("--initial-servers", nargs="*",
                   default=DEFAULT_INITIAL_SERVERS)
    p.add_argument("--no-reload-chrony", dest="reload_chrony",
                   action="store_false", default=True)
    p.add_argument("--notify-email", default=DEFAULT_NOTIFY_EMAIL,
                   help="Email address for reference server outage alerts")
    p.add_argument("--smtp-host", default=DEFAULT_SMTP_HOST)
    p.add_argument("--smtp-port", type=int, default=DEFAULT_SMTP_PORT)
    p.add_argument("--reference-timeout-s", type=float,
                   default=DEFAULT_REFERENCE_TIMEOUT_S,
                   help="Seconds a preferred server must be absent before alerting")
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    cfg = parse_args(argv)
    run_daemon(cfg)
    return 0


if __name__ == "__main__":
    sys.exit(main())
