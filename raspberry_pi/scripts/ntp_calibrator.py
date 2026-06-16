"""NTP offset calibration daemon.

Two-phase operation:
  1. CALIBRATING — run for a configurable period (default 6h), collect per-server
     offsets, pick the best WAN server as reference (offset=0), write calibrated
     offset terms for all backups into chrony.conf, restart chrony.
  2. MONITORING — log per-server data continuously; enter FROZEN when the reference
     disappears (offsets stay locked); notify via ntfy if it's gone >6h.

Reference hierarchy: GNSS > LAN NTP > best WAN server.
"""
from __future__ import annotations

import csv
import json
import logging
import os
import re
import subprocess
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean, stdev
from typing import Optional

logger = logging.getLogger("ntp_calibrator")

# ---------------------------------------------------------------------------
# Constants / defaults
# ---------------------------------------------------------------------------

BLOCK_BEGIN = "# BEGIN ntp-calibrator-managed"
BLOCK_END = "# END ntp-calibrator-managed"

GNSS_REF_IDS = frozenset({"PPS", "GPS"})

DEFAULT_CALIBRATION_DURATION_S = 6 * 3600
DEFAULT_POLL_INTERVAL_S = 60
DEFAULT_FREEZE_TIMEOUT_S = 6 * 3600
DEFAULT_MIN_SAMPLES = 10
DEFAULT_POLL = 6  # minpoll / maxpoll → 64 s

DEFAULT_SERVERS = [
    "time.apple.com",
    "time.google.com",
    "time.cloudflare.com",
    "time.aws.amazon.com",
    "time.windows.com",
]
DEFAULT_POOL = "pool.ntp.org"

# ---------------------------------------------------------------------------
# Unit parsing helper
# ---------------------------------------------------------------------------

_UNIT_MULTIPLIERS = {"ns": 1e-9, "us": 1e-6, "ms": 1e-3, "s": 1.0}


def _parse_time_value(s: str) -> Optional[float]:
    """Convert a chronyc time string like '+49us', '-1.23ms', '300ns' to seconds."""
    m = re.match(r"^([+-]?\d+(?:\.\d+)?)(ns|us|ms|s)$", s.strip())
    if not m:
        return None
    return float(m.group(1)) * _UNIT_MULTIPLIERS[m.group(2)]


def _parse_iso_utc(s: str) -> datetime:
    """Parse an ISO 8601 UTC string to a timezone-aware datetime (Python 3.8 compat)."""
    clean = re.sub(r"([+-]\d{2}:\d{2}|Z)$", "", s)
    return datetime.fromisoformat(clean).replace(tzinfo=timezone.utc)


# ---------------------------------------------------------------------------
# Group A — parse_sourcestats
# ---------------------------------------------------------------------------

def parse_sourcestats(text: str) -> dict:
    """Parse ``chronyc sourcestats [-n]`` output.

    Returns {server: {"offset_s": float, "stdev_s": float, "np": int}}.
    """
    result = {}
    for line in text.splitlines():
        if not line or line.startswith("Name/") or line.startswith("="):
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        name = parts[0]
        # chronyc truncates names longer than 25 chars with '>'; skip these —
        # they are pool.ntp.org members with long reverse-DNS names that cannot
        # be written as valid server lines in chrony.conf.
        if name.endswith(">"):
            continue
        try:
            np_val = int(parts[1])
        except ValueError:
            continue
        offset_s = _parse_time_value(parts[6])
        stdev_s = _parse_time_value(parts[7])
        if offset_s is None or stdev_s is None:
            continue
        result[name] = {"offset_s": offset_s, "stdev_s": stdev_s, "np": np_val}
    return result


# ---------------------------------------------------------------------------
# Group B — parse_sources
# ---------------------------------------------------------------------------

def parse_sources(text: str) -> dict:
    """Parse ``chronyc sources [-n]`` output.

    Returns {server: {"state": str, "stratum": int, "reach": int}}.
    State chars: '*' selected, '+' acceptable, '-' excluded, '?' unreachable.
    """
    result = {}
    for line in text.splitlines():
        # Lines look like: ^* 17.253.2.35   1   6   377  ...
        m = re.match(r"^\^([*+\-? ~x])\s+(\S+)\s+(\d+)\s+\d+\s+(\d+)", line)
        if not m:
            continue
        state, name = m.group(1), m.group(2)
        stratum, reach = int(m.group(3)), int(m.group(4))
        result[name] = {"state": state, "stratum": stratum, "reach": reach}
    return result


# ---------------------------------------------------------------------------
# Group C — parse_tracking
# ---------------------------------------------------------------------------

def parse_tracking(text: str) -> dict:
    """Parse ``chronyc tracking`` output.

    Returns {"ref_id_name": str, "stratum": int, "synchronized": bool}.
    ref_id_name is the parenthetical name shown by chronyc (e.g. "PPS", "17.253.2.35").
    """
    ref_id_name = ""
    stratum = 0
    synchronized = False

    for line in text.splitlines():
        if line.startswith("Reference ID"):
            m = re.search(r":\s+[0-9A-Fa-f]+\s+\(([^)]*)\)", line)
            if m:
                ref_id_name = m.group(1).strip()
        elif line.startswith("Stratum"):
            m = re.search(r":\s+(\d+)", line)
            if m:
                stratum = int(m.group(1))
        elif line.startswith("Leap status"):
            synchronized = "Normal" in line

    return {"ref_id_name": ref_id_name, "stratum": stratum, "synchronized": synchronized}


# ---------------------------------------------------------------------------
# Group D — detect_reference_tier
# ---------------------------------------------------------------------------

def detect_reference_tier(tracking: dict, lan_servers: list) -> str:
    """Return the current reference tier: "GNSS", "LAN", "WAN", or "NONE".

    GNSS beats LAN beats WAN; NONE when chrony is not synchronized.
    """
    if not tracking.get("synchronized"):
        return "NONE"
    name = tracking.get("ref_id_name", "")
    if name in GNSS_REF_IDS:
        return "GNSS"
    if name in lan_servers:
        return "LAN"
    return "WAN"


# ---------------------------------------------------------------------------
# Group E — compute_server_stats
# ---------------------------------------------------------------------------

def compute_server_stats(samples: list, min_samples: int = DEFAULT_MIN_SAMPLES) -> dict:
    """Compute mean and stdev of offset_s per server from sample dicts.

    Each sample: {"server": str, "offset_s": float, ...}.
    Returns {server: {"mean_offset_s": float, "stdev_s": float, "n": int}}.
    Servers with < min_samples are excluded.
    """
    by_server: dict = {}
    for s in samples:
        by_server.setdefault(s["server"], []).append(s["offset_s"])

    result = {}
    for srv, offsets in by_server.items():
        n = len(offsets)
        if n < min_samples:
            continue
        m = mean(offsets)
        sd = stdev(offsets) if n > 1 else 0.0
        result[srv] = {"mean_offset_s": m, "stdev_s": sd, "n": n}
    return result


def compute_stdev_stats(samples: list, min_samples: int = DEFAULT_MIN_SAMPLES) -> dict:
    """Compute mean, stdev, and CV of the sourcestats Std Dev column per server.

    Each sample must have {"server": str, "stdev_s": float}.
    Returns {server: {"mean_stdev_s": float, "stdev_of_stdev_s": float, "cv": float, "n": int}}.
    CV = stdev_of_stdev_s / mean_stdev_s; used for the stability gate in server scoring.
    Servers with < min_samples are excluded.
    """
    by_server: dict = {}
    for s in samples:
        by_server.setdefault(s["server"], []).append(s["stdev_s"])

    result = {}
    for srv, vals in by_server.items():
        n = len(vals)
        if n < min_samples:
            continue
        m = mean(vals)
        sd = stdev(vals) if n > 1 else 0.0
        cv = sd / m if m > 0 else 0.0
        result[srv] = {"mean_stdev_s": m, "stdev_of_stdev_s": sd, "cv": cv, "n": n}
    return result


# ---------------------------------------------------------------------------
# Group F — select_reference_server
# ---------------------------------------------------------------------------

# Stratum penalty factors for composite score.
# A stratum-2 server must have a 25% better raw score to beat stratum 1.
# Stratum 3 is strongly disfavored; stratum 4+ is effectively excluded.
_STRATUM_FACTORS = {1: 1.0, 2: 4 / 3, 3: 4.0}
_STRATUM_FACTOR_DEFAULT = 10.0
_CV_GATE = 2.0  # servers with CV >= this are excluded


def compute_composite_score(mean_stdev_s: float, cv: float, stratum: int) -> float:
    """Return the composite server quality score (lower = better).

    score = mean_stdev_s * (1 + cv) * stratum_factor
    Returns inf when cv >= _CV_GATE (server is too erratic to trust).
    """
    if cv >= _CV_GATE:
        return float("inf")
    factor = _STRATUM_FACTORS.get(stratum, _STRATUM_FACTOR_DEFAULT)
    return mean_stdev_s * (1 + cv) * factor


def score_servers(stdev_stats: dict, strata: dict) -> dict:
    """Compute composite scores for all servers in stdev_stats.

    stdev_stats: output of compute_stdev_stats.
    strata: {server: stratum_int} from parse_sources.
    Returns {server: composite_score}.
    Servers missing from strata are treated as high-stratum (factor 10.0).
    """
    result = {}
    for srv, info in stdev_stats.items():
        stratum = strata.get(srv, 4)  # unknown → high penalty
        result[srv] = compute_composite_score(info["mean_stdev_s"], info["cv"], stratum)
    return result


def select_reference_server(scores: dict) -> Optional[str]:
    """Return the server with the lowest finite composite score.

    scores: {server: composite_score} — output of score_servers.
    Returns None when scores is empty or all scores are inf.
    """
    finite = {s: v for s, v in scores.items() if v < float("inf")}
    if not finite:
        return None
    return min(finite, key=lambda s: finite[s])


# ---------------------------------------------------------------------------
# Group G — compute_relative_offsets
# ---------------------------------------------------------------------------

def compute_relative_offsets(stats: dict, reference_server: str) -> dict:
    """Compute each server's offset relative to reference_server.

    Returns {server: offset_s} where reference_server maps to 0.0 and
    every other server maps to (its_mean - ref_mean).
    Returns {} when reference_server is not in stats.
    """
    if reference_server not in stats:
        return {}
    ref_mean = stats[reference_server]["mean_offset_s"]
    return {srv: info["mean_offset_s"] - ref_mean for srv, info in stats.items()}


# ---------------------------------------------------------------------------
# Group H — build_server_line
# ---------------------------------------------------------------------------

def build_server_line(server: str, is_reference: bool, is_noselect: bool,
                      offset_s: Optional[float], poll: int = DEFAULT_POLL) -> str:
    """Format one chrony.conf server line.

    - Reference: adds ``prefer``, omits offset term.
    - Backup with offset: adds ``offset <value>``.
    - Noselect: adds ``noselect``, no offset.
    - Calibration-phase (no flags, no offset): plain server line.
    """
    parts = [f"server {server} iburst minpoll {poll} maxpoll {poll}"]
    if is_reference:
        parts.append("prefer")
    elif is_noselect:
        parts.append("noselect")
    elif offset_s is not None:
        parts.append(f"offset {offset_s:+.9f}")
    return " ".join(parts)


# ---------------------------------------------------------------------------
# Group O — select_active_servers
# ---------------------------------------------------------------------------

def select_active_servers(scores: dict, active_pool_size: int = 3):
    """Split servers into active (top-N by composite score) and noselect (the rest).

    scores: {server: composite_score} — output of score_servers.
    CV-gated servers (score == inf) always go to noselect regardless of pool_size.
    Returns (active_servers, noselect_servers) as a pair of sets.
    """
    if not scores:
        return set(), set()
    finite = {s: v for s, v in scores.items() if v < float("inf")}
    gated = set(scores) - set(finite)
    sorted_finite = sorted(finite, key=lambda s: finite[s])
    active = set(sorted_finite[:active_pool_size])
    noselect = set(sorted_finite[active_pool_size:]) | gated
    return active, noselect


# ---------------------------------------------------------------------------
# Group N — build_calibrated_conf_lines
# ---------------------------------------------------------------------------

def build_calibrated_conf_lines(reference_server: str, calibrated_offsets: dict,
                                 noselect_servers: set,
                                 poll: int = DEFAULT_POLL,
                                 noselect_poll: int = 10) -> list:
    """Build the managed-block server lines after a completed calibration.

    calibrated_offsets: {server: offset_s} — active servers with offset terms.
      The reference server has offset_s == 0.0.
    noselect_servers: polled infrequently for monitoring only.
    noselect_poll: minpoll/maxpoll for noselect servers (default 10 ≈ 17 min).
    """
    lines = []
    for srv, offset_s in calibrated_offsets.items():
        lines.append(build_server_line(
            srv,
            is_reference=(srv == reference_server),
            is_noselect=False,
            offset_s=offset_s,
            poll=poll,
        ))
    for srv in sorted(noselect_servers):
        lines.append(build_server_line(
            srv, is_reference=False, is_noselect=True, offset_s=None,
            poll=noselect_poll,
        ))
    return lines


# ---------------------------------------------------------------------------
# Group I — chrony.conf managed block read/write
# ---------------------------------------------------------------------------

def read_managed_block(conf_path: str) -> list:
    """Return the non-blank server lines inside the managed block.

    Returns [] when the file is missing or no block is present.
    """
    try:
        text = Path(conf_path).read_text()
    except OSError:
        return []

    in_block = False
    lines = []
    for line in text.splitlines():
        if line.strip() == BLOCK_BEGIN:
            in_block = True
            continue
        if line.strip() == BLOCK_END:
            break
        if in_block and line.strip():
            lines.append(line)
    return lines


def write_managed_block(conf_path: str, server_lines: list) -> None:
    """Replace the managed block in chrony.conf with server_lines.

    Creates the block at the end of the file when none exists.
    Writes atomically via a .tmp sidecar.
    """
    path = Path(conf_path)
    try:
        text = path.read_text()
    except OSError:
        text = ""

    block = "\n".join([BLOCK_BEGIN] + list(server_lines) + [BLOCK_END])

    if BLOCK_BEGIN in text:
        # Splice out the old block and insert the new one in its place.
        out_lines = []
        in_block = False
        for line in text.splitlines():
            if line.strip() == BLOCK_BEGIN:
                in_block = True
                out_lines.append(block)
                continue
            if in_block:
                if line.strip() == BLOCK_END:
                    in_block = False
                continue
            out_lines.append(line)
        new_text = "\n".join(out_lines)
    else:
        new_text = text.rstrip("\n") + "\n\n" + block

    if not new_text.endswith("\n"):
        new_text += "\n"

    tmp_path = path.with_suffix(".tmp")
    tmp_path.write_text(new_text)
    tmp_path.rename(path)


# ---------------------------------------------------------------------------
# Group J — send_ntfy
# ---------------------------------------------------------------------------

def load_ntfy_config(config_path: str) -> Optional[dict]:
    """Load ntfy config from a JSON file. Returns None on any error."""
    try:
        with open(config_path) as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return None


def send_ntfy(config: dict, title: str, message: str,
              priority: str = "default", tags: str = "") -> None:
    """POST a notification to ntfy. Swallows connection errors."""
    url = f"{config['server']}/{config['topic']}"
    req = urllib.request.Request(url, data=message.encode(), method="POST")
    req.add_header("Title", title)
    req.add_header("Priority", priority)
    req.add_header("Tags", tags)
    try:
        with urllib.request.urlopen(req):
            pass
    except Exception as exc:
        logger.warning("ntfy notification failed: %s", exc)


# ---------------------------------------------------------------------------
# Group K — state persistence
# ---------------------------------------------------------------------------

def default_state() -> dict:
    """Return the initial daemon state for a fresh calibration run."""
    return {
        "mode": "CALIBRATING",
        "calibration_start_utc": None,
        "calibration_complete_utc": None,
        "reference_server": None,
        "reference_tier": None,
        "calibrated_offsets": {},
        "calibration_composite_score": None,
        "frozen_since_utc": None,
        "notified_utc": None,
        "last_daily_check_utc": None,
    }


_VALID_MODES = frozenset({"CALIBRATING", "MONITORING", "FROZEN", "NOTIFIED"})


def load_state(state_path: str) -> dict:
    """Load daemon state from JSON.

    Returns default_state() on any error or if the state has an unrecognized
    mode (e.g. a state file left by the previous SHM-based daemon).
    """
    try:
        with open(state_path) as f:
            state = json.load(f)
        if state.get("mode") not in _VALID_MODES:
            return default_state()
        return state
    except (OSError, json.JSONDecodeError):
        return default_state()


def save_state(state: dict, state_path: str) -> None:
    """Atomically write state dict to state_path as JSON."""
    path = Path(state_path)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(state, indent=2))
    tmp.rename(path)


# ---------------------------------------------------------------------------
# Group L — check_reference_reachable
# ---------------------------------------------------------------------------

def check_reference_reachable(sources: dict, reference_server: str) -> bool:
    """Return True when reference_server is in sources with reach > 0."""
    entry = sources.get(reference_server)
    return entry is not None and entry["reach"] > 0


# ---------------------------------------------------------------------------
# Group M — update_freeze_state
# ---------------------------------------------------------------------------

def update_freeze_state(state: dict, is_reachable: bool,
                         now_utc: datetime, freeze_timeout_s: float) -> dict:
    """Return a new state dict after applying the reachability event.

    Transitions:
      MONITORING + unreachable  → FROZEN (records frozen_since_utc)
      MONITORING + reachable    → MONITORING (no change)
      FROZEN + reachable        → MONITORING (clears frozen_since_utc)
      FROZEN + unreachable, elapsed < timeout → FROZEN
      FROZEN + unreachable, elapsed >= timeout → NOTIFIED (records notified_utc)
      NOTIFIED + reachable      → MONITORING (clears both timestamps)
      NOTIFIED + unreachable    → NOTIFIED (no change)
    """
    state = dict(state)  # shallow copy — never mutate caller's dict
    mode = state["mode"]
    now_iso = now_utc.isoformat()

    if mode == "MONITORING":
        if not is_reachable:
            state["mode"] = "FROZEN"
            state["frozen_since_utc"] = now_iso

    elif mode == "FROZEN":
        if is_reachable:
            state["mode"] = "MONITORING"
            state["frozen_since_utc"] = None
        else:
            frozen_since = _parse_iso_utc(state["frozen_since_utc"])
            elapsed = (now_utc - frozen_since).total_seconds()
            if elapsed >= freeze_timeout_s:
                state["mode"] = "NOTIFIED"
                state["notified_utc"] = now_iso

    elif mode == "NOTIFIED":
        if is_reachable:
            state["mode"] = "MONITORING"
            state["frozen_since_utc"] = None
            state["notified_utc"] = None

    return state


# ---------------------------------------------------------------------------
# Group S — check_daily_criteria
# ---------------------------------------------------------------------------

# Threshold multipliers / absolute limits for daily re-evaluation.
_CRIT1_SCORE_FACTOR = 2.0    # criterion 1: reference score > N× calibration baseline
_CRIT2_OFFSET_DRIFT_S = 5e-4  # criterion 2: active server offset drift threshold (0.5 ms)
_CRIT3_SCORE_RATIO = 0.75    # criterion 3: non-reference score < this fraction of reference


def check_daily_criteria(state: dict, current_scores: dict,
                          calibrated_offsets_now: dict) -> list:
    """Return a list of triggered daily-check criterion numbers (1, 2, 3).

    Criterion 1: reference composite score has degraded beyond 2× its calibration baseline.
    Criterion 2: any active server's current mean offset has drifted >0.5 ms from calibrated.
    Criterion 3: any non-reference server scores better than 75% of the reference score
                 (i.e., a different server would be a substantially better reference).
    """
    reference = state.get("reference_server")
    baseline = state.get("calibration_composite_score")
    calibrated_offsets = state.get("calibrated_offsets", {})
    triggered = []

    # Criterion 1 — reference performance degraded
    ref_score = current_scores.get(reference, float("inf"))
    if baseline and ref_score > _CRIT1_SCORE_FACTOR * baseline:
        triggered.append(1)

    # Criterion 2 — active server offset drift
    for srv, cal_offset in calibrated_offsets.items():
        if srv == reference:
            continue
        now_offset = calibrated_offsets_now.get(srv)
        if now_offset is not None and abs(now_offset - cal_offset) > _CRIT2_OFFSET_DRIFT_S:
            triggered.append(2)
            break  # one trigger is enough

    # Criterion 3 — a non-reference server is substantially better
    threshold = ref_score * _CRIT3_SCORE_RATIO
    for srv, score in current_scores.items():
        if srv != reference and score < threshold:
            triggered.append(3)
            break

    return triggered


# ---------------------------------------------------------------------------
# Group T — log_daily_evaluation
# ---------------------------------------------------------------------------

def log_daily_evaluation(log_path: str, timestamp: float, server_scores: dict,
                          stdev_stats: dict, offset_drifts: dict) -> None:
    """Append one JSONL record to the daily evaluation log.

    Each record captures a timestamped snapshot of all server quality metrics,
    composite scores, and offset drifts from calibration. Designed for
    post-hoc threshold tuning across multiple deployments.
    """
    record = {
        "timestamp": timestamp,
        "server_scores": {k: v for k, v in server_scores.items()},
        "stdev_stats": stdev_stats,
        "offset_drifts": offset_drifts,
    }
    with open(log_path, "a") as f:
        f.write(json.dumps(record) + "\n")


# ---------------------------------------------------------------------------
# Daemon infrastructure (not covered by unit tests — requires chrony / root)
# ---------------------------------------------------------------------------

def _run_chronyc(*args) -> str:
    """Run chronyc; return stdout or '' on failure."""
    try:
        r = subprocess.run(["chronyc"] + list(args),
                           capture_output=True, text=True, timeout=10)
        return r.stdout
    except Exception as exc:
        logger.warning("chronyc %s failed: %s", " ".join(args), exc)
        return ""


def run_chronyc_sourcestats() -> str:
    return _run_chronyc("sourcestats", "-n")


def run_chronyc_sources() -> str:
    return _run_chronyc("sources", "-n")


def run_chronyc_tracking() -> str:
    return _run_chronyc("tracking")


def log_sample(log_path: str, timestamp: float, server: str,
               offset_s: float, stdev_s: float) -> None:
    """Append one sample row to the CSV log file."""
    with open(log_path, "a", newline="") as f:
        csv.writer(f).writerow([timestamp, server, offset_s, stdev_s])


def load_samples(log_path: str, since_timestamp: float) -> list:
    """Load samples from log CSV written at or after since_timestamp."""
    samples = []
    try:
        with open(log_path, newline="") as f:
            for row in csv.reader(f):
                if len(row) < 4:
                    continue
                try:
                    ts = float(row[0])
                    if ts < since_timestamp:
                        continue
                    samples.append({
                        "timestamp": ts,
                        "server": row[1],
                        "offset_s": float(row[2]),
                        "stdev_s": float(row[3]),
                    })
                except (ValueError, IndexError):
                    continue
    except OSError:
        pass
    return samples


def restart_chrony() -> None:
    """Restart chrony via systemctl."""
    try:
        subprocess.run(["systemctl", "restart", "chrony"], check=True, timeout=30)
        logger.info("chrony restarted")
    except Exception as exc:
        logger.error("Failed to restart chrony: %s", exc)


def _run_calibration_loop(config: dict, state: dict, log_path: str) -> dict:
    """Collect per-server offsets, pick reference, write conf, restart chrony."""
    duration = config.get("calibration_duration_s", DEFAULT_CALIBRATION_DURATION_S)
    poll_s = config.get("poll_interval_s", DEFAULT_POLL_INTERVAL_S)
    min_samples = config.get("min_samples", DEFAULT_MIN_SAMPLES)
    lan_servers = config.get("lan_servers", [])
    conf_path = config.get("chrony_conf_path", "/etc/chrony/chrony.conf")
    poll = config.get("poll", DEFAULT_POLL)
    ntfy_cfg = (load_ntfy_config(config["ntfy_config_path"])
                if config.get("ntfy_config_path") else None)

    start = time.monotonic()
    logger.info("Calibration started; running for %.0fs", duration)

    while time.monotonic() - start < duration:
        stats = parse_sourcestats(run_chronyc_sourcestats())
        now = time.time()
        for srv, info in stats.items():
            log_sample(log_path, now, srv, info["offset_s"], info["stdev_s"])
        time.sleep(poll_s)

    tracking = parse_tracking(run_chronyc_tracking())
    reference_tier = detect_reference_tier(tracking, lan_servers)

    since = time.time() - duration
    samples = load_samples(log_path, since)

    # Offset stats — for computing chrony.conf offset correction terms.
    offset_stats = compute_server_stats(samples, min_samples)
    # Stdev stats — for composite scoring and server selection.
    stdev_stats = compute_stdev_stats(samples, min_samples)

    sources = parse_sources(run_chronyc_sources())
    strata = {srv: info["stratum"] for srv, info in sources.items()}
    scores = score_servers(stdev_stats, strata)
    reference_server = select_reference_server(scores)

    if reference_server is None:
        logger.error("No stable servers found after calibration; will retry")
        return state

    ref_score = scores[reference_server]
    active_pool_size = config.get("active_pool_size", 3)
    noselect_poll = config.get("noselect_poll", 10)
    active_servers, noselect_servers = select_active_servers(scores, active_pool_size)
    active_offset_stats = {s: offset_stats[s] for s in active_servers if s in offset_stats}
    calibrated_offsets = compute_relative_offsets(active_offset_stats, reference_server)
    lines = build_calibrated_conf_lines(
        reference_server, calibrated_offsets,
        noselect_servers=noselect_servers, poll=poll, noselect_poll=noselect_poll,
    )
    write_managed_block(conf_path, lines)
    restart_chrony()

    state = dict(state)
    state["mode"] = "MONITORING"
    state["calibration_complete_utc"] = datetime.now(timezone.utc).isoformat()
    state["reference_server"] = reference_server
    state["reference_tier"] = reference_tier
    state["calibrated_offsets"] = calibrated_offsets
    state["calibration_composite_score"] = ref_score
    logger.info("Calibration complete; reference=%s score=%.3fus tier=%s",
                reference_server, ref_score * 1e6, reference_tier)

    if ntfy_cfg:
        offset_lines = "\n".join(
            f"  {srv}: {offset_s * 1000:+.3f} ms"
            for srv, offset_s in calibrated_offsets.items()
        )
        send_ntfy(
            ntfy_cfg,
            title="NTP calibration complete",
            message=(
                f"Reference: {reference_server} (score={ref_score*1e6:.1f}µs "
                f"tier={reference_tier})\n"
                f"Calibrated offsets:\n{offset_lines}"
            ),
            priority="default",
            tags="white_check_mark",
        )

    return state


def _run_daily_check(config: dict, state: dict, log_path: str,
                      ntfy_cfg: Optional[dict]) -> dict:
    """Run the daily re-evaluation: score all servers, check criteria, notify if needed."""
    daily_log_path = config.get("daily_log_path",
                                 "/var/log/ntp-calibrator/daily_eval.jsonl")
    min_samples_daily = config.get("min_samples_daily", 30)

    now_ts = time.time()
    samples = load_samples(log_path, now_ts - 86400)
    stdev_stats = compute_stdev_stats(samples, min_samples_daily)
    offset_stats = compute_server_stats(samples, min_samples_daily)

    sources = parse_sources(run_chronyc_sources())
    strata = {srv: info["stratum"] for srv, info in sources.items()}
    current_scores = score_servers(stdev_stats, strata)

    reference = state.get("reference_server")
    calibrated = state.get("calibrated_offsets", {})
    now_offsets = {srv: offset_stats[srv]["mean_offset_s"]
                   for srv in calibrated if srv in offset_stats}
    offset_drifts = {srv: now_offsets.get(srv, 0.0) - calibrated.get(srv, 0.0)
                     for srv in calibrated}

    criteria = check_daily_criteria(state, current_scores, now_offsets)
    log_daily_evaluation(daily_log_path, now_ts, current_scores, stdev_stats, offset_drifts)

    if criteria and ntfy_cfg:
        ref_score = current_scores.get(reference, float("inf"))
        score_lines = "\n".join(
            f"  {srv}: {sc * 1e6:.1f}µs composite"
            for srv, sc in sorted(current_scores.items(), key=lambda x: x[1])
            if sc < float("inf")
        )
        criteria_desc = {
            1: f"Reference '{reference}' composite score degraded "
               f"({ref_score*1e6:.1f}µs vs calibration "
               f"{state.get('calibration_composite_score', 0)*1e6:.1f}µs baseline)",
            2: "Active server offset has drifted >0.5 ms from calibrated value",
            3: "A non-reference server is now substantially better than the reference",
        }
        msg = "\n".join(criteria_desc[c] for c in sorted(criteria))
        msg += f"\n\nCurrent server scores:\n{score_lines}"
        msg += "\n\nConsider running with --recalibrate to update server selection."
        send_ntfy(ntfy_cfg, title="NTP calibration needs attention",
                  message=msg, priority="high", tags="warning")
        logger.warning("Daily check: criteria %s triggered", criteria)
    else:
        logger.info("Daily check complete; no criteria triggered")

    state = dict(state)
    state["last_daily_check_utc"] = datetime.now(timezone.utc).isoformat()
    return state


def _run_monitoring_loop(config: dict, state: dict, log_path: str) -> None:
    """Continuously monitor reference reachability; run daily re-evaluation."""
    poll_s = config.get("poll_interval_s", DEFAULT_POLL_INTERVAL_S)
    freeze_timeout_s = config.get("freeze_timeout_s", DEFAULT_FREEZE_TIMEOUT_S)
    daily_check_interval_s = config.get("daily_check_interval_s", 86400)
    state_path = config.get("state_path", "/var/lib/ntp-calibrator/state.json")
    ntfy_cfg = (load_ntfy_config(config["ntfy_config_path"])
                if config.get("ntfy_config_path") else None)

    reference_server = state["reference_server"]

    while True:
        sources = parse_sources(run_chronyc_sources())
        is_reachable = check_reference_reachable(sources, reference_server)

        now_utc = datetime.now(timezone.utc)
        prev_mode = state["mode"]
        state = update_freeze_state(state, is_reachable, now_utc, freeze_timeout_s)

        if state["mode"] == "NOTIFIED" and prev_mode == "FROZEN" and ntfy_cfg:
            send_ntfy(
                ntfy_cfg,
                title="NTP reference lost",
                message=(
                    f"Reference server {reference_server} has been unreachable "
                    f"for >{freeze_timeout_s / 3600:.0f}h. "
                    "Restart with --recalibrate if it will not return."
                ),
                priority="high",
                tags="warning",
            )

        raw = parse_sourcestats(run_chronyc_sourcestats())
        now_ts = time.time()
        for srv, info in raw.items():
            log_sample(log_path, now_ts, srv, info["offset_s"], info["stdev_s"])

        # Daily re-evaluation
        last_check_utc = state.get("last_daily_check_utc")
        last_check_ts = (
            _parse_iso_utc(last_check_utc).timestamp() if last_check_utc else 0.0
        )
        if now_ts - last_check_ts >= daily_check_interval_s:
            state = _run_daily_check(config, state, log_path, ntfy_cfg)

        save_state(state, state_path)
        time.sleep(poll_s)


def run_daemon(config: dict) -> None:
    """Top-level daemon entry point."""
    state_path = config.get("state_path", "/var/lib/ntp-calibrator/state.json")
    log_path = config.get("log_path", "/var/log/ntp-calibrator/samples.csv")
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    os.makedirs(os.path.dirname(state_path), exist_ok=True)

    state = load_state(state_path)
    if config.get("recalibrate"):
        logger.info("--recalibrate: resetting state")
        state = default_state()

    if state["mode"] == "CALIBRATING":
        servers = config.get("servers", DEFAULT_SERVERS)
        pool = config.get("pool", DEFAULT_POOL)
        conf_path = config.get("chrony_conf_path", "/etc/chrony/chrony.conf")
        poll = config.get("poll", DEFAULT_POLL)

        initial_lines = [
            build_server_line(s, is_reference=False, is_noselect=False,
                              offset_s=None, poll=poll)
            for s in servers
        ]
        if pool:
            initial_lines.append(
                f"pool {pool} iburst minpoll {poll} maxpoll {poll} maxsources 4"
            )
        write_managed_block(conf_path, initial_lines)
        state = _run_calibration_loop(config, state, log_path)
        save_state(state, state_path)

    if state["mode"] in ("MONITORING", "FROZEN", "NOTIFIED"):
        _run_monitoring_loop(config, state, log_path)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args(argv=None):
    import argparse
    p = argparse.ArgumentParser(description="NTP offset calibration daemon")
    p.add_argument("--config", default="/etc/ntp-calibrator.json")
    p.add_argument("--recalibrate", action="store_true",
                   help="Reset state and redo calibration from scratch")
    p.add_argument("--log-level", default="INFO",
                   choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return p.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )
    config = {}
    if os.path.exists(args.config):
        with open(args.config) as f:
            config = json.load(f)
    if args.recalibrate:
        config["recalibrate"] = True
    run_daemon(config)


if __name__ == "__main__":
    main()
