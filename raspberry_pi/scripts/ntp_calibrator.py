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
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean, median, stdev
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

    Returns a dict with keys:
      ref_id_name, stratum, synchronized,
      rms_offset_s, root_delay_s, root_dispersion_s, freq_ppm, update_interval_s
    Numeric fields are None when the line is absent or unparseable.
    freq_ppm is positive for "slow", negative for "fast" (chrony convention).
    """
    ref_id_name = ""
    stratum = 0
    synchronized = False
    rms_offset_s = None
    root_delay_s = None
    root_dispersion_s = None
    freq_ppm = None
    update_interval_s = None

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
        elif line.startswith("RMS offset"):
            m = re.search(r":\s+([+-]?\d+(?:\.\d+)?)\s+seconds", line)
            if m:
                rms_offset_s = float(m.group(1))
        elif line.startswith("Root delay"):
            m = re.search(r":\s+([+-]?\d+(?:\.\d+)?)\s+seconds", line)
            if m:
                root_delay_s = float(m.group(1))
        elif line.startswith("Root dispersion"):
            m = re.search(r":\s+([+-]?\d+(?:\.\d+)?)\s+seconds", line)
            if m:
                root_dispersion_s = float(m.group(1))
        elif line.startswith("Frequency"):
            m = re.search(r":\s+([+-]?\d+(?:\.\d+)?)\s+ppm\s+(slow|fast)", line)
            if m:
                val = float(m.group(1))
                freq_ppm = val if m.group(2) == "slow" else -val
        elif line.startswith("Update interval"):
            m = re.search(r":\s+([+-]?\d+(?:\.\d+)?)\s+seconds", line)
            if m:
                update_interval_s = float(m.group(1))

    return {
        "ref_id_name": ref_id_name,
        "stratum": stratum,
        "synchronized": synchronized,
        "rms_offset_s": rms_offset_s,
        "root_delay_s": root_delay_s,
        "root_dispersion_s": root_dispersion_s,
        "freq_ppm": freq_ppm,
        "update_interval_s": update_interval_s,
    }


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
_CV_GATE = 3.0  # servers with CV >= this are excluded (WAN NTP stdev varies 10-60× daily)


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
    """Return the initial daemon state."""
    return {
        "mode": "LOGGING",
        "started_utc": None,
        "last_heartbeat_utc": None,
    }


_VALID_MODES = frozenset({"LOGGING", "CALIBRATING", "MONITORING", "FROZEN", "NOTIFIED"})


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
# Group U — compute_modal_ref_id
# ---------------------------------------------------------------------------

def compute_modal_ref_id(tracking_rows: list, start_ts: float, end_ts: float) -> Optional[str]:
    """Return the most frequent non-empty ref_id within [start_ts, end_ts].

    tracking_rows: list of dicts with "timestamp" (float) and "ref_id" (str).
    Returns None if no rows fall in the window or all ref_ids are empty.
    """
    counts: Counter = Counter()
    for row in tracking_rows:
        ts = row.get("timestamp", 0.0)
        if start_ts <= ts <= end_ts:
            ref = row.get("ref_id", "")
            if ref:
                counts[ref] += 1
    if not counts:
        return None
    return counts.most_common(1)[0][0]


# ---------------------------------------------------------------------------
# Group V — compute_precision_metrics
# ---------------------------------------------------------------------------

def _percentile(values: list, pct: float) -> float:
    """Linear-interpolation percentile of a list (pct in 0–100)."""
    if not values:
        return float("nan")
    sorted_vals = sorted(values)
    k = (len(sorted_vals) - 1) * pct / 100.0
    lo = int(k)
    hi = min(lo + 1, len(sorted_vals) - 1)
    frac = k - lo
    return sorted_vals[lo] * (1.0 - frac) + sorted_vals[hi] * frac


def compute_precision_metrics(samples: list, ref_id: str,
                               window_start_ts: float, window_end_ts: float,
                               recent_window_s: float = 1800.0) -> dict:
    """Compute typical, recent, and worst-case stdev for ref_id in the given window.

    samples: list of dicts with "timestamp", "server", "stdev_s".
    recent_window_s: seconds before window_end_ts to call "recent" (default 30 min).
    Returns dict with keys typical_stdev_s, recent_stdev_s, worst_case_stdev_s.
    All values are None when no data is available for ref_id in the window.
    """
    window_vals = [
        s["stdev_s"] for s in samples
        if s.get("server") == ref_id
        and window_start_ts <= s.get("timestamp", 0.0) <= window_end_ts
    ]
    if not window_vals:
        return {"typical_stdev_s": None, "recent_stdev_s": None, "worst_case_stdev_s": None}

    recent_start = window_end_ts - recent_window_s
    recent_vals = [
        s["stdev_s"] for s in samples
        if s.get("server") == ref_id
        and recent_start <= s.get("timestamp", 0.0) <= window_end_ts
    ]

    return {
        "typical_stdev_s": median(window_vals),
        "recent_stdev_s": median(recent_vals) if recent_vals else None,
        "worst_case_stdev_s": _percentile(window_vals, 95),
    }


# ---------------------------------------------------------------------------
# Group W — check_source_diversity
# ---------------------------------------------------------------------------

def check_source_diversity(sources: dict, min_reachable: int = 3,
                            min_reach: int = 200) -> dict:
    """Return source diversity status based on reach register values.

    sources: output of parse_sources {server: {reach, state, stratum}}.
    min_reach: minimum reach value to consider a source reachable (octal 377 = 255 = all polls).
    Returns {"reachable_count", "is_adequate", "low_reach_servers"}.
    """
    reachable = [srv for srv, info in sources.items() if info.get("reach", 0) >= min_reach]
    low_reach = [srv for srv, info in sources.items() if info.get("reach", 0) < min_reach]
    return {
        "reachable_count": len(reachable),
        "is_adequate": len(reachable) >= min_reachable,
        "low_reach_servers": low_reach,
    }


# ---------------------------------------------------------------------------
# Group X — check_stall_indicators
# ---------------------------------------------------------------------------

# 3× maxpoll-6 (64s) — update_interval above this signals a chrony stall
STALL_UPDATE_INTERVAL_S = 200.0

# Total calibration window; 2nd half (12-24h) used for server selection and metrics
CALIBRATION_WINDOW_S = 24 * 3600.0


def check_stall_indicators(tracking: dict, prev_tracking: Optional[dict] = None) -> list:
    """Return a list of stall condition strings from chrony tracking snapshots.

    Conditions:
      "stall"              — update_interval_s > STALL_UPDATE_INTERVAL_S
      "source_switch"      — ref_id_name changed vs prev_tracking
      "dispersion_growing" — root_dispersion_s increased vs prev_tracking
    prev_tracking is optional; source_switch and dispersion_growing require it.
    """
    conditions = []

    update_interval = tracking.get("update_interval_s")
    if update_interval is not None and update_interval > STALL_UPDATE_INTERVAL_S:
        conditions.append("stall")

    if prev_tracking is not None:
        curr_ref = tracking.get("ref_id_name", "")
        prev_ref = prev_tracking.get("ref_id_name", "")
        if curr_ref and prev_ref and curr_ref != prev_ref:
            conditions.append("source_switch")

        curr_disp = tracking.get("root_dispersion_s")
        prev_disp = prev_tracking.get("root_dispersion_s")
        if curr_disp is not None and prev_disp is not None and curr_disp > prev_disp:
            conditions.append("dispersion_growing")

    return conditions


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


_SAMPLE_HEADER = ["timestamp", "server", "offset_s", "stdev_s", "stratum", "reach", "np"]
_TRACKING_HEADER = [
    "timestamp", "ref_id", "stratum", "synchronized",
    "rms_offset_s", "root_delay_s", "root_dispersion_s", "freq_ppm", "update_interval_s",
]


def _write_csv_row(log_path: str, header: list, row: list) -> None:
    """Append one row; write header first if the file does not yet exist."""
    path = Path(log_path)
    write_header = not path.exists() or path.stat().st_size == 0
    with open(log_path, "a", newline="") as f:
        w = csv.writer(f)
        if write_header:
            w.writerow(header)
        w.writerow(row)


def log_sample(log_path: str, timestamp: float, server: str,
               offset_s: float, stdev_s: float,
               stratum: int, reach: int, np_val: int) -> None:
    """Append one sourcestats row (with stratum/reach/np) to the sample log."""
    _write_csv_row(log_path, _SAMPLE_HEADER,
                   [timestamp, server, offset_s, stdev_s, stratum, reach, np_val])


def log_tracking_sample(log_path: str, timestamp: float, tracking: dict) -> None:
    """Append one chrony tracking snapshot to the tracking log."""
    _write_csv_row(log_path, _TRACKING_HEADER, [
        timestamp,
        tracking.get("ref_id_name", ""),
        tracking.get("stratum", ""),
        tracking.get("synchronized", ""),
        tracking.get("rms_offset_s", ""),
        tracking.get("root_delay_s", ""),
        tracking.get("root_dispersion_s", ""),
        tracking.get("freq_ppm", ""),
        tracking.get("update_interval_s", ""),
    ])


def load_samples(log_path: str, since_timestamp: float) -> list:
    """Load samples from log CSV written at or after since_timestamp.

    Handles both the old headerless 4-column format and the new 7-column
    format with a header row.
    """
    samples = []
    try:
        with open(log_path, newline="") as f:
            for row in csv.reader(f):
                if not row:
                    continue
                # Skip header row
                if row[0] == "timestamp":
                    continue
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


def load_tracking_samples(log_path: str, since_timestamp: float) -> list:
    """Load tracking.csv rows at or after since_timestamp.

    Returns list of dicts with keys matching _TRACKING_HEADER.
    Handles missing files and malformed rows gracefully.
    """
    rows = []
    try:
        with open(log_path, newline="") as f:
            for row in csv.reader(f):
                if not row:
                    continue
                if row[0] == "timestamp":
                    continue
                if len(row) < 2:
                    continue
                try:
                    ts = float(row[0])
                    if ts < since_timestamp:
                        continue
                    rows.append({
                        "timestamp": ts,
                        "ref_id": row[1] if len(row) > 1 else "",
                        "stratum": int(row[2]) if len(row) > 2 and row[2] else 0,
                        "synchronized": row[3] == "True" if len(row) > 3 else False,
                        "rms_offset_s": float(row[4]) if len(row) > 4 and row[4] else None,
                        "root_delay_s": float(row[5]) if len(row) > 5 and row[5] else None,
                        "root_dispersion_s": float(row[6]) if len(row) > 6 and row[6] else None,
                        "freq_ppm": float(row[7]) if len(row) > 7 and row[7] else None,
                        "update_interval_s": float(row[8]) if len(row) > 8 and row[8] else None,
                    })
                except (ValueError, IndexError):
                    continue
    except OSError:
        pass
    return rows


def restart_chrony() -> None:  # pragma: no cover
    """Restart chrony via systemctl (used by legacy calibration loop only)."""
    try:
        subprocess.run(["systemctl", "restart", "chrony"], check=True, timeout=30)
        logger.info("chrony restarted")
    except Exception as exc:
        logger.error("Failed to restart chrony: %s", exc)


# ---------------------------------------------------------------------------
# Legacy loop functions — kept for reference; not called by run_daemon.
# These implemented the old CALIBRATING → MONITORING state machine.
# ---------------------------------------------------------------------------

def _run_calibration_loop(config: dict, state: dict, log_path: str) -> dict:  # pragma: no cover
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


def _run_daily_check(config: dict, state: dict, log_path: str,  # pragma: no cover
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


def _run_monitoring_loop(config: dict, state: dict, log_path: str) -> None:  # pragma: no cover
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
    """Top-level daemon entry point.

    Runs indefinitely as a pure logger: every poll_interval_s seconds it
    collects chronyc sourcestats, sources, and tracking and appends one row
    per server to samples.csv and one row to tracking.csv.  No chrony.conf
    is modified and no server selection is performed.
    """
    state_path = config.get("state_path", "/var/lib/ntp-calibrator/state.json")
    log_path = config.get("log_path", "/var/log/ntp-calibrator/samples.csv")
    tracking_log_path = config.get(
        "tracking_log_path", "/var/log/ntp-calibrator/tracking.csv"
    )
    poll_s = config.get("poll_interval_s", DEFAULT_POLL_INTERVAL_S)
    heartbeat_interval_s = config.get("heartbeat_interval_s", 86400)

    for path in (log_path, tracking_log_path, state_path):
        os.makedirs(os.path.dirname(path), exist_ok=True)

    ntfy_cfg = (load_ntfy_config(config["ntfy_config_path"])
                if config.get("ntfy_config_path") else None)

    state = default_state()
    state["started_utc"] = datetime.now(timezone.utc).isoformat()
    save_state(state, state_path)

    if ntfy_cfg:
        send_ntfy(ntfy_cfg,
                  title="NTP logger started",
                  message=f"ntp-calibrator logging daemon started on {os.uname().nodename}",
                  priority="default", tags="satellite")

    logger.info("Logging daemon started; poll_interval=%ss", poll_s)

    prev_tracking: Optional[dict] = None
    last_diversity_alert_ts: float = 0.0
    _DIVERSITY_ALERT_INTERVAL_S = 3600.0

    while True:
        now_ts = time.time()
        now_utc = datetime.now(timezone.utc)

        sourcestats = parse_sourcestats(run_chronyc_sourcestats())
        sources = parse_sources(run_chronyc_sources())
        tracking = parse_tracking(run_chronyc_tracking())

        for srv, info in sourcestats.items():
            src = sources.get(srv, {})
            log_sample(
                log_path, now_ts, srv,
                info["offset_s"], info["stdev_s"],
                src.get("stratum", 0),
                src.get("reach", 0),
                info["np"],
            )

        log_tracking_sample(tracking_log_path, now_ts, tracking)

        # Source diversity check — alert immediately if < 3 sources have reach >= 200
        diversity = check_source_diversity(sources)
        if not diversity["is_adequate"]:
            logger.warning(
                "Source diversity inadequate: %d sources with reach >= 200 (need 3); "
                "low-reach: %s",
                diversity["reachable_count"],
                ", ".join(diversity["low_reach_servers"]) or "none",
            )
            if ntfy_cfg and now_ts - last_diversity_alert_ts >= _DIVERSITY_ALERT_INTERVAL_S:
                send_ntfy(
                    ntfy_cfg,
                    title="NTP source diversity alert",
                    message=(
                        f"Only {diversity['reachable_count']} NTP sources reachable on "
                        f"{os.uname().nodename} (minimum 3). "
                        "Single-source failure risk elevated."
                    ),
                    priority="high",
                    tags="warning",
                )
                last_diversity_alert_ts = now_ts

        # Stall indicator check
        stall_conditions = check_stall_indicators(tracking, prev_tracking)
        for condition in stall_conditions:
            logger.warning(
                "Stall indicator: %s (update_interval=%.1fs ref=%s)",
                condition,
                tracking.get("update_interval_s") or 0.0,
                tracking.get("ref_id_name", "?"),
            )
        prev_tracking = tracking

        # After 24h, compute calibration metrics from the 2nd-half window (hours 12-24)
        start_ts = _parse_iso_utc(state["started_utc"]).timestamp()
        if not state.get("calibration_complete") and now_ts - start_ts >= CALIBRATION_WINDOW_S:
            window_start = start_ts + CALIBRATION_WINDOW_S / 2
            window_end = start_ts + CALIBRATION_WINDOW_S

            t_rows = load_tracking_samples(tracking_log_path, window_start)
            modal_ref = compute_modal_ref_id(t_rows, window_start, window_end)

            cal_samples = load_samples(log_path, window_start)
            cal_samples = [s for s in cal_samples if s["timestamp"] <= window_end]
            metrics = (
                compute_precision_metrics(cal_samples, modal_ref, window_start, window_end)
                if modal_ref
                else {"typical_stdev_s": None, "recent_stdev_s": None, "worst_case_stdev_s": None}
            )

            state["calibration_complete"] = True
            state["calibration_ref_id"] = modal_ref
            state["precision_typical_ms"] = (
                metrics["typical_stdev_s"] * 1000 if metrics["typical_stdev_s"] else None
            )
            state["precision_recent_ms"] = (
                metrics["recent_stdev_s"] * 1000 if metrics["recent_stdev_s"] else None
            )
            state["precision_worst_case_ms"] = (
                metrics["worst_case_stdev_s"] * 1000 if metrics["worst_case_stdev_s"] else None
            )
            save_state(state, state_path)
            logger.info(
                "24h calibration window complete: ref=%s typical=%.3fms worst=%.3fms",
                modal_ref,
                state["precision_typical_ms"] or 0.0,
                state["precision_worst_case_ms"] or 0.0,
            )
            if ntfy_cfg and modal_ref:
                send_ntfy(
                    ntfy_cfg,
                    title="NTP calibration metrics ready",
                    message=(
                        f"24h calibration complete on {os.uname().nodename}\n"
                        f"Reference: {modal_ref}\n"
                        f"Typical precision: {state['precision_typical_ms']:.3f} ms\n"
                        f"Worst-case precision: {state['precision_worst_case_ms']:.3f} ms"
                    ),
                    priority="default",
                    tags="white_check_mark",
                )

        # Daily heartbeat notification
        last_hb = state.get("last_heartbeat_utc")
        last_hb_ts = _parse_iso_utc(last_hb).timestamp() if last_hb else 0.0
        if now_ts - last_hb_ts >= heartbeat_interval_s and ntfy_cfg:
            sample_count = sum(1 for _ in open(log_path)) if os.path.exists(log_path) else 0
            send_ntfy(ntfy_cfg,
                      title="NTP logger heartbeat",
                      message=(
                          f"ntp-calibrator running on {os.uname().nodename}\n"
                          f"Tracking: {tracking.get('ref_id_name', '?')} "
                          f"(stratum {tracking.get('stratum', '?')})\n"
                          f"Samples logged: {sample_count}"
                      ),
                      priority="default", tags="bar_chart")
            state["last_heartbeat_utc"] = now_utc.isoformat()
            save_state(state, state_path)

        time.sleep(poll_s)


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
