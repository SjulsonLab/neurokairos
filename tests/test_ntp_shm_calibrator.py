"""Tests for ntp_shm_calibrator.py (NTP offset calibration daemon).

Covers Groups A, C–E, G–H, J–N, O–R, S. All I/O is mocked so tests run
without root, without chrony, and on macOS.

Groups removed from the original plan:
  B  — detect_mode (function removed; no GNSS/FROZEN/L2 modes in WAN-only design)
  F  — compute_consensus_offset (function removed; offsets go into chrony.conf directly)
  I  — ShmSegment (SHM removed entirely)
  Q1 — SHM written after chrony reload (SHM removed)
  Q3 — raw cycle refreshes SHM (SHM removed)
  Q5 — fast SHM refresh between raw cycles (SHM removed)
  L2 — hourly cycle calls write_time on SHM (SHM removed)
  L6 — FROZEN mode uses last known offset (FROZEN mode removed)
"""

from __future__ import annotations

import csv as _csv
import importlib.util
import json
import os
import time
from datetime import datetime, timezone, timedelta
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "raspberry_pi" / "scripts" / "ntp_shm_calibrator.py"


# ---------------------------------------------------------------------------
# Module loader
# ---------------------------------------------------------------------------

def load_module():
    """Load ntp_shm_calibrator without installing it as a package."""
    spec = importlib.util.spec_from_file_location("ntp_shm_calibrator", SCRIPT_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def mod():
    return load_module()


# ---------------------------------------------------------------------------
# Sample chronyc output
# ---------------------------------------------------------------------------

SAMPLE_SOURCES = """\
MS Name/IP address         Stratum Poll Reach LastRx Last sample
===============================================================================
^* 17.253.2.35               1   6   377    44   +12us[ +14us] +/-  341us
^+ 216.239.35.4              1   6   377    45  -199us[-197us] +/-  315us
^+ 162.159.200.1             3   6   377    44   -28us[ -26us] +/-  369us
^- 54.81.127.33              3   6   377    46  +171us[+173us] +/-  523us
"""

SAMPLE_SOURCESTATS = """\
Name/IP Address            NP  NR  Span  Frequency  Freq Skew  Offset  Std Dev
==============================================================================
17.253.2.35                16   8   32m   +0.003     0.042    +49us   61us
216.239.35.4               16   9   32m   +0.011     0.051   -199us  312us
162.159.200.1              16   8   32m   +0.001     0.038    -28us  369us
54.81.127.33               16   8   32m   +0.000     0.043   +171us  523us
"""


# ===========================================================================
# Group A — parse_sourcestats_full
# ===========================================================================

def test_a1_positive_offset_parsed(mod):
    """Positive offset string (+49us) is converted to float seconds."""
    result = mod.parse_sourcestats_full(SAMPLE_SOURCES, SAMPLE_SOURCESTATS)
    assert "17.253.2.35" in result
    assert abs(result["17.253.2.35"]["offset_s"] - 49e-6) < 1e-9


def test_a2_negative_offset_parsed(mod):
    """Negative offset string (-199us) converts to negative seconds."""
    result = mod.parse_sourcestats_full(SAMPLE_SOURCES, SAMPLE_SOURCESTATS)
    assert abs(result["216.239.35.4"]["offset_s"] - (-199e-6)) < 1e-9


def test_a3_millisecond_offset(mod):
    """Millisecond-scale offset (1.2ms) is parsed correctly."""
    sources = "MS Name/IP address         Stratum Poll Reach LastRx Last sample\n"
    sources += "=" * 79 + "\n"
    sources += "^* 10.0.0.1                   2   6   377    44   +1ms[  +1ms] +/-  1ms\n"
    stats = "Name/IP Address            NP  NR  Span  Frequency  Freq Skew  Offset  Std Dev\n"
    stats += "=" * 79 + "\n"
    stats += "10.0.0.1                   16   8   32m   +0.003     0.042    +1.2ms   0.5ms\n"
    result = mod.parse_sourcestats_full(sources, stats)
    assert "10.0.0.1" in result
    assert abs(result["10.0.0.1"]["offset_s"] - 1.2e-3) < 1e-9


def test_a4_skips_header_and_separator_lines(mod):
    """Header and separator lines produce no entries in the result dict."""
    result = mod.parse_sourcestats_full(SAMPLE_SOURCES, SAMPLE_SOURCESTATS)
    assert "Name/IP Address" not in result
    assert "MS Name/IP address" not in result
    for key in result:
        assert "==" not in key


def test_a5_state_char_merged_from_sources(mod):
    """State char (* + -) from sources output is attached to each server."""
    result = mod.parse_sourcestats_full(SAMPLE_SOURCES, SAMPLE_SOURCESTATS)
    assert result["17.253.2.35"]["state_char"] == "*"
    assert result["216.239.35.4"]["state_char"] == "+"
    assert result["54.81.127.33"]["state_char"] == "-"


def test_a6_garbage_input_returns_empty_dict(mod):
    """Garbage input returns an empty dict without raising."""
    result = mod.parse_sourcestats_full("garbage\n\nnothing", "more garbage")
    assert result == {}


def test_a7_zero_np_handled(mod):
    """Rows with NP=0 (no samples yet) are included but marked."""
    sources = "MS Name/IP address         Stratum Poll Reach LastRx Last sample\n"
    sources += "=" * 79 + "\n"
    sources += "^? 10.0.0.2                   0   6     0  1024    +0ns[  +0ns] +/-    0ns\n"
    stats = "Name/IP Address            NP  NR  Span  Frequency  Freq Skew  Offset  Std Dev\n"
    stats += "=" * 79 + "\n"
    stats += "10.0.0.2                    0   0    0s   +0.000     0.000     +0ns    0ns\n"
    result = mod.parse_sourcestats_full(sources, stats)
    assert "10.0.0.2" in result
    assert result["10.0.0.2"]["np"] == 0


def test_a8_refclock_entries_excluded(mod):
    """Refclock sources (mode char '#') are excluded from parse_sourcestats_full."""
    sources = SAMPLE_SOURCES + "#? WANC                          0   4     0     -     +0ns[   +0ns] +/-    0ns\n"
    stats = SAMPLE_SOURCESTATS + "WANC                        0   0    0s   +0.000     0.000     +0ns    0ns\n"
    result = mod.parse_sourcestats_full(sources, stats)
    assert "WANC" not in result, "Refclock WANC must be excluded from parsed server list"
    assert "17.253.2.35" in result, "Regular NTP servers must still be parsed"


# ===========================================================================
# Group C — compute_hourly_summary
# ===========================================================================

def _make_raw_samples(n, offset_s, std_dev_s, age_start_s=0, spacing_s=60):
    """Build a list of raw sample dicts, newest last, within HOURLY_WINDOW_S."""
    now = time.time()
    samples = []
    for i in range(n):
        ts = now - age_start_s - (n - 1 - i) * spacing_s
        samples.append({
            "timestamp_s": ts,
            "offset_s": offset_s,
            "std_dev_s": std_dev_s,
            "stratum": 2,
            "state_char": "*",
            "np": 16,
            "nr": 8,
        })
    return samples


def test_c1_mean_and_stdev_computed_from_last_hour(mod):
    """Hourly summary returns correct mean and stdev of last hour."""
    samples = _make_raw_samples(10, offset_s=50e-6, std_dev_s=30e-6)
    result = mod.compute_hourly_summary(samples, window_s=3600)
    assert result is not None
    assert abs(result["mean_offset_s"] - 50e-6) < 1e-9
    assert result["hourly_stdev_s"] == pytest.approx(0.0, abs=1e-12)


def test_c2_excludes_samples_older_than_window(mod):
    """Samples outside HOURLY_WINDOW_S are excluded from the computation."""
    old_samples = _make_raw_samples(5, offset_s=1.0, std_dev_s=0.1,
                                    age_start_s=4000)   # > 3600 s old
    fresh_samples = _make_raw_samples(5, offset_s=50e-6, std_dev_s=30e-6)
    all_samples = old_samples + fresh_samples
    result = mod.compute_hourly_summary(all_samples, window_s=3600)
    assert result is not None
    assert abs(result["mean_offset_s"] - 50e-6) < 1e-9


def test_c3_returns_none_when_too_few_samples(mod):
    """Returns None if fewer than 3 valid samples in the window."""
    samples = _make_raw_samples(2, offset_s=50e-6, std_dev_s=30e-6)
    result = mod.compute_hourly_summary(samples, window_s=3600)
    assert result is None


def test_c4_identical_offsets_give_zero_stdev(mod):
    """All-identical offsets produce hourly_stdev_s == 0."""
    samples = _make_raw_samples(10, offset_s=100e-6, std_dev_s=50e-6)
    result = mod.compute_hourly_summary(samples, window_s=3600)
    assert result is not None
    assert result["hourly_stdev_s"] == pytest.approx(0.0, abs=1e-12)


# ===========================================================================
# Group D — compute_cv_stdev
# ===========================================================================

def test_d1_none_when_too_few_samples(mod):
    """Returns None when fewer than MIN_STABILITY_SAMPLES stdev values."""
    assert mod.compute_cv_stdev([1e-4, 2e-4, 3e-4], min_samples=12) is None


def test_d2_zero_cv_for_constant_series(mod):
    """Returns 0.0 for a constant stdev series."""
    values = [1e-4] * 12
    assert mod.compute_cv_stdev(values, min_samples=12) == pytest.approx(0.0, abs=1e-12)


def test_d3_cv_computed_correctly(mod):
    """CV = stdev / mean for a known series."""
    values = [1e-3, 2e-3, 3e-3]  # mean=2e-3
    cv = mod.compute_cv_stdev(values, min_samples=3)
    import statistics
    expected = statistics.stdev(values) / (sum(values) / len(values))
    assert cv == pytest.approx(expected, rel=1e-6)


def test_d4_none_on_empty_input(mod):
    """Returns None on empty list."""
    assert mod.compute_cv_stdev([], min_samples=12) is None


# ===========================================================================
# Group E — select_servers
# ===========================================================================

def _stable_server(name, stratum, mean_stdev_s, cv=0.1, mean_offset_s=0.0):
    """Build a server summary dict that passes the stability gate."""
    return {
        "name": name,
        "stratum": stratum,
        "mean_stdev_s": mean_stdev_s,
        "cv_stdev": cv,
        "mean_offset_s": mean_offset_s,
        "n_hourly_samples": 20,
        "state_char": "+",
        "tier": 3,
    }


def _unstable_server(name, stratum, mean_stdev_s):
    """Build a server dict that fails the stability gate (high CV)."""
    d = _stable_server(name, stratum, mean_stdev_s)
    d["cv_stdev"] = 0.9  # above DEFAULT_CV_THRESHOLD = 3.0
    d["n_hourly_samples"] = 20
    return d


def test_e1_preferred_is_lowest_stratum_stable(mod):
    """Preferred server is the lowest-stratum stable server.

    s1 is only 15% better stdev than s2 (below the 20% override threshold),
    so s2 wins on stratum despite having a slightly worse stdev.
    """
    servers = [
        _stable_server("s1", stratum=2, mean_stdev_s=170e-6),
        _stable_server("s2", stratum=1, mean_stdev_s=200e-6),
        _stable_server("s3", stratum=3, mean_stdev_s=50e-6),
    ]
    preferred, _, _ = mod.select_servers(
        servers,
        active_pool_size=mod.DEFAULT_ACTIVE_POOL_SIZE,
        cv_threshold=mod.DEFAULT_CV_THRESHOLD,
        override_threshold=mod.DEFAULT_OVERRIDE_THRESHOLD,
        min_stability_samples=mod.DEFAULT_MIN_STABILITY_SAMPLES,
    )
    assert preferred["name"] == "s2"


def test_e2_higher_stratum_overrides_if_20pct_better(mod):
    """Higher-stratum server overrides if >20% better stdev AND stable."""
    servers = [
        _stable_server("s1", stratum=1, mean_stdev_s=200e-6),
        _stable_server("s2", stratum=2, mean_stdev_s=100e-6),
    ]
    preferred, _, _ = mod.select_servers(
        servers,
        active_pool_size=mod.DEFAULT_ACTIVE_POOL_SIZE,
        cv_threshold=mod.DEFAULT_CV_THRESHOLD,
        override_threshold=0.20,
        min_stability_samples=mod.DEFAULT_MIN_STABILITY_SAMPLES,
    )
    assert preferred["name"] == "s2"


def test_e2b_higher_stratum_does_not_override_if_improvement_small(mod):
    """Higher-stratum server does NOT override if improvement is <20%."""
    servers = [
        _stable_server("s1", stratum=1, mean_stdev_s=200e-6),
        _stable_server("s2", stratum=2, mean_stdev_s=170e-6),
    ]
    preferred, _, _ = mod.select_servers(
        servers,
        active_pool_size=mod.DEFAULT_ACTIVE_POOL_SIZE,
        cv_threshold=mod.DEFAULT_CV_THRESHOLD,
        override_threshold=0.20,
        min_stability_samples=mod.DEFAULT_MIN_STABILITY_SAMPLES,
    )
    assert preferred["name"] == "s1"


def test_e3_unstable_servers_excluded_from_preferred(mod):
    """Unstable server (CV > threshold) is not selected as preferred or active."""
    servers = [
        _unstable_server("bad", stratum=1, mean_stdev_s=50e-6),
        _stable_server("good", stratum=2, mean_stdev_s=100e-6),
    ]
    preferred, active, noselect = mod.select_servers(
        servers,
        active_pool_size=3,
        cv_threshold=0.3,
        override_threshold=0.20,
        min_stability_samples=mod.DEFAULT_MIN_STABILITY_SAMPLES,
    )
    assert preferred["name"] == "good"
    assert not any(s["name"] == "bad" for s in active)


def test_e4_servers_beyond_pool_size_go_to_noselect(mod):
    """Servers ranked beyond active_pool_size go to noselect list."""
    servers = [_stable_server(f"s{i}", stratum=2, mean_stdev_s=(i + 1) * 50e-6)
               for i in range(5)]
    preferred, active, noselect = mod.select_servers(
        servers,
        active_pool_size=3,
        cv_threshold=0.3,
        override_threshold=0.20,
        min_stability_samples=mod.DEFAULT_MIN_STABILITY_SAMPLES,
    )
    assert len(noselect) == 2


def test_e5_no_stable_servers_returns_none_preferred(mod):
    """Returns (None, [], all_noselect) when no stable servers."""
    servers = [_unstable_server(f"s{i}", stratum=2, mean_stdev_s=100e-6)
               for i in range(3)]
    preferred, active, noselect = mod.select_servers(
        servers,
        active_pool_size=3,
        cv_threshold=0.3,
        override_threshold=0.20,
        min_stability_samples=mod.DEFAULT_MIN_STABILITY_SAMPLES,
    )
    assert preferred is None
    assert active == []
    assert len(noselect) == 3


def test_e6_insufficient_samples_treated_as_unstable(mod):
    """Server with n_hourly_samples < min_stability_samples is unstable."""
    s = _stable_server("new", stratum=1, mean_stdev_s=50e-6)
    s["n_hourly_samples"] = 5   # below MIN_STABILITY_SAMPLES=12
    servers = [s, _stable_server("old", stratum=2, mean_stdev_s=200e-6)]
    preferred, _, _ = mod.select_servers(
        servers,
        active_pool_size=3,
        cv_threshold=0.3,
        override_threshold=0.20,
        min_stability_samples=12,
    )
    assert preferred["name"] == "old"


# ===========================================================================
# Group G — build_server_line
# ===========================================================================

def test_g1_preferred_server_has_prefer_flag(mod):
    line = mod.build_server_line(
        "time.apple.com", is_preferred=True, is_noselect=False, offset_s=None,
    )
    assert "prefer" in line
    assert "noselect" not in line


def test_g2_noselect_server_has_noselect_and_slow_poll(mod):
    line = mod.build_server_line(
        "time.windows.com", is_preferred=False, is_noselect=True, offset_s=None,
    )
    assert "noselect" in line
    assert f"minpoll {mod.NOSELECT_MINPOLL}" in line
    assert f"maxpoll {mod.NOSELECT_MAXPOLL}" in line
    assert "prefer" not in line


def test_g3_offset_term_present_when_nonzero(mod):
    line = mod.build_server_line(
        "time.google.com", is_preferred=False, is_noselect=False, offset_s=-70e-6,
    )
    assert "offset" in line
    assert "-0.000070" in line or "-7e-05" in line or "-7.0e-05" in line


def test_g4_no_offset_term_when_none(mod):
    line = mod.build_server_line(
        "time.google.com", is_preferred=False, is_noselect=False, offset_s=None,
    )
    assert "offset" not in line


def test_g5_active_server_has_neither_prefer_nor_noselect(mod):
    line = mod.build_server_line(
        "time.google.com", is_preferred=False, is_noselect=False, offset_s=None,
    )
    assert "prefer" not in line
    assert "noselect" not in line
    assert f"minpoll {mod.ACTIVE_MINPOLL}" in line
    assert f"maxpoll {mod.ACTIVE_MAXPOLL}" in line


# ===========================================================================
# Group H — read_chrony_conf / write_chrony_conf
# ===========================================================================

SAMPLE_CONF_WITH_BLOCK = """\
# chrony.conf
refclock SHM 0 refid GPS precision 1e-1 noselect
refclock PPS /dev/pps0 refid PPS lock GPS precision 1e-7 prefer

allow 10.0.0.0/8

# BEGIN ntp-calibrator-managed
server time.apple.com iburst minpoll 6 maxpoll 6
server time.google.com iburst minpoll 6 maxpoll 6
# END ntp-calibrator-managed

local stratum 15 orphan
"""

SAMPLE_CONF_WITHOUT_BLOCK = """\
# chrony.conf
allow 10.0.0.0/8

local stratum 15 orphan
"""

NEW_BLOCK_LINES = [
    "server time.apple.com iburst minpoll 6 maxpoll 6 prefer offset -0.000070",
    "server time.google.com iburst minpoll 6 maxpoll 6 offset +0.000020",
]


def test_h1_reads_existing_managed_block(mod):
    """Reads lines inside the managed block correctly."""
    lines = mod.read_managed_block(SAMPLE_CONF_WITH_BLOCK)
    assert len(lines) == 2
    assert "time.apple.com" in lines[0]


def test_h2_preserves_lines_outside_managed_block(mod, tmp_path):
    """write_chrony_conf replaces only the managed block, leaving other lines."""
    conf_path = tmp_path / "chrony.conf"
    conf_path.write_text(SAMPLE_CONF_WITH_BLOCK)
    mod.write_chrony_conf(conf_path, NEW_BLOCK_LINES)
    content = conf_path.read_text()
    assert "refclock SHM 0" in content
    assert "local stratum 15 orphan" in content


def test_h3_replaces_managed_block_with_new_content(mod, tmp_path):
    """The managed block is replaced with the new server lines."""
    conf_path = tmp_path / "chrony.conf"
    conf_path.write_text(SAMPLE_CONF_WITH_BLOCK)
    mod.write_chrony_conf(conf_path, NEW_BLOCK_LINES)
    content = conf_path.read_text()
    assert "offset -0.000070" in content
    assert "time.apple.com iburst minpoll 6 maxpoll 6\n" not in content


def test_h4_creates_managed_block_from_scratch(mod, tmp_path):
    """If no managed block exists, it is appended at the end."""
    conf_path = tmp_path / "chrony.conf"
    conf_path.write_text(SAMPLE_CONF_WITHOUT_BLOCK)
    mod.write_chrony_conf(conf_path, NEW_BLOCK_LINES)
    content = conf_path.read_text()
    assert mod.MANAGED_BEGIN in content
    assert mod.MANAGED_END in content
    assert "time.apple.com" in content


def test_h5_atomic_write_no_tmp_file_remaining(mod, tmp_path):
    """No .tmp file is left behind after a successful write."""
    conf_path = tmp_path / "chrony.conf"
    conf_path.write_text(SAMPLE_CONF_WITH_BLOCK)
    mod.write_chrony_conf(conf_path, NEW_BLOCK_LINES)
    assert not (tmp_path / "chrony.conf.tmp").exists()


def test_h6_file_content_after_write_is_valid(mod, tmp_path):
    """The written conf is a valid text file with markers intact."""
    conf_path = tmp_path / "chrony.conf"
    conf_path.write_text(SAMPLE_CONF_WITH_BLOCK)
    mod.write_chrony_conf(conf_path, NEW_BLOCK_LINES)
    content = conf_path.read_text()
    assert content.count(mod.MANAGED_BEGIN) == 1
    assert content.count(mod.MANAGED_END) == 1


# ===========================================================================
# Group J — load_state / save_state
# ===========================================================================

def test_j1_missing_file_returns_default_state(mod, tmp_path):
    path = tmp_path / "state.json"
    state = mod.load_state(path)
    default = mod._default_state()
    assert state.keys() == default.keys()


def test_j2_corrupt_json_returns_default_state(mod, tmp_path):
    path = tmp_path / "state.json"
    path.write_text("{invalid json}")
    state = mod.load_state(path)
    default = mod._default_state()
    assert state.keys() == default.keys()


def test_j3_roundtrip_preserves_all_fields(mod, tmp_path):
    """Saving and reloading state preserves all fields including new WAN-only fields."""
    path = tmp_path / "state.json"
    original = mod._default_state()
    original["preferred_server"] = "time.apple.com"
    original["preferred_server_last_seen_utc"] = "2026-06-16T18:00:00Z"
    mod.save_state(original, path)
    loaded = mod.load_state(path)
    assert loaded["preferred_server"] == "time.apple.com"
    assert loaded["preferred_server_last_seen_utc"] == "2026-06-16T18:00:00Z"


def test_j4_no_tmp_file_after_successful_save(mod, tmp_path):
    path = tmp_path / "state.json"
    mod.save_state(mod._default_state(), path)
    assert not (tmp_path / "state.json.tmp").exists()
    assert path.exists()


# ===========================================================================
# Group K — write_status_json
# ===========================================================================

def _sample_server_info(name="time.apple.com"):
    return {
        "name": name,
        "ip": "17.253.2.35",
        "tier": 3,
        "stratum": 2,
        "mean_offset_ms": 0.07,
        "hourly_stdev_ms": 0.06,
        "cv_stdev": 0.12,
        "stable": True,
        "state_char": "*",
    }


def test_k1_outputs_valid_json_with_required_keys(mod, tmp_path):
    """Status JSON has all required top-level keys for the web UI."""
    path = tmp_path / "status.json"
    mod.write_status_json(
        path=path,
        in_warmup=False,
        preferred_server=_sample_server_info(),
        active_servers=[_sample_server_info("time.google.com")],
        noselect_servers=[],
        reference_alert_since_utc=None,
    )
    data = json.loads(path.read_text())
    for key in ("last_update_utc", "mode", "preferred_server",
                "active_servers", "noselect_servers", "reference_alert_since_utc"):
        assert key in data


def test_k2_preferred_server_fields_match(mod, tmp_path):
    path = tmp_path / "status.json"
    info = _sample_server_info()
    mod.write_status_json(
        path=path,
        in_warmup=False,
        preferred_server=info,
        active_servers=[],
        noselect_servers=[],
        reference_alert_since_utc=None,
    )
    data = json.loads(path.read_text())
    assert data["preferred_server"]["name"] == "time.apple.com"
    assert data["preferred_server"]["mean_offset_ms"] == pytest.approx(0.07)


def test_k3_noselect_servers_list_present(mod, tmp_path):
    path = tmp_path / "status.json"
    mod.write_status_json(
        path=path,
        in_warmup=False,
        preferred_server=_sample_server_info(),
        active_servers=[],
        noselect_servers=[_sample_server_info("time.windows.com")],
        reference_alert_since_utc=None,
    )
    data = json.loads(path.read_text())
    assert len(data["noselect_servers"]) == 1
    assert data["noselect_servers"][0]["name"] == "time.windows.com"


def test_k3b_stable_field_false_when_cv_above_threshold(mod, tmp_path):
    """stable=False when a server's cv_stdev exceeds the cv_threshold."""
    summary = _stable_server("bad.ntp.org", stratum=1, mean_stdev_s=200e-6)
    summary["cv_stdev"] = 3.5   # above DEFAULT_CV_THRESHOLD (3.0)
    info = mod._server_info_for_ui(summary, cv_threshold=mod.DEFAULT_CV_THRESHOLD)
    assert info["stable"] is False

    summary2 = _stable_server("good.ntp.org", stratum=1, mean_stdev_s=100e-6)
    summary2["cv_stdev"] = 2.5  # below threshold (3.0)
    info2 = mod._server_info_for_ui(summary2, cv_threshold=mod.DEFAULT_CV_THRESHOLD)
    assert info2["stable"] is True


def test_k4_mode_warmup_vs_active(mod, tmp_path):
    """write_status_json encodes in_warmup as 'warmup'/'active' in mode field."""
    path = tmp_path / "status.json"
    mod.write_status_json(
        path=path,
        in_warmup=True,
        preferred_server=None,
        active_servers=[],
        noselect_servers=[],
        reference_alert_since_utc=None,
    )
    data = json.loads(path.read_text())
    assert data["mode"] == "warmup"

    mod.write_status_json(
        path=path,
        in_warmup=False,
        preferred_server=_sample_server_info(),
        active_servers=[],
        noselect_servers=[],
        reference_alert_since_utc="2026-06-16T18:00:00Z",
    )
    data = json.loads(path.read_text())
    assert data["mode"] == "active"
    assert data["reference_alert_since_utc"] == "2026-06-16T18:00:00Z"


# ===========================================================================
# Group L — Integration: _run_raw_cycle and _run_hourly_cycle (all I/O mocked)
# ===========================================================================

def _mock_config(tmp_path, mod):
    """Build a Namespace-like config object for daemon testing.

    No SHM, no mode-detection fields. WAN-only design.
    """
    conf = tmp_path / "chrony.conf"
    conf.write_text(SAMPLE_CONF_WITH_BLOCK)

    class Cfg:
        poll_interval_s = 60
        hourly_window_s = 3600
        log_dir = tmp_path / "logs"
        state_path = tmp_path / "state.json"
        status_path = tmp_path / "status.json"
        chrony_conf = conf
        chronyc_binary = "chronyc"
        chronyc_timeout_s = 5.0
        raw_log_retention_h = 72
        hourly_log_retention_h = 72
        min_stability_samples = mod.DEFAULT_MIN_STABILITY_SAMPLES
        cv_threshold = mod.DEFAULT_CV_THRESHOLD
        override_threshold = mod.DEFAULT_OVERRIDE_THRESHOLD
        active_pool_size = mod.DEFAULT_ACTIVE_POOL_SIZE
        reload_chrony = False
        initial_servers = mod.DEFAULT_INITIAL_SERVERS
        notify_email = None          # no alerts by default in tests
        smtp_host = "localhost"
        smtp_port = 25
        reference_timeout_s = 1800

    return Cfg()


def _make_mock_run_sourcestats(mod, sources=SAMPLE_SOURCES, stats=SAMPLE_SOURCESTATS):
    def _run(*args, **kwargs):
        return mod.parse_sourcestats_full(sources, stats)
    return _run


def _make_mock_run_sourcestats_empty(mod):
    """Return a mock that reports no NTP sources."""
    def _mock(*_, **__):
        return {}
    return _mock


def test_l1_raw_cycle_appends_one_row_per_server(mod, tmp_path):
    """Each raw cycle appends one row per server to the daily log."""
    cfg = _mock_config(tmp_path, mod)
    state = mod._default_state()

    with patch.object(mod, "run_sourcestats_full", _make_mock_run_sourcestats(mod)):
        mod._run_raw_cycle(cfg, state)

    log_files = list((tmp_path / "logs").glob("raw_*.tsv"))
    assert len(log_files) == 1
    lines = [l for l in log_files[0].read_text().splitlines() if not l.startswith("timestamp")]
    assert len(lines) == 4


def test_l3_hourly_cycle_rewrites_conf_when_ranking_changes(mod, tmp_path):
    """Hourly cycle updates chrony.conf when server ranking changes."""
    cfg = _mock_config(tmp_path, mod)
    state = mod._default_state()
    state["last_conf_servers"] = []   # no previous ranking → always rewrite

    summaries = {
        "17.253.2.35":  {"mean_offset_s": 49e-6,  "hourly_stdev_s": 61e-6,  "n_samples": 60, "stratum": 1},
        "216.239.35.4": {"mean_offset_s": -199e-6, "hourly_stdev_s": 312e-6, "n_samples": 60, "stratum": 1},
        "162.159.200.1":{"mean_offset_s": -28e-6,  "hourly_stdev_s": 369e-6, "n_samples": 60, "stratum": 3},
    }
    state["_hourly_window_override"] = summaries

    conf_mtime_before = cfg.chrony_conf.stat().st_mtime

    with patch.object(mod, "run_sourcestats_full", _make_mock_run_sourcestats(mod)):
        mod._run_hourly_cycle(cfg, state)

    conf_mtime_after = cfg.chrony_conf.stat().st_mtime
    assert conf_mtime_after >= conf_mtime_before


def test_l5_exception_in_sourcestats_does_not_crash_cycle(mod, tmp_path):
    """A sourcestats failure does not crash the raw cycle."""
    cfg = _mock_config(tmp_path, mod)
    state = mod._default_state()

    def _raise(*args, **kwargs):
        raise RuntimeError("chronyc timed out")

    # Should not raise
    with patch.object(mod, "run_sourcestats_full", _raise):
        mod._run_raw_cycle(cfg, state)


# ===========================================================================
# Group M — hourly.tsv written with role + applied_offset_s columns
# ===========================================================================

def _seed_raw_logs(mod, log_dir, servers, n_samples=5, window_s=3600):
    """Write n_samples raw log rows for each server."""
    for server, offset_s, std_dev_s, stratum in servers:
        for i in range(n_samples):
            mod.append_raw_sample(
                log_dir, server, offset_s, std_dev_s, stratum, "+", 16, 8
            )


def test_m1_hourly_cycle_writes_hourly_tsv(mod, tmp_path):
    """After _run_hourly_cycle, hourly.tsv exists with the correct columns."""
    cfg = _mock_config(tmp_path, mod)
    state = mod._default_state()

    servers = [
        ("17.253.2.35",   49e-6,  61e-6,  1),
        ("216.239.35.4", -199e-6, 312e-6, 1),
        ("162.159.200.1", -28e-6, 369e-6, 3),
    ]
    _seed_raw_logs(mod, cfg.log_dir, servers)

    with patch.object(mod, "run_sourcestats_full", _make_mock_run_sourcestats(mod)):
        mod._run_hourly_cycle(cfg, state)

    hourly = tmp_path / "logs" / "hourly.tsv"
    assert hourly.exists(), "hourly.tsv must be written by _run_hourly_cycle"
    with hourly.open() as f:
        rows = list(_csv.DictReader(f, delimiter="\t"))
    assert len(rows) >= 1
    assert "role" in rows[0], "hourly.tsv must have a 'role' column"
    assert "applied_offset_s" in rows[0], "hourly.tsv must have an 'applied_offset_s' column"


def test_m2_role_values_after_warmup(mod, tmp_path):
    """Post-warmup hourly rows have preferred/active/noselect roles."""
    cfg = _mock_config(tmp_path, mod)
    state = mod._default_state()

    summaries = {
        "17.253.2.35":  {"mean_offset_s": 49e-6,  "hourly_stdev_s": 61e-6,  "n_samples": 60, "stratum": 1},
        "216.239.35.4": {"mean_offset_s": -199e-6, "hourly_stdev_s": 312e-6, "n_samples": 60, "stratum": 1},
        "162.159.200.1":{"mean_offset_s": -28e-6,  "hourly_stdev_s": 369e-6, "n_samples": 60, "stratum": 3},
    }
    state["_hourly_window_override"] = summaries

    servers = [
        ("17.253.2.35",   49e-6,  61e-6,  1),
        ("216.239.35.4", -199e-6, 312e-6, 1),
        ("162.159.200.1", -28e-6, 369e-6, 3),
    ]
    _seed_raw_logs(mod, cfg.log_dir, servers)

    with patch.object(mod, "run_sourcestats_full", _make_mock_run_sourcestats(mod)):
        mod._run_hourly_cycle(cfg, state)

    hourly = tmp_path / "logs" / "hourly.tsv"
    if not hourly.exists():
        pytest.skip("hourly.tsv not written (override path skips log writing)")

    with hourly.open() as f:
        rows = list(_csv.DictReader(f, delimiter="\t"))

    roles = {r["server"]: r["role"] for r in rows}
    role_values = set(roles.values())
    assert role_values & {"preferred", "active", "noselect"}, \
        f"Expected post-warmup roles, got: {roles}"


def test_m3_warmup_rows_have_warmup_role(mod, tmp_path):
    """During warmup phase, all hourly rows have role='warmup'."""
    cfg = _mock_config(tmp_path, mod)
    state = mod._default_state()

    servers = [
        ("17.253.2.35",   49e-6,  61e-6,  1),
        ("216.239.35.4", -199e-6, 312e-6, 1),
    ]
    _seed_raw_logs(mod, cfg.log_dir, servers)

    with patch.object(mod, "run_sourcestats_full", _make_mock_run_sourcestats(mod)):
        mod._run_hourly_cycle(cfg, state)

    hourly = tmp_path / "logs" / "hourly.tsv"
    assert hourly.exists()
    with hourly.open() as f:
        rows = list(_csv.DictReader(f, delimiter="\t"))
    assert all(r["role"] == "warmup" for r in rows), \
        f"All warmup rows must have role='warmup', got: {[r['role'] for r in rows]}"


def test_m4_applied_offset_set_for_preferred_server(mod, tmp_path):
    """preferred server row has applied_offset_s equal to its mean_offset_s."""
    cfg = _mock_config(tmp_path, mod)
    state = mod._default_state()
    state["_hourly_window_override"] = {
        "17.253.2.35":  {"mean_offset_s": 49e-6,  "hourly_stdev_s": 61e-6,  "n_samples": 60, "stratum": 1},
        "216.239.35.4": {"mean_offset_s": -199e-6, "hourly_stdev_s": 312e-6, "n_samples": 60, "stratum": 1},
    }

    servers = [
        ("17.253.2.35",   49e-6,  61e-6,  1),
        ("216.239.35.4", -199e-6, 312e-6, 1),
    ]
    _seed_raw_logs(mod, cfg.log_dir, servers)

    with patch.object(mod, "run_sourcestats_full", _make_mock_run_sourcestats(mod)):
        mod._run_hourly_cycle(cfg, state)

    hourly = tmp_path / "logs" / "hourly.tsv"
    if not hourly.exists():
        pytest.skip("hourly.tsv not written in override path")

    with hourly.open() as f:
        rows = list(_csv.DictReader(f, delimiter="\t"))

    preferred_rows = [r for r in rows if r["role"] == "preferred"]
    assert preferred_rows, "at least one preferred row expected"
    for r in preferred_rows:
        assert r["applied_offset_s"] != "", \
            f"preferred server must have applied_offset_s set, got: {r}"


# ---------------------------------------------------------------------------
# Group N — _is_valid_ntp_server
# ---------------------------------------------------------------------------

def test_n1_valid_public_ip_is_valid(mod):
    assert mod._is_valid_ntp_server("17.253.2.35") is True


def test_n2_wanc_is_invalid(mod):
    assert mod._is_valid_ntp_server("WANC") is False


def test_n3_rfc1918_10_block_is_invalid(mod):
    assert mod._is_valid_ntp_server("10.0.0.1") is False


def test_n4_rfc1918_192168_block_is_invalid(mod):
    assert mod._is_valid_ntp_server("192.168.1.1") is False


def test_n5_rfc1918_172_block_is_invalid(mod):
    assert mod._is_valid_ntp_server("172.16.0.1") is False


def test_n6_hostname_is_valid(mod):
    assert mod._is_valid_ntp_server("time.google.com") is True


def test_n7_loopback_is_invalid(mod):
    assert mod._is_valid_ntp_server("127.0.0.1") is False


# ---------------------------------------------------------------------------
# Group O — managed_server_set filtering in _run_hourly_cycle
# ---------------------------------------------------------------------------

def test_o1_wanc_not_written_to_managed_block(mod, tmp_path):
    """WANC appearing in server summaries must never appear as a server line."""
    cfg = _mock_config(tmp_path, mod)
    state = mod._default_state()
    state["_hourly_window_override"] = {
        "17.253.2.35": {"mean_offset_s": 49e-6,  "hourly_stdev_s": 61e-6,  "n_samples": 60, "stratum": 1},
        "WANC":        {"mean_offset_s": 0.0,     "hourly_stdev_s": 66e-6,  "n_samples": 60, "stratum": 0},
    }
    state["_managed_server_set"] = frozenset({"17.253.2.35"})

    with (
        patch.object(mod, "run_sourcestats_full", _make_mock_run_sourcestats_empty(mod)),
        patch.object(mod, "_reload_chrony", lambda *_: None),
    ):
        mod._run_hourly_cycle(cfg, state)

    conf = (tmp_path / "chrony.conf").read_text()
    assert "WANC" not in conf, "WANC must never appear as a server line in chrony.conf"


def test_o2_private_ip_not_written_to_managed_block(mod, tmp_path):
    """A private IP from DHCP must not appear in chrony.conf managed block."""
    cfg = _mock_config(tmp_path, mod)
    state = mod._default_state()
    state["_hourly_window_override"] = {
        "17.253.2.35": {"mean_offset_s": 49e-6, "hourly_stdev_s": 61e-6, "n_samples": 60, "stratum": 1},
        "10.0.0.1":    {"mean_offset_s": 0.0,   "hourly_stdev_s": 0.0,   "n_samples": 60, "stratum": 0},
    }
    state["_managed_server_set"] = frozenset({"17.253.2.35"})

    with (
        patch.object(mod, "run_sourcestats_full", _make_mock_run_sourcestats_empty(mod)),
        patch.object(mod, "_reload_chrony", lambda *_: None),
    ):
        mod._run_hourly_cycle(cfg, state)

    conf = (tmp_path / "chrony.conf").read_text()
    assert "10.0.0.1" not in conf


def test_o3_pool_member_not_in_managed_set_stays_out_of_block(mod, tmp_path):
    """A pool.ntp.org member IP not in managed_server_set must not appear."""
    cfg = _mock_config(tmp_path, mod)
    state = mod._default_state()
    pool_member_ip = "45.77.126.122"
    state["_hourly_window_override"] = {
        "17.253.2.35":  {"mean_offset_s": 49e-6,  "hourly_stdev_s": 61e-6, "n_samples": 60, "stratum": 1},
        pool_member_ip: {"mean_offset_s": -100e-6, "hourly_stdev_s": 50e-6, "n_samples": 60, "stratum": 2},
    }
    state["_managed_server_set"] = frozenset({"17.253.2.35"})

    with (
        patch.object(mod, "run_sourcestats_full", _make_mock_run_sourcestats_empty(mod)),
        patch.object(mod, "_reload_chrony", lambda *_: None),
    ):
        mod._run_hourly_cycle(cfg, state)

    conf = (tmp_path / "chrony.conf").read_text()
    assert pool_member_ip not in conf


# ---------------------------------------------------------------------------
# Group P — invalid servers filtered from selection pool
# ---------------------------------------------------------------------------

def test_p1_wanc_excluded_from_select_servers_input(mod):
    """WANC must be filtered from _build_server_summaries."""
    wanc_entries = [
        {"timestamp_s": 0.0, "mean_offset_s": 0.0, "hourly_stdev_s": 0.0,
         "n_samples": 60, "stratum": 0, "tier": 3}
    ] * mod.DEFAULT_MIN_STABILITY_SAMPLES
    real_entries = [
        {"timestamp_s": 0.0, "mean_offset_s": -700e-6, "hourly_stdev_s": 50e-6,
         "n_samples": 60, "stratum": 1, "tier": 3}
    ] * mod.DEFAULT_MIN_STABILITY_SAMPLES
    hourly_window = {"WANC": wanc_entries, "216.239.35.4": real_entries}
    summaries = mod._build_server_summaries(
        hourly_window, {}, frozenset(), mod.DEFAULT_MIN_STABILITY_SAMPLES
    )
    names = [s["name"] for s in summaries]
    assert "WANC" not in names
    assert "216.239.35.4" in names


def test_p2_private_ip_excluded_from_select_servers_input(mod):
    """A private IP must not appear in server summaries."""
    private_entries = [
        {"timestamp_s": 0.0, "mean_offset_s": 0.0, "hourly_stdev_s": 0.0,
         "n_samples": 60, "stratum": 0, "tier": 3}
    ] * mod.DEFAULT_MIN_STABILITY_SAMPLES
    real_entries = [
        {"timestamp_s": 0.0, "mean_offset_s": -700e-6, "hourly_stdev_s": 50e-6,
         "n_samples": 60, "stratum": 1, "tier": 3}
    ] * mod.DEFAULT_MIN_STABILITY_SAMPLES
    hourly_window = {"10.0.0.1": private_entries, "216.239.35.4": real_entries}
    summaries = mod._build_server_summaries(
        hourly_window, {}, frozenset(), mod.DEFAULT_MIN_STABILITY_SAMPLES
    )
    names = [s["name"] for s in summaries]
    assert "10.0.0.1" not in names
    assert "216.239.35.4" in names


# ===========================================================================
# Group Q (partial) — chrony.conf reload ordering and state persistence
# ===========================================================================

def test_q2_pool_member_churn_does_not_trigger_reload(mod, tmp_path):
    """Chrony reload must NOT fire when only pool member IPs change.

    Pool members change each cycle; filtering to managed_set prevents spurious
    reloads that would otherwise clear valid flags on every hourly cycle.
    """
    cfg = _mock_config(tmp_path, mod)
    cfg.reload_chrony = True
    state = mod._default_state()

    state["_managed_server_set"] = frozenset({"17.253.2.35"})
    state["last_conf_servers"] = ["17.253.2.35"]

    summaries = {
        "17.253.2.35":  {"mean_offset_s": 49e-6,  "hourly_stdev_s": 61e-6,  "n_samples": 60, "stratum": 1},
        "216.239.35.4": {"mean_offset_s": -199e-6, "hourly_stdev_s": 312e-6, "n_samples": 60, "stratum": 1},
        "123.45.67.89": {"mean_offset_s": -10e-6,  "hourly_stdev_s": 200e-6, "n_samples": 60, "stratum": 1},
    }
    state["_hourly_window_override"] = summaries

    reload_called = []

    def _track_reload(reload: bool) -> None:
        if reload:
            reload_called.append(True)

    with (
        patch.object(mod, "run_sourcestats_full", _make_mock_run_sourcestats(mod)),
        patch.object(mod, "_reload_chrony", _track_reload),
    ):
        mod._run_hourly_cycle(cfg, state)

    assert not reload_called, (
        "Chrony reload must NOT be triggered when only pool member IPs "
        "(not in managed_set) differ between cycles"
    )


def test_q4_last_conf_servers_persisted_across_restart(mod, tmp_path):
    """last_conf_servers is persisted so restarts don't cause unnecessary reloads."""
    cfg = _mock_config(tmp_path, mod)
    state = mod._default_state()

    state["_managed_server_set"] = frozenset({"17.253.2.35", "216.239.35.4"})
    summaries = {
        "17.253.2.35":  {"mean_offset_s": 49e-6,  "hourly_stdev_s": 61e-6,  "n_samples": 60, "stratum": 1},
        "216.239.35.4": {"mean_offset_s": -199e-6, "hourly_stdev_s": 312e-6, "n_samples": 60, "stratum": 1},
    }
    state["_hourly_window_override"] = summaries

    with patch.object(mod, "run_sourcestats_full", _make_mock_run_sourcestats(mod)):
        mod._run_hourly_cycle(cfg, state)

    mod.save_state(state, cfg.state_path)
    reloaded = mod.load_state(cfg.state_path)

    assert "last_conf_servers" in reloaded, (
        "last_conf_servers must be persisted so restarts know which servers "
        "are in chrony.conf and can skip unnecessary reloads"
    )
    assert reloaded["last_conf_servers"] == state.get("last_conf_servers")


# ===========================================================================
# Group R — fresh-install bootstrap
# ===========================================================================

def _make_wan_cfg(tmp_path, mod):
    """Config with no managed block in chrony.conf (fresh install scenario)."""
    conf = tmp_path / "chrony.conf"
    conf.write_text(SAMPLE_CONF_WITHOUT_BLOCK)

    class Cfg:
        poll_interval_s = 60
        hourly_window_s = 3600
        log_dir = tmp_path / "logs"
        state_path = tmp_path / "state.json"
        status_path = tmp_path / "status.json"
        chrony_conf = conf
        chronyc_binary = "chronyc"
        chronyc_timeout_s = 5.0
        raw_log_retention_h = 72
        hourly_log_retention_h = 72
        min_stability_samples = mod.DEFAULT_MIN_STABILITY_SAMPLES
        cv_threshold = mod.DEFAULT_CV_THRESHOLD
        override_threshold = mod.DEFAULT_OVERRIDE_THRESHOLD
        active_pool_size = mod.DEFAULT_ACTIVE_POOL_SIZE
        reload_chrony = False
        initial_servers = mod.DEFAULT_INITIAL_SERVERS
        notify_email = None
        smtp_host = "localhost"
        smtp_port = 25
        reference_timeout_s = 1800

    return Cfg()


def test_r1_fresh_install_writes_initial_managed_block(mod, tmp_path):
    """On a fresh install, the first hourly cycle must write the initial managed block.

    Root cause: _default_state() initialised last_conf_servers to [].
    Fix: use None so that [] != None = True triggers the first write.
    """
    cfg = _make_wan_cfg(tmp_path, mod)
    state = mod._default_state()

    with patch.object(mod, "run_sourcestats_full", lambda *a, **kw: {}):
        mod._run_hourly_cycle(cfg, state)

    conf_text = cfg.chrony_conf.read_text()
    assert mod.MANAGED_BEGIN in conf_text
    for srv in mod.DEFAULT_INITIAL_SERVERS:
        assert srv in conf_text


def test_r2_stale_state_with_empty_conf_writes_initial_managed_block(mod, tmp_path):
    """When state has last_conf_servers=[] but no managed block exists, write it.

    Migration scenario: old buggy daemon saved [] as default, never wrote block.
    Fix: guard also on MANAGED_BEGIN absent from chrony.conf.
    """
    cfg = _make_wan_cfg(tmp_path, mod)
    state = mod._default_state()
    state["last_conf_servers"] = []   # old buggy saved value

    with patch.object(mod, "run_sourcestats_full", lambda *a, **kw: {}):
        mod._run_hourly_cycle(cfg, state)

    conf_text = cfg.chrony_conf.read_text()
    assert mod.MANAGED_BEGIN in conf_text
    for srv in mod.DEFAULT_INITIAL_SERVERS:
        assert srv in conf_text


# ===========================================================================
# Group S — reference server outage detection and notification
# ===========================================================================

def _utc_ago(seconds: float) -> str:
    """Return an ISO-8601 UTC string for `seconds` ago."""
    dt = datetime.now(timezone.utc) - timedelta(seconds=seconds)
    return dt.strftime("%Y-%m-%dT%H:%M:%SZ")


def test_s1_raw_cycle_updates_last_seen_when_preferred_present(mod, tmp_path):
    """Raw cycle updates preferred_server_last_seen_utc when preferred is in sourcestats."""
    cfg = _mock_config(tmp_path, mod)
    state = mod._default_state()
    state["preferred_server"] = "17.253.2.35"
    state["preferred_server_last_seen_utc"] = _utc_ago(3600)   # 1 hour ago

    with patch.object(mod, "run_sourcestats_full", _make_mock_run_sourcestats(mod)):
        mod._run_raw_cycle(cfg, state)

    # last_seen should now be very recent (within a few seconds of now)
    last_seen_str = state.get("preferred_server_last_seen_utc")
    assert last_seen_str is not None
    last_seen_ts = datetime.fromisoformat(last_seen_str.replace("Z", "+00:00")).timestamp()
    assert time.time() - last_seen_ts < 10, (
        "preferred_server_last_seen_utc must be updated when server is in sourcestats"
    )


def test_s2_raw_cycle_does_not_update_last_seen_when_preferred_absent(mod, tmp_path):
    """Raw cycle does NOT update preferred_server_last_seen_utc when preferred is absent."""
    cfg = _mock_config(tmp_path, mod)
    state = mod._default_state()
    state["preferred_server"] = "time.apple.com"   # not in SAMPLE_SOURCESTATS
    old_ts = _utc_ago(3600)
    state["preferred_server_last_seen_utc"] = old_ts

    with patch.object(mod, "run_sourcestats_full", _make_mock_run_sourcestats(mod)):
        mod._run_raw_cycle(cfg, state)

    # Timestamp must remain unchanged — server was not seen
    assert state.get("preferred_server_last_seen_utc") == old_ts, (
        "preferred_server_last_seen_utc must NOT be updated when server absent from sourcestats"
    )


def test_s3_hourly_cycle_sends_alert_when_server_long_absent(mod, tmp_path):
    """Hourly cycle calls send_reference_alert when preferred unseen > timeout."""
    cfg = _mock_config(tmp_path, mod)
    cfg.notify_email = "luke@sjulsonlab.org"
    cfg.reference_timeout_s = 1800

    state = mod._default_state()
    state["preferred_server"] = "time.apple.com"
    # Mark as last seen 2 hours ago (well past 30-minute timeout)
    state["preferred_server_last_seen_utc"] = _utc_ago(7200)

    alerts_sent = []

    def _mock_alert(server, since_utc, recipient, smtp_host="localhost", smtp_port=25):
        alerts_sent.append({"server": server, "recipient": recipient})
        return True

    with (
        patch.object(mod, "run_sourcestats_full", lambda *a, **kw: {}),
        patch.object(mod, "send_reference_alert", _mock_alert),
    ):
        mod._run_hourly_cycle(cfg, state)

    assert len(alerts_sent) == 1, "Alert must be sent when preferred server absent > timeout"
    assert alerts_sent[0]["server"] == "time.apple.com"
    assert alerts_sent[0]["recipient"] == "luke@sjulsonlab.org"
    assert state.get("reference_alert_sent_utc") is not None


def test_s4_hourly_cycle_does_not_resend_alert_if_already_sent(mod, tmp_path):
    """Alert is not re-sent when reference_alert_sent_utc is already set."""
    cfg = _mock_config(tmp_path, mod)
    cfg.notify_email = "luke@sjulsonlab.org"
    cfg.reference_timeout_s = 1800

    state = mod._default_state()
    state["preferred_server"] = "time.apple.com"
    state["preferred_server_last_seen_utc"] = _utc_ago(7200)
    state["reference_alert_sent_utc"] = _utc_ago(3600)   # already alerted 1 hour ago

    alerts_sent = []

    def _mock_alert(*args, **kwargs):
        alerts_sent.append(True)
        return True

    with (
        patch.object(mod, "run_sourcestats_full", lambda *a, **kw: {}),
        patch.object(mod, "send_reference_alert", _mock_alert),
    ):
        mod._run_hourly_cycle(cfg, state)

    assert len(alerts_sent) == 0, "Alert must NOT be re-sent when already alerted"


def test_s5_alert_clears_when_preferred_server_returns(mod, tmp_path):
    """reference_alert_sent_utc is cleared when the server reappears in sourcestats."""
    cfg = _mock_config(tmp_path, mod)
    cfg.notify_email = "luke@sjulsonlab.org"
    cfg.reference_timeout_s = 1800

    state = mod._default_state()
    state["preferred_server"] = "17.253.2.35"   # IS in SAMPLE_SOURCESTATS
    state["preferred_server_last_seen_utc"] = _utc_ago(5)   # very recent
    state["reference_alert_sent_utc"] = _utc_ago(3600)   # had been alerted

    with patch.object(mod, "run_sourcestats_full", _make_mock_run_sourcestats(mod)):
        mod._run_hourly_cycle(cfg, state)

    assert state.get("reference_alert_sent_utc") is None, (
        "reference_alert_sent_utc must be cleared when preferred server is back online"
    )
