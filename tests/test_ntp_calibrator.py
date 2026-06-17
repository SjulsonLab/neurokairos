"""Tests for ntp_calibrator.py (NTP offset calibration daemon).

Covers Groups A–N. All I/O is mocked so tests run without root,
without chrony, and on macOS.
"""
from __future__ import annotations

import importlib.util
import json
import urllib.error
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "raspberry_pi" / "scripts" / "ntp_calibrator.py"

# ---------------------------------------------------------------------------
# Module loader
# ---------------------------------------------------------------------------

def load_module():
    """Load ntp_calibrator without installing it as a package."""
    spec = importlib.util.spec_from_file_location("ntp_calibrator", SCRIPT_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def mod():
    return load_module()


# ---------------------------------------------------------------------------
# Sample chronyc outputs
# ---------------------------------------------------------------------------

SAMPLE_SOURCESTATS = """\
Name/IP Address            NP  NR  Span  Frequency  Freq Skew  Offset  Std Dev
==============================================================================
17.253.2.35                16   8   32m   +0.003     0.042    +49us   61us
216.239.35.4               16   9   32m   +0.011     0.051   -199us  312us
162.159.200.1              16   8   32m   +0.001     0.038    -28us  369us
54.81.127.33                0   0    0s   +0.000     1.000      +0ns  312us
"""

SAMPLE_SOURCES = """\
MS Name/IP address         Stratum Poll Reach LastRx Last sample
===============================================================================
^* 17.253.2.35               1   6   377    44   +49us[ +49us] +/-  200us
^+ 216.239.35.4              1   6   377    45  -199us[-199us] +/-  315us
^- 162.159.200.1             3   6   377    44   -28us[ -28us] +/-  369us
^? 54.81.127.33              3   6     0    46  +171us[+171us] +/-  523us
"""

SAMPLE_TRACKING_NTP = """\
Reference ID    : 11FD023B (17.253.2.35)
Stratum         : 2
Ref time (UTC)  : Sun Jun 15 12:00:00 2026
System time     : 0.000000049 seconds slow of NTP time
Last offset     : -0.000000049 seconds
RMS offset      : 0.000000061 seconds
Frequency       : 1.234 ppm slow
Residual freq   : 0.005 ppm
Skew            : 0.100 ppm
Root delay      : 0.001234567 seconds
Root dispersion : 0.000234567 seconds
Update interval : 64.2 seconds
Leap status     : Normal
"""

SAMPLE_TRACKING_PPS = """\
Reference ID    : 50505300 (PPS)
Stratum         : 1
Ref time (UTC)  : Sun Jun 15 12:00:00 2026
System time     : 0.000000001 seconds slow of NTP time
Last offset     : -0.000000001 seconds
RMS offset      : 0.000000002 seconds
Frequency       : 0.001 ppm slow
Residual freq   : 0.000 ppm
Skew            : 0.010 ppm
Root delay      : 0.000000100 seconds
Root dispersion : 0.000000050 seconds
Update interval : 16.0 seconds
Leap status     : Normal
"""

SAMPLE_TRACKING_GPS = """\
Reference ID    : 47505300 (GPS)
Stratum         : 1
Ref time (UTC)  : Sun Jun 15 12:00:00 2026
System time     : 0.000000002 seconds slow of NTP time
Last offset     : +0.000000002 seconds
RMS offset      : 0.000000003 seconds
Frequency       : 0.002 ppm slow
Residual freq   : 0.000 ppm
Skew            : 0.010 ppm
Root delay      : 0.000000100 seconds
Root dispersion : 0.000000050 seconds
Update interval : 16.0 seconds
Leap status     : Normal
"""

SAMPLE_TRACKING_UNSYNC = """\
Reference ID    : 00000000 ()
Stratum         : 0
Ref time (UTC)  : Thu Jan  1 00:00:00 1970
System time     : 0.000000000 seconds slow of NTP time
Last offset     : +0.000000000 seconds
RMS offset      : 0.000000000 seconds
Frequency       : 0.000 ppm slow
Residual freq   : +0.000 ppm
Skew            : 0.000 ppm
Root delay      : 0.000000000 seconds
Root dispersion : 0.000000000 seconds
Update interval : 0.0 seconds
Leap status     : Not synchronised
"""

CONF_WITH_BLOCK = """\
# Basic chrony configuration
pool 2.debian.pool.ntp.org iburst
makestep 1.0 3
rtcsync
driftfile /var/lib/chrony/chrony.drift

# BEGIN ntp-calibrator-managed
server time.apple.com iburst minpoll 6 maxpoll 6 prefer
server time.google.com iburst minpoll 6 maxpoll 6 offset -0.000199
# END ntp-calibrator-managed

logdir /var/log/chrony
"""

CONF_WITHOUT_BLOCK = """\
# Basic chrony configuration
pool 2.debian.pool.ntp.org iburst
makestep 1.0 3
rtcsync
driftfile /var/lib/chrony/chrony.drift
logdir /var/log/chrony
"""


# ---------------------------------------------------------------------------
# Group A — parse_sourcestats
# ---------------------------------------------------------------------------

def test_a1_positive_offset_us(mod):
    """Positive microsecond offset parses to seconds correctly."""
    result = mod.parse_sourcestats(SAMPLE_SOURCESTATS)
    assert abs(result["17.253.2.35"]["offset_s"] - 49e-6) < 1e-9


def test_a2_negative_offset_us(mod):
    """Negative microsecond offset parses correctly."""
    result = mod.parse_sourcestats(SAMPLE_SOURCESTATS)
    assert abs(result["216.239.35.4"]["offset_s"] - (-199e-6)) < 1e-9


def test_a3_stdev_us(mod):
    """Standard deviation in microseconds parses correctly."""
    result = mod.parse_sourcestats(SAMPLE_SOURCESTATS)
    assert abs(result["17.253.2.35"]["stdev_s"] - 61e-6) < 1e-9


def test_a4_nanosecond_offset(mod):
    """Nanosecond-scale offset and stdev parse correctly."""
    text = (
        "Name/IP Address            NP  NR  Span  Frequency  Freq Skew"
        "  Offset  Std Dev\n"
        "==============================================================================\n"
        "1.2.3.4                    12   6   12m   +0.000     0.010   +300ns   50ns\n"
    )
    result = mod.parse_sourcestats(text)
    assert abs(result["1.2.3.4"]["offset_s"] - 300e-9) < 1e-12
    assert abs(result["1.2.3.4"]["stdev_s"] - 50e-9) < 1e-12


def test_a5_millisecond_offset(mod):
    """Millisecond-scale offset and stdev parse correctly."""
    text = (
        "Name/IP Address            NP  NR  Span  Frequency  Freq Skew"
        "  Offset  Std Dev\n"
        "==============================================================================\n"
        "1.2.3.4                    12   6   12m   +0.000     0.010   +1.23ms  0.45ms\n"
    )
    result = mod.parse_sourcestats(text)
    assert abs(result["1.2.3.4"]["offset_s"] - 1.23e-3) < 1e-9
    assert abs(result["1.2.3.4"]["stdev_s"] - 0.45e-3) < 1e-9


def test_a6_multiple_servers(mod):
    """All four servers parsed from sample output."""
    result = mod.parse_sourcestats(SAMPLE_SOURCESTATS)
    assert set(result.keys()) == {"17.253.2.35", "216.239.35.4", "162.159.200.1", "54.81.127.33"}


def test_a7_np_field(mod):
    """NP (number of points) field captured for each server."""
    result = mod.parse_sourcestats(SAMPLE_SOURCESTATS)
    assert result["17.253.2.35"]["np"] == 16
    assert result["54.81.127.33"]["np"] == 0


def test_a8_header_lines_skipped(mod):
    """Header and separator lines produce no server entries."""
    text = (
        "Name/IP Address            NP  NR  Span  Frequency  Freq Skew"
        "  Offset  Std Dev\n"
        "==============================================================================\n"
    )
    assert mod.parse_sourcestats(text) == {}


def test_a9_garbage_input(mod):
    """Completely invalid input returns empty dict."""
    assert mod.parse_sourcestats("not chronyc output\ngarbage") == {}


def test_a10_truncated_names_skipped(mod):
    """Hostnames truncated by chronyc (ending '>') are skipped.

    chronyc column width is 25 chars; long pool.ntp.org reverse-DNS names
    get truncated to fit, producing invalid hostnames like 'usnyc3-ntp-001.aaplimg.c>'.
    """
    text = (
        "Name/IP Address            NP  NR  Span  Frequency  Freq Skew"
        "  Offset  Std Dev\n"
        "==============================================================================\n"
        "time2.google.com           16   8   32m   +0.003     0.042    +49us   61us\n"
        "usnyc3-ntp-001.aaplimg.c>  16   8   32m   +0.001     0.038    -28us  369us\n"
    )
    result = mod.parse_sourcestats(text)
    assert "time2.google.com" in result
    assert not any(k.endswith(">") for k in result)


# ---------------------------------------------------------------------------
# Group B — parse_sources
# ---------------------------------------------------------------------------

def test_b1_state_chars(mod):
    """State chars *, +, -, ? extracted for each server."""
    result = mod.parse_sources(SAMPLE_SOURCES)
    assert result["17.253.2.35"]["state"] == "*"
    assert result["216.239.35.4"]["state"] == "+"
    assert result["162.159.200.1"]["state"] == "-"
    assert result["54.81.127.33"]["state"] == "?"


def test_b2_stratum(mod):
    """Stratum field extracted correctly."""
    result = mod.parse_sources(SAMPLE_SOURCES)
    assert result["17.253.2.35"]["stratum"] == 1
    assert result["162.159.200.1"]["stratum"] == 3


def test_b3_reach(mod):
    """Reach field extracted correctly."""
    result = mod.parse_sources(SAMPLE_SOURCES)
    assert result["17.253.2.35"]["reach"] == 377
    assert result["54.81.127.33"]["reach"] == 0


def test_b4_all_servers_present(mod):
    """All four servers appear in result."""
    result = mod.parse_sources(SAMPLE_SOURCES)
    assert len(result) == 4


def test_b5_garbage_input(mod):
    """Invalid input returns empty dict."""
    assert mod.parse_sources("garbage\nno sources here") == {}


# ---------------------------------------------------------------------------
# Group C — parse_tracking
# ---------------------------------------------------------------------------

def test_c1_pps_reference(mod):
    """PPS hex ref ID decoded to name 'PPS' with synchronized=True."""
    result = mod.parse_tracking(SAMPLE_TRACKING_PPS)
    assert result["ref_id_name"] == "PPS"
    assert result["synchronized"] is True


def test_c2_gps_reference(mod):
    """GPS hex ref ID decoded to name 'GPS'."""
    result = mod.parse_tracking(SAMPLE_TRACKING_GPS)
    assert result["ref_id_name"] == "GPS"


def test_c3_ntp_server_reference(mod):
    """NTP server tracking output reports IP as ref_id_name."""
    result = mod.parse_tracking(SAMPLE_TRACKING_NTP)
    assert result["ref_id_name"] == "17.253.2.35"
    assert result["synchronized"] is True


def test_c4_not_synchronized(mod):
    """Leap status 'Not synchronised' → synchronized=False."""
    result = mod.parse_tracking(SAMPLE_TRACKING_UNSYNC)
    assert result["synchronized"] is False


def test_c5_stratum(mod):
    """Stratum field parsed from tracking output."""
    assert mod.parse_tracking(SAMPLE_TRACKING_NTP)["stratum"] == 2
    assert mod.parse_tracking(SAMPLE_TRACKING_PPS)["stratum"] == 1


def test_c6_garbage_tracking(mod):
    """Unparseable tracking output returns synchronized=False."""
    result = mod.parse_tracking("not chronyc output")
    assert result["synchronized"] is False


# ---------------------------------------------------------------------------
# Group D — detect_reference_tier
# ---------------------------------------------------------------------------

def test_d1_pps_is_gnss(mod):
    """PPS reference → GNSS tier."""
    tracking = {"ref_id_name": "PPS", "synchronized": True, "stratum": 1}
    assert mod.detect_reference_tier(tracking, lan_servers=[]) == "GNSS"


def test_d2_gps_is_gnss(mod):
    """GPS reference → GNSS tier."""
    tracking = {"ref_id_name": "GPS", "synchronized": True, "stratum": 1}
    assert mod.detect_reference_tier(tracking, lan_servers=[]) == "GNSS"


def test_d3_lan_server(mod):
    """Reference IP in lan_servers list → LAN tier."""
    tracking = {"ref_id_name": "192.168.1.10", "synchronized": True, "stratum": 2}
    assert mod.detect_reference_tier(tracking, lan_servers=["192.168.1.10"]) == "LAN"


def test_d4_wan_server(mod):
    """Synchronized non-GNSS non-LAN reference → WAN tier."""
    tracking = {"ref_id_name": "17.253.2.35", "synchronized": True, "stratum": 2}
    assert mod.detect_reference_tier(tracking, lan_servers=[]) == "WAN"


def test_d5_not_synchronized(mod):
    """Not synchronized → NONE tier."""
    tracking = {"ref_id_name": "", "synchronized": False, "stratum": 0}
    assert mod.detect_reference_tier(tracking, lan_servers=[]) == "NONE"


def test_d6_lan_ip_without_config(mod):
    """LAN-looking IP not in lan_servers list → WAN tier."""
    tracking = {"ref_id_name": "192.168.1.10", "synchronized": True, "stratum": 2}
    assert mod.detect_reference_tier(tracking, lan_servers=[]) == "WAN"


# ---------------------------------------------------------------------------
# Group E — compute_server_stats
# ---------------------------------------------------------------------------

def _make_samples(server, offsets_s, base_ts=1_000_000.0):
    """Build a list of sample dicts for one server."""
    return [
        {"server": server, "offset_s": o, "stdev_s": 60e-6, "timestamp": base_ts + i * 60}
        for i, o in enumerate(offsets_s)
    ]


def test_e1_mean_and_stdev(mod):
    """Mean and stdev computed correctly from samples."""
    offsets = [100e-6, 200e-6, 300e-6, 400e-6, 500e-6,
               100e-6, 200e-6, 300e-6, 400e-6, 500e-6]
    samples = _make_samples("1.2.3.4", offsets)
    stats = mod.compute_server_stats(samples, min_samples=5)
    assert abs(stats["1.2.3.4"]["mean_offset_s"] - 300e-6) < 1e-9
    assert stats["1.2.3.4"]["n"] == 10


def test_e2_fewer_than_min_samples_excluded(mod):
    """Server with < min_samples not included in result."""
    samples = _make_samples("1.2.3.4", [100e-6, 200e-6])
    stats = mod.compute_server_stats(samples, min_samples=5)
    assert "1.2.3.4" not in stats


def test_e3_zero_stdev_for_identical_offsets(mod):
    """Identical offsets yield stdev ≈ 0."""
    samples = _make_samples("1.2.3.4", [100e-6] * 10)
    stats = mod.compute_server_stats(samples, min_samples=5)
    assert stats["1.2.3.4"]["stdev_s"] < 1e-12


def test_e4_multiple_servers(mod):
    """Stats computed independently for each server."""
    s1 = _make_samples("1.1.1.1", [100e-6] * 10)
    s2 = _make_samples("2.2.2.2", [200e-6] * 10)
    stats = mod.compute_server_stats(s1 + s2, min_samples=5)
    assert abs(stats["1.1.1.1"]["mean_offset_s"] - 100e-6) < 1e-9
    assert abs(stats["2.2.2.2"]["mean_offset_s"] - 200e-6) < 1e-9


def test_e5_empty_samples(mod):
    """Empty sample list returns empty stats."""
    assert mod.compute_server_stats([], min_samples=5) == {}


# ---------------------------------------------------------------------------
# Group F — select_reference_server
# ---------------------------------------------------------------------------

def test_f1_picks_lowest_composite_score(mod):
    """Server with lowest composite score selected as reference."""
    scores = {"1.1.1.1": 100e-6, "2.2.2.2": 50e-6, "3.3.3.3": 200e-6}
    assert mod.select_reference_server(scores) == "2.2.2.2"


def test_f2_empty_scores_returns_none(mod):
    """Empty scores dict returns None."""
    assert mod.select_reference_server({}) is None


def test_f3_single_server(mod):
    """Single finite-scored server is selected as reference."""
    assert mod.select_reference_server({"1.1.1.1": 60e-6}) == "1.1.1.1"


def test_f4_inf_score_not_selected_when_alternative_exists(mod):
    """CV-gated server (inf score) is never chosen when a finite-score server exists."""
    scores = {"good.com": 60e-6, "gated.com": float("inf")}
    assert mod.select_reference_server(scores) == "good.com"


def test_f5_all_inf_returns_none(mod):
    """All servers gated (inf score) → returns None."""
    scores = {"a.com": float("inf"), "b.com": float("inf")}
    assert mod.select_reference_server(scores) is None


# ---------------------------------------------------------------------------
# Group G — compute_relative_offsets
# ---------------------------------------------------------------------------

def test_g1_reference_gets_zero(mod):
    """Reference server always gets relative offset 0.0."""
    stats = {
        "1.1.1.1": {"mean_offset_s": 50e-6, "stdev_s": 30e-6, "n": 10},
        "2.2.2.2": {"mean_offset_s": 150e-6, "stdev_s": 60e-6, "n": 10},
    }
    offsets = mod.compute_relative_offsets(stats, "1.1.1.1")
    assert offsets["1.1.1.1"] == 0.0


def test_g2_backup_offset_relative_to_reference(mod):
    """Backup offset = backup_mean - reference_mean."""
    stats = {
        "1.1.1.1": {"mean_offset_s": 50e-6, "stdev_s": 30e-6, "n": 10},
        "2.2.2.2": {"mean_offset_s": 150e-6, "stdev_s": 60e-6, "n": 10},
    }
    offsets = mod.compute_relative_offsets(stats, "1.1.1.1")
    assert abs(offsets["2.2.2.2"] - 100e-6) < 1e-12


def test_g3_negative_relative_offset(mod):
    """Negative relative offset computed correctly."""
    stats = {
        "1.1.1.1": {"mean_offset_s": 150e-6, "stdev_s": 30e-6, "n": 10},
        "2.2.2.2": {"mean_offset_s": 50e-6, "stdev_s": 60e-6, "n": 10},
    }
    offsets = mod.compute_relative_offsets(stats, "1.1.1.1")
    assert abs(offsets["2.2.2.2"] - (-100e-6)) < 1e-12


def test_g4_reference_not_in_stats(mod):
    """Reference not in stats → empty dict."""
    stats = {"1.1.1.1": {"mean_offset_s": 50e-6, "stdev_s": 30e-6, "n": 10}}
    assert mod.compute_relative_offsets(stats, "9.9.9.9") == {}


def test_g5_all_servers_included(mod):
    """All servers in stats appear in output dict."""
    stats = {
        "1.1.1.1": {"mean_offset_s": 0.0, "stdev_s": 30e-6, "n": 10},
        "2.2.2.2": {"mean_offset_s": 100e-6, "stdev_s": 60e-6, "n": 10},
        "3.3.3.3": {"mean_offset_s": -50e-6, "stdev_s": 90e-6, "n": 10},
    }
    offsets = mod.compute_relative_offsets(stats, "1.1.1.1")
    assert set(offsets.keys()) == {"1.1.1.1", "2.2.2.2", "3.3.3.3"}


# ---------------------------------------------------------------------------
# Group H — build_server_line
# ---------------------------------------------------------------------------

def test_h1_reference_has_prefer_no_offset(mod):
    """Reference server line: 'prefer' present, 'offset' absent."""
    line = mod.build_server_line(
        "time.apple.com", is_reference=True, is_noselect=False, offset_s=0.0, poll=6
    )
    assert "prefer" in line
    assert "offset" not in line
    assert "time.apple.com" in line


def test_h2_backup_has_offset_no_prefer(mod):
    """Backup server line: 'offset' present, 'prefer' absent."""
    line = mod.build_server_line(
        "time.google.com", is_reference=False, is_noselect=False, offset_s=-199e-6, poll=6
    )
    assert "offset" in line
    assert "prefer" not in line
    assert "noselect" not in line


def test_h3_noselect_flag(mod):
    """Noselect server line includes 'noselect', not 'prefer'."""
    line = mod.build_server_line(
        "time.windows.com", is_reference=False, is_noselect=True, offset_s=None, poll=6
    )
    assert "noselect" in line
    assert "prefer" not in line


def test_h4_calibration_phase_no_flags(mod):
    """Calibration-phase server (no reference, no noselect, no offset): no special flags."""
    line = mod.build_server_line(
        "time.apple.com", is_reference=False, is_noselect=False, offset_s=None, poll=6
    )
    assert "prefer" not in line
    assert "noselect" not in line
    assert "offset" not in line


def test_h5_offset_precision(mod):
    """Offset formatted with enough decimal places to represent sub-ms values."""
    line_pos = mod.build_server_line(
        "x.com", is_reference=False, is_noselect=False, offset_s=199e-6, poll=6
    )
    line_neg = mod.build_server_line(
        "x.com", is_reference=False, is_noselect=False, offset_s=-199e-6, poll=6
    )
    assert "0.000199" in line_pos
    assert "-0.000199" in line_neg


def test_h6_poll_in_line(mod):
    """minpoll and maxpoll appear in the line with the given poll value."""
    line = mod.build_server_line(
        "time.apple.com", is_reference=True, is_noselect=False, offset_s=None, poll=6
    )
    assert "minpoll 6" in line
    assert "maxpoll 6" in line


# ---------------------------------------------------------------------------
# Group I — chrony.conf managed block read/write
# ---------------------------------------------------------------------------

def test_i1_read_managed_block(mod, tmp_path):
    """read_managed_block returns server lines inside the block."""
    conf = tmp_path / "chrony.conf"
    conf.write_text(CONF_WITH_BLOCK)
    lines = mod.read_managed_block(str(conf))
    assert len(lines) == 2
    assert any("time.apple.com" in l for l in lines)
    assert any("time.google.com" in l for l in lines)


def test_i2_read_managed_block_absent(mod, tmp_path):
    """read_managed_block returns [] when no managed block present."""
    conf = tmp_path / "chrony.conf"
    conf.write_text(CONF_WITHOUT_BLOCK)
    assert mod.read_managed_block(str(conf)) == []


def test_i3_write_managed_block_replaces(mod, tmp_path):
    """write_managed_block replaces the existing managed block content."""
    conf = tmp_path / "chrony.conf"
    conf.write_text(CONF_WITH_BLOCK)
    new_lines = ["server time.cloudflare.com iburst minpoll 6 maxpoll 6 prefer"]
    mod.write_managed_block(str(conf), new_lines)
    result = mod.read_managed_block(str(conf))
    assert len(result) == 1
    assert "time.cloudflare.com" in result[0]
    assert "time.apple.com" not in conf.read_text()


def test_i4_write_preserves_outside_lines(mod, tmp_path):
    """Lines outside the managed block are untouched after write."""
    conf = tmp_path / "chrony.conf"
    conf.write_text(CONF_WITH_BLOCK)
    mod.write_managed_block(str(conf), ["server 1.2.3.4 iburst minpoll 6 maxpoll 6"])
    text = conf.read_text()
    assert "makestep 1.0 3" in text
    assert "driftfile /var/lib/chrony/chrony.drift" in text
    assert "logdir /var/log/chrony" in text


def test_i5_write_creates_block_if_absent(mod, tmp_path):
    """write_managed_block inserts a new block when none previously exists."""
    conf = tmp_path / "chrony.conf"
    conf.write_text(CONF_WITHOUT_BLOCK)
    mod.write_managed_block(str(conf), ["server 1.2.3.4 iburst minpoll 6 maxpoll 6"])
    result = mod.read_managed_block(str(conf))
    assert len(result) == 1
    assert "1.2.3.4" in result[0]


def test_i6_write_is_atomic(mod, tmp_path):
    """No leftover .tmp file after write_managed_block."""
    conf = tmp_path / "chrony.conf"
    conf.write_text(CONF_WITH_BLOCK)
    mod.write_managed_block(str(conf), ["server 1.2.3.4 iburst minpoll 6 maxpoll 6"])
    assert list(tmp_path.glob("*.tmp")) == []


# ---------------------------------------------------------------------------
# Group J — send_ntfy
# ---------------------------------------------------------------------------

def test_j1_post_to_correct_url(mod):
    """send_ntfy POSTs to <server>/<topic>."""
    config = {"server": "https://ntfy.sh", "topic": "test-topic-abc"}
    with patch("urllib.request.urlopen") as mock_open:
        mock_open.return_value.__enter__ = lambda s: s
        mock_open.return_value.__exit__ = MagicMock(return_value=False)
        mod.send_ntfy(config, title="Test", message="Hello")
    req = mock_open.call_args[0][0]
    assert "ntfy.sh/test-topic-abc" in req.full_url


def test_j2_required_headers(mod):
    """send_ntfy includes Title, Priority, and Tags headers."""
    config = {"server": "https://ntfy.sh", "topic": "test-topic-abc"}
    with patch("urllib.request.urlopen") as mock_open:
        mock_open.return_value.__enter__ = lambda s: s
        mock_open.return_value.__exit__ = MagicMock(return_value=False)
        mod.send_ntfy(config, title="Alert", message="Ref gone",
                      priority="high", tags="warning")
    req = mock_open.call_args[0][0]
    assert req.get_header("Title") == "Alert"
    assert req.get_header("Priority") == "high"
    assert req.get_header("Tags") == "warning"


def test_j3_connection_error_swallowed(mod):
    """send_ntfy does not raise on connection failure."""
    config = {"server": "https://ntfy.sh", "topic": "test-topic-abc"}
    with patch("urllib.request.urlopen", side_effect=urllib.error.URLError("refused")):
        mod.send_ntfy(config, title="Test", message="Hello")  # must not raise


# ---------------------------------------------------------------------------
# Group K — load_state / save_state / default_state
# ---------------------------------------------------------------------------

def test_k1_missing_file_returns_default(mod, tmp_path):
    """load_state returns default_state() when file does not exist."""
    state = mod.load_state(str(tmp_path / "state.json"))
    assert state["mode"] == mod.default_state()["mode"]


def test_k2_corrupt_json_returns_default(mod, tmp_path):
    """load_state returns default_state() on corrupt JSON, does not raise."""
    path = tmp_path / "state.json"
    path.write_text("not json {{{")
    state = mod.load_state(str(path))
    assert state["mode"] == mod.default_state()["mode"]


def test_k2b_unrecognized_mode_returns_default(mod, tmp_path):
    """load_state returns default_state() when mode is not a recognized value.

    Guards against state files left by the old SHM-based daemon (mode='WAN_CONSENSUS').
    """
    path = tmp_path / "state.json"
    path.write_text(json.dumps({"mode": "WAN_CONSENSUS", "preferred_server": "1.2.3.4"}))
    state = mod.load_state(str(path))
    assert state["mode"] == "LOGGING"


def test_k3_roundtrip(mod, tmp_path):
    """save_state + load_state preserves all fields exactly."""
    path = tmp_path / "state.json"
    state = mod.default_state()
    state["mode"] = "MONITORING"
    state["reference_server"] = "1.2.3.4"
    state["calibrated_offsets"] = {"1.2.3.4": 0.0, "2.2.2.2": -199e-6}
    mod.save_state(state, str(path))
    loaded = mod.load_state(str(path))
    assert loaded["mode"] == "MONITORING"
    assert loaded["reference_server"] == "1.2.3.4"
    assert abs(loaded["calibrated_offsets"]["2.2.2.2"] - (-199e-6)) < 1e-12


def test_k4_no_tmp_leftover(mod, tmp_path):
    """No .tmp file remains after save_state."""
    mod.save_state(mod.default_state(), str(tmp_path / "state.json"))
    assert list(tmp_path.glob("*.tmp")) == []


def test_k5_default_state_is_logging(mod):
    """default_state() starts in LOGGING mode (pure data collection, no decisions)."""
    assert mod.default_state()["mode"] == "LOGGING"


# ---------------------------------------------------------------------------
# Group L — check_reference_reachable
# ---------------------------------------------------------------------------

def test_l1_reachable_when_selected(mod):
    """Reference with state '*' and reach > 0 → reachable."""
    sources = {"1.2.3.4": {"state": "*", "reach": 377, "stratum": 1}}
    assert mod.check_reference_reachable(sources, "1.2.3.4") is True


def test_l2_reachable_when_backup(mod):
    """Reference with state '+' and reach > 0 → reachable."""
    sources = {"1.2.3.4": {"state": "+", "reach": 377, "stratum": 1}}
    assert mod.check_reference_reachable(sources, "1.2.3.4") is True


def test_l3_reachable_when_deprioritized(mod):
    """State '-' with reach > 0 → reachable (server is up, just deprioritized)."""
    sources = {"1.2.3.4": {"state": "-", "reach": 100, "stratum": 1}}
    assert mod.check_reference_reachable(sources, "1.2.3.4") is True


def test_l4_not_reachable_when_reach_zero(mod):
    """reach=0 → not reachable regardless of state."""
    sources = {"1.2.3.4": {"state": "?", "reach": 0, "stratum": 1}}
    assert mod.check_reference_reachable(sources, "1.2.3.4") is False


def test_l5_not_reachable_when_absent(mod):
    """Reference absent from sources dict → not reachable."""
    sources = {"2.2.2.2": {"state": "*", "reach": 377, "stratum": 1}}
    assert mod.check_reference_reachable(sources, "1.2.3.4") is False


# ---------------------------------------------------------------------------
# Group M — update_freeze_state
# ---------------------------------------------------------------------------

def _monitoring_state():
    s = {
        "mode": "MONITORING",
        "reference_server": "1.2.3.4",
        "calibrated_offsets": {},
        "frozen_since_utc": None,
        "notified_utc": None,
    }
    return s


def _frozen_state(frozen_since_iso):
    s = _monitoring_state()
    s["mode"] = "FROZEN"
    s["frozen_since_utc"] = frozen_since_iso
    return s


def test_m1_enters_frozen_on_reference_loss(mod):
    """MONITORING → FROZEN when reference becomes unreachable."""
    state = _monitoring_state()
    now = datetime(2026, 6, 15, 12, 0, 0, tzinfo=timezone.utc)
    new_state = mod.update_freeze_state(
        state, is_reachable=False, now_utc=now, freeze_timeout_s=21600
    )
    assert new_state["mode"] == "FROZEN"
    assert new_state["frozen_since_utc"] is not None


def test_m2_stays_monitoring_when_reachable(mod):
    """MONITORING stays MONITORING when reference is reachable."""
    state = _monitoring_state()
    now = datetime(2026, 6, 15, 12, 0, 0, tzinfo=timezone.utc)
    new_state = mod.update_freeze_state(
        state, is_reachable=True, now_utc=now, freeze_timeout_s=21600
    )
    assert new_state["mode"] == "MONITORING"


def test_m3_notified_after_timeout(mod):
    """FROZEN → NOTIFIED after freeze_timeout_s elapses."""
    state = _frozen_state("2026-06-15T06:00:00+00:00")
    now = datetime(2026, 6, 15, 13, 0, 0, tzinfo=timezone.utc)  # 7h later
    new_state = mod.update_freeze_state(
        state, is_reachable=False, now_utc=now, freeze_timeout_s=21600
    )
    assert new_state["mode"] == "NOTIFIED"


def test_m4_stays_frozen_before_timeout(mod):
    """FROZEN stays FROZEN before freeze_timeout_s elapses."""
    state = _frozen_state("2026-06-15T06:00:00+00:00")
    now = datetime(2026, 6, 15, 9, 0, 0, tzinfo=timezone.utc)   # 3h later
    new_state = mod.update_freeze_state(
        state, is_reachable=False, now_utc=now, freeze_timeout_s=21600
    )
    assert new_state["mode"] == "FROZEN"


def test_m5_unfreezes_when_reference_returns(mod):
    """FROZEN → MONITORING when reference becomes reachable again."""
    state = _frozen_state("2026-06-15T06:00:00+00:00")
    now = datetime(2026, 6, 15, 9, 0, 0, tzinfo=timezone.utc)
    new_state = mod.update_freeze_state(
        state, is_reachable=True, now_utc=now, freeze_timeout_s=21600
    )
    assert new_state["mode"] == "MONITORING"
    assert new_state["frozen_since_utc"] is None


def test_m6_stays_notified_when_still_gone(mod):
    """NOTIFIED stays NOTIFIED when reference still absent."""
    state = _frozen_state("2026-06-15T00:00:00+00:00")
    state["mode"] = "NOTIFIED"
    state["notified_utc"] = "2026-06-15T06:00:00+00:00"
    now = datetime(2026, 6, 15, 14, 0, 0, tzinfo=timezone.utc)
    new_state = mod.update_freeze_state(
        state, is_reachable=False, now_utc=now, freeze_timeout_s=21600
    )
    assert new_state["mode"] == "NOTIFIED"


def test_m7_notified_unfreezes_when_reference_returns(mod):
    """NOTIFIED → MONITORING when reference becomes reachable again."""
    state = _frozen_state("2026-06-15T00:00:00+00:00")
    state["mode"] = "NOTIFIED"
    state["notified_utc"] = "2026-06-15T06:00:00+00:00"
    now = datetime(2026, 6, 15, 14, 0, 0, tzinfo=timezone.utc)
    new_state = mod.update_freeze_state(
        state, is_reachable=True, now_utc=now, freeze_timeout_s=21600
    )
    assert new_state["mode"] == "MONITORING"
    assert new_state["frozen_since_utc"] is None
    assert new_state["notified_utc"] is None


# ---------------------------------------------------------------------------
# Group N — build_calibrated_conf_lines (integration of build_server_line)
# ---------------------------------------------------------------------------

def test_n1_line_count(mod):
    """One line produced per server (calibrated + noselect)."""
    offsets = {"1.1.1.1": 0.0, "2.2.2.2": -100e-6, "3.3.3.3": 50e-6}
    noselect = {"4.4.4.4", "5.5.5.5"}
    lines = mod.build_calibrated_conf_lines(
        reference_server="1.1.1.1",
        calibrated_offsets=offsets,
        noselect_servers=noselect,
        poll=6,
    )
    assert len(lines) == 5


def test_n2_reference_line_has_prefer(mod):
    """Reference IP line includes 'prefer'."""
    offsets = {"1.1.1.1": 0.0, "2.2.2.2": -100e-6}
    lines = mod.build_calibrated_conf_lines(
        reference_server="1.1.1.1",
        calibrated_offsets=offsets,
        noselect_servers=set(),
        poll=6,
    )
    ref_line = next(l for l in lines if "1.1.1.1" in l)
    assert "prefer" in ref_line
    assert "offset" not in ref_line


def test_n3_backup_line_has_offset(mod):
    """Backup IP line includes 'offset' and not 'prefer'."""
    offsets = {"1.1.1.1": 0.0, "2.2.2.2": -100e-6}
    lines = mod.build_calibrated_conf_lines(
        reference_server="1.1.1.1",
        calibrated_offsets=offsets,
        noselect_servers=set(),
        poll=6,
    )
    backup_line = next(l for l in lines if "2.2.2.2" in l)
    assert "offset" in backup_line
    assert "prefer" not in backup_line


def test_n4_noselect_servers_flagged(mod):
    """Servers in noselect_servers get 'noselect' flag."""
    offsets = {"1.1.1.1": 0.0}
    lines = mod.build_calibrated_conf_lines(
        reference_server="1.1.1.1",
        calibrated_offsets=offsets,
        noselect_servers={"2.2.2.2"},
        poll=6,
    )
    noselect_line = next(l for l in lines if "2.2.2.2" in l)
    assert "noselect" in noselect_line


def test_n5_all_server_names_in_output(mod):
    """Every server IP appears in exactly one output line."""
    offsets = {"1.1.1.1": 0.0, "2.2.2.2": -100e-6}
    noselect = {"3.3.3.3"}
    lines = mod.build_calibrated_conf_lines(
        reference_server="1.1.1.1",
        calibrated_offsets=offsets,
        noselect_servers=noselect,
        poll=6,
    )
    for ip in ["1.1.1.1", "2.2.2.2", "3.3.3.3"]:
        assert sum(1 for l in lines if ip in l) == 1


# ---------------------------------------------------------------------------
# Group O — select_active_servers (limits calibrated pool to top-N by score)
# ---------------------------------------------------------------------------

def test_o1_limits_to_active_pool_size(mod):
    """Only top active_pool_size servers by composite score are active."""
    scores = {
        "a.com": 30e-6,   # best
        "b.com": 60e-6,
        "c.com": 90e-6,
        "d.com": 120e-6,  # worst
    }
    active, noselect = mod.select_active_servers(scores, active_pool_size=2)
    assert len(active) == 2
    assert "a.com" in active
    assert "d.com" in noselect


def test_o2_best_server_always_in_active(mod):
    """The best-scoring server is always in the active set."""
    scores = {"best.com": 30e-6, "ok.com": 60e-6, "bad.com": 200e-6}
    active, noselect = mod.select_active_servers(scores, active_pool_size=2)
    assert "best.com" in active


def test_o3_all_active_when_pool_size_exceeds_count(mod):
    """When pool_size >= number of servers, noselect is empty."""
    scores = {"a.com": 30e-6, "b.com": 60e-6}
    active, noselect = mod.select_active_servers(scores, active_pool_size=5)
    assert len(active) == 2
    assert noselect == set()


def test_o4_empty_scores_returns_empty(mod):
    """Empty scores → empty active and noselect."""
    active, noselect = mod.select_active_servers({}, active_pool_size=3)
    assert len(active) == 0
    assert noselect == set()


def test_o5_inf_scored_servers_always_noselect(mod):
    """CV-gated servers (inf score) always go to noselect regardless of pool size."""
    scores = {"good.com": 60e-6, "gated.com": float("inf")}
    active, noselect = mod.select_active_servers(scores, active_pool_size=3)
    assert "good.com" in active
    assert "gated.com" in noselect


# ---------------------------------------------------------------------------
# Group N — additional: noselect_poll parameter
# ---------------------------------------------------------------------------

def test_n6_noselect_servers_use_separate_poll(mod):
    """Noselect servers use noselect_poll, not the active poll."""
    offsets = {"1.1.1.1": 0.0}
    lines = mod.build_calibrated_conf_lines(
        reference_server="1.1.1.1",
        calibrated_offsets=offsets,
        noselect_servers={"2.2.2.2"},
        poll=6,
        noselect_poll=10,
    )
    ref_line = next(l for l in lines if "1.1.1.1" in l)
    noselect_line = next(l for l in lines if "2.2.2.2" in l)
    assert "minpoll 6" in ref_line
    assert "minpoll 10" in noselect_line
    assert "maxpoll 10" in noselect_line


# ---------------------------------------------------------------------------
# Group P — compute_stdev_stats
# ---------------------------------------------------------------------------

def _make_samples_stdev(server, stdev_s_values, base_ts=1_000_000.0):
    """Build sample dicts with explicit per-sample stdev_s values."""
    return [
        {"server": server, "offset_s": 0.0, "stdev_s": s,
         "timestamp": base_ts + i * 60}
        for i, s in enumerate(stdev_s_values)
    ]


def test_p1_mean_stdev(mod):
    """mean_stdev_s is the mean of all stdev_s samples for the server."""
    samples = _make_samples_stdev("1.2.3.4", [60e-6, 80e-6, 100e-6] * 4)
    stats = mod.compute_stdev_stats(samples, min_samples=5)
    assert abs(stats["1.2.3.4"]["mean_stdev_s"] - 80e-6) < 1e-12


def test_p2_stdev_of_stdev(mod):
    """stdev_of_stdev_s is the sample stdev of stdev_s values."""
    from statistics import stdev
    vals = [60e-6, 80e-6, 100e-6] * 4
    samples = _make_samples_stdev("1.2.3.4", vals)
    stats = mod.compute_stdev_stats(samples, min_samples=5)
    assert abs(stats["1.2.3.4"]["stdev_of_stdev_s"] - stdev(vals)) < 1e-12


def test_p3_cv_is_ratio(mod):
    """CV = stdev_of_stdev_s / mean_stdev_s."""
    vals = [60e-6, 80e-6, 100e-6] * 4
    samples = _make_samples_stdev("1.2.3.4", vals)
    stats = mod.compute_stdev_stats(samples, min_samples=5)
    s = stats["1.2.3.4"]
    assert abs(s["cv"] - s["stdev_of_stdev_s"] / s["mean_stdev_s"]) < 1e-9


def test_p4_fewer_than_min_samples_excluded(mod):
    """Server with < min_samples is excluded from result."""
    samples = _make_samples_stdev("1.2.3.4", [60e-6, 80e-6])
    assert mod.compute_stdev_stats(samples, min_samples=5) == {}


def test_p5_constant_stdev_gives_zero_variation(mod):
    """Identical stdev_s values yield stdev_of_stdev ≈ 0 and CV ≈ 0."""
    samples = _make_samples_stdev("1.2.3.4", [60e-6] * 12)
    stats = mod.compute_stdev_stats(samples, min_samples=5)
    assert stats["1.2.3.4"]["stdev_of_stdev_s"] < 1e-15
    assert stats["1.2.3.4"]["cv"] < 1e-9


def test_p6_multiple_servers_independent(mod):
    """Stats are computed independently for each server."""
    s1 = _make_samples_stdev("a.com", [60e-6] * 10)
    s2 = _make_samples_stdev("b.com", [120e-6] * 10)
    stats = mod.compute_stdev_stats(s1 + s2, min_samples=5)
    assert abs(stats["a.com"]["mean_stdev_s"] - 60e-6) < 1e-12
    assert abs(stats["b.com"]["mean_stdev_s"] - 120e-6) < 1e-12


def test_p7_empty_samples_returns_empty(mod):
    """Empty sample list returns empty dict."""
    assert mod.compute_stdev_stats([], min_samples=5) == {}


# ---------------------------------------------------------------------------
# Group Q — compute_composite_score and score_servers
# ---------------------------------------------------------------------------

def test_q1_stratum1_cv0_equals_mean_stdev(mod):
    """Stratum 1, CV=0: score = mean_stdev_s (factor 1.0, no penalty)."""
    assert abs(mod.compute_composite_score(60e-6, cv=0.0, stratum=1) - 60e-6) < 1e-12


def test_q2_stratum2_factor(mod):
    """Stratum 2: score = mean_stdev_s * (4/3) — requires 25% better raw score to beat stratum 1."""
    expected = 60e-6 * (4 / 3)
    assert abs(mod.compute_composite_score(60e-6, cv=0.0, stratum=2) - expected) < 1e-12


def test_q3_cv_above_gate_returns_inf(mod):
    """CV > 2.0 → returns inf (server excluded from selection)."""
    assert mod.compute_composite_score(60e-6, cv=2.1, stratum=1) == float("inf")


def test_q4_cv_at_gate_returns_inf(mod):
    """CV == 2.0 → returns inf (gate is strict: CV must be strictly < 2.0)."""
    assert mod.compute_composite_score(60e-6, cv=2.0, stratum=1) == float("inf")


def test_q5_stratum3_factor(mod):
    """Stratum 3: score = mean_stdev_s * 4.0 (strongly disfavored)."""
    assert abs(mod.compute_composite_score(60e-6, cv=0.0, stratum=3) - 60e-6 * 4.0) < 1e-12


def test_q6_stratum4_factor(mod):
    """Stratum 4+: score = mean_stdev_s * 10.0 (effectively excluded)."""
    assert abs(mod.compute_composite_score(60e-6, cv=0.0, stratum=4) - 60e-6 * 10.0) < 1e-12


def test_q7_cv_penalty(mod):
    """Non-zero CV multiplies score by (1 + CV)."""
    assert abs(mod.compute_composite_score(100e-6, cv=0.5, stratum=1) - 100e-6 * 1.5) < 1e-12


def test_q8_score_servers_lower_stdev_wins(mod):
    """score_servers: server with lower mean_stdev gets lower composite score."""
    stdev_stats = {
        "a.com": {"mean_stdev_s": 60e-6, "cv": 0.0, "n": 10},
        "b.com": {"mean_stdev_s": 120e-6, "cv": 0.0, "n": 10},
    }
    scores = mod.score_servers(stdev_stats, strata={"a.com": 1, "b.com": 1})
    assert scores["a.com"] < scores["b.com"]


def test_q9_score_servers_cv_gated(mod):
    """score_servers: server with CV > 2.0 gets inf score."""
    stdev_stats = {
        "a.com": {"mean_stdev_s": 60e-6, "cv": 2.5, "n": 10},
        "b.com": {"mean_stdev_s": 120e-6, "cv": 0.0, "n": 10},
    }
    scores = mod.score_servers(stdev_stats, strata={"a.com": 1, "b.com": 1})
    assert scores["a.com"] == float("inf")


def test_q10_score_servers_stratum_applied(mod):
    """score_servers: same raw stats, stratum 2 gets higher score than stratum 1."""
    stdev_stats = {
        "s1.com": {"mean_stdev_s": 60e-6, "cv": 0.0, "n": 10},
        "s2.com": {"mean_stdev_s": 60e-6, "cv": 0.0, "n": 10},
    }
    scores = mod.score_servers(stdev_stats, strata={"s1.com": 1, "s2.com": 2})
    assert scores["s1.com"] < scores["s2.com"]


def test_q11_score_servers_missing_stratum_penalized(mod):
    """score_servers: server absent from strata dict gets a high-stratum penalty."""
    stdev_stats = {"unknown.com": {"mean_stdev_s": 60e-6, "cv": 0.0, "n": 10}}
    scores = mod.score_servers(stdev_stats, strata={})
    assert scores["unknown.com"] > 60e-6  # penalized vs clean stratum-1 score


def test_q12_score_servers_empty(mod):
    """score_servers: empty input returns empty dict."""
    assert mod.score_servers({}, {}) == {}


# ---------------------------------------------------------------------------
# Group S — check_daily_criteria
# ---------------------------------------------------------------------------

def _daily_state(reference_server="ref.com", calibration_score=60e-6,
                 calibrated_offsets=None):
    """Minimal MONITORING state for daily-criteria tests."""
    return {
        "mode": "MONITORING",
        "reference_server": reference_server,
        "calibration_composite_score": calibration_score,
        "calibrated_offsets": calibrated_offsets or {reference_server: 0.0},
    }


def test_s1_criterion1_fires_when_reference_degraded(mod):
    """Criterion 1 fires when reference composite score exceeds 2× calibration baseline."""
    state = _daily_state(calibration_score=60e-6)
    # 130e-6 > 2 × 60e-6 = 120e-6
    criteria = mod.check_daily_criteria(state, {"ref.com": 130e-6}, {})
    assert 1 in criteria


def test_s2_criterion1_silent_below_threshold(mod):
    """Criterion 1 silent when reference score is within 2× baseline."""
    state = _daily_state(calibration_score=60e-6)
    # 110e-6 < 2 × 60e-6 = 120e-6
    criteria = mod.check_daily_criteria(state, {"ref.com": 110e-6}, {})
    assert 1 not in criteria


def test_s3_criterion2_fires_on_offset_drift(mod):
    """Criterion 2 fires when active server offset drifts > 0.5 ms from calibration."""
    state = _daily_state(calibrated_offsets={"ref.com": 0.0, "backup.com": 0.0001})
    # drift = |0.0007 - 0.0001| = 0.6 ms > 0.5 ms
    now_offsets = {"ref.com": 0.0, "backup.com": 0.0007}
    criteria = mod.check_daily_criteria(
        state, {"ref.com": 60e-6, "backup.com": 80e-6}, now_offsets
    )
    assert 2 in criteria


def test_s4_criterion2_silent_within_threshold(mod):
    """Criterion 2 silent when offsets within 0.5 ms of calibrated values."""
    state = _daily_state(calibrated_offsets={"ref.com": 0.0, "backup.com": 0.0001})
    # drift = |0.0005 - 0.0001| = 0.4 ms < 0.5 ms
    now_offsets = {"ref.com": 0.0, "backup.com": 0.0005}
    criteria = mod.check_daily_criteria(
        state, {"ref.com": 60e-6, "backup.com": 80e-6}, now_offsets
    )
    assert 2 not in criteria


def test_s5_criterion3_fires_when_scout_substantially_better(mod):
    """Criterion 3 fires when any non-reference server scores < 75% of reference."""
    state = _daily_state(reference_server="ref.com")
    # scout at 50% of reference score — substantially better
    scores = {"ref.com": 100e-6, "scout.com": 50e-6}
    criteria = mod.check_daily_criteria(state, scores, {})
    assert 3 in criteria


def test_s6_criterion3_silent_when_alternatives_not_better_enough(mod):
    """Criterion 3 silent when no non-reference server beats reference by >25%."""
    state = _daily_state(reference_server="ref.com")
    # close.com at 80% — not better enough (75% threshold)
    scores = {"ref.com": 100e-6, "close.com": 80e-6, "worse.com": 120e-6}
    criteria = mod.check_daily_criteria(state, scores, {})
    assert 3 not in criteria


def test_s7_multiple_criteria_can_fire_together(mod):
    """Multiple criteria can trigger in the same daily check."""
    state = _daily_state(
        calibration_score=60e-6,
        calibrated_offsets={"ref.com": 0.0, "backup.com": 0.0},
    )
    scores = {"ref.com": 200e-6, "scout.com": 30e-6}   # crit 1 and crit 3
    now_offsets = {"ref.com": 0.0, "backup.com": 0.001}  # crit 2
    criteria = mod.check_daily_criteria(state, scores, now_offsets)
    assert 1 in criteria
    assert 2 in criteria
    assert 3 in criteria


def test_s8_no_criteria_when_healthy(mod):
    """No criteria fire when reference is healthy and offsets are stable."""
    state = _daily_state(
        calibration_score=60e-6,
        calibrated_offsets={"ref.com": 0.0, "backup.com": 0.0001},
    )
    scores = {"ref.com": 70e-6, "backup.com": 90e-6, "worse.com": 200e-6}
    now_offsets = {"ref.com": 0.0, "backup.com": 0.00015}
    criteria = mod.check_daily_criteria(state, scores, now_offsets)
    assert criteria == []


# ---------------------------------------------------------------------------
# Group T — log_daily_evaluation
# ---------------------------------------------------------------------------

def test_t1_daily_log_appends_jsonl(mod, tmp_path):
    """log_daily_evaluation appends a parseable JSON line to the log file."""
    import json as _json
    log_path = str(tmp_path / "daily_eval.jsonl")
    mod.log_daily_evaluation(
        log_path=log_path,
        timestamp=1_000_000.0,
        server_scores={"ref.com": 60e-6, "backup.com": 80e-6},
        stdev_stats={"ref.com": {"mean_stdev_s": 60e-6, "cv": 0.1, "n": 1440}},
        offset_drifts={"backup.com": 0.0001},
    )
    lines = Path(log_path).read_text().splitlines()
    assert len(lines) == 1
    row = _json.loads(lines[0])
    assert row["timestamp"] == 1_000_000.0
    assert "server_scores" in row
    assert "stdev_stats" in row
    assert "offset_drifts" in row


def test_t2_daily_log_multiple_appends(mod, tmp_path):
    """Each call appends one line; N calls → N lines."""
    log_path = str(tmp_path / "daily_eval.jsonl")
    for i in range(3):
        mod.log_daily_evaluation(
            log_path=log_path,
            timestamp=float(i),
            server_scores={},
            stdev_stats={},
            offset_drifts={},
        )
    lines = Path(log_path).read_text().splitlines()
    assert len(lines) == 3


# ---------------------------------------------------------------------------
# Group C (extended) — parse_tracking new fields
# ---------------------------------------------------------------------------

SAMPLE_TRACKING_FAST = """\
Reference ID    : 11FD023B (17.253.2.35)
Stratum         : 2
Ref time (UTC)  : Sun Jun 15 12:00:00 2026
System time     : 0.000000050 seconds fast of NTP time
Last offset     : +0.000000050 seconds
RMS offset      : 0.000000100 seconds
Frequency       : 2.500 ppm fast
Residual freq   : -0.002 ppm
Skew            : 0.050 ppm
Root delay      : 0.002000000 seconds
Root dispersion : 0.000500000 seconds
Update interval : 128.0 seconds
Leap status     : Normal
"""


def test_c7_rms_offset_s(mod):
    """RMS offset parsed to seconds."""
    result = mod.parse_tracking(SAMPLE_TRACKING_NTP)
    assert abs(result["rms_offset_s"] - 61e-9) < 1e-15


def test_c8_root_delay_s(mod):
    """Root delay parsed to seconds."""
    result = mod.parse_tracking(SAMPLE_TRACKING_NTP)
    assert abs(result["root_delay_s"] - 0.001234567) < 1e-9


def test_c9_root_dispersion_s(mod):
    """Root dispersion parsed to seconds."""
    result = mod.parse_tracking(SAMPLE_TRACKING_NTP)
    assert abs(result["root_dispersion_s"] - 0.000234567) < 1e-9


def test_c10_freq_ppm_slow_is_positive(mod):
    """'ppm slow' frequency stored as positive value."""
    result = mod.parse_tracking(SAMPLE_TRACKING_NTP)
    assert abs(result["freq_ppm"] - 1.234) < 1e-6


def test_c11_freq_ppm_fast_is_negative(mod):
    """'ppm fast' frequency stored as negative value."""
    result = mod.parse_tracking(SAMPLE_TRACKING_FAST)
    assert abs(result["freq_ppm"] - (-2.500)) < 1e-6


def test_c12_update_interval_s(mod):
    """Update interval parsed to seconds (float)."""
    assert abs(mod.parse_tracking(SAMPLE_TRACKING_NTP)["update_interval_s"] - 64.2) < 1e-6
    assert abs(mod.parse_tracking(SAMPLE_TRACKING_FAST)["update_interval_s"] - 128.0) < 1e-6


def test_c13_none_fields_when_garbage(mod):
    """Unparseable input returns None for all numeric fields."""
    result = mod.parse_tracking("not chronyc output")
    assert result["rms_offset_s"] is None
    assert result["root_delay_s"] is None
    assert result["root_dispersion_s"] is None
    assert result["freq_ppm"] is None
    assert result["update_interval_s"] is None


# ---------------------------------------------------------------------------
# Group R — log_sample and log_tracking_sample (extended logging)
# ---------------------------------------------------------------------------

def test_r1_log_sample_header_written_for_new_file(mod, tmp_path):
    """A CSV header row is written when the log file does not yet exist."""
    log_path = str(tmp_path / "samples.csv")
    mod.log_sample(log_path, 1000.0, "time.google.com", 0.0001, 0.0002, 1, 377, 16)
    with open(log_path) as f:
        first_line = f.readline().strip()
    assert first_line.startswith("timestamp")


def test_r2_log_sample_no_duplicate_header(mod, tmp_path):
    """Calling log_sample twice does not write a second header row."""
    log_path = str(tmp_path / "samples.csv")
    mod.log_sample(log_path, 1000.0, "time.google.com", 0.0001, 0.0002, 1, 377, 16)
    mod.log_sample(log_path, 1060.0, "time.google.com", 0.0001, 0.0002, 1, 377, 16)
    with open(log_path) as f:
        lines = [l for l in f.read().splitlines() if l.strip()]
    header_count = sum(1 for l in lines if l.startswith("timestamp"))
    assert header_count == 1
    assert len(lines) == 3  # header + 2 data rows


def test_r3_log_sample_row_columns(mod, tmp_path):
    """Data row contains all expected columns in correct order."""
    import csv as csv_mod
    log_path = str(tmp_path / "samples.csv")
    mod.log_sample(log_path, 1781647292.0, "ntp.test.com", -0.000123, 0.000456, 2, 255, 8)
    with open(log_path, newline="") as f:
        reader = csv_mod.DictReader(f)
        row = next(reader)
    assert float(row["timestamp"]) == pytest.approx(1781647292.0)
    assert row["server"] == "ntp.test.com"
    assert float(row["offset_s"]) == pytest.approx(-0.000123)
    assert float(row["stdev_s"]) == pytest.approx(0.000456)
    assert int(row["stratum"]) == 2
    assert int(row["reach"]) == 255
    assert int(row["np"]) == 8


def test_r4_log_tracking_sample_header_for_new_file(mod, tmp_path):
    """A CSV header row is written when the tracking log does not yet exist."""
    log_path = str(tmp_path / "tracking.csv")
    tracking = {
        "ref_id_name": "17.253.2.35", "stratum": 2, "synchronized": True,
        "rms_offset_s": 61e-9, "root_delay_s": 0.001, "root_dispersion_s": 0.0002,
        "freq_ppm": 1.234, "update_interval_s": 64.2,
    }
    mod.log_tracking_sample(log_path, 1781647292.0, tracking)
    with open(log_path) as f:
        first_line = f.readline().strip()
    assert first_line.startswith("timestamp")


def test_r5_log_tracking_sample_row_columns(mod, tmp_path):
    """Tracking row contains all expected columns."""
    import csv as csv_mod
    log_path = str(tmp_path / "tracking.csv")
    tracking = {
        "ref_id_name": "17.253.2.35", "stratum": 2, "synchronized": True,
        "rms_offset_s": 61e-9, "root_delay_s": 0.001234, "root_dispersion_s": 0.000234,
        "freq_ppm": -2.5, "update_interval_s": 64.2,
    }
    mod.log_tracking_sample(log_path, 1781647292.5, tracking)
    with open(log_path, newline="") as f:
        reader = csv_mod.DictReader(f)
        row = next(reader)
    assert float(row["timestamp"]) == pytest.approx(1781647292.5)
    assert row["ref_id"] == "17.253.2.35"
    assert int(row["stratum"]) == 2
    assert row["synchronized"] == "True"
    assert float(row["rms_offset_s"]) == pytest.approx(61e-9, rel=1e-6)
    assert float(row["root_delay_s"]) == pytest.approx(0.001234, rel=1e-6)
    assert float(row["root_dispersion_s"]) == pytest.approx(0.000234, rel=1e-6)
    assert float(row["freq_ppm"]) == pytest.approx(-2.5, rel=1e-6)
    assert float(row["update_interval_s"]) == pytest.approx(64.2, rel=1e-6)
