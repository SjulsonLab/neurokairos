"""Tests for raspberry_pi/scripts/nk_status_agent.py.

Loaded via importlib like the other Pi scripts. Pure functions are tested
directly; file paths are monkeypatched to tmp_path; subprocess/system actions
are mocked. No real sockets, systemctl, or chronyc are invoked.
"""
import importlib.util
import json
from pathlib import Path
from unittest.mock import MagicMock

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
SCRIPT_PATH = REPO_ROOT / "raspberry_pi" / "scripts" / "nk_status_agent.py"
CALIB_PATH = REPO_ROOT / "raspberry_pi" / "scripts" / "ntp_calibrator.py"


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


@pytest.fixture(scope="module")
def mod():
    return _load(SCRIPT_PATH, "nk_status_agent")


@pytest.fixture(scope="module")
def calib():
    return _load(CALIB_PATH, "ntp_calibrator")


# Sample chronyc tracking (synchronized, ~0.4 ms dispersion)
SAMPLE_TRACKING = """\
Reference ID    : C0A80101 (ntp.example.com)
Stratum         : 3
Ref time (UTC)  : Wed Jul 23 00:00:00 2026
System time     : 0.000012345 seconds slow of NTP time
Last offset     : +0.000004000 seconds
RMS offset      : 0.000050000 seconds
Frequency       : 12.345 ppm slow
Residual freq   : +0.001 ppm
Skew            : 0.100 ppm
Root delay      : 0.001200000 seconds
Root dispersion : 0.000400000 seconds
Update interval : 64.5 seconds
Leap status     : Normal
"""

SAMPLE_SOURCES = """\
MS Name/IP address         Stratum Poll Reach LastRx Last sample
===============================================================================
^* 192.168.1.1                   2   6   377    12    +1us[  +2us] +/-  500us
^+ 10.0.0.2                       2   6   377    18    +5us[  +6us] +/-  600us
^? 203.0.113.9                    0   6     0     -     +0ns[  +0ns] +/-    0ns
"""


# --- quality_level ---------------------------------------------------------

@pytest.mark.parametrize("synced,disp_s,expected", [
    (False, 0.0001, "unsynced"),
    (True, None, "unknown"),
    (True, 0.0002, "good"),      # 0.2 ms < 0.5 ms
    (True, 0.0006, "marginal"),  # 0.6 ms in [0.5,1.0)
    (True, 0.0015, "bad"),       # 1.5 ms >= 1.0 ms
])
def test_quality_level(mod, synced, disp_s, expected):
    assert mod.quality_level(synced, disp_s, warn_ms=1.0) == expected


# --- build_status ----------------------------------------------------------

def test_build_status_shape(mod, calib):
    tracking = calib.parse_tracking(SAMPLE_TRACKING)
    sources = calib.parse_sources(SAMPLE_SOURCES)
    st = mod.build_status(
        name="lab-1", hostname="neurokairos-sender", ip="192.168.1.5",
        role="client", tracking=tracking,
        tier=calib.detect_reference_tier(tracking, []),
        sources=sources, diversity=calib.check_source_diversity(sources),
        services={"irig-sender": True, "chrony": True}, calibrator=None,
    )
    assert st["name"] == "lab-1"
    assert st["role"] == "client"
    assert st["timing"]["synchronized"] is True
    assert st["timing"]["stratum"] == 3
    assert st["timing"]["quality"] == "good"          # 0.4 ms
    assert st["timing"]["reference"] == "ntp.example.com"
    assert st["sources"]["count"] == 3
    assert st["services"]["irig-sender"] is True


# --- role ------------------------------------------------------------------

def test_get_role(mod):
    assert mod.get_role("neurokairos-server") == "server"
    assert mod.get_role("neurokairos-sender") == "client"
    assert mod.get_role("anything-else") == "client"


# --- calibrator metrics ----------------------------------------------------

def test_read_calibrator_metrics(mod, monkeypatch, tmp_path):
    p = tmp_path / "state.json"
    monkeypatch.setattr(mod, "CALIBRATOR_STATE_PATH", str(p))
    assert mod.read_calibrator_metrics() is None          # missing
    p.write_text("not json")
    assert mod.read_calibrator_metrics() is None          # bad json
    p.write_text(json.dumps({"precision_typical_ms": 0.3, "unrelated": 1}))
    m = mod.read_calibrator_metrics()
    assert m == {"precision_typical_ms": 0.3}


# --- auth ------------------------------------------------------------------

def test_auth_roundtrip(mod, monkeypatch, tmp_path):
    monkeypatch.setattr(mod, "AUTH_PATH", str(tmp_path / "agent-auth"))
    assert mod.check_auth("anything") is False            # nothing stored
    h = mod.hash_password("hunter2")
    mod.store_auth_hash(h)
    assert mod.check_auth(h) is True
    assert mod.check_auth("wrong") is False


def test_handle_set_auth(mod, monkeypatch, tmp_path):
    monkeypatch.setattr(mod, "AUTH_PATH", str(tmp_path / "agent-auth"))
    h1 = mod.hash_password("first")
    code, _ = mod.handle_set_auth({"hash": h1})
    assert code == 200 and mod.load_auth_hash() == h1
    # changing requires proving the current hash
    h2 = mod.hash_password("second")
    code, _ = mod.handle_set_auth({"hash": h2})
    assert code == 403
    code, _ = mod.handle_set_auth({"hash": h2, "current": h1})
    assert code == 200 and mod.load_auth_hash() == h2
    # malformed hash rejected
    code, _ = mod.handle_set_auth({"hash": "short"})
    assert code == 400


# --- control dispatch ------------------------------------------------------

def _authed(mod, monkeypatch, tmp_path):
    monkeypatch.setattr(mod, "AUTH_PATH", str(tmp_path / "agent-auth"))
    h = mod.hash_password("pw")
    mod.store_auth_hash(h)
    return h


def test_control_requires_auth(mod, monkeypatch, tmp_path):
    _authed(mod, monkeypatch, tmp_path)
    code, body = mod.handle_control({"action": "reboot", "auth": "nope"})
    assert code == 403 and "auth" in body["error"]


def test_control_unknown_action(mod, monkeypatch, tmp_path):
    code, body = mod.handle_control({"action": "explode"})
    assert code == 400


def test_control_rename(mod, monkeypatch, tmp_path):
    h = _authed(mod, monkeypatch, tmp_path)
    monkeypatch.setattr(mod, "NAME_PATH", str(tmp_path / "name"))
    code, body = mod.handle_control(
        {"action": "rename", "auth": h, "params": {"name": "Rig A"}})
    assert code == 200 and body["result"]["name"] == "Rig A"
    assert Path(mod.NAME_PATH).read_text() == "Rig A"


def test_control_restart_allowlist(mod, monkeypatch, tmp_path):
    h = _authed(mod, monkeypatch, tmp_path)
    run = MagicMock()
    monkeypatch.setattr(mod, "_run", run)
    # allowed service
    code, _ = mod.handle_control(
        {"action": "restart", "auth": h, "params": {"service": "chrony"}})
    assert code == 200
    run.assert_called_once_with(["systemctl", "restart", "chrony"])
    # disallowed service rejected (never shells out)
    run.reset_mock()
    code, body = mod.handle_control(
        {"action": "restart", "auth": h, "params": {"service": "sshd"}})
    assert code == 400 and run.call_count == 0


def test_control_reboot(mod, monkeypatch, tmp_path):
    h = _authed(mod, monkeypatch, tmp_path)
    run = MagicMock()
    monkeypatch.setattr(mod, "_run", run)
    code, _ = mod.handle_control({"action": "reboot", "auth": h})
    assert code == 200
    run.assert_called_once_with(["systemctl", "reboot"])


def test_set_ntp_writes_and_reloads(mod, monkeypatch, tmp_path):
    h = _authed(mod, monkeypatch, tmp_path)
    src = tmp_path / "sources.d" / "neurokairos.sources"
    monkeypatch.setattr(mod, "SOURCES_FILE", str(src))
    sp = MagicMock()
    monkeypatch.setattr(mod.subprocess, "run", sp)
    code, body = mod.handle_control(
        {"action": "set-ntp", "auth": h, "params": {"server": "192.168.1.50"}})
    assert code == 200
    assert "server 192.168.1.50 iburst prefer" in src.read_text()
    sp.assert_called_once()  # chronyc reload sources
    # clearing
    code, _ = mod.handle_control({"action": "set-ntp", "auth": h, "params": {"server": ""}})
    assert code == 200
    assert "server " not in src.read_text()


# --- gather_status integration (real parser, mocked chronyc) ---------------

def test_gather_status_integration(mod, calib, monkeypatch, tmp_path):
    monkeypatch.setattr(calib, "run_chronyc_tracking", lambda: SAMPLE_TRACKING)
    monkeypatch.setattr(calib, "run_chronyc_sources", lambda: SAMPLE_SOURCES)
    monkeypatch.setattr(mod, "NAME_PATH", str(tmp_path / "name"))
    monkeypatch.setattr(mod, "CALIBRATOR_STATE_PATH", str(tmp_path / "state.json"))
    monkeypatch.setattr(mod, "systemctl_is_active", lambda u: True)
    st = mod.gather_status(calib=calib)
    assert st["timing"]["stratum"] == 3
    assert st["timing"]["synchronized"] is True
    assert st["services"]["chrony"] is True
