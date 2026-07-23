"""Tests for raspberry_pi/scripts/nk_dashboard.py (server-only dashboard).

importlib-loaded; file paths monkeypatched to tmp_path; discovery/fetch/forward
injected or mocked. No real sockets or mDNS.
"""
import importlib.util
from pathlib import Path
from unittest.mock import MagicMock

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
SCRIPT_PATH = REPO_ROOT / "raspberry_pi" / "scripts" / "nk_dashboard.py"


def _load():
    spec = importlib.util.spec_from_file_location("nk_dashboard", SCRIPT_PATH)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


@pytest.fixture(scope="module")
def mod():
    return _load()


SAMPLE_BROWSE = (
    "+;eth0;IPv4;NK\\032status;_neurokairos-status._tcp;local\n"
    "=;eth0;IPv4;NK\\032status;_neurokairos-status._tcp;local;a.local;192.168.1.5;8080;\n"
    "=;wlan0;IPv4;NK\\032status;_neurokairos-status._tcp;local;a.local;192.168.1.5;8080;\n"
    "=;eth0;IPv6;NK\\032status;_neurokairos-status._tcp;local;b.local;fe80::1;8080;\n"
    "=;eth0;IPv4;NK\\032status;_neurokairos-status._tcp;local;b.local;10.0.0.2;8080;\n"
)


# --- discovery parse -------------------------------------------------------

def test_parse_status_services(mod):
    assert mod.parse_status_services(SAMPLE_BROWSE) == [
        ("10.0.0.2", 8080), ("192.168.1.5", 8080)
    ]


def test_parse_status_services_empty(mod):
    assert mod.parse_status_services("") == []


# --- UI asset --------------------------------------------------------------

def test_read_index_serves_ui(mod):
    html = mod.read_index()
    assert "NeuroKairos" in html
    assert "/api/all" in html          # the poll endpoint the page uses
    assert "UI asset missing" not in html   # repo-layout fallback found it


# --- aggregation -----------------------------------------------------------

def test_build_all_sorts_servers_first(mod):
    statuses = [
        {"hostname": "c1", "role": "client", "name": "Zeta", "online": True},
        {"hostname": "s1", "role": "server", "name": "Alpha", "online": True},
        {"hostname": "c2", "role": "client", "name": "Beta", "online": True},
    ]
    pis = mod.build_all(statuses)["pis"]
    assert [p["hostname"] for p in pis] == ["s1", "c2", "c1"]  # server, then name


def test_build_all_dedupes_prefers_online(mod):
    statuses = [
        {"hostname": "x", "ip": "1.1.1.1", "online": False},
        {"hostname": "x", "ip": "1.1.1.1", "online": True, "role": "client", "name": "X"},
    ]
    pis = mod.build_all(statuses)["pis"]
    assert len(pis) == 1 and pis[0]["online"] is True


def test_collect_uses_injected_fetcher(mod):
    fetcher = MagicMock(side_effect=lambda ip, port: {"ip": ip, "online": True})
    out = mod.collect([("1.2.3.4", 8080), ("5.6.7.8", 8080)], fetcher=fetcher)
    assert [s["ip"] for s in out] == ["1.2.3.4", "5.6.7.8"]
    assert fetcher.call_count == 2


# --- password store + auth-status ------------------------------------------

def test_password_store_roundtrip(mod, monkeypatch, tmp_path):
    monkeypatch.setattr(mod, "AUTH_PATH", str(tmp_path / "auth.json"))
    assert mod.load_password_hash() == ""
    _, body = mod.handle_auth_status()
    assert body["configured"] is False
    mod.store_password_hash(mod.hash_password("secret"))
    assert mod.load_password_hash() == mod.hash_password("secret")
    _, body = mod.handle_auth_status()
    assert body["configured"] is True


# --- tokens ----------------------------------------------------------------

def test_token_lifecycle(mod):
    t = mod.create_token(now=1000, ttl=100)
    assert mod.check_token(t, now=1050) is True
    assert mod.check_token(t, now=1101) is False   # expired
    assert mod.check_token("bogus", now=1000) is False


# --- set-password ----------------------------------------------------------

def test_set_password_first_run_pushes(mod, monkeypatch, tmp_path):
    monkeypatch.setattr(mod, "AUTH_PATH", str(tmp_path / "auth.json"))
    pusher = MagicMock()
    code, _ = mod.handle_set_password({"password": "letmein"}, pusher=pusher)
    assert code == 200
    assert mod.load_password_hash() == mod.hash_password("letmein")
    pusher.assert_called_once()


def test_set_password_too_short(mod, monkeypatch, tmp_path):
    monkeypatch.setattr(mod, "AUTH_PATH", str(tmp_path / "auth.json"))
    code, _ = mod.handle_set_password({"password": "ab"}, pusher=MagicMock())
    assert code == 400


def test_set_password_change_requires_current(mod, monkeypatch, tmp_path):
    monkeypatch.setattr(mod, "AUTH_PATH", str(tmp_path / "auth.json"))
    mod.handle_set_password({"password": "first"}, pusher=MagicMock())
    # wrong current
    code, _ = mod.handle_set_password(
        {"password": "second", "current": "nope"}, pusher=MagicMock())
    assert code == 403
    # right current
    code, _ = mod.handle_set_password(
        {"password": "second", "current": "first"}, pusher=MagicMock())
    assert code == 200
    assert mod.load_password_hash() == mod.hash_password("second")


# --- login -----------------------------------------------------------------

def test_login_flow(mod, monkeypatch, tmp_path):
    monkeypatch.setattr(mod, "AUTH_PATH", str(tmp_path / "auth.json"))
    code, _ = mod.handle_login({"password": "x"})
    assert code == 409                               # not configured
    mod.store_password_hash(mod.hash_password("openme"))
    code, body = mod.handle_login({"password": "wrong"})
    assert code == 403
    code, body = mod.handle_login({"password": "openme"}, now=2000)
    assert code == 200 and mod.check_token(body["token"], now=2000)


# --- control ---------------------------------------------------------------

def test_control_requires_token(mod, monkeypatch, tmp_path):
    monkeypatch.setattr(mod, "AUTH_PATH", str(tmp_path / "auth.json"))
    code, _ = mod.handle_control({"target": "1.2.3.4", "action": "reboot"})
    assert code == 401


def test_control_needs_target_and_action(mod, monkeypatch, tmp_path):
    monkeypatch.setattr(mod, "AUTH_PATH", str(tmp_path / "auth.json"))
    tok = mod.create_token(now=500)
    code, _ = mod.handle_control({"token": tok, "action": "reboot"}, now=500)
    assert code == 400


# --- pulse-cadence monitoring ----------------------------------------------

EP = "10.0.0.5:8080"   # endpoint key (ip:port)


@pytest.fixture
def monitor(mod, monkeypatch, tmp_path):
    monkeypatch.setattr(mod, "ALERTS_LOG", str(tmp_path / "anomalies.log"))
    mod.reset_monitor_state()
    mod.record_poll_meta([
        {"ip": "10.0.0.5", "endpoint": EP, "name": "Rig A", "role": "client",
         "services": {"irig-sender": True}},
    ])
    return mod


def test_ingest_good_intervals_no_alert(monitor):
    assert monitor.ingest_onset(EP, 1000.0, 1000.0) is None
    assert monitor.ingest_onset(EP, 1001.0, 1001.0) is None    # 1.00 s
    assert monitor.ingest_onset(EP, 1002.05, 1002.05) is None  # 1.05 s ok
    assert monitor.active_alerts() == []


def test_ingest_long_interval_alerts(monitor):
    monitor.ingest_onset(EP, 1000.0, 1000.0)
    a = monitor.ingest_onset(EP, 1001.6, 1001.6)   # 1.6 s gap
    assert a is not None and a["kind"] == "interval"
    assert a["interval_s"] == 1.6
    assert a["sender_name"] == "Rig A"
    assert len(monitor.active_alerts()) == 1


def test_ingest_short_interval_alerts(monitor):
    monitor.ingest_onset(EP, 1000.0, 1000.0)
    a = monitor.ingest_onset(EP, 1000.3, 1000.3)   # 0.3 s gap
    assert a is not None and a["interval_s"] == 0.3


def test_ingest_dedup(monitor):
    monitor.ingest_onset(EP, 1000.0, 1000.0)
    monitor.ingest_onset(EP, 1001.6, 1001.6)
    n = len(monitor._ALERTS)
    monitor.ingest_onset(EP, 1001.6, 1001.6)   # idempotent id -> no new alert
    assert len(monitor._ALERTS) == n


def test_watchdog_stopped_and_resume(monitor):
    monitor.ingest_onset(EP, 1000.0, 1000.0)
    monitor.ingest_onset(EP, 1001.0, 1001.0)
    raised = monitor.watchdog_once(1006.0)          # 5 s silent while sending
    assert len(raised) == 1 and raised[0]["kind"] == "stopped"
    assert len(monitor.active_alerts()) == 1
    assert monitor.watchdog_once(1007.0) == []      # no duplicate
    monitor.ingest_onset(EP, 1007.0, 1007.0)        # resume clears stopped
    assert [a for a in monitor.active_alerts() if a["kind"] == "stopped"] == []


def test_watchdog_ignores_non_sending(monitor):
    monitor.record_poll_meta([
        {"ip": "10.0.0.9", "endpoint": "10.0.0.9:8080", "name": "Idle",
         "role": "client", "services": {"irig-sender": False}}])
    monitor.ingest_onset("10.0.0.9:8080", 1000.0, 1000.0)
    assert monitor.watchdog_once(1010.0) == []


def test_dismiss_requires_token(mod, monkeypatch, tmp_path, monitor):
    monkeypatch.setattr(mod, "AUTH_PATH", str(tmp_path / "auth.json"))
    monitor.ingest_onset(EP, 1000.0, 1000.0)
    a = monitor.ingest_onset(EP, 1002.0, 1002.0)   # 2 s -> alert
    code, _ = mod.dismiss_alert(a["id"], "bogus")
    assert code == 401 and len(mod.active_alerts()) == 1
    tok = mod.create_token(now=5)
    code, _ = mod.dismiss_alert(a["id"], tok, now=5)
    assert code == 200 and mod.active_alerts() == []


def test_handle_onset_validation(monitor):
    assert monitor.handle_onset({"ip": "10.0.0.5"}, 1000.0)[0] == 400
    assert monitor.handle_onset({"onset": 1.0}, 1000.0)[0] == 400
    assert monitor.handle_onset({"ip": "10.0.0.5", "port": 8080, "onset": 1000.0}, 1000.0)[0] == 200


def test_control_forwards_with_auth_hash(mod, monkeypatch, tmp_path):
    monkeypatch.setattr(mod, "AUTH_PATH", str(tmp_path / "auth.json"))
    mod.store_password_hash(mod.hash_password("pw"))
    tok = mod.create_token(now=500)
    forwarder = MagicMock(return_value={"ok": True})
    code, body = mod.handle_control(
        {"token": tok, "target": "192.168.1.9", "action": "restart",
         "params": {"service": "chrony"}},
        forwarder=forwarder, now=500)
    assert code == 200 and body["ok"] is True
    args = forwarder.call_args[0]
    assert args[0] == "192.168.1.9"              # target
    assert args[2] == "restart"                  # action
    assert args[4] == mod.hash_password("pw")    # auth hash forwarded
