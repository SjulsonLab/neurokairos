"""Tests for the headless Wi-Fi onboarding scripts.

nk_wifi_onboard.py (tier-decision) and nk_wifi_portal.py (setup hotspot) are
loaded via importlib like the other Pi scripts. Only the pure logic is exercised
here — config parsing, the "already networked" predicate, scan parsing, nmcli
arg construction, page rendering, and the tier decision in main() with all I/O
mocked. No real nmcli/rfkill/systemctl/sockets are invoked.
"""
import importlib.util
from pathlib import Path
from unittest.mock import MagicMock

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
ONBOARD_PATH = REPO_ROOT / "raspberry_pi" / "scripts" / "nk_wifi_onboard.py"
PORTAL_PATH = REPO_ROOT / "raspberry_pi" / "scripts" / "nk_wifi_portal.py"


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


@pytest.fixture(scope="module")
def ob():
    return _load(ONBOARD_PATH, "nk_wifi_onboard")


@pytest.fixture(scope="module")
def portal():
    return _load(PORTAL_PATH, "nk_wifi_portal")


# ---------------------------------------------------------------------------
# parse_wifi_config
# ---------------------------------------------------------------------------

def test_parse_basic(ob):
    cfg = ob.parse_wifi_config("country=US\nssid=MyNet\npassword=secret\n")
    assert cfg == {"country": "US", "ssid": "MyNet", "password": "secret"}


def test_parse_psk_alias_and_quotes(ob):
    cfg = ob.parse_wifi_config('ssid="My Net"\npsk=\'p@ss word\'\n')
    assert cfg["ssid"] == "My Net"
    assert cfg["password"] == "p@ss word"


def test_parse_ignores_comments_blanks_and_unknown(ob):
    cfg = ob.parse_wifi_config("# comment\n\n  \nfoo=bar\nssid=X\nnonsense line\n")
    assert cfg == {"ssid": "X"}


def test_parse_last_wins_and_case_insensitive_key(ob):
    cfg = ob.parse_wifi_config("SSID=first\nssid=second\n")
    assert cfg["ssid"] == "second"


def test_parse_empty_password_kept(ob):
    cfg = ob.parse_wifi_config("ssid=Open\npassword=\n")
    assert cfg == {"ssid": "Open", "password": ""}


# ---------------------------------------------------------------------------
# networked_from_status
# ---------------------------------------------------------------------------

def test_networked_true_for_connected_eth(ob):
    assert ob.networked_from_status("eth0:connected\nwlan0:disconnected\nlo:unmanaged")


def test_networked_ignores_loopback(ob):
    # `lo` is 'connected' in NM's sense but must never count.
    assert not ob.networked_from_status("lo:connected\neth0:unavailable")


def test_networked_false_when_nothing_connected(ob):
    assert not ob.networked_from_status("eth0:disconnected\nwlan0:disconnected")


# ---------------------------------------------------------------------------
# main() tier decision (all I/O mocked)
# ---------------------------------------------------------------------------

def test_main_noop_when_already_networked(ob, monkeypatch):
    monkeypatch.setattr(ob, "wait_online", lambda *a, **k: None)
    monkeypatch.setattr(ob, "networked", lambda: True)
    start = MagicMock()
    prov = MagicMock()
    monkeypatch.setattr(ob, "start_portal", start)
    monkeypatch.setattr(ob, "provision_from_file", prov)
    assert ob.main() == 0
    start.assert_not_called()
    prov.assert_not_called()


def test_main_uses_boot_file_when_present(ob, monkeypatch, tmp_path):
    cfg = tmp_path / "neurokairos-wifi.txt"
    cfg.write_text("ssid=X\n")
    monkeypatch.setattr(ob, "wait_online", lambda *a, **k: None)
    monkeypatch.setattr(ob, "networked", lambda: False)
    monkeypatch.setattr(ob, "BOOT_CFG", str(cfg))
    prov = MagicMock(return_value=True)
    start = MagicMock()
    monkeypatch.setattr(ob, "provision_from_file", prov)
    monkeypatch.setattr(ob, "start_portal", start)
    assert ob.main() == 0
    prov.assert_called_once()
    start.assert_not_called()


def test_main_starts_portal_when_no_file_and_unprovisioned(ob, monkeypatch, tmp_path):
    monkeypatch.setattr(ob, "wait_online", lambda *a, **k: None)
    monkeypatch.setattr(ob, "networked", lambda: False)
    monkeypatch.setattr(ob, "BOOT_CFG", str(tmp_path / "absent.txt"))
    monkeypatch.setattr(ob, "provisioned", lambda: False)
    start = MagicMock()
    monkeypatch.setattr(ob, "start_portal", start)
    assert ob.main() == 0
    start.assert_called_once()


def test_main_does_not_reenter_ap_when_provisioned(ob, monkeypatch, tmp_path):
    monkeypatch.setattr(ob, "wait_online", lambda *a, **k: None)
    monkeypatch.setattr(ob, "networked", lambda: False)
    monkeypatch.setattr(ob, "BOOT_CFG", str(tmp_path / "absent.txt"))
    monkeypatch.setattr(ob, "provisioned", lambda: True)
    start = MagicMock()
    monkeypatch.setattr(ob, "start_portal", start)
    assert ob.main() == 0
    start.assert_not_called()


# ---------------------------------------------------------------------------
# provision_from_file
# ---------------------------------------------------------------------------

def test_provision_moves_file_and_marks_on_success(ob, monkeypatch, tmp_path):
    cfg = tmp_path / "neurokairos-wifi.txt"
    cfg.write_text("country=US\nssid=Net\npassword=pw\n")
    calls = {}
    monkeypatch.setattr(ob, "set_country", lambda cc: calls.setdefault("cc", cc))
    monkeypatch.setattr(ob, "wifi_rescan", lambda: None)
    monkeypatch.setattr(ob, "wifi_connect", lambda s, p: calls.setdefault("conn", (s, p)) or True)
    monkeypatch.setattr(ob, "networked", lambda: True)
    monkeypatch.setattr(ob, "mark_provisioned", lambda: calls.setdefault("marked", True))
    monkeypatch.setattr(ob.time, "sleep", lambda *_: None)
    assert ob.provision_from_file(str(cfg)) is True
    assert calls["cc"] == "US"
    assert calls["conn"] == ("Net", "pw")
    assert calls["marked"] is True
    assert not cfg.exists()
    assert (tmp_path / "neurokairos-wifi.txt.applied").exists()


def test_provision_returns_false_without_ssid(ob, monkeypatch, tmp_path):
    cfg = tmp_path / "neurokairos-wifi.txt"
    cfg.write_text("country=US\n")
    assert ob.provision_from_file(str(cfg)) is False
    # file left in place for the user to fix
    assert cfg.exists()


def test_provision_no_mark_when_connect_fails(ob, monkeypatch, tmp_path):
    cfg = tmp_path / "neurokairos-wifi.txt"
    cfg.write_text("ssid=Net\npassword=pw\n")
    marked = MagicMock()
    monkeypatch.setattr(ob, "set_country", lambda cc: None)
    monkeypatch.setattr(ob, "wifi_rescan", lambda: None)
    monkeypatch.setattr(ob, "wifi_connect", lambda s, p: False)
    monkeypatch.setattr(ob, "networked", lambda: False)
    monkeypatch.setattr(ob, "mark_provisioned", marked)
    monkeypatch.setattr(ob.time, "sleep", lambda *_: None)
    assert ob.provision_from_file(str(cfg)) is False
    marked.assert_not_called()
    assert cfg.exists()  # not moved, so a retry/next boot can try again


# ---------------------------------------------------------------------------
# portal: parse_scan
# ---------------------------------------------------------------------------

def test_parse_scan_orders_by_signal_and_dedups(portal):
    out = "HomeNet:42\nHomeNet:90\nCafe:55\n:33\n"
    assert portal.parse_scan(out) == ["HomeNet", "Cafe"]


def test_parse_scan_drops_own_ap(portal):
    out = f"{portal.AP_SSID}:99\nReal:20\n"
    assert portal.parse_scan(out) == ["Real"]


def test_parse_scan_handles_nonint_signal(portal):
    out = "A:notanumber\nB:70\n"
    assert portal.parse_scan(out) == ["B", "A"]


# ---------------------------------------------------------------------------
# portal: connect_args
# ---------------------------------------------------------------------------

def test_connect_args_with_password(portal):
    assert portal.connect_args("Net", "pw") == [
        "nmcli", "device", "wifi", "connect", "Net", "password", "pw"]


def test_connect_args_open_network(portal):
    assert portal.connect_args("Open", "") == [
        "nmcli", "device", "wifi", "connect", "Open"]


# ---------------------------------------------------------------------------
# portal: render_page
# ---------------------------------------------------------------------------

def test_render_page_lists_ssids_and_escapes(portal):
    page = portal.render_page(["Home", "A&B<x>"])
    assert "<option value=\"Home\">Home</option>" in page
    assert "A&amp;B&lt;x&gt;" in page  # HTML-escaped
    assert "NeuroKairos Wi-Fi setup" in page


def test_render_page_shows_message(portal):
    assert "please choose" in portal.render_page([], "please choose").lower()
