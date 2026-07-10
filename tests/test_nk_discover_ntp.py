"""Tests for the LAN NTP-server discovery script (raspberry_pi/scripts/nk_discover_ntp.py).

Follows the style of tests/test_ntp_calibrator.py: the script is loaded via
importlib (it is not an installed package), pure functions are exercised with
verbatim sample `avahi-browse` output, file I/O uses tmp_path, and the discovery
subprocess calls are mocked.
"""
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
SCRIPT_PATH = REPO_ROOT / "raspberry_pi" / "scripts" / "nk_discover_ntp.py"


def load_module():
    spec = importlib.util.spec_from_file_location("nk_discover_ntp", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def mod():
    return load_module()


# --- Sample avahi-browse -rpt output ---------------------------------------

# One resolved server. Note avahi escapes spaces as \032 in the name field.
SAMPLE_ONE = (
    "+;eth0;IPv4;NeuroKairos\\032NTP\\032on\\032nk-server;_neurokairos-ntp._udp;local\n"
    "=;eth0;IPv4;NeuroKairos\\032NTP\\032on\\032nk-server;_neurokairos-ntp._udp;local;"
    "nk-server.local;192.168.1.50;123;\n"
)

# Two resolved servers, one seen on two interfaces (same address → deduped).
SAMPLE_TWO = (
    "=;eth0;IPv4;NK\\032a;_neurokairos-ntp._udp;local;a.local;192.168.1.50;123;\n"
    "=;wlan0;IPv4;NK\\032a;_neurokairos-ntp._udp;local;a.local;192.168.1.50;123;\n"
    "=;eth0;IPv4;NK\\032b;_neurokairos-ntp._udp;local;b.local;10.0.0.7;123;\n"
)

# IPv6 resolved record — must be ignored (chrony sources file is IPv4 here).
SAMPLE_IPV6 = (
    "=;eth0;IPv6;NK\\032a;_neurokairos-ntp._udp;local;a.local;fe80::dead:beef;123;\n"
)

SAMPLE_EMPTY = ""

SAMPLE_MALFORMED = (
    "=;eth0;IPv4;too;few;fields\n"          # < 9 fields
    "=;eth0;IPv4;NK;_neurokairos-ntp._udp;local;a.local;not.an.ip.addr;123;\n"  # bad addr
    "garbage line without leading equals\n"
)


# --- parse_avahi_browse ----------------------------------------------------

def test_parse_single(mod):
    assert mod.parse_avahi_browse(SAMPLE_ONE) == {"192.168.1.50"}


def test_parse_multiple_and_dedupe(mod):
    assert mod.parse_avahi_browse(SAMPLE_TWO) == {"192.168.1.50", "10.0.0.7"}


def test_parse_ignores_ipv6(mod):
    assert mod.parse_avahi_browse(SAMPLE_IPV6) == set()


def test_parse_ignores_malformed(mod):
    assert mod.parse_avahi_browse(SAMPLE_MALFORMED) == set()


def test_parse_empty(mod):
    assert mod.parse_avahi_browse(SAMPLE_EMPTY) == set()


def test_parse_ignores_unresolved_plus_lines(mod):
    # A '+' (found but not yet resolved) line carries no address; only '=' counts.
    only_plus = "+;eth0;IPv4;NK;_neurokairos-ntp._udp;local\n"
    assert mod.parse_avahi_browse(only_plus) == set()


# --- _is_ipv4 --------------------------------------------------------------

@pytest.mark.parametrize("addr,ok", [
    ("192.168.1.50", True),
    ("10.0.0.7", True),
    ("255.255.255.255", True),
    ("0.0.0.0", True),
    ("256.1.1.1", False),
    ("1.2.3", False),
    ("1.2.3.4.5", False),
    ("a.b.c.d", False),
    ("192.168.1.", False),
    ("", False),
])
def test_is_ipv4(mod, addr, ok):
    assert mod._is_ipv4(addr) is ok


# --- render_sources --------------------------------------------------------

def test_render_sorted_and_prefer(mod):
    out = mod.render_sources({"192.168.1.50", "10.0.0.7"})
    lines = [ln for ln in out.splitlines() if not ln.startswith("#")]
    assert lines == [
        "server 10.0.0.7 iburst prefer",
        "server 192.168.1.50 iburst prefer",
    ]


def test_render_deterministic(mod):
    a = mod.render_sources(["10.0.0.7", "192.168.1.50"])
    b = mod.render_sources(["192.168.1.50", "10.0.0.7"])
    assert a == b


def test_render_empty_has_header_only(mod):
    out = mod.render_sources(set())
    assert out.startswith("#")
    assert "server " not in out


def test_render_ends_with_newline(mod):
    out = mod.render_sources({"192.168.1.50"})
    assert out.endswith("\n")


# --- discover_and_update ---------------------------------------------------

def _sources(tmp_path):
    return str(tmp_path / "sources.d" / "neurokairos.sources")


def test_update_writes_and_reloads_when_new(mod, tmp_path):
    path = _sources(tmp_path)
    with patch.object(mod, "run_avahi_browse", return_value=SAMPLE_ONE), \
         patch.object(mod, "reload_chrony") as reload:
        changed = mod.discover_and_update(path)
    assert changed is True
    assert reload.call_count == 1
    assert "server 192.168.1.50 iburst prefer" in Path(path).read_text()


def test_update_noop_when_unchanged(mod, tmp_path):
    path = _sources(tmp_path)
    with patch.object(mod, "run_avahi_browse", return_value=SAMPLE_ONE), \
         patch.object(mod, "reload_chrony") as reload:
        assert mod.discover_and_update(path) is True   # first write
        assert mod.discover_and_update(path) is False  # identical → no reload
    assert reload.call_count == 1


def test_update_empty_leaves_file_untouched(mod, tmp_path):
    path = _sources(tmp_path)
    # Seed a good file first.
    with patch.object(mod, "run_avahi_browse", return_value=SAMPLE_ONE), \
         patch.object(mod, "reload_chrony"):
        mod.discover_and_update(path)
    before = Path(path).read_text()
    # A subsequent empty browse must NOT wipe the good config or reload.
    with patch.object(mod, "run_avahi_browse", return_value=SAMPLE_EMPTY), \
         patch.object(mod, "reload_chrony") as reload:
        assert mod.discover_and_update(path) is False
    assert Path(path).read_text() == before
    assert reload.call_count == 0


def test_update_rewrites_on_changed_set(mod, tmp_path):
    path = _sources(tmp_path)
    with patch.object(mod, "run_avahi_browse", return_value=SAMPLE_ONE), \
         patch.object(mod, "reload_chrony"):
        mod.discover_and_update(path)
    with patch.object(mod, "run_avahi_browse", return_value=SAMPLE_TWO), \
         patch.object(mod, "reload_chrony") as reload:
        assert mod.discover_and_update(path) is True
        assert reload.call_count == 1
    text = Path(path).read_text()
    assert "10.0.0.7" in text and "192.168.1.50" in text


def test_update_writes_atomically_no_tmp_left(mod, tmp_path):
    path = _sources(tmp_path)
    with patch.object(mod, "run_avahi_browse", return_value=SAMPLE_ONE), \
         patch.object(mod, "reload_chrony"):
        mod.discover_and_update(path)
    assert list((tmp_path / "sources.d").glob("*.tmp")) == []
