"""Guard tests for the dev-only dashboard simulator (tools/dashboard_sim.py).

Ensures the fake agents keep producing status in the real dashboard schema, so
the simulator stays a faithful stand-in as the real code evolves. The simulator
is not shipped in any image; these tests just keep it honest.
"""
import importlib.util
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
SIM_PATH = REPO_ROOT / "tools" / "dashboard_sim.py"


def _load():
    spec = importlib.util.spec_from_file_location("dashboard_sim", SIM_PATH)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


@pytest.fixture(scope="module")
def sim():
    return _load()


def test_profiles_have_a_server_and_clients(sim):
    roles = [p["role"] for p in sim._profiles()]
    assert "server" in roles and "client" in roles
    assert len(roles) >= 3


def test_fake_status_matches_real_schema(sim):
    prof = sim._profiles()[0]                     # the server profile
    st = sim.FakeAgent(prof, "127.0.0.1").status()
    # Same top-level + nested keys the real agent's build_status emits.
    assert set(st) >= {"name", "hostname", "ip", "role", "led", "timing",
                       "sources", "services", "calibrator"}
    assert set(st["timing"]) >= {"synchronized", "stratum", "reference",
                                 "reference_tier", "root_dispersion_s", "quality"}


def test_quality_reflects_profile(sim):
    profs = {p["name"]: p for p in sim._profiles()}
    good = sim.FakeAgent(profs["Lab GPS server"], "127.0.0.1").status()
    bad = sim.FakeAgent(profs["Rig Gamma"], "127.0.0.1").status()
    unsynced = sim.FakeAgent(profs["Rig Delta"], "127.0.0.1").status()
    assert good["timing"]["quality"] == "good"
    assert bad["timing"]["quality"] == "bad"
    assert unsynced["timing"]["quality"] == "unsynced"
    assert good["role"] == "server"


def test_control_requires_matching_auth(sim):
    agent = sim.FakeAgent(sim._profiles()[1], "127.0.0.1")
    # no auth set yet -> control rejected
    code, _ = agent.control({"action": "rename", "params": {"name": "X"}})
    assert code == 403
    # set auth, then a wrong hash is rejected, right hash accepted
    agent.set_auth({"hash": "a" * 64})
    code, _ = agent.control({"action": "rename", "auth": "b" * 64, "params": {"name": "X"}})
    assert code == 403
    code, _ = agent.control({"action": "rename", "auth": "a" * 64, "params": {"name": "New"}})
    assert code == 200 and agent.name == "New"


def test_reboot_marks_down(sim):
    agent = sim.FakeAgent(sim._profiles()[0], "127.0.0.1")
    agent.set_auth({"hash": "c" * 64})
    assert agent.down_until == 0.0
    agent.control({"action": "reboot", "auth": "c" * 64})
    assert agent.down_until > 0.0
