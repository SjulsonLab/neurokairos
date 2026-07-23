#!/usr/bin/env python3
"""Local simulation harness for the NeuroKairos dashboard.

DEV/TEST ONLY. This is not referenced by any image stage, so it never ships in
a released Pi image — it exists purely so you can see and click the dashboard on
your own machine without any hardware, chrony, or mDNS.

What it does:
  * spins up several *fake* status agents (a mix of servers and clients with
    varied, gently-jittering timing) on 127.0.0.1 high ports;
  * runs the REAL dashboard (raspberry_pi/scripts/nk_dashboard.py) on
    127.0.0.1, with its mDNS discovery monkeypatched to the fake agents;
  * everything binds to loopback only — nothing is broadcast to the network.

Because it reuses the real dashboard UI/aggregation and the real
nk_status_agent.build_status(), what you see matches production.

Usage:
    python3 tools/dashboard_sim.py          # then open the printed URL
    python3 tools/dashboard_sim.py --port 9000

Ctrl-C to stop.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import random
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = REPO_ROOT / "raspberry_pi" / "scripts"


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


AGENT = _load("nk_status_agent", SCRIPTS / "nk_status_agent.py")
DASH = _load("nk_dashboard", SCRIPTS / "nk_dashboard.py")


# ---------------------------------------------------------------------------
# Fake Pi profiles — varied so the grid wraps and shows every quality/LED state
# ---------------------------------------------------------------------------

def _profiles():
    # 1 server + 7 senders, with varied quality / stratum / reference tier.
    return [
        dict(hostname="neurokairos-server", name="Lab GPS server", role="server",
             tier="GNSS", ref="PPS", stratum=1, synced=True,
             disp=0.00005, rms=0.00003, precision=0.06),
        dict(hostname="neurokairos-sender", name="Rig Alpha", role="client",
             tier="LAN", ref="neurokairos-server", stratum=2, synced=True,
             disp=0.00020, rms=0.00008, precision=0.15),
        dict(hostname="neurokairos-sender", name="Rig Bravo", role="client",
             tier="LAN", ref="neurokairos-server", stratum=2, synced=True,
             disp=0.00018, rms=0.00007, precision=0.13),
        dict(hostname="neurokairos-sender", name="Rig Charlie", role="client",
             tier="WAN", ref="time.cloudflare.com", stratum=3, synced=True,
             disp=0.00060, rms=0.00030, precision=0.42),
        dict(hostname="neurokairos-sender", name="Rig Delta", role="client",
             tier="WAN", ref="time.google.com", stratum=3, synced=True,
             disp=0.00072, rms=0.00035, precision=0.50),
        dict(hostname="neurokairos-sender", name="Rig Echo", role="client",
             tier="WAN", ref="time.apple.com", stratum=3, synced=True,
             disp=0.00150, rms=0.00080, precision=1.10),
        dict(hostname="neurokairos-sender", name="Rig Foxtrot", role="client",
             tier="NONE", ref="", stratum=0, synced=False,
             disp=None, rms=None, precision=None),
        dict(hostname="neurokairos-sender", name="Rig Golf", role="client",
             tier="LAN", ref="neurokairos-server", stratum=2, synced=True,
             disp=0.00022, rms=0.00009, precision=0.16),
    ]


class FakeAgent:
    """Holds mutable state for one simulated Pi and renders real-schema status."""

    def __init__(self, profile, ip):
        self.p = dict(profile)
        self.ip = ip
        self.name = profile["name"]
        self.ntp_override = None
        self.auth_hash = None       # set via /api/set-auth (pushed by dashboard)
        self.down_until = 0.0       # simulated reboot/shutdown window

    def _jitter(self, base, frac=0.25):
        if base is None:
            return None
        return max(0.0, base * (1.0 + random.uniform(-frac, frac)))

    def status(self):
        p = self.p
        tracking = {
            "ref_id_name": self.ntp_override or p["ref"],
            "stratum": p["stratum"],
            "synchronized": p["synced"],
            "rms_offset_s": self._jitter(p["rms"]),
            "root_delay_s": self._jitter(0.0008) if p["synced"] else None,
            "root_dispersion_s": self._jitter(p["disp"]),
            "freq_ppm": round(random.uniform(-20, 20), 2) if p["synced"] else None,
            "update_interval_s": 64.0,
        }
        sources = {f"src{i}": {"state": "*" if i == 0 else "+", "stratum": p["stratum"],
                               "reach": 377} for i in range(4 if p["synced"] else 1)}
        diversity = {"is_adequate": p["synced"], "reachable_count": len(sources)}
        calibrator = ({"precision_typical_ms": p["precision"]}
                      if p["precision"] is not None else None)
        return AGENT.build_status(
            name=self.name, hostname=p["hostname"], ip=self.ip, role=p["role"],
            tracking=tracking, tier=p["tier"], sources=sources, diversity=diversity,
            services={"irig-sender": True, "chrony": True}, calibrator=calibrator)

    def set_auth(self, payload):
        h = (payload.get("hash") or "").strip()
        if len(h) != 64:
            return 400, {"error": "invalid hash"}
        if self.auth_hash and payload.get("current") != self.auth_hash:
            return 403, {"error": "current auth required"}
        self.auth_hash = h
        return 200, {"ok": True}

    def control(self, payload):
        if not self.auth_hash or payload.get("auth") != self.auth_hash:
            return 403, {"error": "authentication required"}
        action = payload.get("action")
        params = payload.get("params", {})
        if action == "rename":
            self.name = (params.get("name") or self.name).strip()
        elif action == "restart":
            pass  # simulated no-op
        elif action in ("reboot", "shutdown"):
            self.down_until = time.time() + (8 if action == "reboot" else 3600)
        elif action == "set-ntp":
            self.ntp_override = params.get("server") or None
            self.p["tier"] = "LAN" if self.ntp_override else self.p["tier"]
        else:
            return 400, {"error": f"unknown action: {action}"}
        print(f"[sim] {self.name}: {action} {params or ''}".rstrip())
        return 200, {"ok": True, "result": {"action": action}}


def _agent_server(agent: FakeAgent, host: str, port: int) -> ThreadingHTTPServer:
    class H(BaseHTTPRequestHandler):
        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _read(self):
            n = int(self.headers.get("Content-Length", "0"))
            try:
                return json.loads(self.rfile.read(n).decode()) if n else {}
            except json.JSONDecodeError:
                return {}

        def do_GET(self):
            if self.path == "/api/status":
                if time.time() < agent.down_until:
                    self.send_error(503)   # simulated reboot/shutdown -> "offline"
                    return
                self._json(agent.status())
            elif self.path == "/health":
                self._json({"ok": True})
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            body = self._read()
            if self.path == "/api/set-auth":
                code, resp = agent.set_auth(body)
                self._json(resp, code)
            elif self.path == "/api/control":
                code, resp = agent.control(body)
                self._json(resp, code)
            else:
                self._json({"error": "not found"}, 404)

        def log_message(self, *a):
            return

    return ThreadingHTTPServer((host, port), H)


def build_agents(base_port=8101):
    agents, servers, listing = [], [], []
    for i, prof in enumerate(_profiles()):
        port = base_port + i
        ip = f"127.0.0.1"
        agent = FakeAgent(prof, ip)
        srv = _agent_server(agent, "127.0.0.1", port)
        agents.append((agent, port))
        servers.append(srv)
        listing.append(("127.0.0.1", port))
    return agents, servers, listing


def main(argv=None):
    p = argparse.ArgumentParser(description="Local NeuroKairos dashboard simulator")
    p.add_argument("--port", type=int, default=8090, help="dashboard port (default 8090)")
    args = p.parse_args(argv)

    agents, agent_servers, listing = build_agents()

    # Point the REAL dashboard at our fake agents instead of mDNS, and keep its
    # auth file in a temp location (no root needed). Start fresh each run.
    DASH.discover_agents = lambda: list(listing)
    auth_file = os.path.join(tempfile.gettempdir(), "nk_sim_dashboard_auth.json")
    if os.path.exists(auth_file):
        os.remove(auth_file)
    DASH.AUTH_PATH = auth_file

    threads = []
    for srv in agent_servers:
        t = threading.Thread(target=srv.serve_forever, daemon=True)
        t.start()
        threads.append((srv, t))

    dash = DASH.make_server("127.0.0.1", args.port)
    dt = threading.Thread(target=dash.serve_forever, daemon=True)
    dt.start()

    url = f"http://127.0.0.1:{args.port}"
    print("=" * 60)
    print(" NeuroKairos dashboard simulator (local only — no network)")
    print(f"   {len(agents)} fake Pis on 127.0.0.1:{listing[0][1]}-{listing[-1][1]}")
    print(f"   Open:  {url}")
    print("   First visit sets a dashboard password; then try renaming,")
    print("   restarting, rebooting (a box goes 'offline' ~8s), set-NTP.")
    print("   Ctrl-C to stop.")
    print("=" * 60)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[sim] shutting down")
    finally:
        dash.shutdown()
        for srv, _ in threads:
            srv.shutdown()


if __name__ == "__main__":
    main()
