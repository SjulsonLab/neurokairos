#!/usr/bin/env python3
"""NeuroKairos status agent.

Runs on *every* NeuroKairos Pi (sender/client and server). It is deliberately
tiny and low-priority so it can never preempt the real-time IRIG-H sender
(SCHED_FIFO 80): a normal-priority HTTP handler cannot interrupt the RT thread.

Endpoints (pure-stdlib http.server):
  GET  /health          -> {"ok": true}
  GET  /api/status      -> this Pi's status JSON (timing, LED, services, name)
  POST /api/control     -> perform a password-gated action (rename / restart /
                           reboot / shutdown / set-ntp)
  POST /api/set-auth    -> store the shared password hash pushed by a dashboard

The heavy dashboard/UI lives only on server Pis (nk_dashboard.py); this agent
just reports and executes. Timing parsing is reused from the co-installed
ntp_calibrator.py.
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import socket
import subprocess
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

AGENT_PORT = 8080

NAME_PATH = "/var/lib/neurokairos/name"
AUTH_PATH = "/var/lib/neurokairos/agent-auth"
CALIBRATOR_STATE_PATH = "/var/lib/ntp-calibrator/state.json"
SOURCES_FILE = "/etc/chrony/sources.d/neurokairos.sources"

# Services the web UI is allowed to restart (allowlist — never arbitrary units).
RESTARTABLE = {"chrony", "irig-sender"}
DEFAULT_WARN_MS = 1.0  # mirrors the sender's -w default


# ---------------------------------------------------------------------------
# Reuse chrony parsing from the co-installed ntp_calibrator.py
# ---------------------------------------------------------------------------

def _load_calibrator():
    """Load ntp_calibrator.py from the same directory (installed or repo)."""
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "ntp_calibrator.py")
    if not os.path.exists(path):
        return None
    spec = importlib.util.spec_from_file_location("ntp_calibrator", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# Small I/O helpers
# ---------------------------------------------------------------------------

def read_text(path: str) -> str:
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ""


def get_primary_ip() -> str:
    """Best-effort primary LAN IPv4 (no traffic actually sent)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.0.2.1", 9))  # TEST-NET-1, unroutable
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def get_role(hostname: str) -> str:
    return "server" if "server" in hostname.lower() else "client"


def systemctl_is_active(unit: str) -> bool:
    try:
        r = subprocess.run(["systemctl", "is-active", unit],
                           capture_output=True, text=True, timeout=5)
        return r.stdout.strip() == "active"
    except (OSError, subprocess.SubprocessError):
        return False


# ---------------------------------------------------------------------------
# Pure status assembly (unit-tested)
# ---------------------------------------------------------------------------

def quality_level(synced: bool, root_dispersion_s, warn_ms: float = DEFAULT_WARN_MS) -> str:
    """Coarse timing-quality bucket used to color a box.

    Mirrors the sender's LED logic: unsynced -> 'unsynced'; synced but
    dispersion >= warn threshold -> 'bad'; within threshold -> 'good', with a
    'marginal' band in the top half of the threshold.
    """
    if not synced:
        return "unsynced"
    if root_dispersion_s is None:
        return "unknown"
    disp_ms = root_dispersion_s * 1000.0
    if disp_ms >= warn_ms:
        return "bad"
    if disp_ms >= warn_ms / 2.0:
        return "marginal"
    return "good"


def build_status(*, name, hostname, ip, role, tracking, tier,
                 sources, diversity, services, calibrator, warn_ms=DEFAULT_WARN_MS):
    """Assemble the status dict from already-parsed inputs (no I/O here).

    The dashboard reconstructs each sender's IRIG-H output waveform from the
    clock, so no LED/pin state is reported here.
    """
    tracking = tracking or {}
    synced = bool(tracking.get("synchronized"))
    disp = tracking.get("root_dispersion_s")
    return {
        "name": name,
        "hostname": hostname,
        "ip": ip,
        "role": role,
        "timing": {
            "synchronized": synced,
            "stratum": tracking.get("stratum"),
            "reference": tracking.get("ref_id_name"),
            "reference_tier": tier,  # GNSS | LAN | WAN | NONE
            "rms_offset_s": tracking.get("rms_offset_s"),
            "root_delay_s": tracking.get("root_delay_s"),
            "root_dispersion_s": disp,
            "freq_ppm": tracking.get("freq_ppm"),
            "quality": quality_level(synced, disp, warn_ms),
        },
        "sources": {
            "count": len(sources or {}),
            "diversity_ok": (diversity or {}).get("is_adequate"),
            "reachable": (diversity or {}).get("reachable_count"),
        },
        "services": services,          # {"irig-sender": bool, "chrony": bool}
        "calibrator": calibrator,       # precision metrics or None
    }


def read_calibrator_metrics(path: str = None):
    raw = read_text(path or CALIBRATOR_STATE_PATH)
    if not raw:
        return None
    try:
        st = json.loads(raw)
    except json.JSONDecodeError:
        return None
    keys = ("precision_typical_ms", "precision_recent_ms",
            "precision_worst_case_ms", "calibration_ref_id", "calibration_complete")
    metrics = {k: st.get(k) for k in keys if k in st}
    return metrics or None


# ---------------------------------------------------------------------------
# Full gather (I/O) — glues parsing + files together
# ---------------------------------------------------------------------------

def gather_status(calib=None) -> dict:
    calib = calib or _load_calibrator()
    hostname = socket.gethostname()
    role = get_role(hostname)

    tracking, tier, sources, diversity = {}, "NONE", {}, {}
    if calib is not None:
        tracking = calib.parse_tracking(calib.run_chronyc_tracking())
        sources = calib.parse_sources(calib.run_chronyc_sources())
        tier = calib.detect_reference_tier(tracking, [])
        diversity = calib.check_source_diversity(sources)

    return build_status(
        name=read_text(NAME_PATH) or hostname,
        hostname=hostname,
        ip=get_primary_ip(),
        role=role,
        tracking=tracking,
        tier=tier,
        sources=sources,
        diversity=diversity,
        services={
            "irig-sender": systemctl_is_active("irig-sender"),
            "chrony": systemctl_is_active("chrony"),
        },
        calibrator=read_calibrator_metrics(),
    )


# ---------------------------------------------------------------------------
# Auth (shared password hash, pushed by the dashboard)
# ---------------------------------------------------------------------------

def hash_password(password: str) -> str:
    return hashlib.sha256(password.encode("utf-8")).hexdigest()


def load_auth_hash() -> str:
    return read_text(AUTH_PATH)


def store_auth_hash(hash_hex: str) -> None:
    os.makedirs(os.path.dirname(AUTH_PATH), exist_ok=True)
    tmp = AUTH_PATH + ".tmp"
    with open(tmp, "w") as f:
        f.write(hash_hex)
    os.rename(tmp, AUTH_PATH)


def check_auth(provided_hash: str) -> bool:
    stored = load_auth_hash()
    return bool(stored) and provided_hash == stored


# ---------------------------------------------------------------------------
# Control actions
# ---------------------------------------------------------------------------

def _run(cmd, timeout=15):
    subprocess.run(cmd, check=True, timeout=timeout)


def do_rename(new_name: str) -> dict:
    name = (new_name or "").strip()
    if not name:
        raise ValueError("empty name")
    os.makedirs(os.path.dirname(NAME_PATH), exist_ok=True)
    tmp = NAME_PATH + ".tmp"
    with open(tmp, "w") as f:
        f.write(name)
    os.rename(tmp, NAME_PATH)
    return {"name": name}


def do_restart(service: str) -> dict:
    if service not in RESTARTABLE:
        raise ValueError(f"service not allowed: {service}")
    _run(["systemctl", "restart", service])
    return {"restarted": service}


def do_reboot() -> dict:
    _run(["systemctl", "reboot"])
    return {"rebooting": True}


def do_shutdown() -> dict:
    _run(["systemctl", "poweroff"])
    return {"shutting_down": True}


def do_set_ntp(server: str) -> dict:
    """Set (or clear) the LAN NTP source without restarting chronyd.

    A non-empty server writes a preferred source line into the discovery
    sources file and reloads chrony (no step); empty clears it and falls back
    to the configured public servers.
    """
    os.makedirs(os.path.dirname(SOURCES_FILE), exist_ok=True)
    server = (server or "").strip()
    if server:
        content = f"# Set via NeuroKairos dashboard\nserver {server} iburst prefer\n"
    else:
        content = "# Cleared via NeuroKairos dashboard\n"
    tmp = SOURCES_FILE + ".tmp"
    with open(tmp, "w") as f:
        f.write(content)
    os.rename(tmp, SOURCES_FILE)
    subprocess.run(["chronyc", "reload", "sources"], check=True, timeout=10)
    return {"ntp_server": server or None}


# Dispatch table: action name -> (callable, list of required param keys)
_ACTIONS = {
    "rename": (lambda p: do_rename(p.get("name")), ),
    "restart": (lambda p: do_restart(p.get("service")), ),
    "reboot": (lambda p: do_reboot(), ),
    "shutdown": (lambda p: do_shutdown(), ),
    "set-ntp": (lambda p: do_set_ntp(p.get("server")), ),
}


def handle_control(payload: dict):
    """Validate + dispatch a control request. Returns (http_status, body)."""
    action = payload.get("action")
    if action not in _ACTIONS:
        return 400, {"error": f"unknown action: {action}"}
    if not check_auth(payload.get("auth", "")):
        return 403, {"error": "authentication required"}
    try:
        result = _ACTIONS[action][0](payload.get("params", {}))
        return 200, {"ok": True, "result": result}
    except ValueError as exc:
        return 400, {"error": str(exc)}
    except (OSError, subprocess.SubprocessError) as exc:
        return 500, {"error": f"action failed: {exc}"}


def handle_set_auth(payload: dict):
    """Store the dashboard-provided password hash.

    Allowed if no hash is set yet (first run), or if the caller proves knowledge
    of the current hash (password change).
    """
    new_hash = (payload.get("hash") or "").strip()
    if len(new_hash) != 64:
        return 400, {"error": "invalid hash"}
    existing = load_auth_hash()
    if existing and payload.get("current") != existing:
        return 403, {"error": "current auth required to change"}
    store_auth_hash(new_hash)
    return 200, {"ok": True}


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------

def make_server(host: str, port: int) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def _json(self, payload, status=200):
            body = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def _body(self) -> dict:
            n = int(self.headers.get("Content-Length", "0"))
            if not n:
                return {}
            try:
                return json.loads(self.rfile.read(n).decode("utf-8"))
            except json.JSONDecodeError:
                return {}

        def do_GET(self):  # noqa: N802
            path = urlparse(self.path).path
            if path == "/health":
                self._json({"ok": True})
            elif path == "/api/status":
                self._json(gather_status())
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):  # noqa: N802
            path = urlparse(self.path).path
            if path == "/api/control":
                code, body = handle_control(self._body())
                self._json(body, code)
            elif path == "/api/set-auth":
                code, body = handle_set_auth(self._body())
                self._json(body, code)
            else:
                self._json({"error": "not found"}, 404)

        def log_message(self, *args):  # silence
            return

    return ThreadingHTTPServer((host, port), Handler)


def main(argv=None):
    import argparse
    p = argparse.ArgumentParser(description="NeuroKairos status agent")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=AGENT_PORT)
    args = p.parse_args(argv)
    make_server(args.host, args.port).serve_forever()


if __name__ == "__main__":
    main()
