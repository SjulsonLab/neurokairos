#!/usr/bin/env python3
"""NeuroKairos dashboard.

Runs on SERVER Pis only. Serves a single white-page web UI that shows every
NeuroKairos Pi on the LAN as a rectangle, and proxies password-gated control
actions to each Pi's status agent (nk_status_agent.py, port 8080).

Discovery reuses the existing mDNS layer: every Pi advertises
`_neurokairos-status._tcp`. The dashboard browses that, polls each agent's
`/api/status` server-side (so the browser makes no cross-origin calls), and
serves the aggregate at `/api/all`.

Auth (trusted-LAN model): viewing is open; write-actions require a password set
in the UI on first run. The dashboard stores the password hash and pushes it to
every agent (their `/api/set-auth`) so agents can validate forwarded commands.

Pure-stdlib (http.server + urllib), matching the rest of the repo.
"""
from __future__ import annotations

import hashlib
import json
import os
import secrets
import subprocess
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

DASHBOARD_PORT = 80
AGENT_PORT = 8080
STATUS_SERVICE = "_neurokairos-status._tcp"
AUTH_PATH = "/var/lib/neurokairos/dashboard-auth.json"
TOKEN_TTL_S = 3600
FETCH_TIMEOUT_S = 3

# Where the UI's index.html is installed; falls back to the repo layout so the
# dashboard also runs straight from a source checkout.
_INSTALLED_WEB = "/usr/local/share/neurokairos-dashboard"
_REPO_WEB = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "dashboard", "web")


def web_dir() -> str:
    return _INSTALLED_WEB if os.path.isdir(_INSTALLED_WEB) else os.path.normpath(_REPO_WEB)


# ---------------------------------------------------------------------------
# mDNS discovery of status agents
# ---------------------------------------------------------------------------

def _is_ipv4(s: str) -> bool:
    parts = s.split(".")
    return len(parts) == 4 and all(p.isdigit() and 0 <= int(p) <= 255 for p in parts)


def parse_status_services(text: str):
    """Parse `avahi-browse -rpt _neurokairos-status._tcp` into [(ip, port)].

    Resolved records start with '=': fields are
    =;iface;proto;name;type;domain;host;ADDRESS;PORT;txt. IPv4 only; deduped.
    """
    found = set()
    for line in text.splitlines():
        if not line.startswith("="):
            continue
        f = line.split(";")
        if len(f) < 9 or f[2] != "IPv4":
            continue
        ip, port = f[7].strip(), f[8].strip()
        if _is_ipv4(ip) and port.isdigit():
            found.add((ip, int(port)))
    return sorted(found)


def discover_agents():
    try:
        r = subprocess.run(["avahi-browse", "-rpt", STATUS_SERVICE],
                           capture_output=True, text=True, timeout=30)
        return parse_status_services(r.stdout)
    except (OSError, subprocess.SubprocessError):
        return []


def fetch_status(ip: str, port: int, timeout: float = FETCH_TIMEOUT_S):
    url = f"http://{ip}:{port}/api/status"
    endpoint = f"{ip}:{port}"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        data.update({"online": True, "ip": ip, "port": port, "endpoint": endpoint})
        return data
    except (urllib.error.URLError, OSError, json.JSONDecodeError, ValueError):
        return {"ip": ip, "port": port, "endpoint": endpoint, "online": False}


def collect(agents, fetcher=fetch_status):
    """Fetch status from each (ip, port). fetcher injectable for tests."""
    return [fetcher(ip, port) for ip, port in agents]


def build_all(statuses):
    """Dedupe by reachable endpoint (ip:port), sort (servers first, then name), wrap.

    Identity is the ip:port endpoint, NOT hostname — every sender image ships the
    same hostname (neurokairos-sender), so hostname would wrongly collapse
    distinct client Pis into one. ip:port is unique per reachable agent.
    """
    by_key = {}
    for s in statuses:
        key = s.get("endpoint") or s.get("ip") or s.get("hostname")
        # Prefer an online record over an offline duplicate.
        if key not in by_key or (s.get("online") and not by_key[key].get("online")):
            by_key[key] = s
    pis = sorted(
        by_key.values(),
        key=lambda s: (0 if s.get("role") == "server" else 1,
                       (s.get("name") or s.get("hostname") or s.get("ip") or "")),
    )
    return {"pis": pis}


# ---------------------------------------------------------------------------
# Auth: password store + tokens
# ---------------------------------------------------------------------------

def hash_password(password: str) -> str:
    return hashlib.sha256(password.encode("utf-8")).hexdigest()


def load_password_hash() -> str:
    try:
        with open(AUTH_PATH) as f:
            return json.load(f).get("hash", "")
    except (OSError, json.JSONDecodeError):
        return ""


def store_password_hash(hash_hex: str) -> None:
    os.makedirs(os.path.dirname(AUTH_PATH), exist_ok=True)
    tmp = AUTH_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"hash": hash_hex}, f)
    os.rename(tmp, AUTH_PATH)


_TOKENS = {}  # token -> expiry epoch


def create_token(now: float = None, ttl: int = TOKEN_TTL_S) -> str:
    now = time.time() if now is None else now
    tok = secrets.token_hex(16)
    _TOKENS[tok] = now + ttl
    return tok


def check_token(token: str, now: float = None) -> bool:
    now = time.time() if now is None else now
    exp = _TOKENS.get(token)
    if exp is None:
        return False
    if exp < now:
        _TOKENS.pop(token, None)
        return False
    return True


# ---------------------------------------------------------------------------
# Pulse-cadence monitoring: onsets must be 1.000 s apart
# ---------------------------------------------------------------------------

ALERTS_LOG = "/var/lib/neurokairos/pulse-anomalies.log"
INTERVAL_MIN = 0.9
INTERVAL_MAX = 1.1
STOPPED_THRESHOLD_S = 3.0

_ONSET_STATE = {}   # endpoint "ip:port" -> last onset epoch (float)
_SENDER_META = {}   # endpoint -> {"name": str, "sending": bool}
_ALERTS = {}        # id -> alert dict


def reset_monitor_state():
    _ONSET_STATE.clear()
    _SENDER_META.clear()
    _ALERTS.clear()


def _iso(epoch: float) -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime(epoch))


def _log_alert(alert: dict) -> None:
    try:
        os.makedirs(os.path.dirname(ALERTS_LOG), exist_ok=True)
        with open(ALERTS_LOG, "a") as f:
            f.write(json.dumps(alert) + "\n")
    except OSError:
        pass


def _raise_alert(endpoint, name, kind, onset_epoch, now, interval=None):
    """Create+log an alert if this exact one isn't already active."""
    aid = f"{endpoint}:{kind}:{onset_epoch}"
    if aid in _ALERTS:
        return None
    alert = {
        "id": aid, "endpoint": endpoint, "sender_name": name, "kind": kind,
        "interval_s": None if interval is None else round(interval, 3),
        "onset_iso": _iso(onset_epoch), "created_iso": _iso(now),
        "created_epoch": now, "dismissed": False,
    }
    _ALERTS[aid] = alert
    _log_alert(alert)
    return alert


def record_poll_meta(pis):
    """Cache sender name + sending state from an /api/all poll, keyed by endpoint."""
    for p in pis:
        endpoint = p.get("endpoint") or p.get("ip")
        if not endpoint:
            continue
        _SENDER_META[endpoint] = {
            "name": p.get("name") or p.get("hostname") or endpoint,
            "sending": bool((p.get("services") or {}).get("irig-sender")),
        }


def ingest_onset(endpoint, onset, now):
    """Record an onset; raise an interval alert if spacing is off. Returns alert|None."""
    name = (_SENDER_META.get(endpoint) or {}).get("name", endpoint)
    prev = _ONSET_STATE.get(endpoint)
    _ONSET_STATE[endpoint] = onset
    # Pulses resumed -> clear any active 'stopped' alert for this sender.
    for a in _ALERTS.values():
        if a["endpoint"] == endpoint and a["kind"] == "stopped" and not a["dismissed"]:
            a["dismissed"] = True
    if prev is None:
        return None
    interval = onset - prev
    if interval < INTERVAL_MIN or interval > INTERVAL_MAX:
        return _raise_alert(endpoint, name, "interval", onset, now, interval=interval)
    return None


def watchdog_once(now):
    """Raise a 'stopped' alert for senders that were pulsing but went silent."""
    raised = []
    for endpoint, last in _ONSET_STATE.items():
        meta = _SENDER_META.get(endpoint)
        if not meta or not meta["sending"]:
            continue
        if now - last > STOPPED_THRESHOLD_S:
            a = _raise_alert(endpoint, meta["name"], "stopped", last, now)
            if a:
                raised.append(a)
    return raised


def active_alerts():
    return sorted((a for a in _ALERTS.values() if not a["dismissed"]),
                  key=lambda a: a["created_epoch"], reverse=True)


def dismiss_alert(alert_id, token, now=None):
    if not check_token(token, now):
        return 401, {"error": "not authenticated"}
    a = _ALERTS.get(alert_id)
    if a:
        a["dismissed"] = True
    return 200, {"ok": True}


# ---------------------------------------------------------------------------
# Request handlers (pure-ish: I/O injected where practical)
# ---------------------------------------------------------------------------

def handle_auth_status():
    return 200, {"configured": bool(load_password_hash())}


def push_auth_to_agents(hash_hex: str, current: str = "", agents=None) -> None:
    """Push the password hash to every discovered agent's /api/set-auth."""
    agents = discover_agents() if agents is None else agents
    payload = json.dumps({"hash": hash_hex, "current": current}).encode("utf-8")
    for ip, port in agents:
        try:
            req = urllib.request.Request(
                f"http://{ip}:{port}/api/set-auth", data=payload,
                headers={"Content-Type": "application/json"}, method="POST")
            urllib.request.urlopen(req, timeout=FETCH_TIMEOUT_S)
        except (urllib.error.URLError, OSError):
            continue  # best-effort; a missed agent re-syncs on next change


def handle_set_password(payload, pusher=push_auth_to_agents):
    """First run sets the password; later changes require the current password."""
    new = payload.get("password") or ""
    if len(new) < 4:
        return 400, {"error": "password too short (min 4 chars)"}
    existing = load_password_hash()
    current_hash = ""
    if existing:
        current_hash = hash_password(payload.get("current", ""))
        if current_hash != existing:
            return 403, {"error": "current password required to change"}
    new_hash = hash_password(new)
    store_password_hash(new_hash)
    pusher(new_hash, current=current_hash)
    return 200, {"ok": True}


def handle_login(payload, now: float = None):
    stored = load_password_hash()
    if not stored:
        return 409, {"error": "no password set"}
    if hash_password(payload.get("password", "")) != stored:
        return 403, {"error": "invalid password"}
    return 200, {"token": create_token(now)}


def forward_control(target_ip, port, action, params, auth_hash,
                    timeout: float = FETCH_TIMEOUT_S):
    body = json.dumps({"action": action, "params": params, "auth": auth_hash}).encode("utf-8")
    req = urllib.request.Request(
        f"http://{target_ip}:{port}/api/control", data=body,
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def handle_control(payload, forwarder=forward_control, now: float = None):
    if not check_token(payload.get("token", ""), now):
        return 401, {"error": "not authenticated"}
    target = payload.get("target")
    action = payload.get("action")
    if not target or not action:
        return 400, {"error": "target and action required"}
    auth_hash = load_password_hash()
    try:
        result = forwarder(target, payload.get("port", AGENT_PORT), action,
                          payload.get("params", {}), auth_hash)
        return 200, {"ok": True, "result": result}
    except (urllib.error.URLError, OSError, json.JSONDecodeError, ValueError) as exc:
        return 502, {"error": f"agent unreachable: {exc}"}


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------

def handle_onset(payload, now, src_ip):
    onset = payload.get("onset")
    if not isinstance(onset, (int, float)):
        return 400, {"error": "numeric onset required"}
    # Real senders post just {"onset": ...} — identify them by the request's
    # source IP + the standard agent port. The simulator overrides ip/port so
    # its many fake senders (all on 127.0.0.1) stay distinct.
    ip = payload.get("ip") or src_ip
    endpoint = f"{ip}:{payload.get('port', AGENT_PORT)}"
    ingest_onset(endpoint, float(onset), now)
    return 200, {"ok": True}


def build_all_response():
    result = build_all(collect(discover_agents()))
    record_poll_meta(result["pis"])          # cache name/sending for alerts + watchdog
    result["alerts"] = active_alerts()
    return result


def read_index() -> str:
    path = os.path.join(web_dir(), "index.html")
    try:
        with open(path, encoding="utf-8") as f:
            return f.read()
    except OSError:
        return "<html><body><h1>NeuroKairos</h1><p>UI asset missing.</p></body></html>"


def make_server(host: str, port: int) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def _json(self, payload, status=200):
            body = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _html(self, body):
            enc = body.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(enc)))
            self.end_headers()
            self.wfile.write(enc)

        def _body(self):
            n = int(self.headers.get("Content-Length", "0"))
            if not n:
                return {}
            try:
                return json.loads(self.rfile.read(n).decode("utf-8"))
            except json.JSONDecodeError:
                return {}

        def do_GET(self):  # noqa: N802
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                self._html(read_index())
            elif path == "/health":
                self._json({"ok": True})
            elif path == "/api/auth-status":
                _, body = handle_auth_status()
                self._json(body)
            elif path == "/api/all":
                self._json(build_all_response())
            elif path == "/api/alerts":
                self._json({"alerts": active_alerts()})
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):  # noqa: N802
            path = urlparse(self.path).path
            body = self._body()
            if path == "/api/set-password":
                code, resp = handle_set_password(body)
            elif path == "/api/login":
                code, resp = handle_login(body)
            elif path == "/api/control":
                code, resp = handle_control(body)
            elif path == "/api/onset":
                code, resp = handle_onset(body, time.time(), self.client_address[0])
            elif path == "/api/alerts/dismiss":
                code, resp = dismiss_alert(body.get("id"), body.get("token"), time.time())
            else:
                code, resp = 404, {"error": "not found"}
            self._json(resp, code)

        def log_message(self, *args):
            return

    return ThreadingHTTPServer((host, port), Handler)


def main(argv=None):
    import argparse
    p = argparse.ArgumentParser(description="NeuroKairos dashboard (server)")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=DASHBOARD_PORT)
    args = p.parse_args(argv)

    def _watchdog():
        while True:
            watchdog_once(time.time())
            time.sleep(1)
    threading.Thread(target=_watchdog, daemon=True).start()

    make_server(args.host, args.port).serve_forever()


if __name__ == "__main__":
    main()
