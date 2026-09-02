#!/usr/bin/env python3
"""NeuroKairos Wi-Fi setup hotspot / captive portal (sender image).

Started by nk_wifi_onboard when a fresh unit has no ethernet and no Wi-Fi
config. Brings up a `NeuroKairos-Setup` access point (NetworkManager's built-in
hotspot, which also serves DHCP), and serves a page to pick a network + enter the
password + regulatory country. On submit it replies first (so the phone sees the
result), then tears the AP down, sets the country, joins the chosen network, and
writes the provisioned marker so the AP never comes back on its own.

Single-radio caveat: AP and client can't run at once, so joining drops the setup
hotspot — the page says so. Pure-stdlib http.server (mirrors nk_status_agent.py).
"""
from __future__ import annotations

import html
import os
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs

AP_SSID = "NeuroKairos-Setup"
AP_PASSWORD = "neurokairos"
AP_CON = "nk-setup"
MARKER = "/var/lib/neurokairos/wifi-provisioned"
PORTAL_PORT = 80


def _run(args, timeout=30):
    return subprocess.run(args, capture_output=True, text=True, timeout=timeout)


# ---------------------------------------------------------------------------
# Pure helpers (unit-tested)
# ---------------------------------------------------------------------------

def parse_scan(nmcli_out: str):
    """Parse `nmcli -t -f SSID,SIGNAL device wifi list` into SSIDs, strongest
    first, deduped, blanks/our own AP dropped."""
    best = {}
    for line in nmcli_out.splitlines():
        ssid, _, sig = line.partition(":")
        ssid = ssid.strip()
        if not ssid or ssid == AP_SSID:
            continue
        try:
            sig = int(sig)
        except ValueError:
            sig = 0
        best[ssid] = max(best.get(ssid, -1), sig)
    return [s for s, _ in sorted(best.items(), key=lambda kv: -kv[1])]


def connect_args(ssid: str, password: str):
    args = ["nmcli", "device", "wifi", "connect", ssid]
    if password:
        args += ["password", password]
    return args


def render_page(ssids, message=""):
    opts = "".join(f'<option value="{html.escape(s)}">{html.escape(s)}</option>'
                   for s in ssids)
    msg = f'<p class="msg">{html.escape(message)}</p>' if message else ""
    return f"""<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>NeuroKairos Wi-Fi setup</title>
<style>body{{font:16px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;
 max-width:460px;margin:24px auto;padding:0 16px;color:#111827}}
h1{{font-size:20px}} label{{display:block;margin:14px 0 4px;font-weight:600}}
input,select{{width:100%;padding:9px;border:1px solid #d1d5db;border-radius:8px;font:inherit}}
button{{margin-top:18px;width:100%;padding:11px;border:0;border-radius:8px;background:#2563eb;
 color:#fff;font:inherit;font-weight:600}} .msg{{background:#eff6ff;border:1px solid #bfdbfe;
 padding:10px 12px;border-radius:8px}} .sub{{color:#6b7280;font-size:13px}}</style></head><body>
<h1>NeuroKairos Wi-Fi setup</h1>
<p class="sub">Choose your network so the sender can reach the internet. When it
connects, this hotspot disappears and the device joins your Wi-Fi.</p>
{msg}
<form method="POST" action="/connect">
 <label>Network</label>
 <select name="ssid" required>{opts}<option value="__other__">Other (type below)</option></select>
 <label>…or enter SSID</label><input name="ssid_other" placeholder="Hidden/other network">
 <label>Password</label><input name="password" type="password" placeholder="Wi-Fi password">
 <label>Wi-Fi country (2-letter, e.g. US, GB, DE)</label>
 <input name="country" maxlength="2" placeholder="US" required>
 <button type="submit">Connect</button>
</form></body></html>"""


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def networked() -> bool:
    """True once we've joined a real network (any non-loopback dev 'connected')."""
    try:
        r = _run(["nmcli", "-t", "-f", "DEVICE,STATE", "device", "status"], timeout=10)
    except (OSError, subprocess.SubprocessError):
        return False
    for line in r.stdout.splitlines():
        dev, _, state = line.partition(":")
        if dev and dev != "lo" and state.strip() == "connected":
            return True
    return False


def scan_networks():
    try:
        r = _run(["nmcli", "-t", "-f", "SSID,SIGNAL", "device", "wifi", "list",
                  "--rescan", "yes"], timeout=25)
        return parse_scan(r.stdout)
    except (OSError, subprocess.SubprocessError):
        return []


def start_ap():
    for cmd in (["iw", "reg", "set", "00"], ["rfkill", "unblock", "wifi"],
                ["nmcli", "radio", "wifi", "on"]):
        try:
            _run(cmd, timeout=10)
        except (OSError, subprocess.SubprocessError):
            pass
    try:
        _run(["nmcli", "device", "wifi", "hotspot", "con-name", AP_CON,
              "ssid", AP_SSID, "password", AP_PASSWORD], timeout=30)
    except (OSError, subprocess.SubprocessError) as exc:
        print(f"nk-wifi-portal: could not start AP: {exc}", flush=True)


def stop_ap():
    for cmd in (["nmcli", "connection", "down", AP_CON],
                ["nmcli", "connection", "delete", AP_CON]):
        try:
            _run(cmd, timeout=15)
        except (OSError, subprocess.SubprocessError):
            pass


def do_connect(ssid, password, country):
    """Tear down the AP and join the chosen network (runs off the request thread).

    On success: write the provisioned marker and exit(0) so systemd's
    ExecStopPost restores the status agent (and the AP stays down). On failure:
    bring the setup AP back so the user can try again with a corrected password.
    """
    time.sleep(1)                       # let the HTTP reply flush before the AP drops
    if country:
        for cmd in (["raspi-config", "nonint", "do_wifi_country", country],
                    ["iw", "reg", "set", country]):
            try:
                _run(cmd, timeout=15)
            except (OSError, subprocess.SubprocessError):
                pass
        time.sleep(3)
    stop_ap()
    try:
        _run(["rfkill", "unblock", "wifi"], timeout=10)
        _run(["nmcli", "device", "wifi", "rescan"], timeout=20)
    except (OSError, subprocess.SubprocessError):
        pass
    time.sleep(4)
    try:
        _run(["nmcli", "--wait", "30"] + connect_args(ssid, password)[1:], timeout=45)
    except (OSError, subprocess.SubprocessError):
        pass
    time.sleep(3)
    if networked():
        try:
            os.makedirs("/var/lib/neurokairos", exist_ok=True)
            open(MARKER, "w").close()
        except OSError:
            pass
        print(f"nk-wifi-portal: joined '{ssid}', provisioned", flush=True)
        os._exit(0)     # clean exit -> ExecStopPost restarts the status agent
    print(f"nk-wifi-portal: could not join '{ssid}'; reopening setup AP", flush=True)
    start_ap()


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------

def make_server(host="0.0.0.0", port=PORTAL_PORT):
    class Handler(BaseHTTPRequestHandler):
        def _html(self, body, status=200):
            enc = body.encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(enc)))
            self.end_headers()
            self.wfile.write(enc)

        def do_GET(self):  # noqa: N802
            # Captive-portal friendliness: serve the setup page for any path.
            self._html(render_page(scan_networks()))

        def do_POST(self):  # noqa: N802
            n = int(self.headers.get("Content-Length", "0"))
            form = parse_qs(self.rfile.read(n).decode("utf-8")) if n else {}
            ssid = (form.get("ssid_other", [""])[0].strip()
                    or form.get("ssid", [""])[0].strip())
            password = form.get("password", [""])[0]
            country = form.get("country", [""])[0].strip().upper()
            if not ssid:
                self._html(render_page(scan_networks(), "Please choose a network."))
                return
            # Reply BEFORE switching networks (the AP is about to drop).
            self._html(render_page([], f"Connecting to '{ssid}'… this hotspot will "
                                        "disappear. If it worked, the sender is now on "
                                        "your Wi-Fi — reconnect your device to your "
                                        "normal network and find it at neurokairos-sender.local."))
            threading.Thread(target=do_connect, args=(ssid, password, country),
                             daemon=True).start()

        def log_message(self, *a):
            return

    return ThreadingHTTPServer((host, port), Handler)


def main():
    start_ap()
    make_server().serve_forever()


if __name__ == "__main__":
    main()
