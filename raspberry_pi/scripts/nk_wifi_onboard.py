#!/usr/bin/env python3
"""NeuroKairos headless Wi-Fi onboarding (sender image).

Runs once per boot (oneshot, after NetworkManager) and decides how to get the Pi
onto a network, in three tiers:

  1. Already networked — ethernet carrier or a saved Wi-Fi connected -> nothing.
  2. Boot-file — /boot/firmware/neurokairos-wifi.txt present -> set the Wi-Fi
     regulatory country (required, or the radio stays rfkill-blocked), connect,
     mark provisioned, and move the file aside (keep the PSK off the FAT card).
  3. Setup hotspot — otherwise start nk-wifi-portal.service (the captive AP).

Once provisioned (a marker file), it never auto-starts the setup AP again, so a
transient Wi-Fi outage can't drop a running appliance into AP mode. Ethernet
always takes precedence.

Pure-stdlib. The parse/decision logic is unit-tested; nmcli/rfkill/raspi-config
calls are I/O (mocked in tests).
"""
from __future__ import annotations

import os
import subprocess
import sys
import time

BOOT_CFG = "/boot/firmware/neurokairos-wifi.txt"
MARKER = "/var/lib/neurokairos/wifi-provisioned"
PORTAL_UNIT = "nk-wifi-portal.service"


def log(msg):
    print(f"nk-wifi-onboard: {msg}", flush=True)


# ---------------------------------------------------------------------------
# Pure helpers (unit-tested)
# ---------------------------------------------------------------------------

def parse_wifi_config(text):
    """Parse `key=value` lines. Recognizes country/ssid/password (psk alias).

    Ignores blanks and #comments; tolerates surrounding quotes/space; last wins.
    """
    cfg = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, val = line.partition("=")
        key = key.strip().lower()
        val = val.strip().strip('"').strip("'")
        if key in ("country", "ssid", "password", "psk"):
            cfg["password" if key == "psk" else key] = val
    return cfg


def networked_from_status(nmcli_status: str) -> bool:
    """True if any non-loopback device is 'connected' in `nmcli -t device status`."""
    for line in nmcli_status.splitlines():
        dev, _, state = line.partition(":")
        if dev and dev != "lo" and state.strip() == "connected":
            return True
    return False


# ---------------------------------------------------------------------------
# I/O (mocked in tests)
# ---------------------------------------------------------------------------

def _run(args, timeout=30):
    return subprocess.run(args, capture_output=True, text=True, timeout=timeout)


def networked() -> bool:
    try:
        r = _run(["nmcli", "-t", "-f", "DEVICE,STATE", "device", "status"], timeout=10)
    except (OSError, subprocess.SubprocessError):
        return False
    return networked_from_status(r.stdout)


def set_country(cc: str) -> None:
    if not cc:
        return
    # do_wifi_country persists the country and also unblocks the radio; iw is a
    # belt-and-suspenders in case raspi-config is unavailable. Give the kernel a
    # moment to apply the new regulatory domain before we try to scan/associate —
    # associating before the regdomain is live is what silently failed on-device.
    for cmd in (["raspi-config", "nonint", "do_wifi_country", cc], ["iw", "reg", "set", cc]):
        try:
            _run(cmd, timeout=15)
        except (OSError, subprocess.SubprocessError):
            pass
    for cmd in (["rfkill", "unblock", "wifi"], ["nmcli", "radio", "wifi", "on"]):
        try:
            _run(cmd, timeout=10)
        except (OSError, subprocess.SubprocessError):
            pass
    time.sleep(3)


def wifi_rescan() -> None:
    # A fresh scan is needed after unblocking the radio, or `wifi connect` fails
    # with "No network with SSID found" because nothing is in the scan cache yet.
    try:
        _run(["nmcli", "device", "wifi", "rescan"], timeout=20)
    except (OSError, subprocess.SubprocessError):
        pass
    time.sleep(4)


def wifi_connect(ssid: str, password: str) -> bool:
    args = ["nmcli", "--wait", "30", "device", "wifi", "connect", ssid]
    if password:
        args += ["password", password]
    try:
        r = _run(args, timeout=45)
    except (OSError, subprocess.SubprocessError) as exc:
        log(f"wifi connect error: {exc}")
        return False
    if r.returncode != 0:
        log(f"wifi connect failed: {(r.stderr or r.stdout or '').strip()}")
    return r.returncode == 0


def wait_online(seconds=25):
    try:
        _run(["nm-online", "-s", "-t", str(seconds)], timeout=seconds + 5)
    except (OSError, subprocess.SubprocessError):
        pass


def provisioned() -> bool:
    return os.path.exists(MARKER)


def mark_provisioned() -> None:
    os.makedirs(os.path.dirname(MARKER), exist_ok=True)
    open(MARKER, "w").close()


def provision_from_file(path=BOOT_CFG) -> bool:
    try:
        with open(path) as f:
            cfg = parse_wifi_config(f.read())
    except OSError:
        return False
    if not cfg.get("ssid"):
        log(f"no ssid in {path}")
        return False
    set_country(cfg.get("country", ""))
    log(f"connecting to '{cfg['ssid']}'")
    # Retry: right after the radio comes up the first scan/associate can miss.
    for attempt in range(1, 4):
        wifi_rescan()
        if wifi_connect(cfg["ssid"], cfg.get("password", "")):
            break
        log(f"attempt {attempt} failed; retrying")
    time.sleep(3)
    if networked():
        try:
            os.replace(path, path + ".applied")   # keep the PSK off the FAT card
        except OSError:
            try:
                os.remove(path)
            except OSError:
                pass
        mark_provisioned()
        return True
    return False


def start_portal():
    try:
        _run(["systemctl", "start", PORTAL_UNIT], timeout=15)
    except (OSError, subprocess.SubprocessError) as exc:
        log(f"could not start setup portal: {exc}")


def main():
    wait_online()                       # let ethernet / saved Wi-Fi autoconnect
    if networked():
        log("already networked; nothing to do")
        return 0
    if os.path.exists(BOOT_CFG):
        if provision_from_file():
            log("provisioned from neurokairos-wifi.txt")
            return 0
        log("boot-file provisioning failed")
    if provisioned():
        log("already provisioned; not starting setup AP")
        return 0
    log("no network and no config; starting setup hotspot")
    start_portal()
    return 0


if __name__ == "__main__":
    sys.exit(main())
