#!/usr/bin/env python3
"""NeuroKairos LAN NTP server discovery.

Browses the local network (mDNS/Avahi) for NeuroKairos server-image Pis that
advertise the ``_neurokairos-ntp._udp`` service, then hands the discovered
addresses to chrony as additional NTP sources.

Crucially, this never restarts chronyd and never steps the clock:

  * discovered servers are written to a ``.sources`` file inside chrony's
    ``sourcedir`` (``/etc/chrony/sources.d/``);
  * ``chronyc reload sources`` makes chronyd pick them up at runtime — a running
    daemon *slews* toward a better source, it does not step (``makestep`` only
    steps within the first few updates after chronyd *starts*);
  * the file is rewritten (and chrony reloaded) only when the discovered set
    actually changed, so existing sources keep their measurement history.

If no servers are discovered the sources file is left untouched — a server that
has genuinely gone away simply becomes unreachable and chrony ignores it,
falling back to the public servers in chrony.conf. This avoids a transient empty
browse wiping a good configuration.

Pure-stdlib; designed to run as a root systemd oneshot on a timer.
"""
from __future__ import annotations

import argparse
import logging
import os
import subprocess
import sys
from typing import Iterable, Set

logger = logging.getLogger("nk_discover_ntp")

SERVICE_TYPE = "_neurokairos-ntp._udp"
DEFAULT_SOURCES_PATH = "/etc/chrony/sources.d/neurokairos.sources"

_HEADER = (
    "# Managed by nk_discover_ntp.py — NeuroKairos LAN NTP servers discovered\n"
    "# via mDNS. Do not edit by hand; regenerated on a timer.\n"
)


# ---------------------------------------------------------------------------
# Pure helpers (unit-tested)
# ---------------------------------------------------------------------------

def _is_ipv4(s: str) -> bool:
    """True if s is a dotted-quad IPv4 address with each octet in 0..255."""
    parts = s.split(".")
    if len(parts) != 4:
        return False
    for p in parts:
        if not p.isdigit() or len(p) > 3:
            return False
        if not 0 <= int(p) <= 255:
            return False
    return True


def parse_avahi_browse(text: str) -> Set[str]:
    """Extract resolved IPv4 addresses from ``avahi-browse -rpt`` output.

    The parseable format is semicolon-separated; resolved records begin with
    ``=`` and carry the address in field index 7::

        =;eth0;IPv4;NeuroKairos\\032NTP;_neurokairos-ntp._udp;local;host.local;192.168.1.50;123;""

    IPv6 records and malformed/other lines are ignored.
    """
    addresses: Set[str] = set()
    for line in text.splitlines():
        if not line.startswith("="):
            continue
        fields = line.split(";")
        if len(fields) < 9:
            continue
        if fields[2] != "IPv4":
            continue
        addr = fields[7].strip()
        if _is_ipv4(addr):
            addresses.add(addr)
    return addresses


def render_sources(addresses: Iterable[str]) -> str:
    """Render the chrony ``.sources`` file body for the given addresses.

    Deterministic (sorted) so an unchanged set produces byte-identical output.
    Discovered LAN servers are marked ``prefer`` so chrony prefers them over the
    public fallback sources whenever they are usable.
    """
    lines = [f"server {addr} iburst prefer" for addr in sorted(addresses)]
    return _HEADER + "\n".join(lines) + ("\n" if lines else "")


# ---------------------------------------------------------------------------
# I/O (mocked in tests)
# ---------------------------------------------------------------------------

def run_avahi_browse(service_type: str = SERVICE_TYPE) -> str:
    """Run avahi-browse once (resolve, parseable, terminate) and return stdout."""
    result = subprocess.run(
        ["avahi-browse", "-rpt", service_type],
        capture_output=True, text=True, timeout=30,
    )
    return result.stdout


def reload_chrony() -> None:
    """Ask a running chronyd to re-read its source files (no restart, no step)."""
    subprocess.run(["chronyc", "reload", "sources"], check=True, timeout=10)


def _read_file(path: str) -> str:
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return ""


def _write_atomic(path: str, content: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(content)
    os.rename(tmp, path)


# ---------------------------------------------------------------------------
# Orchestration
# ---------------------------------------------------------------------------

def discover_and_update(sources_path: str, service_type: str = SERVICE_TYPE) -> bool:
    """Discover servers and update the sources file. Returns True if it changed."""
    addresses = parse_avahi_browse(run_avahi_browse(service_type))

    if not addresses:
        logger.info("No NeuroKairos NTP servers discovered; leaving %s unchanged",
                    sources_path)
        return False

    new_content = render_sources(addresses)
    if _read_file(sources_path) == new_content:
        logger.info("Discovered %d server(s), sources file already current: %s",
                    len(addresses), ", ".join(sorted(addresses)))
        return False

    _write_atomic(sources_path, new_content)
    reload_chrony()
    logger.info("Updated %s with %d server(s) and reloaded chrony: %s",
                sources_path, len(addresses), ", ".join(sorted(addresses)))
    return True


def _parse_args(argv=None):
    p = argparse.ArgumentParser(description="Discover NeuroKairos LAN NTP servers")
    p.add_argument("--sources-path", default=DEFAULT_SOURCES_PATH,
                   help="chrony .sources file to write (default: %(default)s)")
    p.add_argument("--service-type", default=SERVICE_TYPE,
                   help="mDNS service type to browse (default: %(default)s)")
    p.add_argument("--log-level", default="INFO",
                   choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return p.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )
    try:
        discover_and_update(args.sources_path, args.service_type)
    except FileNotFoundError as e:
        # avahi-browse / chronyc missing — log and exit non-fatally so the timer
        # simply retries later rather than the unit entering a failed state loop.
        logger.error("Required command not found: %s", e)
        return 1
    except subprocess.SubprocessError as e:
        logger.error("Discovery command failed: %s", e)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
