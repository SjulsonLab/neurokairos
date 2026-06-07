#!/usr/bin/env python3
"""Manual export CLI for NeuroKairos event logger recordings."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import socket
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
LIB_DIR = Path("/usr/local/lib/neurokairos-eventlogger")
for candidate in (SCRIPT_DIR, LIB_DIR):
    if candidate.exists() and str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

from eventlogger_control import DEFAULT_ROOT_DIR, export_recording, normalize_basename, parse_utc_iso_to_ns, validate_basename


def main() -> int:
    parser = argparse.ArgumentParser(description="Export a journal time range to TSV and YAML.")
    parser.add_argument("--root-dir", default=str(DEFAULT_ROOT_DIR))
    parser.add_argument("--basename", default="events")
    parser.add_argument("--start", required=True, help="UTC start time, for example 2026-06-06T14:57:07Z")
    parser.add_argument("--stop", required=True, help="UTC stop time, for example 2026-06-06T14:57:08Z")
    parser.add_argument("--timezone", default=datetime.now().astimezone().tzname() or "UTC")
    args = parser.parse_args()

    start = args.start
    stop = args.stop
    start_local = datetime.fromtimestamp(parse_utc_iso_to_ns(start) / 1_000_000_000).astimezone().replace(
        tzinfo=None
    )
    recording = {
        "basename": normalize_basename(args.basename),
        "start_utc": start,
        "stop_utc": stop,
        "start_ns": parse_utc_iso_to_ns(start),
        "stop_ns": parse_utc_iso_to_ns(stop),
        "start_local": start_local.isoformat(timespec="seconds"),
        "timezone": args.timezone,
        "exported_inputs": [],
        "interrupted": False,
    }
    recording["basename"] = validate_basename(recording["basename"])
    result = export_recording(Path(args.root_dir), recording, hostname=socket.gethostname())
    print(result["tsv_path"])
    print(result["yaml_path"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
