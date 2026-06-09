#!/usr/bin/env python3

"""Log chrony state alongside a one-shot TM2000B NTP offset during an internet-only chrony test.

This logger is intentionally separate from the existing TM2000B observer.
It records the same compact chrony/TM2000B timing fields, but writes to a
different output directory so the internet-only test can run without changing
the baseline logger.
"""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
from typing import Any, Sequence


BASE_SCRIPT_PATH = Path(__file__).with_name("log_tm2000b_observer.py")
DEFAULT_OUTPUT_DIR = Path("/home/pi/tm2000b_internet_observer_logs")


def _load_base_module() -> Any:
    """Load the existing TM2000B observer logger as a helper module."""

    spec = importlib.util.spec_from_file_location("_tm2000b_observer_base", BASE_SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load module from {BASE_SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_base = _load_base_module()

DEFAULT_TARGET_HOST = _base.DEFAULT_TARGET_HOST
DEFAULT_TARGET_PORT = _base.DEFAULT_TARGET_PORT
DEFAULT_CHRONYC_BINARY = _base.DEFAULT_CHRONYC_BINARY
DEFAULT_TIMEOUT_S = _base.DEFAULT_TIMEOUT_S
CSV_HEADER = _base.CSV_HEADER
utc_now = _base.utc_now
iso_utc = _base.iso_utc
parse_chronyc_tracking = _base.parse_chronyc_tracking
run_chronyc_tracking = _base.run_chronyc_tracking
run_chronyc_sourcestats = _base.run_chronyc_sourcestats
query_tm2000b_ntp = _base.query_tm2000b_ntp
build_record = _base.build_record
append_tsv_row = _base.append_tsv_row


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments for the internet-only observer logger."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--tm2000b-host", default=DEFAULT_TARGET_HOST)
    parser.add_argument("--tm2000b-port", type=int, default=DEFAULT_TARGET_PORT)
    parser.add_argument("--chronyc-binary", default=DEFAULT_CHRONYC_BINARY)
    parser.add_argument("--timeout-s", type=float, default=DEFAULT_TIMEOUT_S)
    return parser.parse_args(argv)


def _output_path(output_dir: Path, timestamp_utc) -> Path:
    """Return the daily TSV path for one sample timestamp."""

    return output_dir / f"tm2000b_internet_observer_{timestamp_utc:%Y-%m-%d}.tsv"


def run(config: argparse.Namespace) -> dict[str, Any]:
    """Collect one observer sample and write it to the daily TSV log."""

    timestamp_utc = utc_now()
    chrony_tracking = run_chronyc_tracking(
        chronyc_binary=config.chronyc_binary,
        timeout_s=config.timeout_s,
    )
    chrony_sourcestats = run_chronyc_sourcestats(
        chronyc_binary=config.chronyc_binary,
        timeout_s=config.timeout_s,
    )
    tm2000b_query = query_tm2000b_ntp(
        host=config.tm2000b_host,
        port=config.tm2000b_port,
        timeout_s=config.timeout_s,
    )
    record = build_record(
        timestamp_utc=timestamp_utc,
        chrony_tracking=chrony_tracking,
        tm2000b_query=tm2000b_query,
        chrony_sourcestats=chrony_sourcestats,
    )
    output_path = _output_path(config.output_dir, timestamp_utc)
    append_tsv_row(output_path, CSV_HEADER, record)
    return {"output_path": str(output_path), "record": record}


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entry point."""

    args = parse_args(argv)
    result = run(args)
    print(result["output_path"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
