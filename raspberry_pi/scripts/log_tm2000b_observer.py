#!/usr/bin/env python3

"""Log chrony's local clock estimate alongside a TM2000B offset measurement.

Per sample:
- query `chronyc tracking` to record the Pi's local chrony state
- run `chronyd -Q 'server HOST iburst'` to measure the Pi's clock offset
  from the TM2000B using chronyd's own clock-filter algorithm
- append one TSV row per sample, rotating by UTC date
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence


DEFAULT_OUTPUT_DIR = Path("/home/pi/tm2000b_observer_logs")
DEFAULT_TARGET_HOST = "10.49.98.251"
DEFAULT_TARGET_PORT = 123
DEFAULT_CHRONYC_BINARY = "chronyc"
DEFAULT_TIMEOUT_S = 5.0
CSV_HEADER = [
    "timestamp_utc",
    "chrony_tracking_ok",
    "chrony_tracking_error",
    "chrony_reference_id",
    "chrony_stratum",
    "chrony_leap_status",
    "chrony_system_time_s",
    "chrony_last_offset_s",
    "chrony_rms_offset_s",
    "chrony_frequency_ppm",
    "chrony_skew_ppm",
    "chrony_root_delay_s",
    "chrony_root_dispersion_s",
    "chrony_update_interval_s",
    "tm2000b_query_ok",
    "tm2000b_query_error",
    "tm2000b_host",
    "tm2000b_port",
    "tm2000b_leap_indicator",
    "tm2000b_version",
    "tm2000b_mode",
    "tm2000b_stratum",
    "tm2000b_reference_id",
    "tm2000b_reference_time_utc",
    "tm2000b_receive_time_utc",
    "tm2000b_transmit_time_utc",
    "tm2000b_local_send_time_utc",
    "tm2000b_local_receive_time_utc",
    "tm2000b_offset_s",
    "tm2000b_delay_s",
    "tm2000b_root_delay_s",
    "tm2000b_root_dispersion_s",
    "tm2000b_precision_s",
    "tm2000b_poll_interval_s",
    "tm2000b_response_time_s",
    # sourcestats columns (added after initial deployment; blank in older rows)
    "chrony_selected_src_stdev_s",
    "chrony_n_combined",
    "chrony_sources_json",
]

FIELD_RE = re.compile(r"^(?P<name>[^:]+?)\s*:\s*(?P<value>.+)$")


def utc_now() -> datetime:
    """Return the current UTC timestamp.

    Returns:
        datetime: timezone-aware UTC timestamp.
    """

    return datetime.now(timezone.utc)


def iso_utc(timestamp: datetime) -> str:
    """Format a UTC datetime as `YYYY-MM-DDTHH:MM:SS.ssssssZ`.

    Args:
        timestamp (datetime): timezone-aware UTC timestamp.

    Returns:
        str: ISO-8601 string with trailing `Z`.
    """

    if timestamp.tzinfo is None:
        raise ValueError("timestamp must be timezone-aware")
    return timestamp.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")


def _parse_float_field(value: str) -> float | None:
    """Extract the leading floating-point value from a chrony field string.

    Args:
        value (str): Chrony field text, possibly with units.

    Returns:
        float | None: Parsed floating-point prefix, or `None` if absent.
    """

    match = re.match(r"^[+-]?[0-9.]+(?:[eE][+-]?[0-9]+)?", value.strip())
    if match is None:
        return None
    try:
        return float(match.group(0))
    except ValueError:
        return None


def parse_chronyc_tracking(output: str) -> dict[str, Any]:
    """Parse `chronyc tracking` output into a structured record.

    Args:
        output (str): Raw stdout from `chronyc tracking`.

    Returns:
        dict[str, Any]: Parsed tracking fields plus raw output and a boolean
        `chrony_holdover` flag.
    """

    fields: dict[str, Any] = {"raw_output": output}
    for line in output.splitlines():
        match = FIELD_RE.match(line.strip())
        if match is None:
            continue
        name = match.group("name").strip().lower().replace(" ", "_")
        value = match.group("value").strip()
        if name == "stratum":
            try:
                fields[name] = int(value)
            except ValueError:
                fields[name] = value
        elif name in {
            "root_delay",
            "root_dispersion",
            "last_offset",
            "rms_offset",
            "frequency",
            "residual_freq",
            "skew",
            "system_time",
            "update_interval",
        }:
            parsed_value = _parse_float_field(value)
            fields[name] = parsed_value if parsed_value is not None else value
        else:
            fields[name] = value

    leap_status = fields.get("leap_status")
    if isinstance(leap_status, str):
        fields["chrony_holdover"] = leap_status.lower() != "normal"
    else:
        fields["chrony_holdover"] = True
    return fields


def run_chronyc_tracking(
    *,
    chronyc_binary: str = DEFAULT_CHRONYC_BINARY,
    timeout_s: float = DEFAULT_TIMEOUT_S,
) -> dict[str, Any]:
    """Run `chronyc tracking` and parse its output.

    Args:
        chronyc_binary (str): Path to the `chronyc` executable.
        timeout_s (float): Subprocess timeout in seconds.

    Returns:
        dict[str, Any]: Parsed tracking data with `ok` and `error` fields.
    """

    try:
        completed = subprocess.run(
            [chronyc_binary, "-h", "127.0.0.1", "-n", "tracking"],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
        output = completed.stdout or completed.stderr
        tracking = parse_chronyc_tracking(output)
        tracking["ok"] = completed.returncode == 0
        tracking["returncode"] = completed.returncode
        tracking["error"] = "" if completed.returncode == 0 else output.strip()
        return tracking
    except subprocess.TimeoutExpired as exc:
        output = (exc.stdout or "") + (exc.stderr or "")
        tracking = parse_chronyc_tracking(output)
        tracking["ok"] = False
        tracking["returncode"] = None
        tracking["error"] = "chronyc tracking timed out"
        return tracking
    except OSError as exc:
        return {
            "raw_output": "",
            "ok": False,
            "returncode": None,
            "error": str(exc),
            "chrony_holdover": True,
        }


_CHRONY_DURATION_RE = re.compile(r"^([+-]?[0-9.]+(?:[eE][+-]?[0-9]+)?)(ns|us|ms|s)$")
_CHRONY_DURATION_MULTIPLIERS = {"ns": 1e-9, "us": 1e-6, "ms": 1e-3, "s": 1.0}


def _parse_chrony_duration(s: str) -> float | None:
    """Parse chrony duration strings like '49us', '216us', '1.2ms', '100ns'."""
    match = _CHRONY_DURATION_RE.match(s.strip())
    if match is None:
        return None
    return float(match.group(1)) * _CHRONY_DURATION_MULTIPLIERS[match.group(2)]


def parse_chronyc_sourcestats(
    sources_output: str,
    stats_output: str,
) -> dict[str, Any]:
    """Parse `chronyc sources` and `chronyc sourcestats` into a combined record.

    Merges by row position (both commands emit rows in the same order).

    Args:
        sources_output (str): Raw stdout from `chronyc sources`.
        stats_output (str): Raw stdout from `chronyc sourcestats`.

    Returns:
        dict[str, Any]: Record with selected-source stdev, n_combined, and
        a compact JSON list of all sources.
    """
    # Parse state from `chronyc sources`: state char is column[1] of each data row.
    # Valid states are * + - x ~ ? (mode chars ^ = # must not be treated as states;
    # the === separator line starts with = but its second char = is not a valid state).
    _valid_states = frozenset("*+-x~?")
    states: list[str] = []
    for line in sources_output.splitlines():
        if len(line) >= 2 and line[0] in ("^", "=", "#") and line[1] in _valid_states:
            states.append(line[1])

    # Parse name + stdev (last column) from `chronyc sourcestats`.
    sources: list[dict[str, Any]] = []
    for line in stats_output.splitlines():
        line = line.strip()
        if not line or line.startswith("=") or line.startswith("N"):
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        sources.append({
            "name": parts[0],
            "stdev_s": _parse_chrony_duration(parts[-1]),
        })

    # Merge state into each source entry.
    for i, src in enumerate(sources):
        src["state"] = states[i] if i < len(states) else "?"

    selected = next((s for s in sources if s["state"] == "*"), None)
    # Count sources chrony is combining: the selected (*) plus any blended (+).
    n_combined = sum(1 for s in sources if s["state"] in ("*", "+"))

    return {
        "selected_stdev_s": selected["stdev_s"] if selected else None,
        "n_combined": n_combined,
        "sources_json": json.dumps(
            [{"n": s["name"], "st": s["state"], "sd": s["stdev_s"]} for s in sources],
            separators=(",", ":"),
        ),
    }


def run_chronyc_sourcestats(
    *,
    chronyc_binary: str = DEFAULT_CHRONYC_BINARY,
    timeout_s: float = DEFAULT_TIMEOUT_S,
) -> dict[str, Any]:
    """Run `chronyc sources` and `chronyc sourcestats` and return parsed results.

    Args:
        chronyc_binary (str): Path to the `chronyc` executable.
        timeout_s (float): Subprocess timeout in seconds.

    Returns:
        dict[str, Any]: Parsed sourcestats record, or empty dict on failure.
    """
    def _run(subcmd: str) -> str:
        completed = subprocess.run(
            [chronyc_binary, "-h", "127.0.0.1", "-n", subcmd],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
        return completed.stdout or ""

    try:
        sources_out = _run("sources")
        stats_out = _run("sourcestats")
        return parse_chronyc_sourcestats(sources_out, stats_out)
    except (subprocess.TimeoutExpired, OSError):
        return {}


# Matches: "2026-05-28T14:03:21Z System clock wrong by -0.000308 seconds (ignored)"
_CHRONYD_OFFSET_RE = re.compile(r"System clock wrong by ([+-]?[0-9.]+(?:[eE][+-]?[0-9]+)?) seconds")


def query_tm2000b_ntp(
    *,
    host: str,
    port: int = DEFAULT_TARGET_PORT,
    timeout_s: float = DEFAULT_TIMEOUT_S,
) -> dict[str, Any]:
    """Measure the Pi's clock offset from the TM2000B using chronyd -Q.

    Uses chronyd's iburst clock filter (4-8 packets, picks lowest-delay
    sample) rather than a single raw UDP packet, giving a cleaner estimate.
    Requires the process to run as root.

    Args:
        host (str): TM2000B hostname or IP address.
        port (int): Unused; kept for interface compatibility.
        timeout_s (float): Subprocess timeout in seconds.

    Returns:
        dict[str, Any]: Query result with at least ``ok``, ``error``,
        ``host``, ``port``, and ``offset_s`` keys.
    """

    try:
        result = subprocess.run(
            ["sudo", "chronyd", "-Q", f"server {host} iburst"],
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
        output = result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "chronyd -Q timed out", "host": host, "port": port}
    except OSError as exc:
        return {"ok": False, "error": str(exc), "host": host, "port": port}

    match = _CHRONYD_OFFSET_RE.search(output)
    if match is None:
        return {"ok": False, "error": f"no offset in chronyd output: {output.strip()}", "host": host, "port": port}

    return {
        "ok": True,
        "error": "",
        "host": host,
        "port": port,
        "offset_s": float(match.group(1)),
    }


def build_record(
    *,
    timestamp_utc: datetime,
    chrony_tracking: Mapping[str, Any],
    tm2000b_query: Mapping[str, Any],
    chrony_sourcestats: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Combine chrony and TM2000B measurements into one flat record.

    Args:
        timestamp_utc (datetime): Sample timestamp in UTC.
        chrony_tracking (Mapping[str, Any]): Parsed `chronyc tracking` record.
        tm2000b_query (Mapping[str, Any]): Parsed TM2000B NTP query record.
        chrony_sourcestats (Mapping[str, Any] | None): Parsed sourcestats record,
            or None/empty to leave the new columns blank (backwards-compatible).

    Returns:
        dict[str, Any]: Flat dictionary ready for TSV output.
    """
    ss = chrony_sourcestats or {}
    return {
        "timestamp_utc": iso_utc(timestamp_utc),
        "chrony_tracking_ok": chrony_tracking.get("ok", False),
        "chrony_tracking_error": chrony_tracking.get("error", ""),
        "chrony_reference_id": chrony_tracking.get("reference_id", ""),
        "chrony_stratum": chrony_tracking.get("stratum", ""),
        "chrony_leap_status": chrony_tracking.get("leap_status", ""),
        "chrony_system_time_s": chrony_tracking.get("system_time", ""),
        "chrony_last_offset_s": chrony_tracking.get("last_offset", ""),
        "chrony_rms_offset_s": chrony_tracking.get("rms_offset", ""),
        "chrony_frequency_ppm": chrony_tracking.get("frequency", ""),
        "chrony_skew_ppm": chrony_tracking.get("skew", ""),
        "chrony_root_delay_s": chrony_tracking.get("root_delay", ""),
        "chrony_root_dispersion_s": chrony_tracking.get("root_dispersion", ""),
        "chrony_update_interval_s": chrony_tracking.get("update_interval", ""),
        "tm2000b_query_ok": tm2000b_query.get("ok", False),
        "tm2000b_query_error": tm2000b_query.get("error", ""),
        "tm2000b_host": tm2000b_query.get("host", ""),
        "tm2000b_port": tm2000b_query.get("port", ""),
        "tm2000b_leap_indicator": tm2000b_query.get("leap_indicator", ""),
        "tm2000b_version": tm2000b_query.get("version", ""),
        "tm2000b_mode": tm2000b_query.get("mode", ""),
        "tm2000b_stratum": tm2000b_query.get("stratum", ""),
        "tm2000b_reference_id": tm2000b_query.get("reference_id", ""),
        "tm2000b_reference_time_utc": tm2000b_query.get("reference_time_utc", ""),
        "tm2000b_receive_time_utc": tm2000b_query.get("receive_time_utc", ""),
        "tm2000b_transmit_time_utc": tm2000b_query.get("transmit_time_utc", ""),
        "tm2000b_local_send_time_utc": tm2000b_query.get("local_send_time_utc", ""),
        "tm2000b_local_receive_time_utc": tm2000b_query.get("local_receive_time_utc", ""),
        "tm2000b_offset_s": tm2000b_query.get("offset_s", ""),
        "tm2000b_delay_s": tm2000b_query.get("delay_s", ""),
        "tm2000b_root_delay_s": tm2000b_query.get("root_delay_s", ""),
        "tm2000b_root_dispersion_s": tm2000b_query.get("root_dispersion_s", ""),
        "tm2000b_precision_s": tm2000b_query.get("precision_s", ""),
        "tm2000b_poll_interval_s": tm2000b_query.get("poll_interval_s", ""),
        "tm2000b_response_time_s": tm2000b_query.get("response_time_s", ""),
        "chrony_selected_src_stdev_s": ss.get("selected_stdev_s", ""),
        "chrony_n_combined": ss.get("n_combined", ""),
        "chrony_sources_json": ss.get("sources_json", ""),
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments for the observer logger."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--tm2000b-host", default=DEFAULT_TARGET_HOST)
    parser.add_argument("--tm2000b-port", type=int, default=DEFAULT_TARGET_PORT)
    parser.add_argument("--chronyc-binary", default=DEFAULT_CHRONYC_BINARY)
    parser.add_argument("--timeout-s", type=float, default=DEFAULT_TIMEOUT_S)
    return parser.parse_args(argv)


def _output_path(output_dir: Path, timestamp_utc: datetime) -> Path:
    """Return the daily TSV path for one sample timestamp.

    Args:
        output_dir (Path): Root directory for observer logs.
        timestamp_utc (datetime): Sample timestamp in UTC.

    Returns:
        Path: Daily TSV file path.
    """

    return output_dir / f"tm2000b_observer_{timestamp_utc:%Y-%m-%d}.tsv"


def append_tsv_row(path: Path, fieldnames: Sequence[str], row: Mapping[str, Any]) -> None:
    """Append one TSV row, creating the file and header if needed.

    Args:
        path (Path): Output TSV path.
        fieldnames (Sequence[str]): TSV column order.
        row (Mapping[str, Any]): Flat row to write.
    """

    path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, delimiter="\t", extrasaction="ignore")
        if new_file:
            writer.writeheader()
        writer.writerow(row)


def run(config: argparse.Namespace) -> dict[str, Any]:
    """Collect one observer sample and write it to the daily TSV log.

    Args:
        config (argparse.Namespace): Parsed CLI arguments.

    Returns:
        dict[str, Any]: The record written to disk plus the output path.
    """

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
    """CLI entry point.

    Args:
        argv (Sequence[str] | None): Optional command-line arguments.

    Returns:
        int: Process exit code.
    """

    args = parse_args(argv)
    result = run(args)
    print(result["output_path"])
    return 0


def _main() -> None:
    """Convert exceptions to a non-zero process exit status."""

    try:
        raise SystemExit(main())
    except Exception as exc:  # pragma: no cover - CLI failure path.
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    _main()
