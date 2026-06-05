#!/usr/bin/env python3

"""Log chrony's local clock estimate alongside a one-shot TM2000B NTP offset.

This observer is intentionally simple:

- query `chronyc tracking` on the Raspberry Pi to record chrony's estimate of
  the local clock
- send one NTP request directly to the TM2000B appliance and compute the
  offset from the returned timestamps
- append a single TSV row per sample, rotating by UTC date

The output is designed for drift analysis over multiple days.
"""

from __future__ import annotations

import argparse
import csv
import re
import socket
import struct
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence


DEFAULT_OUTPUT_DIR = Path("/home/pi/tm2000b_observer_logs")
DEFAULT_TARGET_HOST = "10.49.98.251"
DEFAULT_TARGET_PORT = 123
DEFAULT_CHRONYC_BINARY = "chronyc"
DEFAULT_TIMEOUT_S = 5.0
NTP_EPOCH_UNIX_OFFSET = 2_208_988_800
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


def _ntp_timestamp_to_unix(timestamp: int) -> float:
    """Convert an NTP 64-bit timestamp to Unix seconds.

    Args:
        timestamp (int): Unsigned 64-bit NTP timestamp.

    Returns:
        float: Seconds since the Unix epoch.
    """

    seconds = timestamp >> 32
    fraction = timestamp & 0xFFFFFFFF
    return (seconds - NTP_EPOCH_UNIX_OFFSET) + (fraction / 2**32)


def _unix_to_ntp_timestamp(unix_time: float) -> int:
    """Convert Unix seconds to an unsigned 64-bit NTP timestamp.

    Args:
        unix_time (float): Seconds since the Unix epoch.

    Returns:
        int: Unsigned 64-bit NTP timestamp.
    """

    ntp_seconds = int(unix_time + NTP_EPOCH_UNIX_OFFSET)
    fractional = int((unix_time - int(unix_time)) * 2**32) & 0xFFFFFFFF
    return (ntp_seconds << 32) | fractional


def _decode_reference_id(raw_reference_id: int, stratum: int) -> str:
    """Render the NTP reference ID as printable text when possible.

    Args:
        raw_reference_id (int): Unsigned 32-bit NTP reference ID.
        stratum (int): Server stratum from the response.

    Returns:
        str: Human-readable reference ID.
    """

    raw_bytes = raw_reference_id.to_bytes(4, byteorder="big", signed=False)
    decoded = raw_bytes.decode("ascii", errors="replace").rstrip("\x00")
    if stratum <= 1 and all(byte == 0 or 32 <= byte < 127 for byte in raw_bytes):
        return decoded
    return ".".join(str(byte) for byte in raw_bytes)


def _decode_li_vn_mode(value: int) -> tuple[int, int, int]:
    """Split the first NTP header byte into leap, version, and mode bits.

    Args:
        value (int): First byte of an NTP header.

    Returns:
        tuple[int, int, int]: `(leap_indicator, version, mode)`.
    """

    leap = (value >> 6) & 0x3
    version = (value >> 3) & 0x7
    mode = value & 0x7
    return leap, version, mode


def query_tm2000b_ntp(
    *,
    host: str,
    port: int = DEFAULT_TARGET_PORT,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    time_func: Callable[[], float] = time.time,
) -> dict[str, Any]:
    """Send one NTP query to the TM2000B and estimate the time offset.

    Args:
        host (str): TM2000B hostname or IP address.
        port (int): NTP port, usually 123.
        timeout_s (float): Socket timeout in seconds.
        time_func (Callable[[], float]): Function returning Unix seconds.

    Returns:
        dict[str, Any]: Parsed NTP response plus the estimated offset and delay.
    """

    request = bytearray(48)
    request[0] = 0x23  # LI=0, VN=4, Mode=3 (client)

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(timeout_s)
            local_send_time = time_func()
            request[40:48] = _unix_to_ntp_timestamp(local_send_time).to_bytes(8, byteorder="big")
            sock.sendto(request, (host, port))
            payload, _ = sock.recvfrom(512)
            local_receive_time = time_func()
    except socket.timeout as exc:
        return {
            "ok": False,
            "error": "timed out waiting for TM2000B NTP response",
            "host": host,
            "port": port,
        }
    except OSError as exc:
        return {
            "ok": False,
            "error": str(exc),
            "host": host,
            "port": port,
        }

    if len(payload) < 48:
        return {
            "ok": False,
            "error": "short NTP response",
            "host": host,
            "port": port,
        }

    li_vn_mode, stratum, poll, precision, root_delay, root_dispersion, reference_id, reference_time, originate_time, receive_time, transmit_time = struct.unpack(
        "!BBBbII4sQQQQ", payload[:48]
    )
    leap_indicator, version, mode = _decode_li_vn_mode(li_vn_mode)
    receive_time_unix = _ntp_timestamp_to_unix(receive_time)
    transmit_time_unix = _ntp_timestamp_to_unix(transmit_time)
    reference_time_unix = _ntp_timestamp_to_unix(reference_time)
    offset_s = ((receive_time_unix - local_send_time) + (transmit_time_unix - local_receive_time)) / 2.0
    delay_s = (local_receive_time - local_send_time) - (transmit_time_unix - receive_time_unix)

    return {
        "ok": True,
        "error": "",
        "host": host,
        "port": port,
        "leap_indicator": leap_indicator,
        "version": version,
        "mode": mode,
        "stratum": stratum,
        "poll_interval_s": 2**poll if poll >= 0 else None,
        "precision_s": 2**precision if precision < 0 else float(precision),
        "root_delay_s": root_delay / 65536.0,
        "root_dispersion_s": root_dispersion / 65536.0,
        "reference_id": _decode_reference_id(int.from_bytes(reference_id, byteorder="big", signed=False), stratum),
        "reference_time_utc": iso_utc(datetime.fromtimestamp(reference_time_unix, tz=timezone.utc)),
        "receive_time_utc": iso_utc(datetime.fromtimestamp(receive_time_unix, tz=timezone.utc)),
        "transmit_time_utc": iso_utc(datetime.fromtimestamp(transmit_time_unix, tz=timezone.utc)),
        "local_send_time_utc": iso_utc(datetime.fromtimestamp(local_send_time, tz=timezone.utc)),
        "local_receive_time_utc": iso_utc(datetime.fromtimestamp(local_receive_time, tz=timezone.utc)),
        "offset_s": offset_s,
        "delay_s": delay_s,
        "response_time_s": local_receive_time - local_send_time,
    }


def build_record(
    *,
    timestamp_utc: datetime,
    chrony_tracking: Mapping[str, Any],
    tm2000b_query: Mapping[str, Any],
) -> dict[str, Any]:
    """Combine chrony and TM2000B measurements into one flat record.

    Args:
        timestamp_utc (datetime): Sample timestamp in UTC.
        chrony_tracking (Mapping[str, Any]): Parsed `chronyc tracking` record.
        tm2000b_query (Mapping[str, Any]): Parsed TM2000B NTP query record.

    Returns:
        dict[str, Any]: Flat dictionary ready for TSV output.
    """

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
    tm2000b_query = query_tm2000b_ntp(
        host=config.tm2000b_host,
        port=config.tm2000b_port,
        timeout_s=config.timeout_s,
    )
    record = build_record(
        timestamp_utc=timestamp_utc,
        chrony_tracking=chrony_tracking,
        tm2000b_query=tm2000b_query,
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
