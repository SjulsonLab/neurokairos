#!/usr/bin/env python3

"""Collect a compact GNSS and chrony status snapshot once per minute.

This script polls `gpsd` for receiver state and `chronyc` for clock discipline
state, then appends one TSV row per sample to a daily file and only writes a
JSONL event when the compact state changes.
"""

from __future__ import annotations

import argparse
import json
import re
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping


DEFAULT_OUTPUT_DIR = Path("/home/pi/GNSS_logs")
DEFAULT_GPSD_HOST = "127.0.0.1"
DEFAULT_GPSD_PORT = 2947
DEFAULT_SAMPLE_WINDOW_SECONDS = 5.0
TSV_HEADER = [
    "timestamp_utc",
    "receiver_holdover",
    "fix_mode",
    "satellites_visible",
    "satellites_used",
    "satellite_prns",
    "satellite_snr",
    "satellite_elevation",
    "satellite_azimuth",
    "satellite_used",
    "kernel_pps_ok",
    "kernel_pps_assert_count",
    "chrony_holdover",
    "chrony_synced",
    "chrony_stratum",
    "chrony_leap_status",
    "chrony_reference_id",
    "chrony_selected_source",
    "chrony_selected_marker",
    "chrony_pps_reach",
    "chrony_pps_last_rx_s",
    "chrony_pps_sample",
    "chrony_gps_reach",
    "chrony_gps_last_rx_s",
    "pps_sourcestats_np",
    "pps_sourcestats_span_s",
    "pps_sourcestats_frequency_ppm",
    "pps_sourcestats_freq_skew_ppm",
    "pps_sourcestats_offset",
    "pps_sourcestats_std_dev",
]
STATE_FILE_NAME = "gnss_m8t_status_state.json"
EVENT_LOG_PREFIX = "gnss_m8t_status_events_"
GPSD_WATCH_COMMAND = b'?WATCH={"enable":true,"json":true};\n'


def utc_now() -> datetime:
    """Return the current UTC time.

    Returns:
        datetime: timezone-aware UTC timestamp.
    """

    return datetime.now(timezone.utc)


def isoformat_utc(timestamp: datetime) -> str:
    """Format a timezone-aware UTC datetime as an ISO 8601 string.

    Args:
        timestamp (datetime): timezone-aware UTC datetime.

    Returns:
        str: Timestamp in the form `YYYY-MM-DDTHH:MM:SSZ`.
    """

    if timestamp.tzinfo is None:
        raise ValueError("timestamp must be timezone-aware")
    return timestamp.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")


def read_gpsd_messages(
    host: str = DEFAULT_GPSD_HOST,
    port: int = DEFAULT_GPSD_PORT,
    sample_window_seconds: float = DEFAULT_SAMPLE_WINDOW_SECONDS,
) -> list[dict[str, Any]]:
    """Collect JSON messages from gpsd over a short watch window.

    Args:
        host (str): gpsd host name or IP address.
        port (int): gpsd TCP port.
        sample_window_seconds (float): Time window, in seconds, used to collect
            recent gpsd messages.

    Returns:
        list[dict[str, Any]]: Parsed gpsd JSON objects observed during the
        watch window.
    """

    messages: list[dict[str, Any]] = []
    deadline = time.monotonic() + sample_window_seconds

    with socket.create_connection((host, port), timeout=sample_window_seconds) as sock:
        sock.settimeout(sample_window_seconds)
        sock.sendall(GPSD_WATCH_COMMAND)

        with sock.makefile("r", encoding="utf-8", newline="\n") as stream:
            while time.monotonic() < deadline:
                try:
                    line = stream.readline()
                except socket.timeout:
                    break
                if not line:
                    break
                line = line.strip()
                if not line.startswith("{"):
                    continue
                try:
                    payload = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(payload, dict):
                    messages.append(payload)

    return messages


def _count_used_satellites(sky_message: Mapping[str, Any]) -> int:
    """Count the satellites marked as used in a gpsd SKY message."""

    value = sky_message.get("uSat")
    if isinstance(value, int):
        return value

    satellites = sky_message.get("satellites")
    if isinstance(satellites, list):
        return sum(1 for satellite in satellites if isinstance(satellite, Mapping) and satellite.get("used"))
    return 0


def _join_satellite_field(satellites: Any, field_name: str) -> str:
    """Serialize one gpsd satellite field as comma-delimited text."""

    if not isinstance(satellites, list):
        return ""

    values = []
    for satellite in satellites:
        if not isinstance(satellite, Mapping):
            continue
        value = satellite.get(field_name)
        if value is None and field_name == "PRN":
            value = satellite.get("svid")
        if isinstance(value, bool):
            values.append("1" if value else "0")
        elif value is not None:
            values.append(str(value))
        else:
            values.append("")
    return ",".join(values)


def _parse_int_field(value: Any) -> int | None:
    """Return an integer if a gpsd field already looks like one."""

    if isinstance(value, int):
        return value
    if isinstance(value, str):
        try:
            return int(value)
        except ValueError:
            return None
    return None


def summarize_gpsd_messages(messages: Iterable[Mapping[str, Any]]) -> dict[str, Any]:
    """Summarize gpsd TPV and SKY messages into a receiver status record.

    Args:
        messages (Iterable[Mapping[str, Any]]): gpsd JSON payloads.

    Returns:
        dict[str, Any]: Summary fields including fix mode, satellite counts,
        and a receiver-level holdover flag based on no-fix state.
    """

    latest_tpv: Mapping[str, Any] | None = None
    latest_sky: Mapping[str, Any] | None = None

    for message in messages:
        message_class = message.get("class")
        if message_class == "TPV":
            latest_tpv = message
        elif message_class == "SKY":
            latest_sky = message

    fix_mode = None
    device = None
    gps_time_utc = None
    if latest_tpv is not None:
        device = latest_tpv.get("device")
        gps_time_utc = latest_tpv.get("time")
        mode_value = latest_tpv.get("mode")
        if isinstance(mode_value, int):
            fix_mode = mode_value

    if device is None and latest_sky is not None:
        device = latest_sky.get("device")

    visible_satellites = 0
    used_satellites = 0
    if latest_sky is not None:
        n_sat = _parse_int_field(latest_sky.get("nSat"))
        if n_sat is not None:
            visible_satellites = n_sat
        else:
            satellites = latest_sky.get("satellites")
            if isinstance(satellites, list):
                visible_satellites = len(satellites)
        used_satellites = _count_used_satellites(latest_sky)
        satellites = latest_sky.get("satellites")
    else:
        satellites = None

    receiver_holdover = fix_mode is None or fix_mode < 2

    return {
        "device": device,
        "gps_time_utc": gps_time_utc,
        "fix_mode": fix_mode,
        "satellites_visible": visible_satellites,
        "satellites_used": used_satellites,
        "satellite_prns": _join_satellite_field(satellites, "PRN"),
        "satellite_snr": _join_satellite_field(satellites, "ss"),
        "satellite_elevation": _join_satellite_field(satellites, "el"),
        "satellite_azimuth": _join_satellite_field(satellites, "az"),
        "satellite_used": _join_satellite_field(satellites, "used"),
        "receiver_holdover": receiver_holdover,
    }


_FIELD_RE = re.compile(r"^(?P<name>[^:]+?)\s*:\s*(?P<value>.+)$")


def _parse_float_field(value: str) -> float | None:
    """Extract the leading floating-point value from a chrony field string."""

    match = re.match(r"^[+-]?[0-9.]+(?:[eE][+-]?[0-9]+)?", value.strip())
    if match is None:
        return None
    try:
        return float(match.group(0))
    except ValueError:
        return None


def parse_chronyc_tracking(output: str) -> dict[str, Any]:
    """Parse `chronyc tracking` output into structured timing metadata.

    Args:
        output (str): Raw stdout from `chronyc tracking`.

    Returns:
        dict[str, Any]: Parsed tracking fields plus the raw text and a
        conservative `chrony_holdover` flag.
    """

    fields: dict[str, Any] = {"raw_output": output}
    for line in output.splitlines():
        match = _FIELD_RE.match(line.strip())
        if match is None:
            continue
        name = match.group("name").strip().lower().replace(" ", "_")
        value = match.group("value").strip()
        if name == "stratum":
            try:
                fields[name] = int(value)
            except ValueError:
                fields[name] = value
        elif name in {"root_delay", "root_dispersion", "last_offset", "rms_offset", "frequency", "residual_freq", "skew", "system_time"}:
            parsed_value = _parse_float_field(value)
            fields[name] = parsed_value if parsed_value is not None else value
        else:
            fields[name] = value

    leap_status = fields.get("leap_status")
    if isinstance(leap_status, str):
        chrony_holdover = leap_status.lower() != "normal"
    else:
        chrony_holdover = True

    fields["chrony_holdover"] = chrony_holdover
    return fields


def parse_chronyc_sources(output: str) -> dict[str, Any]:
    """Parse `chronyc sources` output and identify the selected source.

    Args:
        output (str): Raw stdout from `chronyc sources`.

    Returns:
        dict[str, Any]: Selected source metadata and parsed source rows.
    """

    sources: list[dict[str, Any]] = []
    selected_source = None
    selected_marker = None

    for line in output.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("MS "):
            continue
        parts = stripped.split()
        if len(parts) < 2:
            continue
        marker = parts[0]
        if marker[0] not in "#^=+?-":
            continue
        name = parts[1]
        row = {
            "marker": marker,
            "name": name,
            "raw": stripped,
            "reach": parts[4] if len(parts) > 4 else None,
            "last_rx_s": _parse_last_rx(parts[5]) if len(parts) > 5 else None,
            "sample": " ".join(parts[6:]) if len(parts) > 6 else None,
        }
        sources.append(row)
        if "*" in marker:
            selected_source = name
            selected_marker = marker

    return {
        "selected_source": selected_source,
        "selected_marker": selected_marker,
        "sources": sources,
    }


def parse_chronyc_sourcestats(output: str) -> dict[str, Any]:
    """Parse `chronyc sourcestats` output by source name.

    Args:
        output (str): Raw stdout from `chronyc sourcestats`.

    Returns:
        dict[str, Any]: Parsed rows keyed by source name, plus raw output.
    """

    rows: dict[str, dict[str, Any]] = {}
    for line in output.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("=") or stripped.startswith("Name/IP"):
            continue
        parts = stripped.split()
        if len(parts) < 8:
            continue
        try:
            rows[parts[0]] = {
                "np": int(parts[1]),
                "nr": int(parts[2]),
                "span_s": float(parts[3]),
                "frequency_ppm": float(parts[4]),
                "freq_skew_ppm": float(parts[5]),
                "offset": parts[6],
                "std_dev": parts[7],
                "raw": stripped,
            }
        except ValueError:
            rows[parts[0]] = {"raw": stripped}
    return {"sources": rows, "raw_output": output}


def read_kernel_pps(device: str = "/dev/pps0", sample_seconds: float = 3.0) -> dict[str, Any]:
    """Check whether the kernel PPS device emits assertion events.

    Args:
        device (str): Kernel PPS device path.
        sample_seconds (float): Maximum observation time, in seconds.

    Returns:
        dict[str, Any]: PPS observation status and raw `ppstest` output.
    """

    try:
        completed = subprocess.run(
            ["ppstest", device],
            check=False,
            capture_output=True,
            text=True,
            timeout=sample_seconds,
        )
        raw_output = completed.stdout or completed.stderr
        returncode = completed.returncode
    except subprocess.TimeoutExpired as exc:
        raw_output = _decode_process_output(exc.stdout) + _decode_process_output(exc.stderr)
        returncode = None
    except OSError as exc:
        raw_output = str(exc)
        returncode = None

    assert_count = len(re.findall(r"\bassert\b", raw_output))
    return {
        "device": device,
        "ok": assert_count > 0,
        "assert_count": assert_count,
        "returncode": returncode,
        "raw_output": raw_output,
    }


def _decode_process_output(value: Any) -> str:
    """Decode subprocess stdout/stderr values that may be bytes or text."""

    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def _parse_last_rx(value: str) -> int | None:
    """Parse chrony source LastRx field into seconds when possible."""

    if value == "-":
        return None
    if value.endswith("m"):
        try:
            return int(float(value[:-1]) * 60)
        except ValueError:
            return None
    if value.endswith("h"):
        try:
            return int(float(value[:-1]) * 3600)
        except ValueError:
            return None
    if value.endswith("d"):
        try:
            return int(float(value[:-1]) * 86400)
        except ValueError:
            return None
    try:
        return int(value)
    except ValueError:
        return None


def _source_by_name(chrony_sources: Mapping[str, Any], source_name: str) -> Mapping[str, Any]:
    """Return a parsed chrony source row by name."""

    sources = chrony_sources.get("sources")
    if not isinstance(sources, list):
        return {}
    for row in sources:
        if isinstance(row, Mapping) and row.get("name") == source_name:
            return row
    return {}


def build_record(
    timestamp_utc: datetime,
    gps_status: Mapping[str, Any],
    chrony_tracking: Mapping[str, Any],
    chrony_sources: Mapping[str, Any],
    chrony_sourcestats: Mapping[str, Any] | None = None,
    pps_status: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Combine GNSS and chrony status into one compact TSV record.

    Args:
        timestamp_utc (datetime): Sample timestamp in UTC.
        gps_status (Mapping[str, Any]): Parsed gpsd summary.
        chrony_tracking (Mapping[str, Any]): Parsed chronyc tracking data.
        chrony_sources (Mapping[str, Any]): Parsed chronyc sources data.
        chrony_sourcestats (Mapping[str, Any] | None): Parsed chronyc
            sourcestats data.
        pps_status (Mapping[str, Any] | None): Kernel PPS observation status.

    Returns:
        dict[str, Any]: TSV-ready logging record.
    """

    selected_source = chrony_sources.get("selected_source")
    selected_marker = chrony_sources.get("selected_marker")
    leap_status = chrony_tracking.get("leap_status")
    leap_is_normal = isinstance(leap_status, str) and leap_status.lower() == "normal"
    chrony_synced = leap_is_normal
    chrony_stratum = chrony_tracking.get("stratum")
    chrony_reference_id = chrony_tracking.get("reference_id")
    selected_source_is_gnss = isinstance(selected_source, str) and selected_source.upper() in {"GPS", "PPS"}
    selected_marker_is_active = isinstance(selected_marker, str) and "*" in selected_marker
    chrony_holdover = not (chrony_synced and selected_source_is_gnss and selected_marker_is_active)
    pps_source = _source_by_name(chrony_sources, "PPS")
    gps_source = _source_by_name(chrony_sources, "GPS")
    sourcestats_sources = chrony_sourcestats.get("sources", {}) if isinstance(chrony_sourcestats, Mapping) else {}
    pps_sourcestats = sourcestats_sources.get("PPS", {}) if isinstance(sourcestats_sources, Mapping) else {}

    record = {
        "timestamp_utc": isoformat_utc(timestamp_utc),
        "fix_mode": gps_status.get("fix_mode"),
        "satellites_visible": gps_status.get("satellites_visible", 0),
        "satellites_used": gps_status.get("satellites_used", 0),
        "satellite_prns": gps_status.get("satellite_prns"),
        "satellite_snr": gps_status.get("satellite_snr"),
        "satellite_elevation": gps_status.get("satellite_elevation"),
        "satellite_azimuth": gps_status.get("satellite_azimuth"),
        "satellite_used": gps_status.get("satellite_used"),
        "kernel_pps_ok": pps_status.get("ok") if isinstance(pps_status, Mapping) else None,
        "kernel_pps_assert_count": pps_status.get("assert_count") if isinstance(pps_status, Mapping) else None,
        "receiver_holdover": gps_status.get("receiver_holdover", True),
        "chrony_holdover": chrony_holdover,
        "chrony_synced": chrony_synced,
        "chrony_stratum": chrony_stratum,
        "chrony_leap_status": leap_status,
        "chrony_reference_id": chrony_reference_id,
        "chrony_selected_source": selected_source,
        "chrony_selected_marker": selected_marker,
        "chrony_pps_reach": pps_source.get("reach"),
        "chrony_pps_last_rx_s": pps_source.get("last_rx_s"),
        "chrony_pps_sample": pps_source.get("sample"),
        "chrony_gps_reach": gps_source.get("reach"),
        "chrony_gps_last_rx_s": gps_source.get("last_rx_s"),
        "pps_sourcestats_np": pps_sourcestats.get("np"),
        "pps_sourcestats_span_s": pps_sourcestats.get("span_s"),
        "pps_sourcestats_frequency_ppm": pps_sourcestats.get("frequency_ppm"),
        "pps_sourcestats_freq_skew_ppm": pps_sourcestats.get("freq_skew_ppm"),
        "pps_sourcestats_offset": pps_sourcestats.get("offset"),
        "pps_sourcestats_std_dev": pps_sourcestats.get("std_dev"),
    }
    return record


def _compact_record_state(record: Mapping[str, Any]) -> dict[str, Any]:
    """Extract the fields that define the logger's persistent compact state.

    Args:
        record (Mapping[str, Any]): A TSV-ready logging record.

    Returns:
        dict[str, Any]: Compact state used to detect transitions between
        samples. Values are JSON-serializable scalars.
    """

    return {
        "receiver_holdover": record.get("receiver_holdover"),
        "fix_mode": record.get("fix_mode"),
        "satellites_visible": record.get("satellites_visible"),
        "satellites_used": record.get("satellites_used"),
        "kernel_pps_ok": record.get("kernel_pps_ok"),
        "chrony_holdover": record.get("chrony_holdover"),
        "chrony_synced": record.get("chrony_synced"),
        "chrony_stratum": record.get("chrony_stratum"),
        "chrony_leap_status": record.get("chrony_leap_status"),
        "chrony_reference_id": record.get("chrony_reference_id"),
        "chrony_selected_source": record.get("chrony_selected_source"),
        "chrony_selected_marker": record.get("chrony_selected_marker"),
        "chrony_pps_reach": record.get("chrony_pps_reach"),
        "chrony_gps_reach": record.get("chrony_gps_reach"),
    }


def _state_file_path(output_dir: Path) -> Path:
    """Return the compact state file path inside the log directory."""

    return output_dir / STATE_FILE_NAME


def _event_log_path(timestamp_utc: datetime, output_dir: Path) -> Path:
    """Return the daily transition-event JSONL path for a UTC timestamp."""

    date_string = timestamp_utc.astimezone(timezone.utc).date().isoformat()
    return output_dir / f"{EVENT_LOG_PREFIX}{date_string}.jsonl"


def _load_previous_state(state_path: Path) -> dict[str, Any] | None:
    """Load the last compact state from disk if it exists."""

    try:
        raw_text = state_path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return None

    try:
        loaded_state = json.loads(raw_text)
    except json.JSONDecodeError:
        return None

    if isinstance(loaded_state, dict):
        return loaded_state
    return None


def _write_state_file(state_path: Path, compact_state: Mapping[str, Any]) -> None:
    """Persist the latest compact state for the next logger invocation."""

    state_path.parent.mkdir(parents=True, exist_ok=True)
    with state_path.open("w", encoding="utf-8") as stream:
        json.dump(compact_state, stream, indent=2, sort_keys=True)
        stream.write("\n")


def _build_state_change_event(
    timestamp_utc: datetime,
    gps_status: Mapping[str, Any],
    chrony_tracking: Mapping[str, Any],
    chrony_sources: Mapping[str, Any],
    previous_state: Mapping[str, Any] | None,
    current_state: Mapping[str, Any],
) -> dict[str, Any]:
    """Build a compact JSONL event describing a state transition.

    Args:
        timestamp_utc (datetime): Sample timestamp in UTC.
        gps_status (Mapping[str, Any]): Parsed gpsd summary for the sample.
        chrony_tracking (Mapping[str, Any]): Parsed chronyc tracking data.
        chrony_sources (Mapping[str, Any]): Parsed chronyc sources data.
        previous_state (Mapping[str, Any] | None): Previously persisted compact
            state, or `None` when no prior sample is available.
        current_state (Mapping[str, Any]): Compact state for the new sample.

    Returns:
        dict[str, Any]: JSON-serializable event record.
    """

    changed_fields: list[str] = []
    if previous_state is not None:
        for field_name in current_state:
            if previous_state.get(field_name) != current_state.get(field_name):
                changed_fields.append(field_name)

    return {
        "timestamp_utc": isoformat_utc(timestamp_utc),
        "event_type": "state_change",
        "changed_fields": changed_fields,
        "previous_state": previous_state,
        "current_state": dict(current_state),
        "gps_status": dict(gps_status),
        "chrony_tracking": dict(chrony_tracking),
        "chrony_sources": dict(chrony_sources),
    }


def write_state_change_event(
    timestamp_utc: datetime,
    gps_status: Mapping[str, Any],
    chrony_tracking: Mapping[str, Any],
    chrony_sources: Mapping[str, Any],
    current_record: Mapping[str, Any],
    output_dir: Path,
    state_path: Path | None = None,
) -> Path | None:
    """Append one event when the compact state changes, then persist state.

    Args:
        timestamp_utc (datetime): Sample timestamp in UTC.
        gps_status (Mapping[str, Any]): Parsed gpsd summary for the sample.
        chrony_tracking (Mapping[str, Any]): Parsed chronyc tracking data.
        chrony_sources (Mapping[str, Any]): Parsed chronyc sources data.
        current_record (Mapping[str, Any]): TSV-ready record for the sample.
        output_dir (Path): Directory where the daily event log should live.
        state_path (Path | None): Optional override for the persisted compact
            state file.

    Returns:
        Path | None: Event file path when a transition was appended, otherwise
        `None`.
    """

    output_dir.mkdir(parents=True, exist_ok=True)
    state_file = state_path or _state_file_path(output_dir)
    previous_state = _load_previous_state(state_file)
    current_state = _compact_record_state(current_record)

    event_path = None
    if previous_state is not None and previous_state != current_state:
        event_path = _event_log_path(timestamp_utc, output_dir)
        event_record = _build_state_change_event(
            timestamp_utc=timestamp_utc,
            gps_status=gps_status,
            chrony_tracking=chrony_tracking,
            chrony_sources=chrony_sources,
            previous_state=previous_state,
            current_state=current_state,
        )
        with event_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(event_record, sort_keys=True))
            stream.write("\n")

    _write_state_file(state_file, current_state)
    return event_path


def _timestamp_to_output_path(timestamp_utc: datetime, output_dir: Path) -> Path:
    """Map a UTC timestamp to the daily TSV file path for that date."""

    date_string = timestamp_utc.astimezone(timezone.utc).date().isoformat()
    return output_dir / f"gnss_m8t_status_{date_string}.tsv"


def write_record(record: Mapping[str, Any], output_dir: Path) -> Path:
    """Append one TSV record to the appropriate daily file.

    Args:
        record (Mapping[str, Any]): JSON-serializable logging record containing
            at least `timestamp_utc`.
        output_dir (Path): Directory where log files should be written.

    Returns:
        Path: The file that received the record.
    """

    timestamp_value = record.get("timestamp_utc")
    if not isinstance(timestamp_value, str):
        raise ValueError("record must contain a string timestamp_utc field")

    timestamp = datetime.fromisoformat(timestamp_value.replace("Z", "+00:00"))
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = _timestamp_to_output_path(timestamp, output_dir)
    with output_path.open("a", encoding="utf-8") as stream:
        if output_path.stat().st_size == 0:
            stream.write("\t".join(TSV_HEADER) + "\n")
        row = [str(record.get(column, "")) for column in TSV_HEADER]
        stream.write("\t".join(row) + "\n")
    return output_path


def collect_snapshot(
    gpsd_host: str = DEFAULT_GPSD_HOST,
    gpsd_port: int = DEFAULT_GPSD_PORT,
    sample_window_seconds: float = DEFAULT_SAMPLE_WINDOW_SECONDS,
    chronyc_binary: str = "chronyc",
) -> dict[str, Any]:
    """Collect one GNSS/chrony snapshot with raw parser outputs.

    Args:
        gpsd_host (str): gpsd host name or IP address.
        gpsd_port (int): gpsd TCP port.
        sample_window_seconds (float): gpsd sampling window in seconds.
        chronyc_binary (str): Chrony client executable.

    Returns:
        dict[str, Any]: Snapshot bundle containing the compact record plus the
        parsed gpsd and chrony payloads used to build it.
    """

    timestamp_utc = utc_now()
    gps_messages = read_gpsd_messages(
        host=gpsd_host,
        port=gpsd_port,
        sample_window_seconds=sample_window_seconds,
    )
    gps_status = summarize_gpsd_messages(gps_messages)

    tracking_output = subprocess.run(
        [chronyc_binary, "-n", "tracking"],
        check=False,
        capture_output=True,
        text=True,
    )
    sources_output = subprocess.run(
        [chronyc_binary, "-n", "sources", "-v"],
        check=False,
        capture_output=True,
        text=True,
    )
    sourcestats_output = subprocess.run(
        [chronyc_binary, "-n", "sourcestats"],
        check=False,
        capture_output=True,
        text=True,
    )
    pps_status = read_kernel_pps()

    chrony_tracking = parse_chronyc_tracking(tracking_output.stdout if tracking_output.stdout else tracking_output.stderr)
    chrony_tracking["returncode"] = tracking_output.returncode
    chrony_sources = parse_chronyc_sources(sources_output.stdout if sources_output.stdout else sources_output.stderr)
    chrony_sources["returncode"] = sources_output.returncode
    chrony_sourcestats = parse_chronyc_sourcestats(
        sourcestats_output.stdout if sourcestats_output.stdout else sourcestats_output.stderr
    )
    chrony_sourcestats["returncode"] = sourcestats_output.returncode

    record = build_record(
        timestamp_utc=timestamp_utc,
        gps_status=gps_status,
        chrony_tracking=chrony_tracking,
        chrony_sources=chrony_sources,
        chrony_sourcestats=chrony_sourcestats,
        pps_status=pps_status,
    )
    return {
        "timestamp_utc": timestamp_utc,
        "record": record,
        "gps_status": gps_status,
        "chrony_tracking": chrony_tracking,
        "chrony_sources": chrony_sources,
        "chrony_sourcestats": chrony_sourcestats,
        "pps_status": pps_status,
    }


def collect_once(
    gpsd_host: str = DEFAULT_GPSD_HOST,
    gpsd_port: int = DEFAULT_GPSD_PORT,
    sample_window_seconds: float = DEFAULT_SAMPLE_WINDOW_SECONDS,
    chronyc_binary: str = "chronyc",
) -> dict[str, Any]:
    """Collect one compact GNSS/chrony snapshot.

    Args:
        gpsd_host (str): gpsd host name or IP address.
        gpsd_port (int): gpsd TCP port.
        sample_window_seconds (float): gpsd sampling window in seconds.
        chronyc_binary (str): Chrony client executable.

    Returns:
        dict[str, Any]: Compact TSV-ready logging record.
    """

    snapshot = collect_snapshot(
        gpsd_host=gpsd_host,
        gpsd_port=gpsd_port,
        sample_window_seconds=sample_window_seconds,
        chronyc_binary=chronyc_binary,
    )
    return snapshot["record"]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments for the logger.

    Args:
        argv (list[str] | None): Optional argument vector; defaults to sys.argv.

    Returns:
        argparse.Namespace: Parsed command-line arguments.
    """

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory where TSV log files are written.",
    )
    parser.add_argument(
        "--gpsd-host",
        default=DEFAULT_GPSD_HOST,
        help="gpsd host name or IP address.",
    )
    parser.add_argument(
        "--gpsd-port",
        type=int,
        default=DEFAULT_GPSD_PORT,
        help="gpsd TCP port.",
    )
    parser.add_argument(
        "--sample-window-seconds",
        type=float,
        default=DEFAULT_SAMPLE_WINDOW_SECONDS,
        help="Duration to watch gpsd for TPV/SKY updates.",
    )
    parser.add_argument(
        "--chronyc-binary",
        default="chronyc",
        help="Path to the chronyc executable.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Run one collection pass and append it to disk.

    Args:
        argv (list[str] | None): Optional CLI argument vector.

    Returns:
        int: Process exit status code.
    """

    args = parse_args(argv)
    try:
        snapshot = collect_snapshot(
            gpsd_host=args.gpsd_host,
            gpsd_port=args.gpsd_port,
            sample_window_seconds=args.sample_window_seconds,
            chronyc_binary=args.chronyc_binary,
        )
        record = snapshot["record"]
        output_path = write_record(record, args.output_dir)
        event_path = write_state_change_event(
            timestamp_utc=snapshot["timestamp_utc"],
            gps_status=snapshot["gps_status"],
            chrony_tracking=snapshot["chrony_tracking"],
            chrony_sources=snapshot["chrony_sources"],
            current_record=record,
            output_dir=args.output_dir,
        )
    except Exception as exc:  # pragma: no cover - exercised in integration
        print(f"GNSS logger failed: {exc}", file=sys.stderr)
        return 1

    if event_path is not None:
        print(f"wrote {output_path}; event {event_path}")
    else:
        print(f"wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
