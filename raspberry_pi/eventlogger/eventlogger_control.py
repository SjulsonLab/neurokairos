#!/usr/bin/env python3
"""NeuroKairos event logger control service and export CLI.

This module provides the localhost/LAN HTTP control surface used by the
temporary event logger test UI. It also contains the export logic shared by the
service and the manual CLI entry point.
"""

from __future__ import annotations

import argparse
import calendar
from datetime import datetime, timedelta, timezone, tzinfo
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import threading
from typing import Callable
from urllib.parse import urlparse


SCRIPT_DIR = Path(__file__).resolve().parent
WEB_DIR = SCRIPT_DIR / "web"
DEFAULT_ROOT_DIR = Path("/var/lib/neurokairos/eventlogger")
ACTIVE_RECORDING_FILE = "active_recording.json"
INPUT_STATE_FILE = "inputs.json"


class ControlError(RuntimeError):
    """Raised when a control action is invalid for the current state."""


VALID_BASENAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")


def normalize_basename(value: str | None) -> str:
    """Normalize user-provided basenames, defaulting empty values to 'events'."""

    if value is None:
        return "events"
    trimmed = value.strip()
    return trimmed or "events"


def validate_basename(value: str) -> str:
    """Reject basenames that are unsafe or awkward for cross-platform filenames."""

    if not VALID_BASENAME_RE.fullmatch(value):
        raise ControlError("basename must start with a letter or digit and contain only letters, digits, '.', '-', and '_'")
    return value


def normalize_notes(value: str | None) -> str:
    """Normalize optional user notes for storage and export."""

    if value is None:
        return ""
    return value.strip()


def normalize_user(value: str | None) -> str:
    """Normalize optional user name, defaulting empty values to 'anonymous'."""

    if value is None:
        return "anonymous"
    trimmed = value.strip()
    return trimmed or "anonymous"


def utc_ns_from_datetime(value: datetime) -> int:
    """Convert an aware UTC datetime to integer nanoseconds since Unix epoch."""

    if value.tzinfo is None:
        raise ValueError("datetime must be timezone-aware")
    value_utc = value.astimezone(timezone.utc)
    return calendar.timegm(value_utc.utctimetuple()) * 1_000_000_000 + value_utc.microsecond * 1000


def format_utc_iso(value: datetime) -> str:
    """Format a UTC datetime as an RFC3339 timestamp with microsecond precision."""

    return value.astimezone(timezone.utc).isoformat(timespec="microseconds").replace("+00:00", "Z")


def parse_utc_iso_to_ns(text: str) -> int:
    """Parse a UTC RFC3339 timestamp with up to nanosecond fractional precision."""

    if not text.endswith("Z"):
        raise ValueError(f"expected UTC timestamp with Z suffix: {text}")
    body = text[:-1]
    if "." in body:
        main, frac = body.split(".", 1)
    else:
        main, frac = body, ""
    dt = datetime.strptime(main, "%Y-%m-%dT%H:%M:%S").replace(tzinfo=timezone.utc)
    frac_digits = "".join(ch for ch in frac if ch.isdigit())[:9].ljust(9, "0")
    return calendar.timegm(dt.utctimetuple()) * 1_000_000_000 + int(frac_digits or "0")


def yaml_bool(value: bool) -> str:
    return "true" if value else "false"


def yaml_string(value: str) -> str:
    return json.dumps(value, ensure_ascii=True)


def yaml_block_string(value: str) -> list[str]:
    if not value:
        return ['notes: ""']
    lines = ["notes: |-"]
    for line in value.splitlines():
        lines.append(f"  {line}")
    return lines


def yaml_named_block_string(name: str, value: str) -> list[str]:
    if not value:
        return [f'{name}: ""']
    lines = [f"{name}: |-"]
    for line in value.splitlines():
        lines.append(f"  {line}")
    return lines


def parse_bool_jsonish(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).lower() == "true"


def load_json(path: Path) -> dict | None:
    if not path.exists():
        return None
    return json.loads(path.read_text())


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(text)
    os.replace(tmp_path, path)


def choose_export_stem(root_dir: Path, basename: str, start_local: str) -> str:
    """Choose a collision-free local-time stem without changing the true start time."""

    recordings_dir = root_dir / "recordings"
    recordings_dir.mkdir(parents=True, exist_ok=True)
    local_dt = datetime.fromisoformat(start_local)
    while True:
        stem = f"{basename}_{local_dt.strftime('%Y-%m-%d_%H%M%S')}"
        tsv_path = recordings_dir / f"{stem}.tsv"
        yaml_path = recordings_dir / f"{stem}.yaml"
        if not tsv_path.exists() and not yaml_path.exists():
            return stem
        local_dt += timedelta(seconds=1)


def recording_filename(recording: dict | None) -> str | None:
    """Return the user-facing TSV filename for a recording dict, if known.

    Inputs:
        recording: dict | None recording metadata. May contain ``export_stem`` or
            enough fields to derive one from ``basename`` and ``start_local``.

    Returns:
        str | None: expected TSV filename, including ``.tsv``, or ``None`` when
        no filename can be determined.
    """

    if not recording:
        return None
    stem = recording.get("export_stem")
    if stem:
        return f"{stem}.tsv"
    basename = recording.get("basename")
    start_local = recording.get("start_local")
    if basename and start_local:
        local_dt = datetime.fromisoformat(start_local)
        return f"{basename}_{local_dt.strftime('%Y-%m-%d_%H%M%S')}.tsv"
    return None


def iter_event_rows(root_dir: Path, start_ns: int, stop_ns: int):
    """Yield journal rows whose UTC timestamps fall inside the requested interval.

    Inputs:
        root_dir: Path to the event logger root directory.
        start_ns: int nanoseconds since Unix epoch, UTC inclusive lower bound.
        stop_ns: int nanoseconds since Unix epoch, UTC inclusive upper bound.

    Yields:
        str rows from the raw journal TSV, including all raw columns.
    """

    journal_dir = root_dir / "journal"
    if not journal_dir.exists():
        return
    for path in sorted(journal_dir.glob("events_*.tsv")):
        with path.open("r", encoding="utf-8") as handle:
            header = handle.readline()
            if not header:
                continue
            for line in handle:
                parts = line.rstrip("\n").split("\t")
                if len(parts) != 5:
                    continue
                event_ns = parse_utc_iso_to_ns(parts[0])
                if start_ns <= event_ns <= stop_ns:
                    yield line


def simplify_event_row(raw_row: str) -> str | None:
    """Convert one raw journal row into the user-facing recording TSV schema.

    Inputs:
        raw_row: str raw journal line with tab-separated columns
            ``utc_time``, ``realtime_ns``, ``monotonic_ns``, ``input``, ``edge``.

    Returns:
        str | None: simplified TSV row containing ``UTC_time``, ``input``, and
        ``edge`` only, or ``None`` if the row is malformed.
    """

    parts = raw_row.rstrip("\n").split("\t")
    if len(parts) != 5:
        return None
    return f"{parts[0]}\t{parts[3]}\t{parts[4]}\n"


def write_metadata_yaml(path: Path, recording: dict, hostname: str) -> None:
    lines = [
        f"basename: {yaml_string(recording['basename'])}",
        f"user: {yaml_string(normalize_user(recording.get('user')))}",
        f"hostname: {yaml_string(hostname)}",
        f"start_utc: {yaml_string(recording['start_utc'])}",
        f"stop_utc: {yaml_string(recording['stop_utc'])}",
        f"start_local: {yaml_string(recording['start_local'])}",
        f"timezone: {yaml_string(recording['timezone'])}",
        "inputs:",
    ]
    for name in recording.get("exported_inputs", []):
        lines.append(f"  - {yaml_string(name)}")
    lines.extend(yaml_named_block_string("notes", recording.get("notes", "")))
    lines.append(f"interrupted: {yaml_bool(parse_bool_jsonish(recording['interrupted']))}")
    path.write_text("\n".join(lines) + "\n")


def export_recording(root_dir: Path, recording: dict, hostname: str) -> dict:
    """Export one recording interval into a user-facing TSV plus YAML metadata.

    Inputs:
        root_dir: Path to the event logger root directory.
        recording: dict with UTC/local recording bounds and metadata. ``start_ns``
            and ``stop_ns`` are integer nanoseconds since Unix epoch when present;
            otherwise ``start_utc`` and ``stop_utc`` are RFC3339 UTC timestamps.
        hostname: str host name to store in the YAML sidecar.

    Returns:
        dict with keys ``recording``, ``tsv_path``, and ``yaml_path``.
    """

    root_dir = Path(root_dir)
    start_ns = int(recording.get("start_ns", parse_utc_iso_to_ns(recording["start_utc"])))
    stop_ns = int(recording.get("stop_ns", parse_utc_iso_to_ns(recording["stop_utc"])))
    stem = recording.get("export_stem") or choose_export_stem(root_dir, recording["basename"], recording["start_local"])
    recordings_dir = root_dir / "recordings"
    recordings_dir.mkdir(parents=True, exist_ok=True)
    tsv_path = recordings_dir / f"{stem}.tsv"
    yaml_path = recordings_dir / f"{stem}.yaml"

    with tsv_path.open("w", encoding="utf-8") as handle:
        handle.write("UTC_time\tinput\tedge\n")
        for row in iter_event_rows(root_dir, start_ns, stop_ns):
            simplified_row = simplify_event_row(row)
            if simplified_row is not None:
                handle.write(simplified_row)

    write_metadata_yaml(yaml_path, recording, hostname)
    return {
        "recording": recording,
        "tsv_path": str(tsv_path),
        "yaml_path": str(yaml_path),
    }


def recover_interrupted_recording(root_dir: Path, recovery_time: datetime, hostname: str) -> dict | None:
    """Finalize an interrupted recording if active state exists at startup."""

    active_path = Path(root_dir) / "control" / ACTIVE_RECORDING_FILE
    active = load_json(active_path)
    if not active:
        return None

    active["stop_utc"] = format_utc_iso(recovery_time)
    active["stop_ns"] = utc_ns_from_datetime(recovery_time)
    active["interrupted"] = True
    result = export_recording(Path(root_dir), active, hostname)
    active_path.unlink(missing_ok=True)
    return result


def read_inputs_state(root_dir: Path) -> list[dict]:
    """Read the input status snapshot written by the C event logger daemon."""

    path = Path(root_dir) / "control" / INPUT_STATE_FILE
    data = load_json(path)
    if not data:
        return []
    return list(data.get("inputs", []))


class EventLoggerControlApp:
    def __init__(
        self,
        root_dir: Path,
        hostname: str,
        now_utc: Callable[[], datetime] | None = None,
        local_timezone_name: str | None = None,
        local_timezone: tzinfo | None = None,
        capture_service_name: str = "neurokairos-eventlogger.service",
    ) -> None:
        self.root_dir = Path(root_dir)
        self.hostname = hostname
        self.now_utc = now_utc or (lambda: datetime.now(timezone.utc))
        self.local_timezone = local_timezone or datetime.now().astimezone().tzinfo or timezone.utc
        self.local_timezone_name = local_timezone_name or datetime.now().astimezone().tzname() or "UTC"
        self.capture_service_name = capture_service_name
        (self.root_dir / "recordings").mkdir(parents=True, exist_ok=True)
        (self.root_dir / "control").mkdir(parents=True, exist_ok=True)

    @property
    def active_path(self) -> Path:
        return self.root_dir / "control" / ACTIVE_RECORDING_FILE

    def capture_service_active(self) -> bool:
        try:
            result = subprocess.run(
                ["systemctl", "is-active", "--quiet", self.capture_service_name],
                check=False,
            )
        except FileNotFoundError:
            return False
        return result.returncode == 0

    def get_active_recording(self) -> dict | None:
        return load_json(self.active_path)

    def set_active_recording(self, recording: dict) -> None:
        atomic_write_text(self.active_path, json.dumps(recording, indent=2, sort_keys=True))

    def clear_active_recording(self) -> None:
        self.active_path.unlink(missing_ok=True)

    def build_status(self) -> dict:
        active = self.get_active_recording()
        return {
            "capture_service_running": self.capture_service_active(),
            "control_service_running": True,
            "active_recording": {
                "active": active is not None,
                "basename": active.get("basename") if active else None,
                "user": active.get("user") if active else "anonymous",
                "notes": active.get("notes") if active else "",
                "start_utc": active.get("start_utc") if active else None,
                "start_local": active.get("start_local") if active else None,
                "timezone": active.get("timezone") if active else None,
                "filename": recording_filename(active),
                "input_rising_baselines": dict(active.get("input_rising_baselines", {})) if active else {},
            },
            "paths": {
                "journal_dir": str(self.root_dir / "journal"),
                "recordings_dir": str(self.root_dir / "recordings"),
            },
            "inputs": read_inputs_state(self.root_dir),
        }

    def start_recording(self, basename: str | None, user: str | None = None, notes: str | None = None) -> dict:
        if self.get_active_recording() is not None:
            raise ControlError("recording already active")

        now = self.now_utc()
        start_ns = utc_ns_from_datetime(now)
        basename_normalized = validate_basename(normalize_basename(basename))
        user_normalized = normalize_user(user)
        notes_normalized = normalize_notes(notes)
        local_start = now.astimezone(self.local_timezone)
        inputs_state = read_inputs_state(self.root_dir)
        recording = {
            "basename": basename_normalized,
            "user": user_normalized,
            "notes": notes_normalized,
            "start_utc": format_utc_iso(now),
            "start_ns": start_ns,
            "start_local": local_start.replace(tzinfo=None).isoformat(timespec="seconds"),
            "timezone": self.local_timezone_name,
            "export_stem": choose_export_stem(
                self.root_dir,
                basename_normalized,
                local_start.replace(tzinfo=None).isoformat(timespec="seconds"),
            ),
            "input_rising_baselines": {
                entry["name"]: int(entry.get("rising_count", 0))
                for entry in inputs_state
            },
            "exported_inputs": [
                entry["name"]
                for entry in inputs_state
                if parse_bool_jsonish(entry.get("enabled", True))
            ],
            "interrupted": False,
        }
        self.set_active_recording(recording)
        return recording

    def stop_recording(self, user: str | None = None, notes: str | None = None) -> dict:
        recording = self.get_active_recording()
        if recording is None:
            raise ControlError("no active recording")

        if user is not None:
            recording["user"] = normalize_user(user)
        if notes is not None:
            recording["notes"] = normalize_notes(notes)
        now = self.now_utc()
        recording["stop_utc"] = format_utc_iso(now)
        recording["stop_ns"] = utc_ns_from_datetime(now)
        result = export_recording(self.root_dir, recording, self.hostname)
        self.clear_active_recording()
        return result


def build_ui_html() -> str:
    return (WEB_DIR / "index.html").read_text(encoding="utf-8")


def make_http_server(host: str, port: int, app: EventLoggerControlApp) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def _send_json(self, payload: dict, status: int = 200) -> None:
            body = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_text(self, body: str, content_type: str = "text/html; charset=utf-8") -> None:
            encoded = body.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)

        def _read_json(self) -> dict:
            content_length = int(self.headers.get("Content-Length", "0"))
            if content_length == 0:
                return {}
            return json.loads(self.rfile.read(content_length).decode("utf-8"))

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path == "/" or path == "/index.html":
                self._send_text(build_ui_html())
                return
            if path == "/health":
                self._send_json({"ok": True})
                return
            if path == "/v1/status":
                self._send_json(app.build_status())
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            try:
                if path == "/v1/recordings/start":
                    payload = self._read_json()
                    recording = app.start_recording(payload.get("basename"), payload.get("user"), payload.get("notes"))
                    self._send_json({"recording": recording})
                    return
                if path == "/v1/recordings/stop":
                    payload = self._read_json()
                    result = app.stop_recording(payload.get("user"), payload.get("notes"))
                    self._send_json(result)
                    return
                self.send_error(HTTPStatus.NOT_FOUND)
            except ControlError as exc:
                self._send_json({"error": str(exc)}, status=400)

        def log_message(self, format: str, *args) -> None:  # noqa: A003
            return

    return ThreadingHTTPServer((host, port), Handler)


def run_server(root_dir: Path, host: str, port: int) -> None:
    app = EventLoggerControlApp(root_dir=root_dir, hostname=socket.gethostname())
    recover_interrupted_recording(root_dir, app.now_utc(), app.hostname)
    httpd = make_http_server(host, port, app)
    shutdown_lock = threading.Lock()
    finalized = {"done": False}

    def finalize_active_recording() -> None:
        with shutdown_lock:
            if finalized["done"]:
                return
            active = app.get_active_recording()
            if active is not None:
                now = app.now_utc()
                active["stop_utc"] = format_utc_iso(now)
                active["stop_ns"] = utc_ns_from_datetime(now)
                active["interrupted"] = True
                export_recording(root_dir, active, app.hostname)
                app.clear_active_recording()
            finalized["done"] = True

    def handle_signal(signum, frame) -> None:  # noqa: ARG001
        finalize_active_recording()
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)
    try:
        httpd.serve_forever()
    finally:
        finalize_active_recording()
        httpd.server_close()


def main() -> int:
    parser = argparse.ArgumentParser(description="NeuroKairos event logger control service")
    parser.add_argument("--root-dir", default=str(DEFAULT_ROOT_DIR))
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=80)
    args = parser.parse_args()
    run_server(Path(args.root_dir), args.host, args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
