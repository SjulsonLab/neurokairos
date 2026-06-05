#!/usr/bin/env python3
"""Collect and normalize TM2000B log information.

The TM2000B exposes two useful sources of information:

* a web-accessible archive of system logs
* a live status page plus a satellite feed (`status.cgi` and `sat.cgi`)

This collector downloads the archive, extracts only relevant state-change
events, scrapes the live status endpoints, and writes two compact TSV logs on
the Raspberry Pi:

* a deduplicated event log
* a periodic satellite-tracking log

Raw snapshots are intentionally not retained.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import html
import io
import json
import re
import sys
import tarfile
import time
import urllib.parse
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Optional, Sequence


DEFAULT_OUTPUT_ROOT = Path("/home/pi/timemachine_logs")
DEFAULT_EVENTS_NAME = "tm2000b_events.tsv"
DEFAULT_TRACKING_NAME = "tm2000b_satellite_tracking.tsv"

_DATE_RE = re.compile(
    r"^(?P<month>[A-Z][a-z]{2})\s+"
    r"(?P<day>\d{1,2})\s+"
    r"(?P<hour>\d{2}):(?P<minute>\d{2}):(?P<second>\d{2})"
)
_EVENT_PATTERNS = [
    (re.compile(r"Booting Linux", re.IGNORECASE), "boot"),
    (re.compile(r"Fix State\s+1\s*->\s*0", re.IGNORECASE), "holdover_start"),
    (re.compile(r"Fix State\s+0\s*->\s*1", re.IGNORECASE), "holdover_end"),
    (re.compile(r"\bGPS\b.*\b(?:lost|lock|holdover|fix)\b", re.IGNORECASE), "gps_state_change"),
]
_STATUS_FIELDS = (
    "time_source",
    "location",
    "status_time_utc",
    "satellites_used",
    "gps_fix",
    "ntp_lookups",
    "holdover_time",
    "mac",
    "uptime",
    "ocxo_correction",
    "version",
)
_CHANGE_FIELDS = ("time_source", "gps_fix", "satellites_used")
_EVENT_FIELDNAMES = (
    "event_key",
    "logged_at_utc",
    "event_timestamp_utc",
    "event_type",
    "source",
    "detail",
    "raw_line",
)
_TRACKING_FIELDNAMES = (
    "collected_at_utc",
    "status_time_utc",
    "time_source",
    "location",
    "satellites_used",
    "gps_fix",
    "ntp_lookups",
    "holdover_time",
    "mac",
    "uptime",
    "ocxo_correction",
    "version",
    "satellite_count",
    "satellite_ids",
    "satellite_snr",
    "satellite_azimuth",
    "satellite_elevation",
)


@dataclass(frozen=True)
class CollectorConfig:
    """Configuration for one collection run.

    Inputs
    ------
    base_url:
        Root URL of the TM2000B web interface, for example
        ``"http://10.49.98.251"``.
    about_url:
        Authenticated page used to trigger the log download.
    username:
        HTTP digest authentication username.
    password:
        HTTP digest authentication password.
    output_root:
        Directory where the consolidated TSV logs are stored.
    events_name:
        Filename for the deduplicated event TSV.
    tracking_name:
        Filename for the periodic satellite-tracking TSV.
    download_action:
        Relative path invoked by the download button.
    archive_name:
        Name of the downloadable tarball exposed by the appliance.
    timeout_s:
        Per-request timeout in seconds.
    poll_delay_s:
        Delay between archive polling attempts in seconds.
    max_poll_attempts:
        Maximum number of archive fetch attempts after the trigger POST.

    Returns
    -------
    CollectorConfig
        Frozen dataclass holding the supplied parameters.
    """

    base_url: str
    about_url: str
    username: str
    password: str
    output_root: Path
    events_name: str = DEFAULT_EVENTS_NAME
    tracking_name: str = DEFAULT_TRACKING_NAME
    download_action: str = "download_logs.cgi"
    archive_name: str = "logs.tar.gz"
    timeout_s: float = 10.0
    poll_delay_s: float = 1.0
    max_poll_attempts: int = 10


RUNNING = True


def stop(_signum, _frame):
    """Request graceful termination on SIGTERM or SIGINT."""

    global RUNNING
    RUNNING = False


def utc_now() -> datetime:
    """Return the current UTC time as a timezone-aware datetime."""

    return datetime.now(timezone.utc)


def iso_utc(value: Optional[datetime] = None) -> str:
    """Format a UTC datetime as compact ISO-8601 text.

    Inputs
    ------
    value:
        Timezone-aware datetime in UTC. If ``None``, the current UTC time is
        used.

    Returns
    -------
    str
        ISO-8601 string with a trailing ``Z``.
    """

    current = value or utc_now()
    return current.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")


def first_float(text: Optional[str]) -> Optional[float]:
    """Return the first floating-point number embedded in a string."""

    if not text:
        return None
    match = re.search(r"[+-]?[0-9]+(?:\.[0-9]*)?(?:[eE][+-]?[0-9]+)?", text)
    if not match:
        return None
    try:
        return float(match.group(0))
    except ValueError:
        return None


def normalize_whitespace(text: str) -> str:
    """Collapse repeated whitespace and trim the ends."""

    return re.sub(r"\s+", " ", text).strip()


def hash_key(parts: Iterable[Optional[str]]) -> str:
    """Create a stable deduplication key from a tuple of text fields."""

    payload = "\0".join("" if part is None else str(part) for part in parts)
    return hashlib.sha1(payload.encode("utf-8")).hexdigest()


def build_auth_opener(base_url: str, username: str, password: str) -> urllib.request.OpenerDirector:
    """Create a digest-authenticated opener for the TM2000B web interface."""

    password_mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()
    password_mgr.add_password(None, base_url, username, password)
    handler = urllib.request.HTTPDigestAuthHandler(password_mgr)
    return urllib.request.build_opener(handler)


def fetch_text(opener: object, url: str, timeout_s: float) -> str:
    """Fetch a UTF-8 text resource over HTTP.

    Inputs
    ------
    opener:
        Object with an ``open(request, timeout=...)`` method.
    url:
        Absolute URL to fetch.
    timeout_s:
        Request timeout in seconds.

    Returns
    -------
    str
        Response body decoded as UTF-8 with replacement for malformed bytes.
    """

    request = urllib.request.Request(url, method="GET")
    with opener.open(request, timeout=timeout_s) as response:
        return response.read().decode("utf-8", errors="replace")


def download_log_bundle(
    *,
    opener: object,
    about_url: str,
    download_action: str,
    archive_name: str,
    timeout_s: float,
    poll_delay_s: float,
    max_poll_attempts: int,
) -> bytes:
    """Trigger the server download action and fetch the archive.

    Inputs
    ------
    opener:
        Object with an ``open(request, timeout=...)`` method, typically a
        ``urllib`` opener.
    about_url:
        URL of the authenticated page that exposes the download action.
    download_action:
        Relative CGI path invoked by the download control.
    archive_name:
        Name of the archive file exposed after the download action runs.
    timeout_s:
        Request timeout in seconds.
    poll_delay_s:
        Seconds to wait between archive fetch attempts.
    max_poll_attempts:
        Maximum number of archive fetch attempts.

    Returns
    -------
    bytes
        Raw contents of the downloaded archive.
    """

    about_parts = urllib.parse.urlsplit(about_url)
    download_url = urllib.parse.urlunsplit(
        (
            about_parts.scheme,
            about_parts.netloc,
            urllib.parse.urljoin(about_parts.path, download_action),
            "",
            "",
        )
    )
    archive_url = urllib.parse.urlunsplit(
        (
            about_parts.scheme,
            about_parts.netloc,
            urllib.parse.urljoin(about_parts.path, archive_name),
            "",
            "",
        )
    )

    trigger_request = urllib.request.Request(download_url, data=b"", method="POST")
    with opener.open(trigger_request, timeout=timeout_s) as response:
        response.read()

    last_error = None
    for attempt in range(1, max_poll_attempts + 1):
        fetch_request = urllib.request.Request(archive_url, method="GET")
        try:
            with opener.open(fetch_request, timeout=timeout_s) as response:
                archive_bytes = response.read()
                if archive_bytes:
                    return archive_bytes
        except Exception as exc:  # pragma: no cover - runtime network concerns.
            last_error = exc
        if attempt < max_poll_attempts and poll_delay_s > 0:
            time.sleep(poll_delay_s)

    if last_error is not None:
        raise RuntimeError(f"failed to download archive from {archive_url}") from last_error
    raise RuntimeError(f"archive {archive_name!r} was empty after {max_poll_attempts} attempts")


def parse_status_page(html_text: str) -> dict[str, str]:
    """Parse the TM2000B status page into a flat dictionary.

    Inputs
    ------
    html_text:
        Raw HTML from ``status.cgi``.

    Returns
    -------
    dict[str, str]
        Normalized status fields from the page.
    """

    fields = {
        "time_source": "",
        "location": "",
        "status_time_utc": "",
        "satellites_used": "",
        "gps_fix": "",
        "ntp_lookups": "",
        "holdover_time": "",
        "mac": "",
        "uptime": "",
        "ocxo_correction": "",
        "version": "",
    }
    for label, key in (
        ("Time Source", "time_source"),
        ("Location", "location"),
        ("Date/Time", "status_time_utc"),
        ("Satellites used", "satellites_used"),
        ("GPS fix", "gps_fix"),
        ("NTP Lookups", "ntp_lookups"),
        ("Holdover Time", "holdover_time"),
        ("MAC", "mac"),
        ("Uptime", "uptime"),
        ("OCXO Correction", "ocxo_correction"),
        ("Version", "version"),
    ):
        raw = _extract_status_cell(html_text, label)
        if key == "status_time_utc":
            fields[key] = _parse_status_datetime(raw)
        elif key == "ocxo_correction":
            fields[key] = _extract_ocxo_correction(raw)
        else:
            fields[key] = raw
    return fields


def parse_satellite_feed(csv_text: str) -> dict[str, str]:
    """Parse the TM2000B satellite feed into compact comma-delimited columns.

    Inputs
    ------
    csv_text:
        Raw text from ``sat.cgi``. The feed contains four comma-separated
        lines: satellite IDs, SNR values, elevation values, and azimuth values.

    Returns
    -------
    dict[str, str]
        Satellite arrays rendered as CSV text for the tracking TSV.
    """

    lines = [line.strip() for line in csv_text.splitlines() if line.strip()]
    if len(lines) < 4:
        return {
            "satellite_count": "0",
            "satellite_ids": "",
            "satellite_snr": "",
            "satellite_azimuth": "",
            "satellite_elevation": "",
        }

    ids = [part.strip() for part in lines[0].split(",")]
    snr = [part.strip() for part in lines[1].split(",")]
    elevation = [part.strip() for part in lines[2].split(",")]
    azimuth = [part.strip() for part in lines[3].split(",")]

    satellites = []
    for satellite_id, snr_value, elevation_value, azimuth_value in zip(ids, snr, elevation, azimuth):
        satellites.append(
            {
                "id": satellite_id,
                "snr": snr_value,
                "elevation": elevation_value,
                "azimuth": azimuth_value,
                "snr_sort": first_float(snr_value) if first_float(snr_value) is not None else float("-inf"),
            }
        )

    satellites.sort(key=lambda item: item["snr_sort"], reverse=True)
    return {
        "satellite_count": str(len(satellites)),
        "satellite_ids": ",".join(item["id"] for item in satellites),
        "satellite_snr": ",".join(item["snr"] for item in satellites),
        "satellite_azimuth": ",".join(item["azimuth"] for item in satellites),
        "satellite_elevation": ",".join(item["elevation"] for item in satellites),
    }


def extract_relevant_events(archive_bytes: bytes) -> list[dict[str, str]]:
    """Extract a small set of relevant events from the downloaded archive.

    Inputs
    ------
    archive_bytes:
        Raw gzipped tar archive downloaded from the TM2000B.

    Returns
    -------
    list[dict[str, str]]
        Event dictionaries ready to append to the event TSV.
    """

    events: list[dict[str, str]] = []
    with tarfile.open(fileobj=io.BytesIO(archive_bytes), mode="r:gz") as archive:
        for member in archive.getmembers():
            if not member.isfile():
                continue
            if not Path(member.name).name.startswith("messages"):
                continue
            fileobj = archive.extractfile(member)
            if fileobj is None:
                continue
            for raw_line in fileobj.read().decode("utf-8", errors="replace").splitlines():
                event = _parse_archive_event_line(member.name, raw_line)
                if event is not None:
                    events.append(event)
    return events


def build_status_change_events(current: dict[str, str], previous: Optional[dict[str, str]], *, logged_at_utc: str) -> list[dict[str, str]]:
    """Create event rows for changes in the live status snapshot.

    Inputs
    ------
    current:
        Current status/tracking sample.
    previous:
        Previous tracking sample, or ``None`` if this is the first run.
    logged_at_utc:
        Timestamp when the collector wrote the current sample.

    Returns
    -------
    list[dict[str, str]]
        Event rows describing the changed fields.
    """

    if previous is None:
        return []

    changes = []
    for field in _CHANGE_FIELDS:
        old_value = previous.get(field, "")
        new_value = current.get(field, "")
        if old_value != new_value:
            changes.append(f"{field}: {old_value or '(blank)'} -> {new_value or '(blank)'}")

    if not changes:
        return []

    event_time = current.get("status_time_utc") or logged_at_utc
    detail = "; ".join(changes)
    return [
        {
            "event_key": hash_key(("status_change", event_time, detail)),
            "logged_at_utc": logged_at_utc,
            "event_timestamp_utc": event_time,
            "event_type": "status_change",
            "source": "status.cgi",
            "detail": detail,
            "raw_line": detail,
        }
    ]


def read_last_tsv_row(path: Path) -> Optional[dict[str, str]]:
    """Return the last data row from a TSV file, or ``None`` if it is empty."""

    if not path.exists() or path.stat().st_size == 0:
        return None

    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle, delimiter="\t")
        last_row = None
        for row in reader:
            last_row = row
        return last_row


def load_existing_keys(path: Path, key_field: str) -> set[str]:
    """Load existing deduplication keys from a TSV file."""

    keys: set[str] = set()
    if not path.exists() or path.stat().st_size == 0:
        return keys

    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle, delimiter="\t")
        for row in reader:
            key = row.get(key_field, "")
            if key:
                keys.add(key)
    return keys


def append_tsv_rows(path: Path, fieldnames: Sequence[str], rows: Sequence[dict[str, str]]) -> None:
    """Append rows to a TSV file and write the header when creating it."""

    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, delimiter="\t", extrasaction="ignore")
        if new_file:
            writer.writeheader()
        for row in rows:
            writer.writerow(row)


def append_deduped_events(path: Path, rows: Sequence[dict[str, str]]) -> int:
    """Append unique event rows to the event TSV.

    Inputs
    ------
    path:
        Target TSV file.
    rows:
        Candidate event rows.

    Returns
    -------
    int
        Number of newly appended rows.
    """

    existing_keys = load_existing_keys(path, "event_key")
    unique_rows = []
    for row in rows:
        key = row["event_key"]
        if key in existing_keys:
            continue
        existing_keys.add(key)
        unique_rows.append(row)

    if not unique_rows and not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=_EVENT_FIELDNAMES, delimiter="\t", extrasaction="ignore")
            writer.writeheader()
        return 0

    append_tsv_rows(path, _EVENT_FIELDNAMES, unique_rows)
    return len(unique_rows)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Parse command-line arguments for the collector."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://10.49.98.251")
    parser.add_argument("--about-url", default="http://10.49.98.251/about.cgi")
    parser.add_argument("--username", default="admin")
    parser.add_argument("--password", default="tmachine")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--download-action", default="download_logs.cgi")
    parser.add_argument("--archive-name", default="logs.tar.gz")
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--poll-delay-s", type=float, default=1.0)
    parser.add_argument("--max-poll-attempts", type=int, default=10)
    parser.add_argument("--events-name", default=DEFAULT_EVENTS_NAME)
    parser.add_argument("--tracking-name", default=DEFAULT_TRACKING_NAME)
    return parser.parse_args(argv)


def run(config: CollectorConfig, *, opener: Optional[object] = None) -> dict[str, object]:
    """Run one collection cycle and write the consolidated TSV logs."""

    opener = opener or build_auth_opener(config.base_url, config.username, config.password)
    archive_bytes = download_log_bundle(
        opener=opener,
        about_url=config.about_url,
        download_action=config.download_action,
        archive_name=config.archive_name,
        timeout_s=config.timeout_s,
        poll_delay_s=config.poll_delay_s,
        max_poll_attempts=config.max_poll_attempts,
    )

    logged_at_utc = iso_utc()
    status_url = urllib.parse.urljoin(config.base_url.rstrip("/") + "/", "status.cgi")
    satellite_url = urllib.parse.urljoin(config.base_url.rstrip("/") + "/", "sat.cgi")
    status_html = fetch_text(opener, status_url, config.timeout_s)
    satellite_text = fetch_text(opener, satellite_url, config.timeout_s)

    status_fields = parse_status_page(status_html)
    satellite_fields = parse_satellite_feed(satellite_text)
    tracking_row = {
        "collected_at_utc": logged_at_utc,
        **{field: status_fields.get(field, "") for field in _STATUS_FIELDS},
        **satellite_fields,
    }

    previous_tracking = read_last_tsv_row(config.output_root / config.tracking_name)
    archive_events = extract_relevant_events(archive_bytes)
    status_events = build_status_change_events(tracking_row, previous_tracking, logged_at_utc=logged_at_utc)
    appended_events = append_deduped_events(config.output_root / config.events_name, archive_events + status_events)
    append_tsv_rows(config.output_root / config.tracking_name, _TRACKING_FIELDNAMES, [tracking_row])

    return {
        "events_path": str(config.output_root / config.events_name),
        "tracking_path": str(config.output_root / config.tracking_name),
        "appended_events": appended_events,
        "tracking_row": tracking_row,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point for the collector."""

    args = parse_args(argv)
    config = CollectorConfig(
        base_url=args.base_url,
        about_url=args.about_url,
        username=args.username,
        password=args.password,
        output_root=args.output_root,
        download_action=args.download_action,
        archive_name=args.archive_name,
        timeout_s=args.timeout_s,
        poll_delay_s=args.poll_delay_s,
        max_poll_attempts=args.max_poll_attempts,
    )

    result = run(config)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


def _extract_status_cell(html_text: str, label: str) -> str:
    """Extract a table cell from the status page and normalize the text."""

    pattern = re.compile(
        rf"<tr>\s*<th>\s*{re.escape(label)}\s*</th>\s*<td>(.*?)</td>\s*</tr>",
        re.IGNORECASE | re.DOTALL,
    )
    match = pattern.search(html_text)
    if match is None:
        return ""
    cell_html = match.group(1)
    plain = re.sub(r"<[^>]+>", " ", cell_html)
    return normalize_whitespace(html.unescape(plain))


def _parse_status_datetime(value: str) -> str:
    """Parse the status page's UTC time string into ISO-8601 text."""

    if not value:
        return ""
    parsed = datetime.strptime(value, "%a %b %d %H:%M:%S UTC %Y")
    return iso_utc(parsed.replace(tzinfo=timezone.utc))


def _extract_ocxo_correction(value: str) -> str:
    """Extract the OCXO correction text from the status page cell."""

    if not value:
        return ""
    match = re.search(r"[+-]?[0-9]+(?:\.[0-9]*)?\s*Hz", value, re.IGNORECASE)
    if match is None:
        return value
    return normalize_whitespace(match.group(0))


def _parse_archive_event_line(source_file: str, raw_line: str) -> Optional[dict[str, str]]:
    """Parse one archive log line into a relevant event row."""

    message = raw_line.strip()
    if not message:
        return None

    timestamp_utc = ""
    parsed_timestamp = _parse_archive_timestamp(message)
    if parsed_timestamp is not None:
        timestamp_utc = iso_utc(parsed_timestamp[0])
        message = message[parsed_timestamp[1] :].strip()

    for pattern, event_type in _EVENT_PATTERNS:
        if not pattern.search(message):
            continue
        detail = normalize_whitespace(message)
        event_key = hash_key((source_file, timestamp_utc, event_type, detail))
        return {
            "event_key": event_key,
            "logged_at_utc": iso_utc(),
            "event_timestamp_utc": timestamp_utc,
            "event_type": event_type,
            "source": source_file,
            "detail": detail,
            "raw_line": raw_line.strip(),
        }
    return None


def _parse_archive_timestamp(line: str) -> Optional[tuple[datetime, int]]:
    """Parse the syslog-like timestamp prefix used by the appliance logs."""

    match = _DATE_RE.match(line)
    if match is None:
        return None

    current_year = utc_now().year
    parsed = datetime.strptime(
        f"{current_year} {match.group('month')} {int(match.group('day')):02d} "
        f"{int(match.group('hour')):02d} {int(match.group('minute')):02d} {int(match.group('second')):02d}",
        "%Y %b %d %H %M %S",
    ).replace(tzinfo=timezone.utc)
    return parsed, match.end()


def _main() -> None:
    """Run the command-line entry point and map exceptions to exit codes."""

    try:
        raise SystemExit(main())
    except Exception as exc:  # pragma: no cover - CLI failure path.
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    _main()
