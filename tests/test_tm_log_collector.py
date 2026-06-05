from __future__ import annotations

import csv
import importlib.util
import io
import tarfile
import sys
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "raspberry_pi" / "scripts" / "download_tm_logs.py"


def load_collector_module():
    """Load the collector script as a module for direct unit testing."""

    spec = importlib.util.spec_from_file_location("download_tm_logs", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load module from {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class FakeResponse:
    """Minimal file-like response object for exercising the downloader."""

    def __init__(self, body: bytes, content_type: str = "text/plain") -> None:
        self._body = body
        self.status = 200
        self.headers = {"Content-Type": content_type}

    def read(self) -> bytes:
        return self._body

    def __enter__(self) -> "FakeResponse":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        return None


class FakeOpener:
    """Record requests and return canned responses in order."""

    def __init__(self, responses: list[FakeResponse]) -> None:
        self.responses = responses
        self.requests = []

    def open(self, request, timeout=None):
        self.requests.append((request.full_url, request.data, request.get_method(), timeout))
        if not self.responses:
            raise AssertionError("unexpected extra request")
        return self.responses.pop(0)


def make_tar_gz(files: dict[str, str]) -> bytes:
    """Create a small gzipped tar archive in memory."""

    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w:gz") as tar:
        for name, text in files.items():
            payload = text.encode("utf-8")
            info = tarfile.TarInfo(name=name)
            info.size = len(payload)
            tar.addfile(info, io.BytesIO(payload))
    return buffer.getvalue()


def sample_status_html() -> str:
    """Return a representative status page payload."""

    return """
    <html><body>
      <table>
        <tr><th>Time Source</th><td>GPS</td></tr>
        <tr><th>Location</th><td>40.85535, -73.84672</td></tr>
        <tr><th>Date/Time</th><td>Thu May 21 19:26:11 UTC 2026</td></tr>
        <tr><th>Satellites used</th><td>6</td></tr>
        <tr><th>GPS fix</th><td>2D</td></tr>
        <tr><th>NTP Lookups</th><td>123915</td></tr>
        <tr><th>Holdover Time</th><td>0 Minutes</td></tr>
        <tr><th>MAC</th><td>74:A5:8C:F0:A9:E6</td></tr>
        <tr><th>Uptime</th><td>6 days 5 hours 8 minutes 8 seconds</td></tr>
        <tr><th>OCXO Correction</th><td><form>-3.1575 Hz <button>Save</button></form></td></tr>
        <tr><th>Version</th><td>0.6.6</td></tr>
      </table>
    </body></html>
    """


def sample_satellite_feed() -> str:
    """Return a representative satellite feed payload."""

    return "\n".join(
        [
            "21,11,30",
            "37,34,34",
            "45,31,14",
            "053,110,076",
        ]
    )


def test_parse_status_page_and_satellite_feed():
    """The live status and satellite endpoints should parse cleanly."""

    module = load_collector_module()
    status = module.parse_status_page(sample_status_html())
    satellites = module.parse_satellite_feed(sample_satellite_feed())

    assert status["time_source"] == "GPS"
    assert status["gps_fix"] == "2D"
    assert status["status_time_utc"] == "2026-05-21T19:26:11Z"
    assert status["ocxo_correction"] == "-3.1575 Hz"
    assert satellites["satellite_count"] == "3"
    assert satellites["satellite_ids"] == "21,11,30"
    assert satellites["satellite_snr"] == "37,34,34"
    assert satellites["satellite_azimuth"] == "053,110,076"
    assert satellites["satellite_elevation"] == "45,31,14"


def test_extract_relevant_events_filters_boot_and_holdover():
    """Archive parsing should keep only the relevant event lines."""

    module = load_collector_module()
    archive_bytes = make_tar_gz(
        {
            "logs/messages.0": "\n".join(
                [
                    "May 21 19:20:00 am335x-evm kernel: Booting Linux on physical CPU 0x0",
                    "May 21 19:25:00 am335x-evm user.alert kernel: [GPS] Fix State 1 -> 0.",
                    "May 21 19:26:00 am335x-evm user.alert kernel: ethholdover_work_func: Next_TAI_Second=1779205000",
                ]
            )
            + "\n"
        }
    )

    events = module.extract_relevant_events(archive_bytes)

    assert [event["event_type"] for event in events] == ["boot", "holdover_start"]
    assert events[0]["source"] == "logs/messages.0"
    assert events[1]["raw_line"].endswith("Fix State 1 -> 0.")


def test_run_appends_tracking_and_deduplicates_events(tmp_path, monkeypatch):
    """Running the collector twice should not duplicate the same events."""

    module = load_collector_module()
    monkeypatch.setattr(module, "utc_now", lambda: module.datetime(2026, 5, 21, 19, 30, 0))

    archive_bytes = make_tar_gz(
        {
            "logs/messages.0": "\n".join(
                [
                    "May 21 19:20:00 am335x-evm kernel: Booting Linux on physical CPU 0x0",
                    "May 21 19:25:00 am335x-evm user.alert kernel: [GPS] Fix State 1 -> 0.",
                ]
            )
            + "\n"
        }
    )

    opener = FakeOpener(
        [
            FakeResponse(b"logs/\ndone\n"),
            FakeResponse(archive_bytes, content_type="application/gzip"),
            FakeResponse(sample_status_html().encode("utf-8")),
            FakeResponse(sample_satellite_feed().encode("utf-8")),
            FakeResponse(b"logs/\ndone\n"),
            FakeResponse(archive_bytes, content_type="application/gzip"),
            FakeResponse(sample_status_html().encode("utf-8")),
            FakeResponse(sample_satellite_feed().encode("utf-8")),
        ]
    )

    config = module.CollectorConfig(
        base_url="http://10.49.98.251",
        about_url="http://10.49.98.251/about.cgi",
        username="admin",
        password="tmachine",
        output_root=tmp_path,
        timeout_s=5.0,
        poll_delay_s=0.0,
        max_poll_attempts=1,
    )

    first = module.run(config, opener=opener)
    second = module.run(config, opener=opener)

    events_path = tmp_path / module.DEFAULT_EVENTS_NAME
    tracking_path = tmp_path / module.DEFAULT_TRACKING_NAME

    with events_path.open("r", encoding="utf-8", newline="") as handle:
        event_rows = list(csv.DictReader(handle, delimiter="\t"))
    with tracking_path.open("r", encoding="utf-8", newline="") as handle:
        tracking_rows = list(csv.DictReader(handle, delimiter="\t"))

    assert first["appended_events"] == 2
    assert second["appended_events"] == 0
    assert len(event_rows) == 2
    assert [row["event_type"] for row in event_rows] == ["boot", "holdover_start"]
    assert len(tracking_rows) == 2
    assert tracking_rows[0]["time_source"] == "GPS"
    assert tracking_rows[0]["satellite_ids"] == "21,11,30"
    assert tracking_rows[1]["status_time_utc"] == "2026-05-21T19:26:11Z"


def test_parse_args_defaults_to_timemachine_logs():
    """The CLI should default to the Pi log archive directory."""

    module = load_collector_module()
    args = module.parse_args([])

    assert args.output_root == Path("/home/pi/timemachine_logs")
