from __future__ import annotations

import csv
import importlib.util
from argparse import Namespace
from datetime import datetime, timezone
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "raspberry_pi"
    / "scripts"
    / "log_tm2000b_internet_observer.py"
)


def load_observer_module():
    """Load the internet-only TM2000B observer logger module."""

    spec = importlib.util.spec_from_file_location("log_tm2000b_internet_observer", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load module from {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_ntp_packet(module, *, stratum=1, leap=0, version=4, mode=4, reference_id=b"GPS\x00", reference_time=999.0, receive_time=1000.1, transmit_time=1000.2, root_delay_s=0.015625, root_dispersion_s=0.03125):
    """Create a minimal NTP response packet for unit testing."""

    packet = bytearray(48)
    packet[0] = (leap << 6) | (version << 3) | mode
    packet[1] = stratum & 0xFF
    packet[2] = 4
    packet[3] = -20 & 0xFF
    packet[4:8] = int(root_delay_s * 65536).to_bytes(4, byteorder="big", signed=False)
    packet[8:12] = int(root_dispersion_s * 65536).to_bytes(4, byteorder="big", signed=False)
    packet[12:16] = reference_id
    packet[16:24] = module._unix_to_ntp_timestamp(reference_time).to_bytes(8, byteorder="big")
    packet[24:32] = module._unix_to_ntp_timestamp(999.5).to_bytes(8, byteorder="big")
    packet[32:40] = module._unix_to_ntp_timestamp(receive_time).to_bytes(8, byteorder="big")
    packet[40:48] = module._unix_to_ntp_timestamp(transmit_time).to_bytes(8, byteorder="big")
    return bytes(packet)


class FakeSocket:
    """Minimal UDP socket stub for a single NTP exchange."""

    def __init__(self, response: bytes) -> None:
        self.response = response
        self.sent = []
        self.timeout = None

    def settimeout(self, timeout):
        self.timeout = timeout

    def sendto(self, payload, address):
        self.sent.append((bytes(payload), address))

    def recvfrom(self, size):
        return self.response, ("10.49.98.251", 123)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return None


def test_parse_chronyc_tracking_parses_numeric_fields():
    """The tracking parser should extract chrony timing estimates."""

    module = load_observer_module()
    tracking = module.parse_chronyc_tracking(
        """
Reference ID    : GPS
Stratum         : 2
Ref time (UTC)  : Mon May 19 12:34:56 2026
System time     : 0.000000096 seconds fast of NTP time
Last offset     : 0.000000048 seconds
RMS offset      : 0.000000120 seconds
Frequency       : 3.253 ppm slow
Skew            : 0.073 ppm
Root delay      : 0.000000001 seconds
Root dispersion : 0.000024310 seconds
Update interval : 16.0 seconds
Leap status     : Normal
"""
    )

    assert tracking["reference_id"] == "GPS"
    assert tracking["stratum"] == 2
    assert tracking["system_time"] == 0.000000096
    assert tracking["last_offset"] == 0.000000048
    assert tracking["frequency"] == 3.253
    assert tracking["skew"] == 0.073
    assert tracking["update_interval"] == 16.0
    assert tracking["chrony_holdover"] is False


def test_query_tm2000b_ntp_computes_offset_and_delay(monkeypatch):
    """The raw NTP query should compute a sensible offset from one response."""

    module = load_observer_module()
    response = make_ntp_packet(module)
    fake_socket = FakeSocket(response)
    times = iter([1000.0, 1000.3])

    monkeypatch.setattr(module.socket, "socket", lambda *args, **kwargs: fake_socket)

    result = module.query_tm2000b_ntp(
        host="10.49.98.251",
        port=123,
        timeout_s=5.0,
        time_func=lambda: next(times),
    )

    assert fake_socket.sent[0][1] == ("10.49.98.251", 123)
    assert result["ok"] is True
    assert result["stratum"] == 1
    assert result["reference_id"] == "GPS"
    assert abs(result["offset_s"]) < 1e-8
    assert abs(result["delay_s"] - 0.2) < 1e-8
    assert abs(result["response_time_s"] - 0.3) < 1e-8
    assert result["reference_time_utc"] == "1970-01-01T00:16:39Z"
    assert result["receive_time_utc"] == "1970-01-01T00:16:40.100000Z"
    assert result["transmit_time_utc"] == "1970-01-01T00:16:40.200000Z"


def test_run_writes_daily_tsv(tmp_path, monkeypatch):
    """The internet-only observer should write a daily TSV row with both estimates."""

    module = load_observer_module()
    monkeypatch.setattr(module, "utc_now", lambda: datetime(2026, 5, 22, 15, 0, 0, tzinfo=timezone.utc))
    monkeypatch.setattr(
        module,
        "run_chronyc_tracking",
        lambda **kwargs: {
            "ok": True,
            "error": "",
            "reference_id": "GPS",
            "stratum": 2,
            "leap_status": "Normal",
            "system_time": 0.000001,
            "last_offset": 0.000002,
            "rms_offset": 0.000003,
            "frequency": 3.2,
            "skew": 0.07,
            "root_delay": 0.0,
            "root_dispersion": 0.0,
            "update_interval": 16.0,
            "chrony_holdover": False,
        },
    )
    monkeypatch.setattr(
        module,
        "query_tm2000b_ntp",
        lambda **kwargs: {
            "ok": True,
            "error": "",
            "host": "10.49.98.251",
            "port": 123,
            "leap_indicator": 0,
            "version": 4,
            "mode": 4,
            "stratum": 1,
            "reference_id": "GPS",
            "reference_time_utc": "2026-05-22T14:59:50Z",
            "receive_time_utc": "2026-05-22T15:00:00.100000Z",
            "transmit_time_utc": "2026-05-22T15:00:00.200000Z",
            "local_send_time_utc": "2026-05-22T15:00:00Z",
            "local_receive_time_utc": "2026-05-22T15:00:00.300000Z",
            "offset_s": 0.0,
            "delay_s": 0.2,
            "root_delay_s": 0.0,
            "root_dispersion_s": 0.0,
            "precision_s": 0.000000119,
            "poll_interval_s": 16.0,
            "response_time_s": 0.3,
        },
    )

    config = Namespace(
        output_dir=tmp_path,
        tm2000b_host="10.49.98.251",
        tm2000b_port=123,
        chronyc_binary="chronyc",
        timeout_s=5.0,
    )

    result = module.run(config)
    output_path = Path(result["output_path"])

    assert output_path.exists()
    with output_path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle, delimiter="\t"))

    assert len(rows) == 1
    row = rows[0]
    assert row["chrony_reference_id"] == "GPS"
    assert row["tm2000b_reference_id"] == "GPS"
    assert row["tm2000b_offset_s"] == "0.0"
    assert row["tm2000b_response_time_s"] == "0.3"

