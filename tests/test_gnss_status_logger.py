from __future__ import annotations

import importlib.util
from datetime import datetime, timezone
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
LOGGER_PATH = (
    REPO_ROOT / "raspberry_pi" / "scripts" / "log_gnss_status.py"
)


def load_logger_module():
    """Load the GNSS status logger module from its script path."""

    spec = importlib.util.spec_from_file_location("log_gnss_status", LOGGER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_parse_gpsd_snapshot_counts_visible_and_used_satellites():
    """The GPS parser should separate receiver fix state from satellite counts."""

    module = load_logger_module()
    messages = [
        {
            "class": "TPV",
            "device": "/dev/ttyAMA0",
            "mode": 3,
            "time": "2026-05-19T12:34:56.000Z",
        },
        {
            "class": "SKY",
            "device": "/dev/ttyAMA0",
            "nSat": 12,
            "uSat": 8,
            "satellites": [
                {"PRN": 1, "used": True},
                {"PRN": 2, "used": False, "ss": 20, "el": 30, "az": 40},
                {"PRN": 3, "used": True, "ss": 35, "el": 50, "az": 60},
            ],
        },
    ]

    status = module.summarize_gpsd_messages(messages)

    assert status["receiver_holdover"] is False
    assert status["fix_mode"] == 3
    assert status["satellites_visible"] == 12
    assert status["satellites_used"] == 8
    assert status["device"] == "/dev/ttyAMA0"
    assert status["satellite_prns"] == "1,2,3"
    assert status["satellite_snr"] == ",20,35"
    assert status["satellite_elevation"] == ",30,50"
    assert status["satellite_azimuth"] == ",40,60"
    assert status["satellite_used"] == "1,0,1"


def test_parse_gpsd_snapshot_detects_no_fix_holdover():
    """A mode 1 receiver should be marked as in GNSS holdover."""

    module = load_logger_module()
    messages = [
        {
            "class": "SKY",
            "device": "/dev/ttyAMA0",
            "nSat": 4,
            "uSat": 0,
            "satellites": [],
        }
    ]

    status = module.summarize_gpsd_messages(messages)

    assert status["receiver_holdover"] is True
    assert status["fix_mode"] is None
    assert status["satellites_visible"] == 4
    assert status["satellites_used"] == 0


def test_parse_chronyc_tracking_and_sources_detects_sync_loss():
    """Chrony parsing should preserve raw state and infer unsynchronised mode."""

    module = load_logger_module()
    tracking_output = """
Reference ID    : PPS
Stratum         : 1
Ref time (UTC)  : Mon May 19 12:34:56 2026
System time     : 0.000000096 seconds fast of NTP time
Last offset     : 0.000000048 seconds
RMS offset      : 0.000000120 seconds
Root delay      : 0.000015 seconds
Root dispersion : 0.000023 seconds
Leap status     : Normal
"""
    sources_output = """
MS Name/IP address         Stratum Poll Reach LastRx Last sample
#* PPS                           0   4   377     1   +0ns[ +0ns] +/-  25ns
#? GPS                           0   4   377     1   +0ns[ +0ns] +/-  25ns
"""

    tracking = module.parse_chronyc_tracking(tracking_output)
    sources = module.parse_chronyc_sources(sources_output)
    record = module.build_record(
        timestamp_utc=datetime(2026, 5, 19, 12, 35, tzinfo=timezone.utc),
        gps_status={"receiver_holdover": False, "fix_mode": 3},
        chrony_tracking=tracking,
        chrony_sources=sources,
        chrony_sourcestats=module.parse_chronyc_sourcestats(
            """
Name/IP Address            NP  NR  Span  Frequency  Freq Skew  Offset  Std Dev
PPS                        12   7   176     -0.003      0.020    -30ns   845ns
"""
        ),
        pps_status={"ok": True, "assert_count": 3},
    )

    assert tracking["leap_status"] == "Normal"
    assert tracking["stratum"] == 1
    assert sources["selected_source"] == "PPS"
    assert record["chrony_holdover"] is False
    assert record["receiver_holdover"] is False
    assert record["chrony_synced"] is True
    assert record["chrony_selected_source"] == "PPS"
    assert record["chrony_selected_marker"] == "#*"
    assert record["chrony_stratum"] == 1
    assert record["chrony_leap_status"] == "Normal"
    assert record["chrony_pps_reach"] == "377"
    assert record["chrony_pps_last_rx_s"] == 1
    assert record["chrony_pps_sample"] == "+0ns[ +0ns] +/- 25ns"
    assert record["kernel_pps_ok"] is True
    assert record["kernel_pps_assert_count"] == 3
    assert record["pps_sourcestats_np"] == 12
    assert record["pps_sourcestats_frequency_ppm"] == -0.003


def test_parse_chronyc_sources_preserves_sample_and_unit_last_rx():
    """The source parser should keep the complete sample field and parse age units."""

    module = load_logger_module()
    sources = module.parse_chronyc_sources(
        """
MS Name/IP address         Stratum Poll Reach LastRx Last sample
#? PPS                           0   4   377   20m   -19ns[ -37ns] +/-  266ns
"""
    )

    pps_source = sources["sources"][0]

    assert pps_source["last_rx_s"] == 1200
    assert pps_source["sample"] == "-19ns[ -37ns] +/- 266ns"


def test_build_record_marks_chrony_holdover_when_it_falls_back_to_local_time():
    """A non-GNSS selected source should count as chrony holdover."""

    module = load_logger_module()
    tracking = module.parse_chronyc_tracking(
        """
Reference ID    : LOCL
Stratum         : 10
Leap status     : Normal
"""
    )
    sources = module.parse_chronyc_sources(
        """
MS Name/IP address         Stratum Poll Reach LastRx Last sample
#* LOCAL                          10   4   377     1   +0ns[ +0ns] +/-  1us
"""
    )
    record = module.build_record(
        timestamp_utc=datetime(2026, 5, 19, 12, 35, tzinfo=timezone.utc),
        gps_status={"receiver_holdover": False, "fix_mode": 3},
        chrony_tracking=tracking,
        chrony_sources=sources,
    )

    assert record["chrony_holdover"] is True
    assert record["chrony_synced"] is True
    assert record["chrony_selected_source"] == "LOCAL"
    assert record["chrony_selected_marker"] == "#*"
    assert record["chrony_stratum"] == 10


def test_write_state_change_event_only_logs_transitions(tmp_path):
    """The event log should stay empty until the compact state changes."""

    module = load_logger_module()
    steady_tracking = module.parse_chronyc_tracking(
        """
Reference ID    : PPS
Stratum         : 1
Leap status     : Normal
"""
    )
    steady_sources = module.parse_chronyc_sources(
        """
MS Name/IP address         Stratum Poll Reach LastRx Last sample
#* PPS                           0   4   377     1   +0ns[ +0ns] +/-  25ns
#? GPS                           0   4   377     1   +0ns[ +0ns] +/-  25ns
"""
    )
    steady_record = module.build_record(
        timestamp_utc=datetime(2026, 5, 19, 12, 35, tzinfo=timezone.utc),
        gps_status={"receiver_holdover": False, "fix_mode": 3, "satellites_visible": 10, "satellites_used": 7},
        chrony_tracking=steady_tracking,
        chrony_sources=steady_sources,
    )
    state_path = tmp_path / "gnss_m8t_status_state.json"

    first_event_path = module.write_state_change_event(
        timestamp_utc=datetime(2026, 5, 19, 12, 35, tzinfo=timezone.utc),
        gps_status={"receiver_holdover": False, "fix_mode": 3, "satellites_visible": 10, "satellites_used": 7},
        chrony_tracking=steady_tracking,
        chrony_sources=steady_sources,
        current_record=steady_record,
        output_dir=tmp_path,
        state_path=state_path,
    )
    second_event_path = module.write_state_change_event(
        timestamp_utc=datetime(2026, 5, 19, 12, 36, tzinfo=timezone.utc),
        gps_status={"receiver_holdover": False, "fix_mode": 3, "satellites_visible": 10, "satellites_used": 7},
        chrony_tracking=steady_tracking,
        chrony_sources=steady_sources,
        current_record=steady_record,
        output_dir=tmp_path,
        state_path=state_path,
    )

    assert first_event_path is None
    assert second_event_path is None
    assert state_path.exists()
    assert not any(tmp_path.glob("gnss_m8t_status_events_*.jsonl"))

    changed_tracking = module.parse_chronyc_tracking(
        """
Reference ID    : LOCL
Stratum         : 10
Leap status     : Normal
"""
    )
    changed_sources = module.parse_chronyc_sources(
        """
MS Name/IP address         Stratum Poll Reach LastRx Last sample
#* LOCAL                         10   4   377     1   +0ns[ +0ns] +/-  1us
"""
    )
    changed_record = module.build_record(
        timestamp_utc=datetime(2026, 5, 19, 12, 37, tzinfo=timezone.utc),
        gps_status={"receiver_holdover": False, "fix_mode": 3, "satellites_visible": 10, "satellites_used": 7},
        chrony_tracking=changed_tracking,
        chrony_sources=changed_sources,
    )

    event_path = module.write_state_change_event(
        timestamp_utc=datetime(2026, 5, 19, 12, 37, tzinfo=timezone.utc),
        gps_status={"receiver_holdover": False, "fix_mode": 3, "satellites_visible": 10, "satellites_used": 7},
        chrony_tracking=changed_tracking,
        chrony_sources=changed_sources,
        current_record=changed_record,
        output_dir=tmp_path,
        state_path=state_path,
    )

    assert event_path is not None
    assert event_path.name == "gnss_m8t_status_events_2026-05-19.jsonl"
    event_lines = event_path.read_text().splitlines()
    assert len(event_lines) == 1
    assert '"event_type": "state_change"' in event_lines[0]
    assert '"previous_state": {' in event_lines[0]
    assert '"current_state": {' in event_lines[0]
    assert '"chrony_selected_source": "PPS"' in event_lines[0]
    assert '"chrony_selected_source": "LOCAL"' in event_lines[0]
    assert '"raw_output": "' in event_lines[0]


def test_record_writer_creates_daily_tsv_file(tmp_path):
    """A sample record should be appended to the requested output directory."""

    module = load_logger_module()
    record = {
        "timestamp_utc": "2026-05-19T12:35:00Z",
        "receiver_holdover": True,
        "chrony_holdover": False,
        "chrony_synced": True,
        "chrony_stratum": 1,
        "chrony_leap_status": "Normal",
        "chrony_reference_id": "PPS",
        "chrony_selected_source": "PPS",
        "chrony_selected_marker": "#*",
        "satellites_visible": 5,
        "satellites_used": 2,
    }

    output_path = module.write_record(record, tmp_path)

    assert output_path.parent == tmp_path
    assert output_path.name == "gnss_m8t_status_2026-05-19.tsv"
    lines = output_path.read_text().splitlines()
    assert lines[0] == "\t".join(
        [
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
    )
    assert lines[1] == "2026-05-19T12:35:00Z\tTrue\t\t5\t2\t\t\t\t\t\t\t\tFalse\tTrue\t1\tNormal\tPPS\tPPS\t#*\t\t\t\t\t\t\t\t\t\t\t"


def test_record_writer_rolls_over_by_day(tmp_path):
    """Records from different UTC dates should go to different TSV files."""

    module = load_logger_module()
    first = {
        "timestamp_utc": "2026-05-19T23:59:59Z",
        "receiver_holdover": False,
        "chrony_holdover": False,
        "chrony_synced": True,
        "chrony_stratum": 1,
        "chrony_leap_status": "Normal",
        "chrony_reference_id": "PPS",
        "chrony_selected_source": "PPS",
        "chrony_selected_marker": "#*",
        "fix_mode": 3,
        "satellites_visible": 11,
        "satellites_used": 8,
    }
    second = {
        "timestamp_utc": "2026-05-20T00:00:01Z",
        "receiver_holdover": True,
        "chrony_holdover": True,
        "chrony_synced": False,
        "chrony_stratum": 10,
        "chrony_leap_status": "Not synchronised",
        "chrony_reference_id": "LOCL",
        "chrony_selected_source": "LOCAL",
        "chrony_selected_marker": "#*",
        "fix_mode": 1,
        "satellites_visible": 4,
        "satellites_used": 0,
    }

    first_path = module.write_record(first, tmp_path)
    second_path = module.write_record(second, tmp_path)

    assert first_path.name == "gnss_m8t_status_2026-05-19.tsv"
    assert second_path.name == "gnss_m8t_status_2026-05-20.tsv"
    assert first_path != second_path


def test_installer_and_timer_target_the_requested_directory():
    """The installation files should wire the logger to /home/pi/GNSS_logs."""

    service_text = (
        REPO_ROOT
        / "raspberry_pi"
        / "systemd"
        / "gnss-status-logger.service"
    ).read_text()
    timer_text = (
        REPO_ROOT
        / "raspberry_pi"
        / "systemd"
        / "gnss-status-logger.timer"
    ).read_text()
    install_text = (
        REPO_ROOT
        / "raspberry_pi"
        / "scripts"
        / "install_gnss_status_logger.sh"
    ).read_text()

    assert "/home/pi/GNSS_logs" in service_text
    assert "OnUnitActiveSec=60s" in timer_text
    assert "gnss-status-logger.timer" in install_text
    assert "log_gnss_status.py" in install_text
