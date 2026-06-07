from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[1]
EVENTLOGGER_DIR = REPO_ROOT / "raspberry_pi" / "eventlogger"


def read_text(relative_path: str) -> str:
    return (REPO_ROOT / relative_path).read_text()


def test_eventlogger_core_c_tests_pass():
    result = subprocess.run(
        ["make", "-C", str(EVENTLOGGER_DIR), "test"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stdout + result.stderr


def test_default_config_uses_expected_paths_and_pins():
    config = read_text("raspberry_pi/eventlogger/eventlogger.conf.example")

    assert "journal_dir = /var/lib/neurokairos/eventlogger/journal" in config
    assert "cleanup_threshold_gb = 5" in config
    assert "cleanup_target_gb = 8" in config
    assert "flush_interval_ms = 1000" in config
    assert "flush_event_count = 1000" in config
    assert "deadtime_ms = 1" in config

    for gpio in (5, 6, 10, 11, 12, 13, 16, 17):
        assert f"gpio = {gpio}" in config


def test_event_tsv_schema_has_no_sequence_column():
    header = read_text("raspberry_pi/eventlogger/eventlogger_journal.c")

    assert r"utc_time\trealtime_ns\tmonotonic_ns\tinput\tedge\n" in header
    assert "sequence" not in header.lower()


def test_systemd_unit_and_install_script_are_standalone():
    service = read_text("raspberry_pi/eventlogger/eventlogger.service")
    install_script = read_text("raspberry_pi/eventlogger/install_eventlogger.sh")

    assert "ExecStart=/usr/local/bin/neurokairos-eventlogger" in service
    assert "User=root" in service
    assert "irig-sender" not in service

    assert "/etc/neurokairos/eventlogger.conf" in install_script
    assert "systemctl enable neurokairos-eventlogger.service" in install_script


def test_makefile_targets_bookworm_libgpiod_v1():
    makefile = read_text("raspberry_pi/eventlogger/Makefile")
    source = read_text("raspberry_pi/eventlogger/eventlogger.c")

    assert "pkg-config --cflags --libs libgpiod" in makefile
    assert "gpiod_line_event_wait_bulk" in source
    assert "gpiod_line_request_config" in source
