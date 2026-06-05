from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_internet_only_observer_install_script_configures_public_ntp_sources():
    """The new installer should configure chrony to use only internet NTP."""

    install_script = (
        REPO_ROOT
        / "raspberry_pi"
        / "scripts"
        / "install_tm2000b_internet_observer.sh"
    ).read_text()

    assert "time.google.com" in install_script
    assert "time.cloudflare.com" in install_script
    assert "time.apple.com" in install_script
    assert "pool.ntp.org" in install_script
    assert "refclock SHM 0" not in install_script
    assert "refclock PPS /dev/pps0" not in install_script
    assert "10.49.98.251" not in install_script
    assert "tm2000b-internet-observer.service" in install_script
    assert "tm2000b-internet-observer.timer" in install_script

    timer_unit = (
        REPO_ROOT
        / "raspberry_pi"
        / "systemd"
        / "tm2000b-internet-observer.timer"
    ).read_text()
    assert "OnUnitActiveSec=10min" in timer_unit


def test_internet_only_observer_systemd_units_run_the_new_logger():
    """The timer should trigger the new logger on a 10-minute cadence."""

    service_unit = (
        REPO_ROOT
        / "raspberry_pi"
        / "systemd"
        / "tm2000b-internet-observer.service"
    ).read_text()
    timer_unit = (
        REPO_ROOT
        / "raspberry_pi"
        / "systemd"
        / "tm2000b-internet-observer.timer"
    ).read_text()

    assert "log_tm2000b_internet_observer.py" in service_unit
    assert "/home/pi/tm2000b_internet_observer_logs" in service_unit
    assert "User=pi" in service_unit
    assert "After=chrony.service network-online.target" in service_unit
    assert "OnBootSec=5min" in timer_unit
    assert "OnUnitActiveSec=10min" in timer_unit
    assert "tm2000b-internet-observer.service" in timer_unit
