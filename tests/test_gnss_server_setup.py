from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_server_chrony_config_uses_pps_and_blocks_lab_clients():
    """The Waveshare M8T test config should use PPS and deny the lab subnets."""

    chrony_conf = (
        REPO_ROOT
        / "image"
        / "stage-neurokairos-server"
        / "00-config-gps-server"
        / "files"
        / "chrony.conf"
    ).read_text()

    assert "refclock SHM 0" in chrony_conf
    assert "refclock PPS /dev/pps0 refid PPS lock GPS precision 1e-7 prefer" in chrony_conf
    assert "refclock SOCK /var/run/chrony.ttyAMA0.sock" not in chrony_conf
    assert "hwtimestamp eth0" in chrony_conf
    assert "allow all" in chrony_conf
    assert "deny 10.49.0.0/16" in chrony_conf
    assert "deny 10.42.0.0/16" in chrony_conf
    assert "allow 192.168.0.0/16" not in chrony_conf
    assert "allow 10.0.0.0/8" not in chrony_conf
    assert "pool pool.ntp.org" not in chrony_conf
    assert "server 10.49.98.251" not in chrony_conf


def test_server_gpsd_config_targets_uart_and_pps_gpio():
    """The gpsd config should target the Pi UART and GPIO PPS device."""

    gpsd_conf = (
        REPO_ROOT
        / "image"
        / "stage-neurokairos-server"
        / "00-config-gps-server"
        / "files"
        / "gpsd"
    ).read_text()

    assert 'DEVICES="/dev/ttyAMA0 /dev/pps0"' in gpsd_conf
    assert 'USBAUTO="false"' in gpsd_conf


def test_boot_script_and_service_configure_m8t_pps():
    """The boot-time GNSS setup should program the receiver with ubxtool."""

    boot_script = (
        REPO_ROOT / "raspberry_pi" / "scripts" / "configure_gnss_pps.sh"
    ).read_text()
    stage_boot_script = (
        REPO_ROOT
        / "image"
        / "stage-neurokairos-server"
        / "00-config-gps-server"
        / "files"
        / "configure_gnss_pps.sh"
    ).read_text()
    service_unit = (
        REPO_ROOT / "raspberry_pi" / "systemd" / "gnss-pps.service"
    ).read_text()

    assert "ubxtool" in boot_script
    assert 'GNSS_DEVICE="${GNSS_DEVICE:-/dev/ttyAMA0}"' in boot_script
    assert 'GNSS_UBX_TARGET="${GNSS_UBX_TARGET:-localhost:2947:${GNSS_DEVICE}}"' in boot_script
    assert "UBX_TP5_PPS_COMMAND" in boot_script
    assert "06,31,00,01,00,00,02,00,00,00,40,42,0f,00,40,42,0f,00,a0,86,01,00,a0,86,01,00,00,00,00,00,77,00,00,00" in boot_script
    assert 'ubxtool ${GNSS_UBX_TARGET} -c "${UBX_TP5_PPS_COMMAND}"' in boot_script
    assert "ubxtool ${GNSS_UBX_TARGET} -e PPS" not in boot_script
    assert "ubxtool ${GNSS_UBX_TARGET} -p CFG-TP5" not in boot_script
    assert "0xf7" not in boot_script
    assert 'UBX_DYNAMIC_MODEL="${UBX_DYNAMIC_MODEL:-2}"' in boot_script
    assert 'MODEL,${UBX_DYNAMIC_MODEL}' in boot_script
    assert "UBX_TMODE2_FIXED_COMMAND" in boot_script
    assert "06,3d,02,00,00,00,ef,cf,02,08,c8,4a,57,e4,29,d5,bc,18,40,42,0f,00,00,00,00,00,00,00,00,00" in boot_script
    assert "ubxtool ${GNSS_UBX_TARGET} -e BINARY" in boot_script
    assert "ubxtool ${GNSS_UBX_TARGET} -d NMEA" in boot_script
    assert "SAVE" in boot_script
    assert "UBX_TMODE2_FIXED_COMMAND" in stage_boot_script
    assert "UBX_TP5_PPS_COMMAND" in stage_boot_script
    assert "06,31,00,01,00,00,02,00,00,00,40,42,0f,00,40,42,0f,00,a0,86,01,00,a0,86,01,00,00,00,00,00,77,00,00,00" in stage_boot_script
    assert 'ubxtool ${GNSS_UBX_TARGET} -c "${UBX_TP5_PPS_COMMAND}"' in stage_boot_script
    assert "ubxtool ${GNSS_UBX_TARGET} -e PPS" not in stage_boot_script
    assert "UBX_PULSE_COMMAND" not in stage_boot_script
    assert "ubxtool ${GNSS_UBX_TARGET} -p CFG-TP5" not in stage_boot_script
    assert "0xf7" not in stage_boot_script
    assert "06,3d,02,00,00,00,ef,cf,02,08,c8,4a,57,e4,29,d5,bc,18,40,42,0f,00,00,00,00,00,00,00,00,00" in stage_boot_script
    assert "After=gpsd.service" in service_unit
    assert "After=chrony.service" in service_unit
    assert "Before=chrony.service" not in service_unit
    assert "ExecStart=/usr/local/sbin/configure_gnss_pps.sh" in service_unit


def test_server_image_uses_waveshare_gpio18_and_installs_logger():
    """The image build should use Waveshare PPS GPIO 18 and persistent logging."""

    image_run = (
        REPO_ROOT
        / "image"
        / "stage-neurokairos-server"
        / "00-config-gps-server"
        / "00-run.sh"
    ).read_text()

    assert "dtoverlay=pps-gpio,gpiopin=18,capture_clear,pull=up" in image_run
    assert "dtoverlay=pps-gpio,gpiopin=18\n" not in image_run
    assert "gpiopin=4" not in image_run
    assert "log_gnss_status.py" in image_run
    assert "gnss-status-logger.timer" in image_run


def test_server_install_script_uses_waveshare_gpio18_and_installs_logger():
    """The manual installer should use Waveshare GPIO 18 and persistent logging."""

    install_script = (
        REPO_ROOT
        / "raspberry_pi"
        / "scripts"
        / "install_chrony_server.sh"
    ).read_text()

    assert 'GNSS_PPS_GPIO="18"' in install_script
    assert 'dtoverlay=pps-gpio,gpiopin=${GNSS_PPS_GPIO},capture_clear,pull=up' in install_script
    assert "hwtimestamp ${ETHERNET_INTERFACE}" in install_script
    assert "network-online.target" in install_script
    assert "allow all" in install_script
    assert "deny 10.49.0.0/16" in install_script
    assert "deny 10.42.0.0/16" in install_script
    assert "gnss-status-logger.timer" in install_script
    assert "log_gnss_status.py" in install_script


def test_server_image_delays_chrony_until_network_online_for_hwtimestamp():
    """The image should avoid starting chrony before eth0 can enable HW timestamps."""

    image_run = (
        REPO_ROOT
        / "image"
        / "stage-neurokairos-server"
        / "00-config-gps-server"
        / "00-run.sh"
    ).read_text()

    assert "chrony.service.d" in image_run
    assert "network-online.target" in image_run


def test_docs_clarify_waveshare_and_adafruit_pps_gpio():
    """Docs should explain the GPIO 18 vs GPIO 4 HAT distinction."""

    readme = (REPO_ROOT / "README.md").read_text()
    server_doc = (REPO_ROOT / "raspberry_pi" / "docs" / "rpi-gps-server.md").read_text()

    assert "Waveshare NEO-M8T" in readme
    assert "BCM GPIO 18" in readme
    assert "Adafruit" in readme
    assert "GPIO 4" in readme
    assert "Waveshare NEO-M8T HAT | GPIO 18" in server_doc
    assert "Adafruit GPS HAT | GPIO 4" in server_doc
