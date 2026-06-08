from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path: str) -> str:
    return (REPO_ROOT / relative_path).read_text()


def test_chrony_server_install_script_supports_pps_override_and_defaults_to_waveshare_pin():
    install_script = read_text("raspberry_pi/scripts/install_chrony_server.sh")

    assert "--pps-pin" in install_script
    assert "PPS_PIN=18" in install_script
    assert "dtoverlay=pps-gpio,gpiopin=${PPS_PIN}" in install_script
    assert "sed -i \"s|^dtoverlay=pps-gpio.*|dtoverlay=pps-gpio,gpiopin=${PPS_PIN}|\"" in install_script


def test_server_image_boot_config_defaults_to_waveshare_pps_pin():
    run_script = read_text("image/stage-neurokairos-server/00-config-gps-server/00-run.sh")

    assert 'PPS_PIN="${PPS_PIN:-18}"' in run_script
    assert "dtoverlay=pps-gpio,gpiopin=__PPS_PIN__" in run_script


def test_sender_warnings_do_not_claim_gpio_4_is_universal_pps_pin():
    sender_source = read_text("raspberry_pi/sender/irig_sender.c")

    assert "Adafruit" in sender_source
    assert "Waveshare" in sender_source
    assert "BCM GPIO 4 is commonly used for PPS on Adafruit-style GPS HATs" in sender_source
