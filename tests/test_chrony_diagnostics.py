from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_chrony_diagnostics_script_includes_pps_and_journal_checks():
    """The chrony diagnostic script should check PPS and system logs together."""

    script_text = (
        REPO_ROOT / "raspberry_pi" / "scripts" / "test_chrony.sh"
    ).read_text()

    assert "ppstest /dev/pps0" in script_text
    assert "chronyc sources -v" in script_text
    assert "journalctl -u chrony -u gpsd -u gnss-pps" in script_text


def test_chrony_diagnostics_script_supports_time_window_arguments():
    """The chrony diagnostic script should accept an optional log window."""

    script_text = (
        REPO_ROOT / "raspberry_pi" / "scripts" / "test_chrony.sh"
    ).read_text()

    assert "--since" in script_text
    assert "--until" in script_text
    assert "usage" in script_text.lower()
