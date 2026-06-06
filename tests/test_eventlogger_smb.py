from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path: str) -> str:
    return (REPO_ROOT / relative_path).read_text()


def test_eventlogger_smb_share_template_is_guest_readonly():
    share = read_text("raspberry_pi/eventlogger/eventlogger.smb.conf.example")

    assert "[neurokairos-eventlogger]" in share
    assert "path = /var/lib/neurokairos/eventlogger" in share
    assert "browseable = yes" in share
    assert "guest ok = yes" in share
    assert "read only = yes" in share


def test_install_script_provisions_samba_share():
    install_script = read_text("raspberry_pi/eventlogger/install_eventlogger.sh")

    assert "apt-get install -y samba" in install_script
    assert "/var/lib/neurokairos/eventlogger" in install_script
    assert "eventlogger.smb.conf.example" in install_script
    assert "neurokairos-eventlogger" in install_script
    assert "systemctl restart smbd" in install_script
