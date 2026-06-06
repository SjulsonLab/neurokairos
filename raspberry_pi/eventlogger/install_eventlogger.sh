#!/bin/bash

# Install the NeuroKairos TTL event logger as a standalone systemd service.
# The default config logs BCM GPIO 5, 6, 10, 11, 12, 13, 16, and 17.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SMB_CONF="/etc/samba/smb.conf"
SMB_SHARE_MARKER_BEGIN="# BEGIN NeuroKairos event logger share"
SMB_SHARE_MARKER_END="# END NeuroKairos event logger share"

echo "Installing NeuroKairos event logger..."

if ! pkg-config --exists libgpiod; then
    echo "Missing libgpiod development files. Install with:"
    echo "  sudo apt-get install -y libgpiod-dev"
    exit 1
fi

if ! command -v testparm >/dev/null 2>&1; then
    echo "Installing Samba support..."
    sudo apt-get update
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y samba
fi

make -C "$SCRIPT_DIR"

sudo install -m 0755 "$SCRIPT_DIR/neurokairos-eventlogger" /usr/local/bin/neurokairos-eventlogger
sudo install -d -m 0755 /etc/neurokairos
sudo install -d -m 0755 /var/lib/neurokairos/eventlogger
sudo install -d -m 0755 /var/lib/neurokairos/eventlogger/journal

if [ ! -f /etc/neurokairos/eventlogger.conf ]; then
    sudo install -m 0644 "$SCRIPT_DIR/eventlogger.conf.example" /etc/neurokairos/eventlogger.conf
    echo "Installed default config at /etc/neurokairos/eventlogger.conf"
else
    echo "Keeping existing /etc/neurokairos/eventlogger.conf"
fi

tmp_smb_conf="$(mktemp)"
awk -v marker_begin="$SMB_SHARE_MARKER_BEGIN" -v marker_end="$SMB_SHARE_MARKER_END" '
    $0 == marker_begin { in_block = 1; next }
    $0 == marker_end { in_block = 0; next }
    in_block { next }
    { print }
' "$SMB_CONF" > "$tmp_smb_conf"
printf '\n' | cat "$tmp_smb_conf" - "$SCRIPT_DIR/eventlogger.smb.conf.example" | sudo tee "$SMB_CONF" > /dev/null
rm -f "$tmp_smb_conf"

sudo testparm -s >/dev/null
sudo systemctl enable smbd
sudo systemctl restart smbd

sudo install -m 0644 "$SCRIPT_DIR/eventlogger.service" /etc/systemd/system/neurokairos-eventlogger.service
sudo systemctl daemon-reload
sudo systemctl enable neurokairos-eventlogger.service
sudo systemctl restart neurokairos-eventlogger.service

echo "Event logger installed."
echo "Useful commands:"
echo "  sudo systemctl status neurokairos-eventlogger"
echo "  sudo journalctl -u neurokairos-eventlogger -f"
echo "  ls -lh /var/lib/neurokairos/eventlogger/journal"
echo "  smbclient -L localhost -N"
