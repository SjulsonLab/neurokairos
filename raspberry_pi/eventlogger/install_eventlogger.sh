#!/bin/bash

# Install the NeuroKairos TTL event logger as a standalone systemd service.
# The default config logs BCM GPIO 5, 6, 13, 19, 26, 16, 20, and 21 to match the Pi header layout.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SMB_CONF="/etc/samba/smb.conf"
SMB_SHARE_MARKER_BEGIN="# BEGIN NeuroKairos event logger share"
SMB_SHARE_MARKER_END="# END NeuroKairos event logger share"
INSTALL_LIB_DIR="/usr/local/lib/neurokairos-eventlogger"
JOURNAL_README_PATH="/var/lib/neurokairos/eventlogger/journal/README.txt"
RECORDINGS_README_PATH="/var/lib/neurokairos/eventlogger/recordings/README.txt"

echo "Installing NeuroKairos event logger..."

if ! pkg-config --exists libgpiod; then
    echo "Missing libgpiod development files. Install with:"
    echo "  sudo apt-get install -y libgpiod-dev"
    exit 1
fi

if ! command -v testparm >/dev/null 2>&1 || ! command -v avahi-publish >/dev/null 2>&1; then
    echo "Installing Samba and mDNS support..."
    sudo apt-get update
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y samba avahi-utils
fi

make -C "$SCRIPT_DIR"

sudo install -m 0755 "$SCRIPT_DIR/neurokairos-eventlogger" /usr/local/bin/neurokairos-eventlogger
sudo install -m 0755 "$SCRIPT_DIR/eventlogger_export.py" /usr/local/bin/neurokairos-eventlogger-export
sudo install -d -m 0755 /etc/neurokairos
sudo install -d -m 0755 /var/lib/neurokairos/eventlogger
sudo install -d -m 0755 /var/lib/neurokairos/eventlogger/control
sudo install -d -m 0755 /var/lib/neurokairos/eventlogger/journal
sudo install -d -m 0755 /var/lib/neurokairos/eventlogger/recordings
sudo install -d -m 0755 /var/lib/neurokairos/eventlogger/journal/2026-06_journal
sudo install -d -m 0755 /var/lib/neurokairos/eventlogger/recordings/2026-06_recordings
sudo tee "$JOURNAL_README_PATH" >/dev/null <<'EOF'
NeuroKairos event journal

This folder contains the raw, append-only TTL journal written by the capture
daemon. Files are organized by month in subfolders such as `2026-06_journal/`.
These logs are the source of truth for diagnostics and later export.
If you forgot to hit record or accidentally deleted your event recording, the
UTC timestamps for the events can be retrieved from these logs.

Retention policy:
- The oldest inactive journal files are deleted automatically when free disk
  space falls below the configured cleanup threshold.
- Active files are never deleted.
- Journal files are not user-editable.
EOF
sudo tee "$RECORDINGS_README_PATH" >/dev/null <<'EOF'
NeuroKairos recordings

This folder contains exported user recordings derived from the raw journal.
Files are organized by month in subfolders such as `2026-06_recordings/`.
Each recording includes a TSV file containing the UTC timestamps of detected
events, as well as a YAML file containing the metadata and user's notes.

Retention policy:
- The oldest inactive recording exports are deleted automatically when free
  disk space falls below the configured cleanup threshold.
- Active recordings are preserved until they are stopped and exported.
- The recording files are user-editable, but we recommend against editing
  them in place.
EOF
sudo install -d -m 0755 "$INSTALL_LIB_DIR"
sudo install -d -m 0755 "$INSTALL_LIB_DIR/web"
sudo install -m 0644 "$SCRIPT_DIR/eventlogger_control.py" "$INSTALL_LIB_DIR/eventlogger_control.py"
sudo install -m 0755 "$SCRIPT_DIR/publish_neurokairos_alias.sh" "$INSTALL_LIB_DIR/publish_neurokairos_alias.sh"
sudo install -m 0644 "$SCRIPT_DIR/web/index.html" "$INSTALL_LIB_DIR/web/index.html"

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
sudo install -m 0644 "$SCRIPT_DIR/eventlogger-control.service" /etc/systemd/system/neurokairos-eventlogger-control.service
sudo install -m 0644 "$SCRIPT_DIR/neurokairos-local-alias.service" /etc/systemd/system/neurokairos-local-alias.service
sudo systemctl daemon-reload
sudo systemctl enable neurokairos-eventlogger.service
sudo systemctl enable neurokairos-eventlogger-control.service
sudo systemctl enable neurokairos-local-alias.service
sudo systemctl restart neurokairos-eventlogger.service
sudo systemctl restart neurokairos-eventlogger-control.service
sudo systemctl restart neurokairos-local-alias.service

echo "Event logger installed."
echo "Useful commands:"
echo "  sudo systemctl status neurokairos-eventlogger"
echo "  sudo systemctl status neurokairos-eventlogger-control"
echo "  sudo journalctl -u neurokairos-eventlogger -f"
echo "  sudo journalctl -u neurokairos-eventlogger-control -f"
echo "  ls -lh /var/lib/neurokairos/eventlogger/journal"
echo "  ls -lh /var/lib/neurokairos/eventlogger/recordings"
echo "  smbclient -L localhost -N"
echo "  open http://neurokairos.local"
