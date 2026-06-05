#!/bin/bash

# Install the GNSS status logger and enable the systemd timer.
#
# This logger samples gpsd and chrony once per minute, writes daily TSV
# snapshots, and keeps a small change-only JSONL event log in /home/pi/GNSS_logs.
#
# Usage:
#   sudo ./install_gnss_status_logger.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

if [ "$(id -u)" -ne 0 ]; then
    echo "Error: This script must be run as root (sudo)."
    exit 1
fi

echo "=== NeuroKairos GNSS Status Logger Installation ==="

install -m 0755 "${REPO_ROOT}/raspberry_pi/scripts/log_gnss_status.py" /usr/local/sbin/log_gnss_status.py
install -m 0644 "${REPO_ROOT}/raspberry_pi/systemd/gnss-status-logger.service" /etc/systemd/system/gnss-status-logger.service
install -m 0644 "${REPO_ROOT}/raspberry_pi/systemd/gnss-status-logger.timer" /etc/systemd/system/gnss-status-logger.timer

mkdir -p /home/pi/GNSS_logs
chmod 0755 /home/pi/GNSS_logs

systemctl daemon-reload
systemctl enable gnss-status-logger.timer
systemctl restart gnss-status-logger.timer

echo "GNSS status logger installed."
echo "Timer: gnss-status-logger.timer"
echo "Logs: /home/pi/GNSS_logs/gnss_m8t_status_YYYY-MM-DD.tsv"
echo "Events: /home/pi/GNSS_logs/gnss_m8t_status_events_YYYY-MM-DD.jsonl"
