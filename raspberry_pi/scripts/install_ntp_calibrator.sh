#!/usr/bin/env bash
# Install the NTP offset calibration daemon on a Raspberry Pi running chrony.
#
# Usage:
#   sudo ./install_ntp_calibrator.sh [--ntfy-topic TOPIC]
#
# What this script does:
#   1. Installs ntp_calibrator.py to /usr/local/sbin/
#   2. Creates log/state directories with correct permissions
#   3. Writes /etc/ntp-calibrator.json config (created if missing)
#   4. Writes /etc/ntp-calibrator-ntfy.json (if --ntfy-topic is given)
#   5. Installs and enables the systemd service

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DAEMON_SRC="${SCRIPT_DIR}/ntp_calibrator.py"
DAEMON_DST="/usr/local/sbin/ntp_calibrator.py"

# Service file: look next to this script first, then in the sibling systemd/ dir
if [[ -f "${SCRIPT_DIR}/ntp-calibrator.service" ]]; then
    SERVICE_SRC="${SCRIPT_DIR}/ntp-calibrator.service"
else
    SERVICE_SRC="${SCRIPT_DIR}/../systemd/ntp-calibrator.service"
fi
SERVICE_DST="/etc/systemd/system/ntp-calibrator.service"

CONFIG_DST="/etc/ntp-calibrator.json"
NTFY_CONFIG_DST="/etc/ntp-calibrator-ntfy.json"
LOG_DIR="/var/log/ntp-calibrator"
LIB_DIR="/var/lib/ntp-calibrator"

NTFY_TOPIC=""

usage() {
    echo "Usage: sudo $0 [--ntfy-topic TOPIC]"
    exit 1
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --ntfy-topic)
            NTFY_TOPIC="$2"
            shift 2
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
done

if [[ $EUID -ne 0 ]]; then
    echo "Error: this script must be run as root" >&2
    exit 1
fi

echo "==> Installing NTP calibration daemon"

# Install the daemon script
install -m 755 "${DAEMON_SRC}" "${DAEMON_DST}"
echo "    Installed ${DAEMON_DST}"

# Create directories
install -d -m 755 "${LOG_DIR}"
install -d -m 755 "${LIB_DIR}"
echo "    Created ${LOG_DIR} and ${LIB_DIR}"

# Write main config if not present
if [[ ! -f "${CONFIG_DST}" ]]; then
    cat > "${CONFIG_DST}" <<'EOF'
{
  "poll": 6,
  "active_pool_size": 3,
  "ntfy_config_path": "/etc/ntp-calibrator-ntfy.json"
}
EOF
    echo "    Created ${CONFIG_DST}"
else
    echo "    ${CONFIG_DST} already exists — not overwriting"
fi

# Write ntfy config if a topic was given
if [[ -n "${NTFY_TOPIC}" ]]; then
    cat > "${NTFY_CONFIG_DST}" <<EOF
{
  "server": "https://ntfy.sh",
  "topic": "${NTFY_TOPIC}"
}
EOF
    echo "    Created ${NTFY_CONFIG_DST} (topic=${NTFY_TOPIC})"
fi

# Install systemd service
install -m 644 "${SERVICE_SRC}" "${SERVICE_DST}"
echo "    Installed ${SERVICE_DST}"

# Enable and start
systemctl daemon-reload
systemctl enable ntp-calibrator.service
systemctl restart chrony.service
systemctl start ntp-calibrator.service

echo ""
echo "==> Done. Check status with:"
echo "    journalctl -u ntp-calibrator -f"
echo "    chronyc sources"
