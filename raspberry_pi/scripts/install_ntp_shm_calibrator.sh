#!/usr/bin/env bash
# Install the NTP SHM offset calibration daemon on a Raspberry Pi running chrony.
#
# Usage:
#   sudo ./install_ntp_shm_calibrator.sh [--privileged-servers "ip1 ip2"]
#
# What this script does:
#   1. Installs ntp_shm_calibrator.py to /usr/local/sbin/
#   2. Creates log/state/status directories with correct permissions
#   3. Adds the SHM refclock line to chrony.conf (once, idempotent)
#   4. Installs and enables the systemd service

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DAEMON_SRC="${SCRIPT_DIR}/ntp_shm_calibrator.py"
DAEMON_DST="/usr/local/sbin/ntp_shm_calibrator.py"
SERVICE_SRC="${SCRIPT_DIR}/../systemd/ntp-shm-calibrator.service"
SERVICE_DST="/etc/systemd/system/ntp-shm-calibrator.service"
CHRONY_CONF="/etc/chrony/chrony.conf"
LOG_DIR="/var/log/ntp-calibrator"
LIB_DIR="/var/lib/ntp-calibrator"

# SHM refclock line added once to chrony.conf.
# Start as noselect; promote to trust after observing WANC for ≥1 day.
SHM_REFCLOCK="refclock SHM 3 refid WANC precision 1e-4 poll 4 noselect"

PRIVILEGED_SERVERS=""

usage() {
    echo "Usage: sudo $0 [--privileged-servers \"ip1 ip2\"]"
    exit 1
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --privileged-servers)
            PRIVILEGED_SERVERS="$2"
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

echo "==> Installing NTP SHM calibration daemon"

# Install the daemon script
install -m 755 "${DAEMON_SRC}" "${DAEMON_DST}"
echo "    Installed ${DAEMON_DST}"

# Create directories
install -d -m 755 "${LOG_DIR}"
install -d -m 755 "${LIB_DIR}"
echo "    Created ${LOG_DIR} and ${LIB_DIR}"

# Add SHM refclock to chrony.conf if not already present
if grep -qF "SHM 3" "${CHRONY_CONF}" 2>/dev/null; then
    echo "    chrony.conf already has SHM 3 refclock entry — skipping"
else
    echo "" >> "${CHRONY_CONF}"
    echo "# NTP SHM calibration daemon — added by install_ntp_shm_calibrator.sh" >> "${CHRONY_CONF}"
    echo "${SHM_REFCLOCK}" >> "${CHRONY_CONF}"
    echo "    Added SHM 3 refclock to ${CHRONY_CONF}"
fi

# Build ExecStart line, adding privileged servers if provided
EXEC_START="/usr/bin/python3 ${DAEMON_DST}"
if [[ -n "${PRIVILEGED_SERVERS}" ]]; then
    EXEC_START="${EXEC_START} --privileged-servers ${PRIVILEGED_SERVERS} --truth-ips ${PRIVILEGED_SERVERS}"
fi

# Install systemd service (substituting ExecStart)
sed "s|ExecStart=.*|ExecStart=${EXEC_START}|" "${SERVICE_SRC}" > "${SERVICE_DST}"
echo "    Installed ${SERVICE_DST}"

# Enable and start
systemctl daemon-reload
systemctl enable ntp-shm-calibrator.service
systemctl restart chrony.service
systemctl start ntp-shm-calibrator.service

echo ""
echo "==> Done. Check status with:"
echo "    journalctl -u ntp-shm-calibrator -f"
echo "    chronyc sources"
echo ""
echo "    WANC appears as 'noselect' until you remove that flag from chrony.conf"
echo "    after observing stable WANC operation for at least 24 hours."
