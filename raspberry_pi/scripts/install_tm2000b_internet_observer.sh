#!/bin/bash

# Install the TM2000B internet-only observer logger and switch chrony to
# internet NTP sources only.
#
# This script deliberately leaves the existing TM2000B observer logger in
# place. It installs a separate logger that writes to its own output
# directory and configures chrony to discipline the Pi from public internet
# servers instead of the GNSS hat or the TM2000B appliance.
#
# Usage:
#   sudo ./install_tm2000b_internet_observer.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT=""
if [ -f "${SCRIPT_DIR}/../../raspberry_pi/scripts/log_tm2000b_internet_observer.py" ]; then
    REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
elif [ -f "/usr/local/share/neurokairos/raspberry_pi/scripts/log_tm2000b_internet_observer.py" ]; then
    REPO_ROOT="/usr/local/share/neurokairos"
else
    echo "Error: cannot locate the NeuroKairos source tree."
    echo "Set NEUROKAIROS_SOURCE_ROOT or stage the files under /usr/local/share/neurokairos."
    exit 1
fi

if [ "$(id -u)" -ne 0 ]; then
    echo "Error: This script must be run as root (sudo)."
    exit 1
fi

CHRONY_CONF="/etc/chrony/chrony.conf"
CHRONY_BACKUP="${CHRONY_CONF}.before-internet-only-test-$(date +%Y%m%d-%H%M%S)"
LOGGER_SCRIPT="/usr/local/sbin/log_tm2000b_internet_observer.py"
LOGGER_SERVICE="/etc/systemd/system/tm2000b-internet-observer.service"
LOGGER_TIMER="/etc/systemd/system/tm2000b-internet-observer.timer"

echo "=== NeuroKairos TM2000B Internet-Only Observer Installation ==="

echo ""
echo "Installing the internet-only observer logger..."
install -m 0755 "${REPO_ROOT}/raspberry_pi/scripts/log_tm2000b_observer.py" "/usr/local/sbin/log_tm2000b_observer.py"
install -m 0755 "${REPO_ROOT}/raspberry_pi/scripts/log_tm2000b_internet_observer.py" "${LOGGER_SCRIPT}"
install -m 0644 "${REPO_ROOT}/raspberry_pi/systemd/tm2000b-internet-observer.service" "${LOGGER_SERVICE}"
install -m 0644 "${REPO_ROOT}/raspberry_pi/systemd/tm2000b-internet-observer.timer" "${LOGGER_TIMER}"

echo ""
echo "Backing up and replacing chrony configuration..."
cp "${CHRONY_CONF}" "${CHRONY_BACKUP}"
cat > "${CHRONY_CONF}" <<EOF
# NeuroKairos internet-only chrony test configuration

server time.google.com iburst prefer
server time.cloudflare.com iburst
server time.apple.com iburst
pool pool.ntp.org iburst maxsources 4

# Allow LAN clients on the two lab subnets while the test is running.
allow 10.49.0.0/16
allow 10.42.0.0/16

local stratum 15 orphan

keyfile /etc/chrony/chrony.keys
driftfile /var/lib/chrony/chrony.drift
ntsdumpdir /var/lib/chrony

maxupdateskew 100.0
rtcsync
makestep 1 3
leapsectz right/UTC

logdir /var/log/chrony
log tracking statistics measurements
EOF

echo ""
echo "Creating the output directory..."
mkdir -p /home/pi/tm2000b_internet_observer_logs
chown pi:pi /home/pi/tm2000b_internet_observer_logs
chmod 0755 /home/pi/tm2000b_internet_observer_logs

echo ""
echo "Reloading systemd and enabling the new timer..."
systemctl daemon-reload
systemctl enable tm2000b-internet-observer.timer
systemctl restart chrony
systemctl restart tm2000b-internet-observer.timer

echo ""
echo "=== chrony tracking ==="
chronyc tracking || true

echo ""
echo "=== chrony sources ==="
chronyc sources || true

echo ""
echo "=== Installation complete ==="
echo "Backed up previous chrony.conf to: ${CHRONY_BACKUP}"
echo "New logger: tm2000b-internet-observer.timer"
echo "Logs: /home/pi/tm2000b_internet_observer_logs"
