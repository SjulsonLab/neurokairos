#!/bin/bash

# Install and configure chrony + gpsd for a GNSS-disciplined Raspberry Pi server.
#
# This script configures the Pi to:
#   - read GNSS time from the u-blox receiver on /dev/ttyAMA0
#   - use Waveshare NEO-M8T PPS on GPIO 18 via /dev/pps0
#   - keep PPS active during GNSS holdover by programming the receiver at boot
#   - refuse NTP client access from the 10.49/10.42 lab subnets during tests
#   - use Ethernet hardware timestamping if the NIC supports it
#   - install persistent GNSS/chrony/PPS status logging
#
# Usage:
#   sudo ./install_chrony_server.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT=""
if [ -f "${SCRIPT_DIR}/../../raspberry_pi/scripts/configure_gnss_pps.sh" ]; then
    REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
elif [ -f "/usr/local/share/neurokairos/raspberry_pi/scripts/configure_gnss_pps.sh" ]; then
    REPO_ROOT="/usr/local/share/neurokairos"
else
    echo "Error: cannot locate the NeuroKairos source tree."
    echo "Set NEUROKAIROS_SOURCE_ROOT or stage the files under /usr/local/share/neurokairos."
    exit 1
fi
GNSS_BOOT_SCRIPT="/usr/local/sbin/configure_gnss_pps.sh"
GNSS_SERVICE_FILE="/etc/systemd/system/gnss-pps.service"
GNSS_LOGGER_SCRIPT="/usr/local/sbin/log_gnss_status.py"
GNSS_LOGGER_SERVICE="/etc/systemd/system/gnss-status-logger.service"
GNSS_LOGGER_TIMER="/etc/systemd/system/gnss-status-logger.timer"
CHRONY_OVERRIDE_DIR="/etc/systemd/system/chrony.service.d"
CHRONY_OVERRIDE_FILE="${CHRONY_OVERRIDE_DIR}/network-online.conf"
GNSS_SERIAL_DEVICE="/dev/ttyAMA0"
GNSS_PPS_DEVICE="/dev/pps0"
GNSS_PPS_GPIO="18"
ETHERNET_INTERFACE="eth0"

if [ "$(id -u)" -ne 0 ]; then
    echo "Error: This script must be run as root (sudo)."
    exit 1
fi

echo "=== NeuroKairos GNSS Chrony Server Installation ==="
echo "GNSS serial device: ${GNSS_SERIAL_DEVICE}"
echo "GNSS PPS device: ${GNSS_PPS_DEVICE}"
echo "GNSS PPS GPIO: ${GNSS_PPS_GPIO}"

# --- Install packages ---
echo ""
echo "Installing chrony, gpsd, and PPS tools..."
apt-get update -qq
apt-get install -y chrony gpsd gpsd-clients pps-tools

# --- Configure gpsd ---
echo ""
echo "Configuring gpsd..."
cat > /etc/default/gpsd <<EOF
# gpsd configuration for NeuroKairos GNSS time server
START_DAEMON="true"
GPSD_OPTIONS="-n"
DEVICES="${GNSS_SERIAL_DEVICE} ${GNSS_PPS_DEVICE}"
USBAUTO="false"
EOF

# --- Enable PPS GPIO overlay ---
echo ""
echo "Configuring PPS GPIO overlay..."
if [ -f /boot/firmware/config.txt ]; then
    BOOT_CONFIG="/boot/firmware/config.txt"
elif [ -f /boot/config.txt ]; then
    BOOT_CONFIG="/boot/config.txt"
else
    BOOT_CONFIG=""
fi

if [ -n "${BOOT_CONFIG}" ]; then
    sed -i '/^dtoverlay=pps-gpio/d' "${BOOT_CONFIG}"
    echo "dtoverlay=pps-gpio,gpiopin=${GNSS_PPS_GPIO},capture_clear,pull=up" >> "${BOOT_CONFIG}"
fi

CMDLINE_TXT=""
if [ -f /boot/firmware/cmdline.txt ]; then
    CMDLINE_TXT="/boot/firmware/cmdline.txt"
elif [ -f /boot/cmdline.txt ]; then
    CMDLINE_TXT="/boot/cmdline.txt"
fi

if [ -n "${CMDLINE_TXT}" ]; then
    sed -i 's/ \?console=serial0,[0-9]\+//g; s/ \?console=ttyAMA0,[0-9]\+//g' "${CMDLINE_TXT}"
fi

# --- Install boot-time GNSS configuration ---
echo ""
echo "Installing GNSS PPS boot script..."
install -m 0755 "${REPO_ROOT}/raspberry_pi/scripts/configure_gnss_pps.sh" "${GNSS_BOOT_SCRIPT}"
install -m 0644 "${REPO_ROOT}/raspberry_pi/systemd/gnss-pps.service" "${GNSS_SERVICE_FILE}"

# --- Configure chrony ---
echo ""
echo "Configuring chrony..."
cat > /etc/chrony/chrony.conf <<EOF
# NeuroKairos Waveshare NEO-M8T test chrony configuration

confdir /etc/chrony/conf.d

hwtimestamp ${ETHERNET_INTERFACE}

# Coarse GNSS time from gpsd shared memory. Chrony uses this only to label PPS
# seconds; it is not selected as the disciplining source.
refclock SHM 0 refid GPS precision 1e-1 noselect

# Direct kernel PPS from the Waveshare NEO-M8T HAT.
refclock PPS ${GNSS_PPS_DEVICE} refid PPS lock GPS precision 1e-7 prefer

# Allow NTP service generally, but refuse the lab client ranges during testing.
allow all
deny 10.49.0.0/16
deny 10.42.0.0/16

# Serve a clearly-low-quality local source if all GNSS/PPS truth is lost.
local stratum 15 orphan

keyfile /etc/chrony/chrony.keys
driftfile /var/lib/chrony/chrony.drift
ntsdumpdir /var/lib/chrony

maxupdateskew 100.0
rtcsync
makestep 1 3
leapsectz right/UTC

# Keep chrony diagnostics available for holdover analysis.
logdir /var/log/chrony
log tracking statistics measurements
EOF

mkdir -p "${CHRONY_OVERRIDE_DIR}"
cat > "${CHRONY_OVERRIDE_FILE}" <<EOF
[Unit]
Wants=network-online.target
After=network-online.target
EOF

# --- Install persistent status logger ---
echo ""
echo "Installing GNSS status logger..."
install -m 0755 "${REPO_ROOT}/raspberry_pi/scripts/log_gnss_status.py" "${GNSS_LOGGER_SCRIPT}"
install -m 0644 "${REPO_ROOT}/raspberry_pi/systemd/gnss-status-logger.service" "${GNSS_LOGGER_SERVICE}"
install -m 0644 "${REPO_ROOT}/raspberry_pi/systemd/gnss-status-logger.timer" "${GNSS_LOGGER_TIMER}"
mkdir -p /home/pi/GNSS_logs
chmod 0755 /home/pi/GNSS_logs

# --- Enable and restart services ---
echo ""
echo "Reloading systemd and enabling services..."
systemctl daemon-reload
systemctl enable gpsd
systemctl enable chrony
systemctl enable gnss-pps
systemctl enable gnss-status-logger.timer

systemctl disable hciuart.service 2>/dev/null || true
systemctl disable serial-getty@ttyAMA0.service 2>/dev/null || true

systemctl restart chrony
systemctl restart gpsd
systemctl restart gnss-pps
systemctl restart gnss-status-logger.timer

# --- Verify GPS and chrony ---
echo ""
echo "Waiting a few seconds for gpsd to acquire data..."
sleep 3

echo ""
echo "=== gpsd status ==="
systemctl is-active --quiet gpsd && echo "gpsd: RUNNING" || echo "gpsd: NOT RUNNING"
if command -v gpspipe >/dev/null 2>&1; then
    timeout 5 gpspipe -w 2>/dev/null | head -3 || echo "(no GPS data yet — receiver may need sky view)"
fi

echo ""
echo "=== chrony tracking ==="
chronyc tracking || true

echo ""
echo "=== chrony sources ==="
chronyc sources || true

echo ""
echo "=== Installation complete ==="
echo "The Pi is now configured as a Waveshare NEO-M8T GNSS chrony test server."
echo "Monitor with: chronyc tracking"
echo "GNSS logs: /home/pi/GNSS_logs"
