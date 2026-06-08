#!/bin/bash

# Install and configure chrony + gpsd on a Raspberry Pi with a GPS timing receiver.
#
# This script sets up GPS-disciplined NTP serving:
#   - gpsd reads NMEA data from the GPS serial port and exposes PPS
#   - chrony uses gpsd shared memory (SHM 0) for coarse GPS time and
#     PPS (/dev/pps0) for sub-microsecond accuracy
#   - Pool servers provide fallback if GPS is unavailable
#
# After installation, the RPi becomes a stratum 1 NTP server that other
# machines on the network can sync to.
#
# Prerequisites:
#   - Raspberry Pi with a GPS receiver connected to serial UART
#   - PPS output from GPS wired to the HAT's PPS GPIO (GPIO 4 by default here; many Waveshare GNSS HATs use GPIO 18) with pps-gpio overlay enabled
#   - Serial console disabled (raspi-config -> Interface -> Serial -> No console, Yes hardware)
#
# Usage:
#   sudo ./install_chrony_server.sh [--serial-device /dev/ttyAMA0]

set -euo pipefail

SERIAL_DEVICE="${1:-/dev/ttyAMA0}"

if [ "$(id -u)" -ne 0 ]; then
    echo "Error: This script must be run as root (sudo)."
    exit 1
fi

echo "=== NeuroKairos Chrony Server Installation ==="
echo "GPS serial device: $SERIAL_DEVICE"

# --- Install packages ---
echo ""
echo "Installing gpsd and chrony..."
apt-get update -qq
apt-get install -y gpsd gpsd-clients chrony pps-tools

# --- Configure gpsd ---
echo ""
echo "Configuring gpsd..."
cat > /etc/default/gpsd <<EOF
# gpsd configuration for NeuroKairos GPS timing receiver
START_DAEMON="true"
GPSD_OPTIONS="-n"
DEVICES="$SERIAL_DEVICE /dev/pps0"
USBAUTO="false"
EOF

# --- Enable pps-gpio device tree overlay if not already present ---
if ! grep -q "^dtoverlay=pps-gpio" /boot/config.txt 2>/dev/null && \
   ! grep -q "^dtoverlay=pps-gpio" /boot/firmware/config.txt 2>/dev/null; then
    echo ""
    echo "Adding pps-gpio overlay to boot config..."
    # Try firmware path first (bookworm+), fall back to /boot
    if [ -f /boot/firmware/config.txt ]; then
        echo "dtoverlay=pps-gpio,gpiopin=4" >> /boot/firmware/config.txt
    elif [ -f /boot/config.txt ]; then
        echo "dtoverlay=pps-gpio,gpiopin=4" >> /boot/config.txt
    fi
    echo "NOTE: A reboot is required for the PPS overlay to take effect."
fi

# --- Configure chrony ---
echo ""
echo "Configuring chrony..."
cat > /etc/chrony/chrony.conf <<EOF
# NeuroKairos chrony configuration — GPS-disciplined stratum 1 server
#
# SHM 0: coarse GPS time from gpsd shared memory (NMEA sentences).
# Offset and delay tuned for typical USB/serial GPS receivers.
refclock SHM 0 refid GPS precision 1e-1 offset 0.0 delay 0.2 noselect

# PPS: sub-microsecond pulse-per-second from GPS receiver via /dev/pps0.
# This is the primary time source; SHM 0 provides the second-of-day context.
refclock PPS /dev/pps0 refid PPS precision 1e-7 lock GPS

# Fallback pool servers in case GPS is temporarily unavailable
pool pool.ntp.org iburst maxsources 4

# Allow NTP clients on the local network
allow 192.168.0.0/16
allow 10.0.0.0/8
allow 172.16.0.0/12

# Serve time even when not fully synchronized (keeps clients running)
local stratum 10 orphan

# Record tracking and statistics logs
logdir /var/log/chrony
log tracking measurements statistics

# Step the clock on startup if offset is > 1 second (3 attempts)
makestep 1.0 3

# Hardware timestamping (if NIC supports it)
hwtimestamp *

# RTC drift file
driftfile /var/lib/chrony/chrony.drift
EOF

# --- Enable and start services ---
echo ""
echo "Enabling and starting services..."
systemctl enable gpsd
systemctl restart gpsd

systemctl enable chrony
systemctl restart chrony

# --- Verify GPS is providing data ---
echo ""
echo "Waiting a few seconds for gpsd to acquire data..."
sleep 3

echo ""
echo "=== gpsd status ==="
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
echo "The RPi is now configured as a stratum 1 NTP server."
echo "It may take several minutes for PPS lock to stabilize."
echo "Monitor with: chronyc tracking"
