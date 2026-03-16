#!/bin/bash

# Install and configure chrony as an NTP client (no GPS hardware required).
#
# Uses public NTP pool servers by default. Optionally specify a custom NTP
# server (e.g., the NeuroKairos RPi acting as a stratum 1 GPS-disciplined server).
#
# Usage:
#   sudo ./install_chrony_client.sh [--server 192.168.1.100]

set -euo pipefail

CUSTOM_SERVER=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --server)
            CUSTOM_SERVER="$2"
            shift 2
            ;;
        *)
            echo "Usage: sudo $0 [--server NTP_SERVER_IP]"
            exit 1
            ;;
    esac
done

if [ "$(id -u)" -ne 0 ]; then
    echo "Error: This script must be run as root (sudo)."
    exit 1
fi

echo "=== NeuroKairos Chrony Client Installation ==="

# --- Install chrony ---
echo ""
echo "Installing chrony..."
apt-get update -qq
apt-get install -y chrony

# --- Configure chrony ---
echo ""
echo "Configuring chrony..."
cat > /etc/chrony/chrony.conf <<EOF
# NeuroKairos chrony client configuration
EOF

if [ -n "$CUSTOM_SERVER" ]; then
    echo "Using custom NTP server: $CUSTOM_SERVER"
    cat >> /etc/chrony/chrony.conf <<EOF

# Primary NTP server (e.g., NeuroKairos RPi with GPS)
server $CUSTOM_SERVER iburst prefer
EOF
fi

cat >> /etc/chrony/chrony.conf <<EOF

# Public pool servers as primary/fallback
pool pool.ntp.org iburst maxsources 4

# Record tracking logs
logdir /var/log/chrony
log tracking measurements statistics

# Step the clock on startup if offset is > 1 second (3 attempts)
makestep 1.0 3

# Hardware timestamping (if NIC supports it; silently ignored if not)
hwtimestamp *

# RTC drift file
driftfile /var/lib/chrony/chrony.drift
EOF

# --- Enable and start chrony ---
echo ""
echo "Enabling and starting chrony..."
systemctl enable chrony
systemctl restart chrony

# --- Verify ---
sleep 2

echo ""
echo "=== chrony tracking ==="
chronyc tracking || true

echo ""
echo "=== chrony sources ==="
chronyc sources || true

echo ""
echo "=== Installation complete ==="
if [ -n "$CUSTOM_SERVER" ]; then
    echo "Chrony configured with preferred server: $CUSTOM_SERVER"
else
    echo "Chrony configured with public pool servers."
fi
echo "Monitor with: chronyc tracking"
