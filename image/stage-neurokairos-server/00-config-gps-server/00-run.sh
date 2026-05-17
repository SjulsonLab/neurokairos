#!/bin/bash -e

install -m 644 files/chrony.conf "${ROOTFS_DIR}/etc/chrony/chrony.conf"
install -m 644 files/gpsd "${ROOTFS_DIR}/etc/default/gpsd"

# Patch boot firmware config for PPS + UART. Bookworm puts /boot at
# /boot/firmware/ inside the rootfs.
CONFIG_TXT="${ROOTFS_DIR}/boot/firmware/config.txt"
CMDLINE_TXT="${ROOTFS_DIR}/boot/firmware/cmdline.txt"

if [ ! -f "${CONFIG_TXT}" ]; then
	echo "error: ${CONFIG_TXT} not found — pi-gen layout may have changed" >&2
	exit 1
fi

cat >> "${CONFIG_TXT}" <<'EOF'

# --- NeuroKairos GPS / PPS configuration ---
# PPS pulse from GPS receiver wired to GPIO 4
dtoverlay=pps-gpio,gpiopin=4
# Enable the primary UART (/dev/ttyAMA0) for the GPS NMEA stream
enable_uart=1
# Free /dev/ttyAMA0 from Bluetooth so the GPS can use it (Pi 3/4)
dtoverlay=disable-bt
EOF

# Drop the serial console from kernel cmdline so it doesn't fight gpsd for the UART.
sed -i 's/ \?console=serial0,[0-9]\+//g; s/ \?console=ttyAMA0,[0-9]\+//g' "${CMDLINE_TXT}"

on_chroot << EOF
systemctl disable hciuart.service || true
systemctl disable serial-getty@ttyAMA0.service || true
systemctl enable chrony
systemctl enable gpsd
EOF

echo "neurokairos-server" > "${ROOTFS_DIR}/etc/hostname"
sed -i 's/127\.0\.1\.1.*/127.0.1.1\tneurokairos-server/' "${ROOTFS_DIR}/etc/hosts"
