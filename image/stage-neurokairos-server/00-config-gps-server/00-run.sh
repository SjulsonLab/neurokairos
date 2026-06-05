#!/bin/bash -e

install -m 644 files/chrony.conf "${ROOTFS_DIR}/etc/chrony/chrony.conf"
install -m 644 files/gpsd "${ROOTFS_DIR}/etc/default/gpsd"
install -m 755 files/configure_gnss_pps.sh "${ROOTFS_DIR}/usr/local/sbin/configure_gnss_pps.sh"
install -m 644 files/gnss-pps.service "${ROOTFS_DIR}/etc/systemd/system/gnss-pps.service"
install -m 755 ../../../raspberry_pi/scripts/log_gnss_status.py "${ROOTFS_DIR}/usr/local/sbin/log_gnss_status.py"
install -m 644 ../../../raspberry_pi/systemd/gnss-status-logger.service "${ROOTFS_DIR}/etc/systemd/system/gnss-status-logger.service"
install -m 644 ../../../raspberry_pi/systemd/gnss-status-logger.timer "${ROOTFS_DIR}/etc/systemd/system/gnss-status-logger.timer"

mkdir -p "${ROOTFS_DIR}/etc/systemd/system/chrony.service.d"
cat > "${ROOTFS_DIR}/etc/systemd/system/chrony.service.d/network-online.conf" <<'EOF'
[Unit]
Wants=network-online.target
After=network-online.target
EOF

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
# PPS pulse from Waveshare NEO-M8T HAT wired to BCM GPIO 18.
# Waveshare documents the PPS edge as capture_clear.
dtoverlay=pps-gpio,gpiopin=18,capture_clear,pull=up
# Enable the primary UART (/dev/ttyAMA0) for the GPS NMEA stream
enable_uart=1
# Free /dev/ttyAMA0 from Bluetooth so the GPS can use it (Pi 3/4)
dtoverlay=disable-bt
EOF

# Drop the serial console from kernel cmdline so it doesn't fight gpsd for the UART.
sed -i 's/ \?console=serial0,[0-9]\+//g; s/ \?console=ttyAMA0,[0-9]\+//g' "${CMDLINE_TXT}"

on_chroot systemctl disable hciuart.service || true
on_chroot systemctl disable serial-getty@ttyAMA0.service || true

on_chroot systemctl enable chrony
on_chroot systemctl enable gpsd
on_chroot systemctl enable gnss-pps
on_chroot systemctl enable gnss-status-logger.timer

echo "neurokairos-server" > "${ROOTFS_DIR}/etc/hostname"
sed -i 's/127\.0\.1\.1.*/127.0.1.1\tneurokairos-server/' "${ROOTFS_DIR}/etc/hosts"
