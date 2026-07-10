#!/bin/bash -e

# Install the LAN NTP-server discovery daemon (client/sender image only).
# It browses mDNS for NeuroKairos servers and adds them to chrony's sourcedir
# at runtime (no chronyd restart). The .py + units are the source of truth under
# raspberry_pi/; the CI workflow (and local-build docs) drop them into files/
# before pi-gen runs, mirroring how ntp_calibrator.py is staged.

install -m 755 files/nk_discover_ntp.py "${ROOTFS_DIR}/usr/local/sbin/nk_discover_ntp.py"
install -m 644 files/nk-discover-ntp.service "${ROOTFS_DIR}/etc/systemd/system/nk-discover-ntp.service"
install -m 644 files/nk-discover-ntp.timer "${ROOTFS_DIR}/etc/systemd/system/nk-discover-ntp.timer"

# chrony reads *.sources from here (see the sourcedir directive in chrony.conf).
install -d -m 755 "${ROOTFS_DIR}/etc/chrony/sources.d"

on_chroot << EOF
systemctl enable avahi-daemon
systemctl enable nk-discover-ntp.timer
EOF
