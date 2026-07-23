#!/bin/bash -e

# Install the NeuroKairos dashboard — SERVER images only. This is the web UI +
# aggregator that discovers every Pi's status agent over mDNS and renders the
# grid, plus the neurokairos.local mDNS alias publisher so the dashboard has one
# friendly URL. Scripts/units/UI are CI-staged from raspberry_pi/ (source of
# truth). avahi-daemon/avahi-utils come from the common status-agent step.

install -m 755 files/nk_dashboard.py "${ROOTFS_DIR}/usr/local/sbin/nk_dashboard.py"
install -m 755 files/nk_publish_alias.sh "${ROOTFS_DIR}/usr/local/sbin/nk_publish_alias.sh"
install -m 644 files/nk-dashboard.service "${ROOTFS_DIR}/etc/systemd/system/nk-dashboard.service"
install -m 644 files/nk-dashboard-alias.service "${ROOTFS_DIR}/etc/systemd/system/nk-dashboard-alias.service"

install -d -m 755 "${ROOTFS_DIR}/usr/local/share/neurokairos-dashboard"
install -m 644 files/index.html "${ROOTFS_DIR}/usr/local/share/neurokairos-dashboard/index.html"

# The dashboard's pulse-anomaly log + auth live here. Senders reach the
# dashboard via the neurokairos.local mDNS alias (nk-dashboard-alias.service).
install -d -m 755 "${ROOTFS_DIR}/var/lib/neurokairos"

on_chroot << EOF
systemctl enable nk-dashboard.service
systemctl enable nk-dashboard-alias.service
EOF
