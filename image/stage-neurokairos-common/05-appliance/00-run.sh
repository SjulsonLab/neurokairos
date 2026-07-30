#!/bin/bash -e

# Appliance basics on BOTH images: force the default user to set a new password
# at first login, show the IP/MAC on the HDMI console before login, and install
# the identity-sanitize tool. (SSH host keys are stripped by pi-gen and
# regenerated per-device on first boot, so each unit gets unique keys.)

install -m 755 files/nk_console_issue.sh "${ROOTFS_DIR}/usr/local/sbin/nk_console_issue.sh"
install -m 755 files/nk_sanitize.sh "${ROOTFS_DIR}/usr/local/sbin/nk_sanitize.sh"
install -m 644 files/nk-console-issue.service "${ROOTFS_DIR}/etc/systemd/system/nk-console-issue.service"
install -m 644 files/nk-console-issue.timer "${ROOTFS_DIR}/etc/systemd/system/nk-console-issue.timer"

on_chroot << EOF
# Expire the default 'neurokairos' password so a new one must be set at first login.
chage -d 0 neurokairos || true
systemctl enable nk-console-issue.timer
EOF
