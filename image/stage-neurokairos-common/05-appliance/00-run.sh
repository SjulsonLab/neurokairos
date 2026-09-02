#!/bin/bash -e

# Appliance basics on BOTH images: force the default user to set a new password
# at first login, show the IP/MAC on the HDMI console before login, and install
# the identity-sanitize tool. (SSH host keys are stripped by pi-gen and
# regenerated per-device on first boot, so each unit gets unique keys.)

install -m 755 files/nk_console_issue.sh "${ROOTFS_DIR}/usr/local/sbin/nk_console_issue.sh"
install -m 755 files/nk_sanitize.sh "${ROOTFS_DIR}/usr/local/sbin/nk_sanitize.sh"
install -m 644 files/nk-console-issue.service "${ROOTFS_DIR}/etc/systemd/system/nk-console-issue.service"
install -m 644 files/nk-console-issue.timer "${ROOTFS_DIR}/etc/systemd/system/nk-console-issue.timer"

# Force a password change on first login via a profile.d prompt (runs passwd as
# root through sudo) instead of `chage -d 0`. Expiry over SSH re-asks for the
# current password and disconnects after the change; this doesn't. A marker file
# arms it; the prompt removes the marker once a new password is set.
install -m 644 files/nk_first_login.sh "${ROOTFS_DIR}/etc/profile.d/nk-first-login.sh"
install -d -m 755 "${ROOTFS_DIR}/var/lib/neurokairos"
touch "${ROOTFS_DIR}/var/lib/neurokairos/default-password"

on_chroot << EOF
systemctl enable nk-console-issue.timer
EOF
