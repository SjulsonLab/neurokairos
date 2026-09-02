#!/bin/bash -e

# Appliance basics on BOTH images: force the default user to set a new password
# at first login, show the IP/MAC on the HDMI console before login, and install
# the identity-sanitize tool. (SSH host keys are stripped by pi-gen and
# regenerated per-device on first boot, so each unit gets unique keys.)

install -m 755 files/nk_console_issue.sh "${ROOTFS_DIR}/usr/local/sbin/nk_console_issue.sh"
install -m 755 files/nk_sanitize.sh "${ROOTFS_DIR}/usr/local/sbin/nk_sanitize.sh"
install -m 644 files/nk-console-issue.service "${ROOTFS_DIR}/etc/systemd/system/nk-console-issue.service"
install -m 644 files/nk-console-issue.timer "${ROOTFS_DIR}/etc/systemd/system/nk-console-issue.timer"

# Force a password change on first login via a profile.d prompt instead of
# `chage -d 0`. Expiry over SSH re-asks for the current password and disconnects
# after the change; this doesn't. The prompt runs a small root helper
# (nk_set_password) through a scoped NOPASSWD sudoers rule, so it works even
# though this image's user does NOT have blanket passwordless sudo. A marker file
# arms it; the helper removes the marker once a new password is set, and refuses
# to run without it, so the standing grant is harmless afterward.
install -m 644 files/nk_first_login.sh "${ROOTFS_DIR}/etc/profile.d/nk-first-login.sh"
install -m 755 files/nk_set_password.sh "${ROOTFS_DIR}/usr/local/sbin/nk_set_password"
install -d -m 755 "${ROOTFS_DIR}/var/lib/neurokairos"
touch "${ROOTFS_DIR}/var/lib/neurokairos/default-password"

# Scoped NOPASSWD rule: neurokairos may run only the password helper without a
# sudo password. sudoers files must be 0440 and syntactically valid.
cat > "${ROOTFS_DIR}/etc/sudoers.d/010_nk-first-login" <<'SUDOERS'
# Allow the first-login prompt to set the initial password without a sudo
# password (the user hasn't set one yet). Scoped to the single helper, which is
# a no-op once the first-login marker is cleared.
neurokairos ALL=(root) NOPASSWD: /usr/local/sbin/nk_set_password
SUDOERS
chmod 0440 "${ROOTFS_DIR}/etc/sudoers.d/010_nk-first-login"

on_chroot << EOF
# Fail the build if the sudoers drop-in is malformed rather than shipping a
# broken sudo config.
visudo -cf /etc/sudoers.d/010_nk-first-login
systemctl enable nk-console-issue.timer
EOF
