#!/bin/bash -e

# Install the cross-compiled irig_sender binary and its systemd unit.
# The binary is cross-compiled on the host in the CI workflow and dropped
# into this stage's files/ directory before pi-gen runs. We do not compile
# inside the rootfs — that path depends on qemu-user-static + binfmt_misc
# working reliably through pi-gen stage boundaries, which it doesn't.

install -m 755 files/irig_sender "${ROOTFS_DIR}/usr/local/bin/irig_sender"
install -m 644 files/irig-sender.service "${ROOTFS_DIR}/etc/systemd/system/irig-sender.service"
install -d -m 755 "${ROOTFS_DIR}/var/log/irig-sender"

on_chroot << EOF
systemctl enable irig-sender.service
EOF
