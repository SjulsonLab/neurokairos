#!/bin/bash -e

# Compile irig_sender inside the rootfs (via QEMU userspace emulation) and
# install it plus its systemd unit. Source files are copied into this stage's
# `files/` directory by the CI workflow before pi-gen runs, so this script
# operates only on files local to the stage.

install -d "${ROOTFS_DIR}/tmp/sender"
install -m 644 files/irig_sender.c files/Makefile "${ROOTFS_DIR}/tmp/sender/"

on_chroot make -C /tmp/sender

install -m 755 "${ROOTFS_DIR}/tmp/sender/irig_sender" "${ROOTFS_DIR}/usr/local/bin/irig_sender"
install -m 644 files/irig-sender.service "${ROOTFS_DIR}/etc/systemd/system/irig-sender.service"
install -d -m 755 "${ROOTFS_DIR}/var/log/irig-sender"

on_chroot systemctl enable irig-sender.service

rm -rf "${ROOTFS_DIR}/tmp/sender"
