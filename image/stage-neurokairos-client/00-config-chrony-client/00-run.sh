#!/bin/bash -e

install -m 644 files/chrony.conf "${ROOTFS_DIR}/etc/chrony/chrony.conf"

on_chroot systemctl enable chrony

echo "neurokairos-sender" > "${ROOTFS_DIR}/etc/hostname"
sed -i 's/127\.0\.1\.1.*/127.0.1.1\tneurokairos-sender/' "${ROOTFS_DIR}/etc/hosts"
