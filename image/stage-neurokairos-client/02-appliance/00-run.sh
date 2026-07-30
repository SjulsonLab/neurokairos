#!/bin/bash -e

# Sender-only: publish the memorable neurokairos.local alias for the ~99%
# single-sender case. Runs at the lowest priority (see the unit) so the avahi
# publisher can never perturb the RT IRIG-H busy-wait. avahi-publish comes from
# avahi-utils (installed in the common status-agent step).

install -m 755 files/nk_publish_alias.sh "${ROOTFS_DIR}/usr/local/sbin/nk_publish_alias.sh"
install -m 644 files/nk-sender-alias.service "${ROOTFS_DIR}/etc/systemd/system/nk-sender-alias.service"

on_chroot << EOF
systemctl enable nk-sender-alias.service
EOF
