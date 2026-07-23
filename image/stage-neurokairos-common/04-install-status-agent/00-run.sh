#!/bin/bash -e

# Install the NeuroKairos status agent on BOTH images. It reports this Pi's
# timing/LED status and performs password-gated control actions, and advertises
# _neurokairos-status._tcp over mDNS so the server-hosted dashboard can discover
# it. avahi-daemon + avahi-utils live here (common) so both variants can
# advertise AND browse. The .py + unit are CI-staged from raspberry_pi/ (source
# of truth); the Avahi service XML is static and committed in files/.

install -m 755 files/nk_status_agent.py "${ROOTFS_DIR}/usr/local/sbin/nk_status_agent.py"
install -m 644 files/nk-status-agent.service "${ROOTFS_DIR}/etc/systemd/system/nk-status-agent.service"

install -d -m 755 "${ROOTFS_DIR}/etc/avahi/services"
install -m 644 files/neurokairos-status.avahi-service \
	"${ROOTFS_DIR}/etc/avahi/services/neurokairos-status.service"

# Persisted friendly name + agent auth hash live here.
install -d -m 755 "${ROOTFS_DIR}/var/lib/neurokairos"

on_chroot << EOF
systemctl enable avahi-daemon
systemctl enable nk-status-agent.service
EOF
