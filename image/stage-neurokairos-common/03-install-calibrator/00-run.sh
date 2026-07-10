#!/bin/bash -e

# Install the NTP calibration/logging daemon. Runs on both image variants.
#
# The daemon (raspberry_pi/scripts/ntp_calibrator.py) is pure-stdlib Python 3;
# in its current form it is a *logger* — it records per-server offset statistics
# and computes precision metrics, and does not itself modify chrony.conf. The
# .py and .service are the source of truth under raspberry_pi/; the CI workflow
# (and the local-build instructions) drop them into this stage's files/ before
# pi-gen runs, mirroring how the irig_sender binary is staged.

install -m 755 files/ntp_calibrator.py "${ROOTFS_DIR}/usr/local/sbin/ntp_calibrator.py"
install -m 644 files/ntp-calibrator.service "${ROOTFS_DIR}/etc/systemd/system/ntp-calibrator.service"

install -d -m 755 "${ROOTFS_DIR}/var/log/ntp-calibrator"
install -d -m 755 "${ROOTFS_DIR}/var/lib/ntp-calibrator"

# Default daemon config. Matches raspberry_pi/scripts/install_ntp_calibrator.sh.
# ntfy alerting stays off until the user drops /etc/ntp-calibrator-ntfy.json
# (load_ntfy_config returns None for a missing file, so this is safe).
cat > "${ROOTFS_DIR}/etc/ntp-calibrator.json" <<'EOF'
{
  "poll": 6,
  "active_pool_size": 3,
  "ntfy_config_path": "/etc/ntp-calibrator-ntfy.json"
}
EOF

on_chroot << EOF
systemctl enable ntp-calibrator.service
EOF
