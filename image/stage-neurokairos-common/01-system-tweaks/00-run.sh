#!/bin/bash -e

# Per-image hostname is set by the variant stage's 00-run.sh (it knows whether
# this is the sender or server image). Here we set a shared banner so a user
# SSHing in immediately sees what kind of system this is.

cat > "${ROOTFS_DIR}/etc/motd" <<'EOF'

NeuroKairos — GPS-disciplined IRIG-H timecode appliance
  Sender status:   systemctl status irig-sender
  Chrony status:   chronyc tracking
  Project:         https://github.com/lukesjulson/irig_unix_timecodes

EOF
