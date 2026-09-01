#!/bin/bash -e

# Sender-only: headless Wi-Fi onboarding. A boot oneshot decides, in order:
# already-networked (ethernet / saved Wi-Fi) -> nothing; /boot/firmware/
# neurokairos-wifi.txt present -> set country + join; else bring up the
# "NeuroKairos-Setup" captive hotspot (nk-wifi-portal, started on demand, NOT
# enabled at boot). The .py + units are the source of truth under raspberry_pi/;
# CI (and the local-build docs) drop them into files/ before pi-gen runs.
#
# Rationale for shipping our own: Raspberry Pi Imager's Wi-Fi provisioning is
# broken on trixie, and the radio stays rfkill-blocked until a regulatory
# country is set — a headless user with no ethernet would otherwise be stuck.

install -m 755 files/nk_wifi_onboard.py "${ROOTFS_DIR}/usr/local/sbin/nk_wifi_onboard.py"
install -m 755 files/nk_wifi_portal.py "${ROOTFS_DIR}/usr/local/sbin/nk_wifi_portal.py"
install -m 644 files/nk-wifi-onboard.service "${ROOTFS_DIR}/etc/systemd/system/nk-wifi-onboard.service"
install -m 644 files/nk-wifi-portal.service "${ROOTFS_DIR}/etc/systemd/system/nk-wifi-portal.service"

# Ship the annotated example on the boot partition so it's visible from any
# computer that mounts the card (rename to neurokairos-wifi.txt to use it).
install -m 644 files/neurokairos-wifi.txt.example \
  "${ROOTFS_DIR}/boot/firmware/neurokairos-wifi.txt.example"

on_chroot << EOF
# iw/rfkill drive the regulatory-domain + radio-unblock dance; raspi-config's
# nonint do_wifi_country is the canonical way to persist the country. All are on
# the RPi OS base, but install defensively so onboarding can't silently no-op.
apt-get install -y --no-install-recommends iw rfkill

# Only the decision oneshot is enabled; the portal is started on demand by it.
systemctl enable nk-wifi-onboard.service
EOF
