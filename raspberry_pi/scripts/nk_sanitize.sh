#!/bin/bash -e

# Wipe machine-specific identity so the running system can be turned into a
# clean golden image, then power off. Run as root: sudo nk_sanitize.sh
#
# Removes: SSH host keys (regenerated on next boot), machine-id, the status
# agent's saved name + web password, discovered NTP sources, chrony drift, and
# rotates the journal. Resets the login password back to the default and re-arms
# the first-login prompt so the next first boot forces a new one.

if [[ $EUID -ne 0 ]]; then echo "run as root" >&2; exit 1; fi

echo "Sanitizing NeuroKairos machine identity..."
systemctl stop irig-sender chrony nk-status-agent 2>/dev/null || true

rm -f /etc/ssh/ssh_host_*                                   # regenerated on next boot
rm -f /var/lib/neurokairos/agent-auth /var/lib/neurokairos/name
rm -f /etc/chrony/sources.d/neurokairos.sources
rm -f /var/lib/chrony/chrony.drift
: > /etc/machine-id
rm -f /var/lib/dbus/machine-id

# Restore the default password and re-arm the first-login change prompt so a
# freshly-imaged card starts from the documented neurokairos/neurokairos state.
echo 'neurokairos:neurokairos' | chpasswd 2>/dev/null || true
mkdir -p /var/lib/neurokairos
: > /var/lib/neurokairos/default-password

journalctl --rotate 2>/dev/null || true
journalctl --vacuum-time=1s 2>/dev/null || true

echo "Done. Powering off — the SD card is now safe to image."
poweroff
