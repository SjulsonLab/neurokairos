#!/bin/bash
# Set the initial NeuroKairos login password. Invoked by the first-login prompt
# (nk_first_login.sh) through a scoped NOPASSWD sudoers rule, so:
#   - the user is NOT asked for a sudo password (they haven't set one yet), and
#   - unlike `passwd` on an expired account, the SSH session is NOT disconnected.
# Running passwd as root also means it never asks for the *current* password.
#
# Guarded by the first-login marker so this can only set the password while the
# device is still on its shipped default; nk_sanitize.sh re-arms the marker when
# preparing a golden image. This keeps the standing NOPASSWD grant harmless.

set -u
MARKER=/var/lib/neurokairos/default-password

if [ ! -f "$MARKER" ]; then
    echo "Password already set; nothing to do." >&2
    exit 0
fi

until passwd neurokairos; do
    echo "Password not changed — it is required. Please try again." >&2
done

rm -f "$MARKER"
exit 0
