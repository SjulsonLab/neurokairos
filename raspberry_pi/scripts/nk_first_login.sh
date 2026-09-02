# NeuroKairos: force a password change on the first interactive login.
#
# Sourced from /etc/profile.d. We deliberately do NOT use account expiry
# (`chage -d 0`): over SSH that re-prompts for the *current* password (which you
# just typed to get in) and then makes OpenSSH drop the connection right after
# the change. Instead we invoke a small root helper via a scoped NOPASSWD sudo
# rule — so it asks only for the new password (twice) and the session continues.
#
# Guarded by a marker file the image ships with (and nk_sanitize.sh restores);
# once the password is set the helper removes the marker and this is a no-op.

if [ -t 0 ] && [ -f /var/lib/neurokairos/default-password ] && [ "$(id -un)" = "neurokairos" ]; then
    echo
    echo "=== NeuroKairos first-time setup ==="
    echo "For security, set a new password for user 'neurokairos' (required)."
    echo

    # Use the scoped NOPASSWD helper if the sudoers rule is in place (checked
    # without prompting); otherwise fall back to a plain passwd so the user is
    # never locked out (that path asks for the current password once).
    if sudo -n -l /usr/local/sbin/nk_set_password >/dev/null 2>&1; then
        sudo -n /usr/local/sbin/nk_set_password
    else
        until passwd; do
            echo "Password not changed — it is required. Please try again."
        done
        rm -f /var/lib/neurokairos/default-password 2>/dev/null || true
    fi

    echo
    echo "Password updated. Continuing..."
    echo
fi
