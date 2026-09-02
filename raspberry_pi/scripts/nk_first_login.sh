# NeuroKairos: force a password change on the first interactive login.
#
# Sourced from /etc/profile.d. We deliberately do NOT use account expiry
# (`chage -d 0`) for this: over SSH that makes PAM re-prompt for the *current*
# password (which you just typed to get in) and then makes OpenSSH drop the
# connection right after the change. Instead we run `passwd` as root via sudo —
# so it asks only for the new password (twice) and the session continues.
#
# Guarded by a marker file that the image ships with and nk_sanitize.sh restores;
# once the password is set the marker is removed and this becomes a no-op.

if [ -t 0 ] && [ -f /var/lib/neurokairos/default-password ] && [ "$(id -un)" = "neurokairos" ]; then
    echo
    echo "=== NeuroKairos first-time setup ==="
    echo "For security, set a new password for user 'neurokairos' (required)."
    echo

    # Prefer running passwd as root (no current-password prompt, no disconnect).
    # Fall back to a normal passwd only if passwordless sudo isn't available.
    if sudo -n true 2>/dev/null; then
        until sudo -n passwd "$(id -un)"; do
            echo "Password not changed — it is required. Please try again."
        done
        sudo -n rm -f /var/lib/neurokairos/default-password
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
