#!/bin/bash -e

# Publish an mDNS address alias (default neurokairos.local) pointing at this
# host's primary IPv4, so the NeuroKairos dashboard is reachable at one friendly
# name regardless of the server's own hostname. Server images only.
#
# avahi-publish (from avahi-utils) runs in the foreground for the lifetime of
# the registration, which is why the systemd unit is Type=simple with Restart.

ALIAS="${1:-neurokairos.local}"

# Determine this host's primary LAN IPv4. `ip route get` is the most precise
# (the source address the kernel would use for outbound LAN traffic) but returned
# nothing on the bench before the default route was up, so fall back to the first
# global address from `hostname -I`. Retry for a while: at boot avahi may start
# before DHCP finishes, and Type=simple + Restart only helps if we actually exit.
detect_ip() {
    local ip
    ip="$(ip -4 route get 192.0.2.1 2>/dev/null | sed -n 's/.* src \([0-9.]\+\).*/\1/p')"
    [ -z "$ip" ] && ip="$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -E '^[0-9]+\.' | head -n1)"
    printf '%s' "$ip"
}

ip=""
for _ in $(seq 1 30); do
    ip="$(detect_ip)"
    [ -n "$ip" ] && break
    sleep 2
done
if [ -z "$ip" ]; then
    echo "nk_publish_alias: could not determine primary IP after retrying" >&2
    exit 1
fi

echo "nk_publish_alias: publishing ${ALIAS} -> ${ip}"
exec avahi-publish -a -R "$ALIAS" "$ip"
