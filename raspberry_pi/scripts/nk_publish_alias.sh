#!/bin/bash -e

# Publish an mDNS address alias (default neurokairos.local) pointing at this
# host's primary IPv4, so the NeuroKairos dashboard is reachable at one friendly
# name regardless of the server's own hostname. Server images only.
#
# avahi-publish (from avahi-utils) runs in the foreground for the lifetime of
# the registration, which is why the systemd unit is Type=simple with Restart.

ALIAS="${1:-neurokairos.local}"

# Primary source IP for outbound LAN traffic (no packet is actually sent).
ip="$(ip -4 route get 192.0.2.1 2>/dev/null | sed -n 's/.* src \([0-9.]\+\).*/\1/p')"
if [ -z "$ip" ]; then
    ip="$(hostname -I | awk '{print $1}')"
fi
if [ -z "$ip" ]; then
    echo "nk_publish_alias: could not determine primary IP" >&2
    exit 1
fi

echo "nk_publish_alias: publishing ${ALIAS} -> ${ip}"
exec avahi-publish -a -R "$ALIAS" "$ip"
