#!/bin/sh

set -eu

ip_address="$(ip -4 route get 8.8.8.8 | awk '/src/ {for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit }}')"
exec /usr/bin/avahi-publish -a -R neurokairos.local "$ip_address"
