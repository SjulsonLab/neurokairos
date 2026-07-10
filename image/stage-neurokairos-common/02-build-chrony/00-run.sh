#!/bin/bash -e

# Build and install chrony 4.8 from source, replacing the distro chrony 4.6
# that trixie ships. We compile *inside the rootfs* via on_chroot so the binary
# links against the target's own libc/nettle — building on the CI host (a
# different Ubuntu release) would produce ABI-mismatched binaries.
#
# Strategy: the distro `chrony` package is installed earlier (01-system-tweaks/
# 00-packages). It provides the `_chrony` user, /etc/chrony, /var/lib/chrony,
# and the systemd unit. We build chrony from source with the same prefix layout
# (--prefix=/usr --sysconfdir=/etc --localstatedir=/var) so `make install`
# overwrites just the binaries (/usr/sbin/chronyd, /usr/bin/chronyc) and the
# existing packaging scaffolding keeps working.
#
# Runtime-library note: we deliberately build only against libraries that the
# distro chrony package already depends on at runtime — nettle (crypto) and
# libcap (privilege dropping). Those runtime libs therefore survive the
# build-dep purge below. We do NOT pull in gnutls (NTS) or libseccomp: their
# runtime libs are not guaranteed to remain after autoremove, which would leave
# our source-built chronyd unable to start. NTS can be revisited once we can
# pin the matching runtime lib.

CHRONY_VERSION="4.8"
CHRONY_TARBALL="chrony-${CHRONY_VERSION}.tar.gz"
CHRONY_URL="https://chrony-project.org/releases/${CHRONY_TARBALL}"
# SHA256 of chrony-4.8.tar.gz. Cross-check against the published PGP signature
# (chrony-4.8-tar-gz-asc.txt) if this ever needs updating.
CHRONY_SHA256="33ea8eb2a4daeaa506e8fcafd5d6d89027ed6f2f0609645c6f149b560d301706"

on_chroot << EOF
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

# Build dependencies (purged again at the end to keep the image small).
# pps-tools provides <sys/timepps.h>, needed to compile chrony's PPS refclock
# support that the server variant relies on; only the header is needed at build
# time, so it is safe to purge afterwards.
BUILD_DEPS="build-essential pkg-config wget ca-certificates nettle-dev libcap-dev pps-tools"
apt-get update
apt-get install -y --no-install-recommends \$BUILD_DEPS

cd /tmp
wget -q "${CHRONY_URL}" -O "${CHRONY_TARBALL}"
echo "${CHRONY_SHA256}  ${CHRONY_TARBALL}" | sha256sum -c -

tar xf "${CHRONY_TARBALL}"
cd "chrony-${CHRONY_VERSION}"

# chrony ships a hand-written configure (not autoconf); these flags match the
# Debian layout so we overwrite the packaged binaries in place. Crypto uses
# nettle (auto-detected); gnutls/seccomp are intentionally absent (see note).
./configure --prefix=/usr --sysconfdir=/etc --localstatedir=/var --with-user=_chrony
make
make install

# Confirm the source build is what's now on PATH.
/usr/sbin/chronyd --version

# Shrink the image back down: drop build deps and caches. The runtime nettle
# and libcap libraries stay because the distro chrony package still depends on
# them.
cd /
rm -rf "/tmp/chrony-${CHRONY_VERSION}" "/tmp/${CHRONY_TARBALL}"
apt-get purge -y \$BUILD_DEPS
apt-get autoremove -y --purge
apt-get clean
# NOTE: do NOT remove /var/lib/apt/lists/* here — later stages (calibrator,
# client discovery, server gpsd) run apt-get install without their own
# apt-get update and rely on this package index. pi-gen clears the lists during
# export, so leaving them costs nothing in the final image.
EOF
