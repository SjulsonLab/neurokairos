# NeuroKairos OS images

Two pre-built Raspberry Pi OS Lite (arm64) images, produced with [pi-gen]:

| Image                     | Use it when…                                                                                       |
| ------------------------- | -------------------------------------------------------------------------------------------------- |
| **`neurokairos-sender`**  | The Pi gets its time from an external NTP server (commercial GPS-NTP appliance, LAN NTP, pool).    |
| **`neurokairos-server`**  | A GPS receiver is wired to the Pi's UART + PPS GPIO; the Pi serves stratum-1 NTP on its network. The server image defaults to GPIO 18 for Waveshare-style HATs; use GPIO 4 only if you override the PPS pin for an Adafruit-style HAT.   |

Both images contain `irig_sender` running as a systemd service from boot, with `chrony` providing the system clock. Pin configuration (BCM GPIO 9 normal output, inverted disabled) matches the defaults in `raspberry_pi/sender/irig_sender.c`; change it with `raspberry_pi/scripts/install.sh -p <pin>` after boot.

## Download → flash → boot

1. Grab the latest `neurokairos-sender-*.img.xz` or `neurokairos-server-*.img.xz` from the [Releases page]. Decompress with `xz -d` or let Raspberry Pi Imager handle it.
2. Flash to a microSD with [Raspberry Pi Imager] or `dd`.
3. Before unmounting the FAT boot partition (`bootfs`), drop two files on it:
   - **`ssh`** — empty file. Enables SSH on first boot.
   - **`userconf.txt`** — one line, `username:hashed-password`. Generate the hash with `openssl passwd -6` or use Raspberry Pi Imager's "Edit custom OS settings" dialog, which writes this file for you.

   (Skipping `userconf.txt` causes first boot to hang on the user-create prompt — this is upstream Pi OS behavior.)
4. Eject, insert into the Pi, power on.

### Sender image — first-boot checklist

- `systemctl status irig-sender` should report `active (running)`.
- `chronyc tracking` should sync within ~30 s to the diverse public NTP sources.
- `chronyd --version` should report **4.8** (built from source; not the distro 4.6).
- `systemctl status ntp-calibrator` should report `active (running)`; it logs per-source offset statistics to `/var/log/ntp-calibrator/`. To enable ntfy push alerts, drop `/etc/ntp-calibrator-ntfy.json` (`{"server":"https://ntfy.sh","topic":"..."}`) and restart the service.
- To use your own NTP server, edit `/etc/chrony/chrony.conf`, uncomment the `server` line at the top, then `sudo systemctl restart chrony`.
- Wire BCM GPIO 9 to your acquisition system's TTL input. Confirm pulses on a scope: 1 Hz frames, 60 pulses per frame.

### Server image — first-boot checklist

- Wire the GPS receiver: NMEA → UART (`/dev/ttyAMA0`), PPS → GPIO 18 by default or GPIO 4 if you intentionally override for an Adafruit-style GPS HAT, ground common.
- `ppstest /dev/pps0` should print a `1 Hz` source.
- `chronyc sources` should show both `#? GPS` and `#* PPS`; `chronyc tracking` should report stratum 1 within a few minutes of GPS lock.
- `gpspipe -w` (one of `gpsd-clients`) shows live NMEA if the receiver is talking.

## Building locally

Requires a **native arm64 Linux host** with Docker — the workflow is built around running on an arm64 machine (Pi 4B/5, Ampere/Graviton VM, Apple Silicon via an arm64 Linux VM, etc.). It does not work on x86_64 without rearchitecting the whole flow: we tried that path and found pi-gen's qemu-user emulation crashes mid-build for Python and other packages.

```bash
# From the neurokairos repo root, on an arm64 Linux host:
sudo apt-get install -y build-essential docker.io

# Native compile the sender binary
install -d build
gcc -O2 -o build/irig_sender raspberry_pi/sender/irig_sender.c -lpthread -lm

# Clone pi-gen at the pinned tag
git clone --depth 1 --branch 2026-06-18-raspios-trixie-arm64 \
    https://github.com/RPi-Distro/pi-gen.git /tmp/pi-gen

# Stage our pi-gen stages
cp -r image/stage-neurokairos-common /tmp/pi-gen/

# Pick a variant
VARIANT=sender   # or: server
case "$VARIANT" in
  sender) cp -r image/stage-neurokairos-client /tmp/pi-gen/; STAGE=stage-neurokairos-client ;;
  server) cp -r image/stage-neurokairos-server /tmp/pi-gen/; STAGE=stage-neurokairos-server ;;
esac

# Drop the binary and systemd unit into the common stage's files/
dst=/tmp/pi-gen/stage-neurokairos-common/00-install-sender/files
install -m 755 build/irig_sender "$dst/"
install -m 644 raspberry_pi/systemd/irig-sender.service "$dst/"

# Drop the NTP calibrator daemon + unit into its install step's files/
cal=/tmp/pi-gen/stage-neurokairos-common/03-install-calibrator/files
install -m 644 raspberry_pi/scripts/ntp_calibrator.py "$cal/"
install -m 644 raspberry_pi/systemd/ntp-calibrator.service "$cal/"

# Don't export the vanilla Pi OS Lite image — only our variant stage exports
rm -f /tmp/pi-gen/stage2/EXPORT_IMAGE

# Render pi-gen/config
sed "s|@VARIANT_STAGE@|$STAGE|" image/config.template > /tmp/pi-gen/config

# Build (this takes ~10 min on a Pi 5, ~30 min on slower arm64 VMs)
cd /tmp/pi-gen && sudo ./build-docker.sh

# Image lands in /tmp/pi-gen/deploy/
ls -lh deploy/
```

To build the other variant cleanly, run `sudo ./build-docker.sh -c clean` (or wipe `/tmp/pi-gen/work/`) before re-running with the new variant.

## How the build is wired

- `image/stage-neurokairos-common/` — always runs. Installs the natively-compiled `irig_sender` binary + systemd unit (`00-install-sender`), the shared packages incl. distro chrony + SSH-banner motd (`01-system-tweaks`), **chrony 4.8 built from source over the distro package** (`02-build-chrony`), and the NTP calibrator logging daemon + unit (`03-install-calibrator`).
- `image/stage-neurokairos-client/` — exports `*-sender.img.xz`. Installs the NTP-client `chrony.conf`, sets hostname `neurokairos-sender`.
- `image/stage-neurokairos-server/` — exports `*-server.img.xz`. Installs gpsd + pps-tools, the stratum-1 `chrony.conf`, patches `/boot/firmware/config.txt` for `pps-gpio` overlay + UART, removes the serial console from `cmdline.txt`, sets hostname `neurokairos-server`.

`raspberry_pi/sender/irig_sender.c` and `raspberry_pi/systemd/irig-sender.service` are the single source of truth. The CI workflow native-compiles the binary on a `ubuntu-24.04-arm` runner and drops it (plus the systemd unit) into the common stage's `files/` directory before pi-gen runs; nothing under `image/stage-neurokairos-common/00-install-sender/files/` is committed except the `.gitkeep` note.

The chrony configs under `image/stage-neurokairos-{client,server}/` are static and intentionally separate from `raspberry_pi/scripts/install_chrony_{client,server}.sh` (those scripts build configs imperatively from CLI flags; the image ships fixed defaults). Comments at the top of each file flag the sibling that must be reviewed alongside.

[pi-gen]: https://github.com/RPi-Distro/pi-gen
[Releases page]: https://github.com/SjulsonLab/neurokairos/releases
[Raspberry Pi Imager]: https://www.raspberrypi.com/software/
