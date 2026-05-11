# NeuroKairos OS images

Two pre-built Raspberry Pi OS Lite (arm64) images, produced with [pi-gen]:

| Image                     | Use it when…                                                                                       |
| ------------------------- | -------------------------------------------------------------------------------------------------- |
| **`neurokairos-sender`**  | The Pi gets its time from an external NTP server (commercial GPS-NTP appliance, LAN NTP, pool).    |
| **`neurokairos-server`**  | A GPS receiver is wired to the Pi's UART + PPS GPIO; the Pi serves stratum-1 NTP on its network.   |

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
- `chronyc tracking` should sync within ~30 s to the public pool.
- To use your own NTP server, edit `/etc/chrony/chrony.conf`, uncomment the `server` line at the top, then `sudo systemctl restart chrony`.
- Wire BCM GPIO 9 to your acquisition system's TTL input. Confirm pulses on a scope: 1 Hz frames, 60 pulses per frame.

### Server image — first-boot checklist

- Wire the GPS receiver: NMEA → UART (`/dev/ttyAMA0`), PPS → GPIO 4, ground common.
- `ppstest /dev/pps0` should print a `1 Hz` source.
- `chronyc sources` should show both `#? GPS` and `#* PPS`; `chronyc tracking` should report stratum 1 within a few minutes of GPS lock.
- `gpspipe -w` (one of `gpsd-clients`) shows live NMEA if the receiver is talking.

## Building locally

Requires Linux (or WSL2) and Docker. Doesn't work natively on macOS — `pi-gen` uses Linux-only binfmt/QEMU plumbing.

```bash
# From the neurokairos repo root
git clone --depth 1 --branch 2024-11-19-raspios-bookworm-arm64 \
    https://github.com/RPi-Distro/pi-gen.git /tmp/pi-gen

# Copy the upstream source-of-truth files into the common stage's files/
cp raspberry_pi/sender/irig_sender.c \
   raspberry_pi/sender/Makefile \
   image/stage-neurokairos-common/00-install-sender/files/
cp raspberry_pi/systemd/irig-sender.service \
   image/stage-neurokairos-common/00-install-sender/files/

# Copy our stages on top of upstream pi-gen
cp -r image/stage-neurokairos-common /tmp/pi-gen/

# Prevent pi-gen's stage2 from exporting a vanilla Pi OS Lite image
rm -f /tmp/pi-gen/stage2/EXPORT_IMAGE

# Pick a variant and render the config
VARIANT=sender   # or: server
case "$VARIANT" in
  sender) cp -r image/stage-neurokairos-client /tmp/pi-gen/; STAGE=stage-neurokairos-client ;;
  server) cp -r image/stage-neurokairos-server /tmp/pi-gen/; STAGE=stage-neurokairos-server ;;
esac
sed "s|@VARIANT_STAGE@|$STAGE|" image/config.template > /tmp/pi-gen/config

# Build (this takes ~30-45 minutes; uses ~10 GB of disk)
cd /tmp/pi-gen
./build-docker.sh

# Image lands in /tmp/pi-gen/deploy/
ls -lh deploy/
```

To build the other variant cleanly, run `./build-docker.sh -c clean` (or wipe `/tmp/pi-gen/work/`) before re-running with the new variant.

## How the build is wired

- `image/stage-neurokairos-common/` — compiles `irig_sender` inside the rootfs (via QEMU userspace), installs the systemd unit, sets the SSH-banner motd. Always runs.
- `image/stage-neurokairos-client/` — exports `*-sender.img.xz`. Installs the NTP-client `chrony.conf`, sets hostname `neurokairos-sender`.
- `image/stage-neurokairos-server/` — exports `*-server.img.xz`. Installs gpsd + pps-tools, the stratum-1 `chrony.conf`, patches `/boot/firmware/config.txt` for `pps-gpio` overlay + UART, removes the serial console from `cmdline.txt`, sets hostname `neurokairos-server`.

`raspberry_pi/sender/` and `raspberry_pi/systemd/irig-sender.service` are the single source of truth for sender code and the systemd unit. The CI workflow copies them into the common stage's `files/` directory before pi-gen runs; nothing under `image/stage-neurokairos-common/00-install-sender/files/` is committed except the `.gitkeep` note.

The chrony configs under `image/stage-neurokairos-{client,server}/` are static and intentionally separate from `raspberry_pi/scripts/install_chrony_{client,server}.sh` (those scripts build configs imperatively from CLI flags; the image ships fixed defaults). Comments at the top of each file flag the sibling that must be reviewed alongside.

[pi-gen]: https://github.com/RPi-Distro/pi-gen
[Releases page]: https://github.com/lukesjulson/irig_unix_timecodes/releases
[Raspberry Pi Imager]: https://www.raspberrypi.com/software/
