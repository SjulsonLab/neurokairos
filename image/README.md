# NeuroKairos OS images

Two pre-built Raspberry Pi OS Lite (arm64) images, produced with [pi-gen]:

| Image                     | Use it when…                                                                                       |
| ------------------------- | -------------------------------------------------------------------------------------------------- |
| **`neurokairos-sender`**  | The Pi gets its time from an external NTP server (commercial GPS-NTP appliance, LAN NTP, pool).    |
| **`neurokairos-server`**  | A GPS receiver is wired to the Pi's UART + PPS GPIO; the Pi serves stratum-1 NTP on its network. The server image defaults to GPIO 18 for Waveshare-style HATs; use GPIO 4 only if you override the PPS pin for an Adafruit-style HAT.   |

Both images contain `irig_sender` running as a systemd service from boot, with `chrony` providing the system clock. Pin configuration (BCM GPIO 9 normal output, inverted disabled) matches the defaults in `raspberry_pi/sender/irig_sender.c`; change it with `raspberry_pi/scripts/install.sh -p <pin>` after boot.

## Download → flash → boot

1. Grab the latest `neurokairos-sender-*.img.xz` or `neurokairos-server-*.img.xz` from the [Releases page]. Decompress with `xz -d` or let Raspberry Pi Imager handle it.
2. Flash to a microSD with [Raspberry Pi Imager] or `dd`. **Use a high-endurance / industrial (A2) card** (e.g. SanDisk High Endurance or Max Endurance) — a timing appliance writes logs continuously and cheap cards wear out and corrupt.
3. Eject, insert into the Pi, power on. **No `ssh`/`userconf.txt` files are needed** — the image ships ready to use.
4. First access: SSH (or console) as **`neurokairos`** / **`neurokairos`**; you'll be prompted to set a new password. Or just open **`http://neurokairos.local`** in a browser. The HDMI console shows the Pi's IP + MAC before login.

> Do NOT use Raspberry Pi Imager's "Edit custom OS settings" to preset a different user — the appliance's first-run behavior (forced password change, `neurokairos.local`) assumes the built-in `neurokairos` user. (Imager's Wi-Fi field is also broken on trixie; use one of the three options below instead.)

### Getting the sender onto your network (headless Wi-Fi)

The sender picks a network automatically, in this order — you only need **one**:

1. **Ethernet (simplest).** Plug in a cable; nothing to configure. Ethernet
   always wins, and a Pi already online never enters Wi-Fi setup.
2. **Wi-Fi config file (headless, no monitor).** Before first boot, on the small
   **`bootfs`** partition that appears when you mount the card, copy
   `neurokairos-wifi.txt.example` to **`neurokairos-wifi.txt`** and fill in
   `country=`, `ssid=`, `password=`. On boot the sender sets the Wi-Fi
   **regulatory country** (required — the radio is blocked until it's set),
   joins, and renames the file to `…​.applied` so your password isn't left on the
   card. (You can also drop this file onto the boot partition of an
   already-running card.)
3. **Setup hotspot (no cable, no file).** If there's no ethernet and no config
   file, the sender broadcasts a Wi-Fi network **`NeuroKairos-Setup`** (password
   `neurokairos`). Join it from a phone/laptop and the setup page **pops up
   automatically** (captive portal, like airport Wi-Fi); if it doesn't, open
   **`http://10.42.0.1`** manually. Pick your network + enter the password and
   country, and submit. The hotspot then
   disappears as the sender joins your Wi-Fi — reconnect your device to your
   normal network and find the sender at `http://neurokairos-sender.local`. Once
   configured this way it won't re-open the hotspot on its own.

> Reaching it afterward: a lone sender is always at **`http://neurokairos-sender.local`**. It also tries to claim the friendlier **`neurokairos.local`**, but if that name doesn't resolve (mDNS caching, or a server on the LAN owning it), use `neurokairos-sender.local` or the Pi's IP.

### Sender image — first-boot checklist

- **Web status (no server needed):** open **`http://neurokairos.local`** (or the
  Pi's IP) for a status page — sync quality, stratum, reference, offsets, whether
  IRIG-H is sending, plus a **Diagnostics** section showing each service's state
  and recent logs (so you can see *why* something failed without SSH). Served at
  the lowest CPU priority; cannot affect pulse timing. Each unit is also reachable
  at its own `neurokairos-sender.local`. Raw JSON: `http://<ip>:8080/api/status`.
  - (On a LAN that also has a NeuroKairos **server**, the server owns
    `neurokairos.local` and hosts the multi-Pi fleet dashboard; a lone sender
    claims `neurokairos.local` for itself.)
- **SSH (on by default):** log in as **`neurokairos`** / password **`neurokairos`**
  — you'll be **required to set a new password** on first login. Unique SSH host
  keys are generated per device on first boot.
- **HDMI/console:** before login, the screen shows the Pi's IP + MAC (or "no IP
  assigned yet"), so you can find it on the network without a scanner.
- **Making a golden image:** `sudo nk_sanitize.sh` wipes machine-specific
  identity (host keys, machine-id, saved name/password) and powers off, so the SD
  card can be re-imaged cleanly.
- `systemctl status irig-sender` should report `active (running)`.
- `chronyc tracking` should sync within ~30 s to the diverse public NTP sources.
- `chronyd --version` should report **4.8** (built from source; not the distro 4.6).
- `systemctl status ntp-calibrator` should report `active (running)`; it logs per-source offset statistics to `/var/log/ntp-calibrator/`. To enable ntfy push alerts, drop `/etc/ntp-calibrator-ntfy.json` (`{"server":"https://ntfy.sh","topic":"..."}`) and restart the service.
- **LAN auto-discovery:** if a NeuroKairos **server** image is running on the same subnet, the sender finds it over mDNS within ~30 s of boot and adds it to chrony as a preferred source — no configuration. Check `chronyc sources` for the LAN server, or `cat /etc/chrony/sources.d/neurokairos.sources`. Discovered servers are added at runtime via `chronyc reload sources`; **chrony is never restarted and the clock is never stepped**, so this is safe to happen mid-recording. Discovery is link-local (a flat subnet/VLAN); it does not cross routers.
- To use a specific NTP server instead, edit `/etc/chrony/chrony.conf`, uncomment the `server` line at the top, then `sudo systemctl restart chrony`.
- Wire BCM GPIO 9 to your acquisition system's TTL input. Confirm pulses on a scope: 1 Hz frames, 60 pulses per frame.

### Server image — first-boot checklist

- Wire the GPS receiver: NMEA → UART (`/dev/ttyAMA0`), PPS → GPIO 18 by default or GPIO 4 if you intentionally override for an Adafruit-style GPS HAT, ground common.
- `ppstest /dev/pps0` should print a `1 Hz` source.
- `chronyc sources` should show both `#? GPS` and `#* PPS`; `chronyc tracking` should report stratum 1 within a few minutes of GPS lock.
- `gpspipe -w` (one of `gpsd-clients`) shows live NMEA if the receiver is talking.

## Status dashboard

Every Pi runs a lightweight status agent; **server** Pis additionally host a web
dashboard. From any device on the same LAN, open **`http://neurokairos.local`**
(published by the server; each server is also reachable at its own
`<hostname>.local`). You get a white page with one rectangle per Pi — servers and
clients — showing timing quality, stratum, reference + tier, offsets, root
dispersion, calibrator precision, and a small blue dot mirroring that Pi's
physical IRIG-H ACT LED (solid = good sync, blinking = marginal/unsynced).

- **First visit** prompts you to set a dashboard password. Viewing is open to
  anyone on the LAN; **write-actions require the password.**
- Per-Pi actions (no SSH needed): **rename**, **restart chrony / irig-sender**,
  **set NTP source**, **reboot**, **shutdown**.
- The agent is low-priority and cannot interrupt a client's real-time IRIG-H
  sending; the dashboard only runs on servers for the same reason.
- Limitations: needs ≥1 server on the LAN to host the UI; mDNS is link-local (a
  flat subnet, not across routers); trust is LAN-scoped.

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

# Sender variant only: stage the LAN discovery daemon + its units
disc=/tmp/pi-gen/stage-neurokairos-client/01-install-discovery/files
if [ -d "$disc" ]; then
  install -m 644 raspberry_pi/scripts/nk_discover_ntp.py "$disc/"
  install -m 644 raspberry_pi/systemd/nk-discover-ntp.service "$disc/"
  install -m 644 raspberry_pi/systemd/nk-discover-ntp.timer "$disc/"
fi

# Both images: stage the status agent into the common stage
agent=/tmp/pi-gen/stage-neurokairos-common/04-install-status-agent/files
install -m 644 raspberry_pi/scripts/nk_status_agent.py "$agent/"
install -m 644 raspberry_pi/systemd/nk-status-agent.service "$agent/"

# Both images: stage the appliance basics (console banner, sanitize, forced pw)
app=/tmp/pi-gen/stage-neurokairos-common/05-appliance/files
install -m 644 raspberry_pi/scripts/nk_console_issue.sh "$app/"
install -m 644 raspberry_pi/scripts/nk_sanitize.sh "$app/"
install -m 644 raspberry_pi/scripts/nk_first_login.sh "$app/"
install -m 644 raspberry_pi/systemd/nk-console-issue.service "$app/"
install -m 644 raspberry_pi/systemd/nk-console-issue.timer "$app/"

# Sender variant only: stage the neurokairos.local alias publisher
appc=/tmp/pi-gen/stage-neurokairos-client/02-appliance/files
if [ -d "$appc" ]; then
  install -m 644 raspberry_pi/scripts/nk_publish_alias.sh "$appc/"
  install -m 644 raspberry_pi/systemd/nk-sender-alias.service "$appc/"
fi

# Sender variant only: stage headless Wi-Fi onboarding (decision oneshot +
# setup-hotspot portal + boot-partition example)
wifi=/tmp/pi-gen/stage-neurokairos-client/03-wifi-onboard/files
if [ -d "$wifi" ]; then
  install -m 644 raspberry_pi/scripts/nk_wifi_onboard.py "$wifi/"
  install -m 644 raspberry_pi/scripts/nk_wifi_portal.py "$wifi/"
  install -m 644 raspberry_pi/scripts/neurokairos-wifi.txt.example "$wifi/"
  install -m 644 raspberry_pi/systemd/nk-wifi-onboard.service "$wifi/"
  install -m 644 raspberry_pi/systemd/nk-wifi-portal.service "$wifi/"
fi

# Server variant only: stage the dashboard (web UI + alias publisher)
dash=/tmp/pi-gen/stage-neurokairos-server/01-install-dashboard/files
if [ -d "$dash" ]; then
  install -m 644 raspberry_pi/scripts/nk_dashboard.py "$dash/"
  install -m 644 raspberry_pi/scripts/nk_publish_alias.sh "$dash/"
  install -m 644 raspberry_pi/systemd/nk-dashboard.service "$dash/"
  install -m 644 raspberry_pi/systemd/nk-dashboard-alias.service "$dash/"
  install -m 644 raspberry_pi/dashboard/web/index.html "$dash/"
fi

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

- `image/stage-neurokairos-common/` — always runs. Installs the natively-compiled `irig_sender` binary + systemd unit (`00-install-sender`), the shared packages incl. distro chrony + SSH-banner motd (`01-system-tweaks`), **chrony 4.8 built from source over the distro package** (`02-build-chrony`), the NTP calibrator logging daemon + unit (`03-install-calibrator`), the **status agent + `avahi-daemon`/`avahi-utils`/`libnss-mdns`** that reports timing status (self-page + `_neurokairos-status._tcp`) (`04-install-status-agent`), and the **appliance basics** — forced first-login password change, the HDMI console IP/MAC banner, and the `nk_sanitize.sh` golden-image tool (`05-appliance`). It also writes `/etc/default/chrony` (empty `DAEMON_OPTS`) so the source-built, seccomp-less chronyd starts.
- `image/stage-neurokairos-client/` — exports `*-sender.img.xz`. Installs the NTP-client `chrony.conf` (with a `sourcedir`), sets hostname `neurokairos-sender`, installs the mDNS **discovery** daemon (`nk_discover_ntp.py` + timer) that finds LAN NeuroKairos servers and feeds them to chrony at runtime (`01-install-discovery`), and publishes the **`neurokairos.local`** alias for the single-sender case at lowest priority (`02-appliance`), and installs **headless Wi-Fi onboarding** — a boot oneshot that joins via `/boot/firmware/neurokairos-wifi.txt` or falls back to the `NeuroKairos-Setup` captive hotspot, setting the Wi-Fi regulatory country either way (`03-wifi-onboard`).
- `image/stage-neurokairos-server/` — exports `*-server.img.xz`. Installs gpsd + pps-tools, the stratum-1 `chrony.conf`, patches `/boot/firmware/config.txt` for `pps-gpio` overlay + UART, removes the serial console from `cmdline.txt`, sets hostname `neurokairos-server`, **advertises** itself over mDNS (`_neurokairos-ntp._udp`) so sender Pis discover it, and hosts the **status dashboard** (`01-install-dashboard`: web UI on port 80 + the `neurokairos.local` mDNS alias).

`raspberry_pi/sender/irig_sender.c` and `raspberry_pi/systemd/irig-sender.service` are the single source of truth. The CI workflow native-compiles the binary on a `ubuntu-24.04-arm` runner and drops it (plus the systemd unit) into the common stage's `files/` directory before pi-gen runs; nothing under `image/stage-neurokairos-common/00-install-sender/files/` is committed except the `.gitkeep` note.

The chrony configs under `image/stage-neurokairos-{client,server}/` are static and intentionally separate from `raspberry_pi/scripts/install_chrony_{client,server}.sh` (those scripts build configs imperatively from CLI flags; the image ships fixed defaults). Comments at the top of each file flag the sibling that must be reviewed alongside.

[pi-gen]: https://github.com/RPi-Distro/pi-gen
[Releases page]: https://github.com/SjulsonLab/neurokairos/releases
[Raspberry Pi Imager]: https://www.raspberrypi.com/software/
