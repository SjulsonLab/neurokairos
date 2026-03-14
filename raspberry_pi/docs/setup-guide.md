# NeuroKairos Setup Guide

Complete instructions for installing the GPS-disciplined IRIG-H timecode system on a Raspberry Pi 4B.

## Overview

This guide covers two scenarios:

- **Scenario A (GPS Server):** A Raspberry Pi with a GPS timing receiver becomes a stratum 1 NTP server and runs the IRIG-H encoder.
- **Scenario B (Client):** A Raspberry Pi on the same LAN (no GPS) synchronises via NTP from the GPS server and runs its own IRIG-H encoder.

Both scenarios result in a systemd service (`irig-sender`) that outputs a continuous pulse-width-modulated TTL signal on a GPIO pin — the encoder.

## Hardware Prerequisites

| Component | Notes |
|-----------|-------|
| Raspberry Pi 4 Model B | Other Pi models may work but are untested |
| GPS receiver with PPS output | e.g. u-blox NEO-M8T, Adafruit Ultimate GPS |
| GPS antenna | Active antenna recommended for indoor use |
| Jumper wires | For connecting PPS and IRIG output |

### GPIO Wiring

| Signal | Default BCM Pin | Direction |
|--------|-----------------|-----------|
| GPS PPS input | GPIO 4 | GPS → Pi |
| IRIG-H output (normal) | GPIO 9 | Pi → recorder |
| IRIG-H output (inverted) | disabled | Pi → recorder (optional) |
| GPS serial TX → Pi RX | GPIO 15 (UART RX) | GPS → Pi |
| GPS serial RX ← Pi TX | GPIO 14 (UART TX) | Pi → GPS |

> **Warning:** Pins 0–1 (I2C HAT ID), 14–15 (UART/GPS), and 4 (PPS) are reserved. The IRIG sender will refuse to use them.

## RPi OS Configuration

Before installing any software, configure the serial port:

```bash
sudo raspi-config
```

1. Navigate to **Interface Options → Serial Port**
2. **Login shell over serial?** → **No**
3. **Serial port hardware enabled?** → **Yes**
4. Reboot when prompted

This frees `/dev/ttyAMA0` for GPS communication while keeping the hardware UART available.

## Scenario A: GPS Server

For the Raspberry Pi that has a GPS receiver attached.

### Step 1: Install chrony + gpsd

```bash
cd raspberry_pi/scripts
sudo ./install_chrony_server.sh
```

This script:
- Installs `chrony`, `gpsd`, and `pps-tools`
- Configures gpsd to use `/dev/ttyAMA0` (override with `--serial-device /dev/ttyXXX`)
- Enables the `pps-gpio` device tree overlay on GPIO 4
- Configures chrony with:
  - SHM refclock from gpsd (stratum 1, offset 0.5s for NMEA latency)
  - PPS refclock from `/dev/pps0` (stratum 1, lock to SHM)
  - Public NTP pool servers as fallback
  - LAN serving enabled (`allow all`)
- Restarts chrony and gpsd

**Reboot required** after installation for the PPS overlay to take effect:

```bash
sudo reboot
```

### Step 2: Verify time synchronisation

After rebooting, wait 2–3 minutes for the GPS to get a fix, then run:

```bash
./test_chrony.sh
```

**What good output looks like:**

- **chrony tracking:** `Stratum` should be `1`. `Root dispersion` should be below 1 ms once PPS is locked. `Leap status` should be `Normal`.
- **chrony sources:** You should see `#* PPS0` (the `*` means it is the selected source). The SHM source (`#+ SHM0` or similar) should also be reachable.
- **PPS test:** `ppstest /dev/pps0` should show timestamps arriving once per second with low jitter (<1 µs typical).
- **GPS data:** `gpspipe` output should show NMEA sentences with a valid fix (mode 2D or 3D).

If sources show `?` in the reach column, GPS hasn't acquired a fix yet — wait longer or check antenna placement.

### Step 3: Install the IRIG sender

```bash
sudo ./install.sh
```

This script:
- Compiles the C sender (`irig_sender.c`) via `make`
- Copies the binary to `/usr/local/bin/irig_sender`
- Installs and enables a systemd service (`irig-sender.service`)
- Runs at Nice -20 with SCHED_FIFO priority 80

The service waits for the next :00 second boundary before transmitting its first frame, ensuring all frames align to minute boundaries.

**Custom pin configuration:**

```bash
# Normal output on GPIO 17 instead of default 11
sudo ./install.sh -p 17

# Normal on 17, inverted on 27
sudo ./install.sh -p 17 -n 27

# Custom LED warning threshold (blink when root dispersion > 2 ms)
sudo ./install.sh -w 2.0
```

### Step 4: Verify IRIG output

Check the service is running:

```bash
sudo systemctl status irig-sender
```

View the live output:

```bash
sudo journalctl -u irig-sender -f
```

You should see log lines showing frame transmission every 60 seconds, along with periodic chrony status updates (stratum and root dispersion).

## Scenario B: Client Setup

For a Raspberry Pi on the same LAN that does **not** have a GPS receiver.

### Step 1: Install chrony as NTP client

```bash
cd raspberry_pi/scripts
sudo ./install_chrony_client.sh --server 192.168.1.100
```

Replace `192.168.1.100` with the IP address of your GPS server Pi from Scenario A.

If you omit `--server`, chrony will use the public NTP pool (less precise, but functional).

### Step 2: Verify synchronisation

```bash
./test_chrony.sh
```

You should see:
- `Stratum` of `2` (one hop from the GPS server)
- `Root dispersion` under a few milliseconds on a local LAN
- The GPS server listed as a source with `*` (selected)

### Step 3: Install the IRIG sender

```bash
sudo ./install.sh
```

Same as Scenario A, Step 3. All the same flags (`-p`, `-n`, `-w`) are available.

## Troubleshooting

### No PPS pulses

- Verify the PPS wire is connected to GPIO 4
- Check the overlay is loaded: `dmesg | grep pps`
- Verify the device exists: `ls /dev/pps0`
- If `/dev/pps0` is missing, the reboot after `install_chrony_server.sh` may not have happened
- Test with: `sudo ppstest /dev/pps0` (should show timestamps every second)

### High root dispersion

- Wait at least 5 minutes after boot for chrony to converge
- Run `chronyc tracking` — root dispersion should decrease over time
- Check `chronyc sources` — if PPS is not selected (`*`), chrony may still be using pool servers
- The RPi ACT LED will blink if root dispersion exceeds the warning threshold (default 1.0 ms, configurable with `-w`)

### chrony not syncing

- `chronyc sources` showing `?` in reach: the source is unreachable
- For GPS server: verify gpsd is running (`systemctl status gpsd`) and has a fix (`gpspipe -w | head -5`)
- For client: verify the server IP is correct and reachable (`ping 192.168.1.100`)
- Check firewall: NTP uses UDP port 123

### Service won't start

- Check logs: `sudo journalctl -u irig-sender -e`
- The sender requires root (for `/dev/mem` GPIO access). Verify the service runs as `User=root`
- Verify the binary exists: `ls -la /usr/local/bin/irig_sender`
- If the binary is missing, re-run `sudo ./install.sh`

### Wrong time in decoded IRIG

- Check for the day-of-year off-by-one bug (fixed in commit `6def02b`). If using an old branch (`less-cpu` or `charlie-irig`), update to `main`
- Verify chrony is actually synced: `chronyc tracking` should show `Leap status: Normal`

## Uninstalling

To remove the IRIG sender service and binary:

```bash
cd raspberry_pi/scripts
sudo ./uninstall.sh
```

This stops the service, disables it from boot, removes the binary from `/usr/local/bin`, and cleans up the log directory.

To also remove chrony and gpsd (GPS server only):

```bash
sudo apt remove --purge chrony gpsd gpsd-clients pps-tools
```
