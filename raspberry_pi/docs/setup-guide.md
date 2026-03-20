# NeuroKairos Setup Guide

Complete instructions for installing the GPS-disciplined IRIG-H timecode system on a Raspberry Pi 4B.

## Overview

The IRIG-H encoder runs on a Raspberry Pi and outputs a pulse-width-modulated TTL signal on a GPIO pin. It synchronises its clock to a GPS-disciplined NTP server on your LAN.

**This guide assumes you already have an NTP server on your LAN.** We recommend a dedicated GPS NTP appliance — see [Choosing an NTP Server](#choosing-an-ntp-server) below. If you want to build your own NTP server from a Raspberry Pi and a GPS HAT, see [RPi GPS NTP Server](rpi-gps-server.md).

## Choosing an NTP Server

We recommend using a commercial GPS NTP appliance rather than a DIY solution. Commercial units are easier to set up, require no maintenance, and — critically — include an **OCXO (oven-controlled crystal oscillator)** holdover clock that keeps accurate time if GPS lock is temporarily lost. A Raspberry Pi with a GPS HAT has no holdover and will drift immediately without a GPS signal.

A good choice is the [Time Machines TM2000 GPS NTP/PTP Network Time Server](https://timemachinescorp.com/product/gps-ntpptp-network-time-server-tm2000/). Plug it into your LAN, attach a GPS antenna with a clear sky view, and it will serve stratum 1 NTP within a few minutes.

Once the appliance is running, note its IP address — you will need it below.

## Hardware Prerequisites

| Component | Notes |
|-----------|-------|
| Raspberry Pi 4 Model B | Other Pi models may work but are untested |
| Jumper wires | For connecting IRIG output to recording hardware |

No GPS hardware is needed on the encoder Pi itself.

### GPIO Wiring

| Signal | Default BCM Pin | Direction |
|--------|-----------------|-----------|
| IRIG-H output (normal) | GPIO 9 | Pi → recorder |
| IRIG-H output (inverted) | disabled | Pi → recorder (optional) |

> **Warning:** Pins 0–1 (I2C HAT ID), 14–15 (UART), and 4 (PPS) are reserved. The IRIG sender will refuse to use them.

## Step 1: Install chrony

```bash
cd raspberry_pi/scripts
sudo ./install_chrony_client.sh --server <NTP_SERVER_IP>
```

Replace `<NTP_SERVER_IP>` with the IP address of your GPS NTP appliance (or other NTP server).

This script:
- Installs `chrony`
- Configures it to use your specified NTP server as the primary source
- Falls back to the public NTP pool if the server is unreachable

## Step 2: Verify time synchronisation

```bash
./test_chrony.sh
```

You should see:
- `Stratum` of `2` (one hop from the GPS appliance)
- `Root dispersion` under a few milliseconds on a local LAN
- Your NTP server listed as a source with `*` (selected)

If the source shows `?` in the reach column, the server is unreachable — verify the IP and check that UDP port 123 is not blocked by a firewall.

## Step 3: Install the IRIG sender

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
# Normal output on GPIO 17 instead of default 9
sudo ./install.sh -p 17

# Normal on 17, inverted on 27
sudo ./install.sh -p 17 -n 27

# Custom LED warning threshold (blink when root dispersion > 2 ms)
sudo ./install.sh -w 2.0
```

## Step 4: Verify IRIG output

Check the service is running:

```bash
sudo systemctl status irig-sender
```

View the live output:

```bash
sudo journalctl -u irig-sender -f
```

You should see log lines showing frame transmission every 60 seconds, along with periodic chrony status updates (stratum and root dispersion).

## Before Starting Recordings

### Wait for the clock to stabilize

After booting, chrony needs several minutes to discipline the clock — especially on first boot when the oscillator has drifted. **Wait at least 5–10 minutes** before starting any recording session. Even if the NTP server is reachable immediately, the control loop takes time to converge and minimize root dispersion.

### Reading the ACT LED

The IRIG sender controls the RPi's green ACT LED to indicate sync quality at a glance:

| LED state | Meaning |
|-----------|---------|
| **Solid on** | Synced and root dispersion is below the warning threshold — safe to record |
| **Blinking (0.5 s on / 0.5 s off)** | Not synced, or root dispersion exceeds the warning threshold — wait before recording |

The LED is updated once at startup and then after every complete 60-bit frame (~every 60 seconds). If it is still blinking after 10 minutes, see [High root dispersion](#high-root-dispersion) below.

The warning threshold defaults to **1.0 ms** and can be changed at install time:

```bash
# Blink only when root dispersion exceeds 2 ms
sudo ./install.sh -w 2.0
```

To check the current dispersion without waiting for the LED cycle, run:

```bash
chronyc tracking
```

Look at `Root dispersion` — once it is well below 1 ms and `Leap status` shows `Normal`, the clock is ready.

## Troubleshooting

### High root dispersion

- Wait at least 5–10 minutes after boot for chrony to converge
- Run `chronyc tracking` — root dispersion should decrease over time
- Check `chronyc sources` — the NTP server should be selected (`*`)
- The RPi ACT LED will blink if root dispersion exceeds the warning threshold (default 1.0 ms, configurable with `-w`)

### chrony not syncing

- `chronyc sources` showing `?` in reach: the server is unreachable
- Verify the server IP is correct and reachable: `ping <NTP_SERVER_IP>`
- Check firewall: NTP uses UDP port 123
- Verify chrony is running: `systemctl status chrony`

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

To also remove chrony:

```bash
sudo apt remove --purge chrony
```
