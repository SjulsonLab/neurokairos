# RPi GPS NTP Server

Instructions for setting up a Raspberry Pi as a GPS-disciplined stratum 1 NTP server. This is an alternative to a commercial GPS NTP appliance.

**Before going this route, read [Choosing an NTP Server](setup-guide.md#choosing-an-ntp-server) in the main setup guide.** Commercial appliances are easier to maintain and include an OCXO holdover clock that keeps accurate time if GPS lock is lost. A Raspberry Pi with a GPS HAT has no holdover and will drift immediately without a GPS signal.

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
| GPS serial TX → Pi RX | GPIO 15 (UART RX) | GPS → Pi |
| GPS serial RX ← Pi TX | GPIO 14 (UART TX) | Pi → GPS |
| IRIG-H output (normal) | GPIO 9 | Pi → recorder |
| IRIG-H output (inverted) | disabled | Pi → recorder (optional) |

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

## Step 1: Install chrony + gpsd

```bash
cd raspberry_pi/scripts
sudo ./install_chrony_server.sh
```

This script:
- Installs `chrony`, `gpsd`, and `pps-tools`
- Configures gpsd to use `/dev/ttyAMA0` (override with `--serial-device /dev/ttyXXX`)
- Enables the `pps-gpio` device tree overlay on GPIO 4
- Configures chrony with:
  - SHM refclock from gpsd (stratum 1, offset 0.5 s for NMEA latency)
  - PPS refclock from `/dev/pps0` (stratum 1, lock to SHM)
  - Public NTP pool servers as fallback
  - LAN serving enabled (`allow all`)
- Restarts chrony and gpsd

**Reboot required** after installation for the PPS overlay to take effect:

```bash
sudo reboot
```

## Step 2: Verify time synchronisation

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

## Step 3: Install the IRIG sender

Follow Steps 3 and 4 from the [main setup guide](setup-guide.md#step-3-install-the-irig-sender).

## Configuring Client Pis

Once this server is running, note its LAN IP address. On each encoder Pi, install chrony pointing to this server:

```bash
sudo ./install_chrony_client.sh --server <THIS_PI_IP>
```

Then proceed with the rest of the [main setup guide](setup-guide.md#step-3-install-the-irig-sender).

## Troubleshooting

### No PPS pulses

- Verify the PPS wire is connected to GPIO 4
- Check the overlay is loaded: `dmesg | grep pps`
- Verify the device exists: `ls /dev/pps0`
- If `/dev/pps0` is missing, the reboot after `install_chrony_server.sh` may not have happened
- Test with: `sudo ppstest /dev/pps0` (should show timestamps every second)

### GPS not getting a fix

- Check antenna placement — the antenna needs a clear view of the sky
- Verify gpsd is running: `systemctl status gpsd`
- Check for NMEA data: `gpspipe -w | head -5` (should show sentences; `mode: 2` or `mode: 3` means a fix)
- Allow up to 15 minutes for cold start acquisition

### chrony not selecting PPS

- Verify `#* PPS0` appears in `chronyc sources` — the `*` means selected
- If SHM is `?`, gpsd may not have a fix yet; PPS cannot lock without it
- Check the chrony log: `journalctl -u chrony`

### High root dispersion or stratum not 1

- Wait at least 5–10 minutes after GPS fix for the PPS control loop to converge
- Run `chronyc tracking` repeatedly and watch `Root dispersion` decrease
- Verify PPS is selected as the active source in `chronyc sources`

## Uninstalling

To remove chrony and gpsd entirely:

```bash
sudo apt remove --purge chrony gpsd gpsd-clients pps-tools
```
