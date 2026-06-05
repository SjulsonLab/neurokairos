# RPi GPS NTP Server

Instructions for setting up a Raspberry Pi as a GPS-disciplined stratum 1 NTP server. This is an alternative to a commercial GPS NTP appliance.

**Before going this route, read [Choosing an NTP Server](setup-guide.md#choosing-an-ntp-server) in the main setup guide.** Commercial appliances are easier to maintain and include an OCXO holdover clock that keeps accurate time if GPS lock is lost. A Raspberry Pi with a GPS HAT has no holdover and will drift immediately without a GPS signal.

## Hardware Prerequisites

This page currently documents the Waveshare NEO-M8T test-server
configuration. It is not a generic GPS HAT configuration.

| Component | Notes |
|-----------|-------|
| Raspberry Pi 4 Model B | Other Pi models may work but are untested |
| GPS receiver with PPS output | Waveshare NEO-M8T GNSS Timing HAT for this test setup |
| GPS antenna | Active antenna recommended for indoor use |
| Jumper wires | For connecting PPS and IRIG output |

### GPIO Wiring

| Signal | Default BCM Pin | Direction |
|--------|-----------------|-----------|
| GPS PPS input, Waveshare NEO-M8T HAT | GPIO 18 | GPS -> Pi |
| GPS PPS input, Adafruit GPS HAT | GPIO 4 | GPS -> Pi; documented here only to avoid confusing the HATs |
| GPS serial TX → Pi RX | GPIO 15 (UART RX) | GPS → Pi |
| GPS serial RX ← Pi TX | GPIO 14 (UART TX) | Pi → GPS |
| IRIG-H output (normal) | GPIO 9 | Pi → recorder |
| IRIG-H output (inverted) | disabled | Pi → recorder (optional) |

> **Warning:** Pins 0–1 (I2C HAT ID), 14–15 (UART/GPS), and 18 (Waveshare PPS) are reserved in this test setup. GPIO 4 is the common Adafruit PPS pin, not the Waveshare PPS pin.

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
- Enables the `pps-gpio` device tree overlay on GPIO 18 for the Waveshare NEO-M8T HAT
- Configures chrony with:
  - SHM refclock from gpsd as coarse GPS time (`noselect`)
  - Direct kernel PPS refclock from `/dev/pps0`, locked to SHM
  - No public pool, no TM2000B server, and no gpsd SOCK PPS refclock
  - LAN serving enabled for `10.49.0.0/16` and `10.42.0.0/16`
- Installs persistent GNSS/chrony/PPS status logging to `/home/pi/GNSS_logs`
- Restarts chrony, gpsd, and the GNSS status logger timer

**Reboot required** after installation for the PPS overlay to take effect:

```bash
sudo reboot
```

## Step 2: Verify time synchronisation

After rebooting, wait 2–3 minutes for the GPS to get a fix, then run:

```bash
./test_chrony.sh
```

To inspect a specific holdover window, pass a journal range:

```bash
./test_chrony.sh --since "2026-05-20 15:00" --until "2026-05-20 15:20"
```

**What good output looks like:**

- **chrony tracking:** `Stratum` should be `1`. `Root dispersion` should be below 1 ms once PPS is locked. `Leap status` should be `Normal`.
- **chrony sources:** You should see `#* PPS` (the `*` means it is the selected source). The GPS/SHM source should be present only as coarse `noselect` time.
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

- Verify the Waveshare NEO-M8T PPS signal is on GPIO 18
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

- Verify `#* PPS` appears in `chronyc sources` — the `*` means selected
- If SHM/GPS is `?`, gpsd may not have a fix yet, so chrony cannot label PPS seconds.
- Check the chrony log: `journalctl -u chrony`
- If the problem is intermittent, rerun `./test_chrony.sh --since ... --until ...` around the event window so the script prints `chronyc`, `ppstest`, and `journalctl` together.
- This test setup uses chrony's direct `/dev/pps0` refclock, not gpsd's chrony SOCK feed.

### Holdover/drift testing

The M8T can continue emitting PPS while GNSS lock is lost. In that state,
chrony may continue selecting PPS even though the receiver is in holdover.
Therefore chrony source selection alone is not proof of GNSS lock.

Use the persistent logs in `/home/pi/GNSS_logs` during antenna disconnect and
reconnect tests. The logs separately record GNSS receiver holdover, chrony
holdover/synchronization state, PPS reachability, kernel PPS observation, and
satellite details.

Clock drift during antenna disconnect cannot be measured from chrony versus
M8T PPS alone, because the M8T PPS is the clock under test. Estimate drift from
the offset/correction observed at GNSS reacquisition, or use a separate
observer reference such as the TM2000B in a logging-only role that does not
discipline the RPi clock.

### High root dispersion or stratum not 1

- Wait at least 5–10 minutes after GPS fix for the PPS control loop to converge
- Run `chronyc tracking` repeatedly and watch `Root dispersion` decrease
- Verify PPS is selected as the active source in `chronyc sources`

## Uninstalling

To remove chrony and gpsd entirely:

```bash
sudo apt remove --purge chrony gpsd gpsd-clients pps-tools
```
