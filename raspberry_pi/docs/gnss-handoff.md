# GNSS / Chrony Handoff

This note captures the Waveshare NEO-M8T test-server configuration that has been validated on the Raspberry Pi 5 at `pi@10.49.98.220`. It is meant to reduce rediscovery the next time the GNSS hat or chrony settings need to change.

## What This Setup Is

- Test-only Waveshare NEO-M8T timing server
- Direct kernel PPS into chrony via `/dev/pps0`
- gpsd is used only for coarse GPS time and receiver state
- Persistent logging is enabled under `/home/pi/GNSS_logs`

## 1. Continued PPS During GNSS Holdover

The setting that matters is the receiver's UBX TP5 configuration. The working configuration is an explicit raw `CFG-TP5` payload that programs the pulse to continue in both locked and unlocked states.

Working behavior:

- 1 Hz PPS
- 100 ms pulse width
- holds PPS output during GNSS unlock / holdover

The raw command that worked is:

```bash
ubxtool localhost:2947:/dev/ttyAMA0 -c "06,31,00,01,00,00,02,00,00,00,40,42,0f,00,40,42,0f,00,a0,86,01,00,a0,86,01,00,00,00,00,00,77,00,00,00" -w 4
```

Relevant fields in that payload:

- `freqPeriod` = `1000000`
- `freqPeriodLock` = `1000000`
- `pulseLenRatio` = `100000`
- `pulseLenRatioLock` = `100000`
- `flags` = `0x77`

Important failure mode:

- `ubxtool -e PPS` was not sufficient in this setup
- it left the unlocked TP5 pulse length at zero
- that produced a receiver state where PPS did not behave as intended during holdover

Boot-time receiver configuration lives in:

- [raspberry_pi/scripts/configure_gnss_pps.sh](/Users/lukesjulson/claudecode/IRIG/neurokairos/raspberry_pi/scripts/configure_gnss_pps.sh)
- [raspberry_pi/systemd/gnss-pps.service](/Users/lukesjulson/claudecode/IRIG/neurokairos/raspberry_pi/systemd/gnss-pps.service)

## 2. Chrony Configuration for PPS

Chrony is configured to read the PPS signal directly and use gpsd only for coarse time labeling.

Working chrony settings:

```conf
confdir /etc/chrony/conf.d
hwtimestamp eth0
refclock SHM 0 refid GPS precision 1e-1 noselect
refclock PPS /dev/pps0 refid PPS lock GPS precision 1e-7 prefer
allow all
deny 10.49.0.0/16
deny 10.42.0.0/16
local stratum 15 orphan
logdir /var/log/chrony
log tracking statistics measurements
```

What matters here:

- `refclock PPS /dev/pps0 ... lock GPS` is the direct PPS path
- `SHM 0` is coarse only and is marked `noselect`
- `hwtimestamp eth0` is present on this Pi 5 setup
- the lab client ranges are denied during the test window
- `local stratum 15 orphan` is present, but it does not mean clients should trust the box during holdover; it only keeps chrony locally alive

Chrony is installed and managed by:

- [raspberry_pi/scripts/install_chrony_server.sh](/Users/lukesjulson/claudecode/IRIG/neurokairos/raspberry_pi/scripts/install_chrony_server.sh)
- [image/stage-neurokairos-server/00-config-gps-server/files/chrony.conf](/Users/lukesjulson/claudecode/IRIG/neurokairos/image/stage-neurokairos-server/00-config-gps-server/files/chrony.conf)

## 3. Other Pertinent Settings

### GPSD

Working gpsd settings:

```conf
START_DAEMON="true"
GPSD_OPTIONS="-n"
DEVICES="/dev/ttyAMA0 /dev/pps0"
USBAUTO="false"
```

### PPS Overlay

The Waveshare hat uses GPIO 18, not GPIO 4.

Boot firmware setting:

```conf
dtoverlay=pps-gpio,gpiopin=18,capture_clear,pull=up
```

Notes:

- GPIO 4 is the Adafruit GPS HAT PPS pin, not the Waveshare NEO-M8T pin
- `capture_clear` and `pull=up` were needed in this setup

### Logging

The status logger runs every 60 seconds and writes to `/home/pi/GNSS_logs`.

It records:

- receiver holdover / GNSS lock state
- chrony sync / holdover state
- selected chrony source
- PPS reach and recent sample timing
- satellite counts and satellite details

Relevant files:

- [raspberry_pi/scripts/log_gnss_status.py](/Users/lukesjulson/claudecode/IRIG/neurokairos/raspberry_pi/scripts/log_gnss_status.py)
- [raspberry_pi/systemd/gnss-status-logger.service](/Users/lukesjulson/claudecode/IRIG/neurokairos/raspberry_pi/systemd/gnss-status-logger.service)
- [raspberry_pi/systemd/gnss-status-logger.timer](/Users/lukesjulson/claudecode/IRIG/neurokairos/raspberry_pi/systemd/gnss-status-logger.timer)

## What We Verified

- `ppstest /dev/pps0` shows 1 Hz assert / clear events
- chrony selects `PPS` when the receiver is healthy
- the explicit TP5 payload restores PPS behavior that `ubxtool -e PPS` did not preserve
- the Pi remains usable for logging during antenna disconnect tests

## Temporary Internet-Only Drift Test

Use this when you want the Pi to ignore both the GNSS hat and the TM2000B as
disciplining sources, while still measuring its offset to the TM2000B in a
separate observer log.

### Chrony configuration

The internet-only test config uses only public NTP sources for discipline:

```conf
server time.google.com iburst prefer
server time.cloudflare.com iburst
server time.apple.com iburst
pool pool.ntp.org iburst maxsources 4

allow 10.49.0.0/16
allow 10.42.0.0/16

local stratum 15 orphan
logdir /var/log/chrony
log tracking statistics measurements
```

### Separate observer logger

This test uses a separate logger and timer so the existing TM2000B observer
remains unchanged:

- [raspberry_pi/scripts/log_tm2000b_internet_observer.py](/Users/lukesjulson/claudecode/IRIG/neurokairos/raspberry_pi/scripts/log_tm2000b_internet_observer.py)
- [raspberry_pi/scripts/install_tm2000b_internet_observer.sh](/Users/lukesjulson/claudecode/IRIG/neurokairos/raspberry_pi/scripts/install_tm2000b_internet_observer.sh)
- [raspberry_pi/systemd/tm2000b-internet-observer.service](/Users/lukesjulson/claudecode/IRIG/neurokairos/raspberry_pi/systemd/tm2000b-internet-observer.service)
- [raspberry_pi/systemd/tm2000b-internet-observer.timer](/Users/lukesjulson/claudecode/IRIG/neurokairos/raspberry_pi/systemd/tm2000b-internet-observer.timer)

It writes daily TSV files under `/home/pi/tm2000b_internet_observer_logs`.

## Cautions

- Chrony selecting PPS is not proof that GNSS lock is present
- Receiver holdover and chrony holdover are different states
- If the goal is to stop clients trusting the Pi, do not assume `local stratum 15 orphan` is enough; use access control or disable serving for the test window
