# NeuroKairos IRIG Sender

The IRIG sender is the part of NeuroKairos that outputs the system time as an
IRIG-H TTL signal. In plain terms, it turns the Raspberry Pi into a timing
source that other devices can use to align their recordings to UTC.

The sender runs continuously as a system service. Once it is started, it keeps
generating the IRIG-H pulse train without any user interaction.

## What it does

- Produces a continuous IRIG-H timecode output
- Encodes the current UTC time into the signal
- Uses the Pi's clock discipline status to indicate whether the timebase looks
  healthy
- Can drive an optional inverted output pin if needed

## Default setup

The default output pin is BCM GPIO 9. The sender can also be configured to use
other GPIO pins when the service is installed.

The service name is:

```text
irig-sender.service
```

Useful commands:

```bash
sudo systemctl status irig-sender
sudo journalctl -u irig-sender -f
sudo systemctl restart irig-sender
```

## How to think about it

The sender is not a recorder. It does not save experimental data. Its job is to
provide a stable, time-coded output that other devices and recording systems can
observe and decode.

If the Pi is being used as a chrony-disciplined time server, the sender reflects
that disciplined timebase in the output signal.

The sender is typically installed from the scripts in `raspberry_pi/scripts/`
and is intended to start automatically after reboot.
