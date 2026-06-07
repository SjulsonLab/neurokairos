# NeuroKairos TTL Event Logger

The NeuroKairos event logger records TTL input changes on Raspberry Pi GPIO pins.
It is intended for experiments where behavioral equipment, cameras, or other
devices emit digital pulses that should be timestamped against the NeuroKairos
timebase.

The logger runs continuously as a system service. Users do not need to press
"record" before events are captured. Whenever a configured input changes state,
the logger writes one row to a daily event journal.

Each logged event includes:

- UTC timestamp
- raw realtime clock timestamp in nanoseconds
- monotonic clock timestamp in nanoseconds
- input name, such as `DIN1`
- edge type, `rising` or `falling`

The event TSV files are written under:

```text
/var/lib/neurokairos/eventlogger/journal
```

Within that directory, files are organized into month buckets such as
`2026-06_journal/`.

The top-level `README.txt` in that directory explains the retention policy:
oldest inactive journal files are deleted automatically when disk space runs
low.

Saved recordings are separate, user-facing exports sliced from that journal.
Those recording TSVs keep only:

- UTC timestamp
- input name
- edge type

They are written under:

```text
/var/lib/neurokairos/eventlogger/recordings
```

Within that directory, exported recordings are organized into month buckets
such as `2026-06_recordings/`.

The top-level `README.txt` in that directory explains the retention policy:
oldest inactive recording exports are deleted automatically when disk space
runs low.

The logger also writes a daily chrony status TSV in the same directory. This
status log records the system clock state once per minute, which helps interpret
event timing quality after an experiment.

When installed, the event logger directory is also shared read-only over SMB so
the logs can be copied from another computer on the network.

## Current Test Setup

For the initial Raspberry Pi 5 test setup, the event logger is configured to use
BCM GPIO 5 as `DIN1`. The IRIG sender uses BCM GPIO 9 by default.

To test the event logger directly from the IRIG sender, connect a jumper from:

```text
BCM GPIO 9  ->  BCM GPIO 5
```

On the 40-pin Raspberry Pi header, this is:

```text
physical pin 21  ->  physical pin 29
```

After the jumper is connected, the event logger should begin writing detected
IRIG pulse edges to the daily `events_YYYY-MM-DD.tsv` file.

## Service

The event logger is installed as:

```text
neurokairos-eventlogger.service
```

Useful commands:

```bash
sudo systemctl status neurokairos-eventlogger
sudo journalctl -u neurokairos-eventlogger -f
ls -lh /var/lib/neurokairos/eventlogger/journal
```

The example configuration file is:

```text
eventlogger.conf.example
```

When installed, the active system configuration is normally:

```text
/etc/neurokairos/eventlogger.conf
```
