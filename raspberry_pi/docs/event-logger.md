# Raspberry Pi Event Logger

## Purpose

The Raspberry Pi event logger continuously records digital input transitions on
configured GPIO pins and assigns each transition a UTC timestamp. It is meant
for cases where cameras, behavioral hardware, or other devices emit TTL pulses
that need to be aligned to the NeuroKairos time base.

The important design choice is that capture is always on. The system does not
depend on a user pressing "record" before data exists.

## Main Components

The event logger is split into two services:

- `neurokairos-eventlogger.service`
  - Runs the C daemon in `raspberry_pi/eventlogger/eventlogger.c`
  - Watches GPIO lines with `libgpiod`
  - Writes the continuous journal and input status snapshots
- `neurokairos-eventlogger-control.service`
  - Runs the Python control/export service in
    `raspberry_pi/eventlogger/eventlogger_control.py`
  - Serves the temporary web UI
  - Tracks an optional active recording window
  - Exports user-facing TSV and YAML files from the continuous journal

## How Event Capture Works

At startup, the C daemon loads `/etc/neurokairos/eventlogger.conf`, which
defines:

- the GPIO chip, usually `gpiochip0`
- the journal directory
- flush and cleanup behavior
- the enabled inputs and their GPIO numbers
- per-input deadtime filtering

The daemon then:

1. Opens the configured GPIO chip.
2. Requests both rising and falling edge events for every enabled input.
3. Reads each line's current logic level.
4. Writes an `inputs.json` snapshot for the control service and UI.
5. Enters a loop waiting for kernel GPIO edge events.

When an edge arrives, the daemon:

1. Reads the kernel event timestamp from the monotonic clock.
2. Applies per-input deadtime filtering to reject duplicate edges that arrive
   too close together.
3. Samples `CLOCK_REALTIME` and `CLOCK_MONOTONIC`.
4. Converts the event's monotonic timestamp into a UTC-based realtime
   nanosecond timestamp by subtracting the monotonic delta from the current
   realtime sample.
5. Appends one row to the raw event journal.
6. Updates the in-memory input state counters and rewrites `inputs.json`.

## Timestamp Model

Each raw event row stores both timing domains:

- `realtime_ns`
  - Nanoseconds since the Unix epoch
  - This is the event's UTC-aligned timestamp source
- `monotonic_ns`
  - Nanoseconds from the monotonic clock
  - Useful for diagnostics and understanding kernel event timing

The human-readable `utc_time` column is derived from `realtime_ns` and written
in UTC with nanosecond text precision, for example:

```text
2026-06-06T18:35:12.123456789Z
```

## Continuous Journal

The daemon writes a raw TSV journal under:

```text
/var/lib/neurokairos/eventlogger/journal
```

Files are organized by UTC month:

```text
journal/
  2026-06_journal/
    all_events_2026-06-06.tsv
    chrony_status_2026-06-06.tsv
```

The raw event file schema is:

```text
utc_time    realtime_ns    monotonic_ns    input    edge
```

`edge` is either `rising` or `falling`.

The daemon keeps file handles open per UTC day and rotates automatically when
the date changes.

## Flush Behavior

Journal writes are buffered and flushed when either threshold is reached:

- `flush_interval_ms`
- `flush_event_count`

With the current example config, that means:

- flush at least once per second
- or sooner if 1000 pending event rows accumulate

This keeps disk I/O reasonable while limiting how much buffered data could be
lost in a crash or sudden power loss.

## Input State Snapshot

The C daemon also writes a live status file:

```text
/var/lib/neurokairos/eventlogger/control/inputs.json
```

This file is rewritten atomically and contains:

- input name
- GPIO number
- enabled flag
- current logic level
- last edge direction
- last edge UTC time
- rising and falling counters

The control service uses this file to populate the web UI and to capture input
baselines when a user starts a recording.

## Chrony Status Logging

Once per `status_interval_s`, the daemon runs `chronyc tracking` and writes a
second daily TSV:

```text
chrony_status_YYYY-MM-DD.tsv
```

This status log records clock health fields such as:

- reference ID
- stratum
- leap status
- system time offset
- RMS offset
- root delay
- root dispersion
- frequency and skew

This is separate from event capture, but it helps interpret timestamp quality
after the experiment.

## Recording Model

The control service does not turn capture on or off. Instead, it defines a time
window inside the always-running journal.

When a user presses start in the UI:

1. The service records the current UTC start time.
2. It stores an `active_recording.json` file under the control directory.
3. It records the requested basename, user, notes, local start time, timezone,
   and the enabled inputs visible at start.

When the user presses stop:

1. The service records the current UTC stop time.
2. It scans the raw journal for rows whose UTC timestamps fall within the start
   and stop bounds.
3. It writes a simplified TSV plus a YAML metadata sidecar into the recordings
   directory.
4. It removes the active recording state file.

If the control service shuts down while a recording is active, it marks the
export as interrupted and still finalizes it on shutdown or next startup.

## Exported Recordings

User-facing exports are written under:

```text
/var/lib/neurokairos/eventlogger/recordings
```

Files are grouped by local month:

```text
recordings/
  2026-06_recordings/
    mouse123_2026-06-06_143512.tsv
    mouse123_2026-06-06_143512.yaml
```

The exported TSV is intentionally simpler than the raw journal:

```text
UTC_time    input    edge
```

The YAML sidecar stores the recording metadata, including:

- basename
- user
- hostname
- start and stop UTC
- local start time
- timezone
- exported inputs
- notes
- interrupted flag

## Cleanup and Retention

The C daemon checks free space every `cleanup_check_interval_s`.

If available space in the journal filesystem drops below
`cleanup_threshold_gb`, it deletes the oldest inactive journal/status TSV files
until free space reaches `cleanup_target_gb`.

Important details:

- active daily files are preserved
- cleanup applies to journal/status TSVs, not exported recordings
- deletions are recorded in `cleanup_audit.tsv`

## Default Configuration

The example config enables eight inputs by default:

- `DIN1` -> BCM 5
- `DIN2` -> BCM 6
- `DIN3` -> BCM 10
- `DIN4` -> BCM 11
- `DIN5` -> BCM 12
- `DIN6` -> BCM 13
- `DIN7` -> BCM 16
- `DIN8` -> BCM 17

The default deadtime is `1 ms` per input. That means back-to-back edges on the
same input that arrive within 1 ms are ignored unless the value is changed to
`0` or another configured number.

## Operational Summary

In practice, the event logger works like this:

1. GPIO edges are captured continuously by the C daemon.
2. Every accepted edge is timestamped and written to the raw journal.
3. The daemon also maintains live input state and clock health logs.
4. The Python control service defines recording windows without affecting
   capture itself.
5. User recordings are exported slices of the raw journal, not separate
   acquisitions.

That architecture is what prevents data loss from forgetting to start a
recording: the raw journal exists first, and recordings are derived from it
later.
