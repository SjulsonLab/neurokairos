# Neurokairos TTL Event Logging and Recording Plan

## Overview

Add an optional **TTL Event Logger** to Neurokairos.

The logger continuously timestamps digital input events (rising/falling edges) and maps them to UTC time using the Neurokairos timebase. The logger operates independently of the existing IRIG output functionality.

The system should be designed so that users never lose data because they forgot to press "Record".

---

## Core Design Philosophy

### Continuous Logging

TTL capture runs continuously whenever Neurokairos is running.

There is no requirement for the user to start or stop acquisition.

Neurokairos maintains a rolling event journal containing all detected TTL transitions.

### User Recordings

The web interface provides optional "Record" and "Stop" buttons.

Pressing Record does **not** start acquisition.

Instead, it creates a named session that marks a start time.

Pressing Stop marks the end time and exports a slice of the continuous journal.

Recordings are therefore named exports derived from the continuous log.

If a user forgets to press Record, they can later retrieve timestamps from the continuous journal.

---

## Hardware Architecture

### Initial Implementation

Use Raspberry Pi 5 GPIO inputs with `libgpiod`.

Use kernel GPIO edge events and kernel timestamps.

No RP2040/Pico required initially.

### Future Possibilities

Future versions may support:

- RP2040/Pico-based hard realtime capture
- External DAQs
- Additional timestamping backends

The logging API should be designed so alternate capture backends can be added later.

---

## Event Storage

Store events in a continuously updated journal.

The continuous journal should retain the full raw event timing fields needed for
diagnostics and later analysis.

Saved recordings exported for users should contain only:

- UTC timestamp
- input identifier
- edge type (rising/falling)

Suggested recording TSV schema:

```text
UTC_time
input
edge
```

UTC timestamps are authoritative. The raw journal remains the source of truth
for higher-resolution timing diagnostics.

---

## Continuous Journal Retention

### Default Policy

Keep all journal data indefinitely.

No automatic deletion based on age.

### Low Disk Space Handling

When available disk space falls below a configurable threshold:

1. Delete oldest journal files first
2. Never delete active journal files
3. Never delete saved recordings by default
4. Continue deleting oldest journal files until free-space target is restored

Suggested defaults:

```text
Cleanup threshold: 10 GB free
Cleanup target: 25 GB free
```

Maintain a deletion log for auditing.

---

## Recording Workflow

### User Interface

User enters:

```text
Basename: mouse123
```

Then clicks:

```text
Start Recording
```

Later:

```text
Stop Recording
```

### Recording Creation

Neurokairos records:

- start UTC
- stop UTC
- basename
- selected inputs

and exports a file containing all events within that interval.

---

## File Naming

Use the user-provided basename plus the **local start time** of the recording.

Format:

```text
{basename}_{YYYY-MM-DD}_{HHMMSS}
```

Example:

```text
mouse123_2026-06-06_143512.tsv
mouse123_2026-06-06_143512.yaml
```

### Time Zone Rules

Filenames use local time.

Event timestamps always use UTC.

No UTC offset or timezone information should appear in filenames.

DST ambiguity is ignored because:

- filenames are for human convenience only
- authoritative timestamps are stored in UTC inside the TSV

---

## Event Export Format

### Primary Data File

TSV format.

Example:

```tsv
utc_time	input	edge
2026-06-06T18:35:12.123456Z	DIN1	rising
2026-06-06T18:35:13.654321Z	DIN1	falling
```

UTC timestamps are required.

---

## Metadata Sidecar

Store metadata in YAML.

Example:

```yaml
basename: mouse123

hostname: neurokairos-0A4C38B7

start_utc: 2026-06-06T18:35:12.123456Z
stop_utc: 2026-06-06T19:12:44.654321Z

start_local: 2026-06-06T14:35:12
timezone: America/New_York

inputs:
  - DIN1
  - DIN2

software_version: 0.1.0
```

YAML is preferred over JSON.

---

## Browser Interface

### Recording Controls

Provide:

```text
Basename field
Start Recording
Stop Recording
```

### Recording Browser

Display recordings using local time.

Example:

```text
mouse123    2026-06-06 14:35:12    37 min
mouse124    2026-06-06 16:02:08    42 min
```

Allow:

- Download TSV
- Download YAML metadata
- Delete recording

### Continuous Log Recovery

Provide UI to export from journal without an existing recording.

Example:

```text
Start Time
End Time
Inputs
Export TSV
```

This allows recovery when the user forgot to press Record.

---

## File Access

### SMB Sharing

SMB should be a standard Neurokairos feature.

Expose a shared folder containing:

```text
recordings/
journal/

Both directories are month-organized in the installed layout, for example
`2026-06_recordings/` and `2026-06_journal/`.
```

Users should be able to access files directly from acquisition computers via SMB.

### Browser Downloads

Also allow downloading recordings through the web interface.

---

## Time Handling Rules

### UTC

Use UTC for:

- Event timestamps
- Internal storage
- APIs
- Synchronization

### Local Time

Use local time for:

- Filenames
- Recording browser display
- User-facing timestamps

UTC remains the authoritative time representation.

---

## Future Extensions

Potential future additions:

- RP2040/Pico capture backend
- DAQ integrations
- REST API access
- Automatic export workflows
- Event annotations/markers
- Additional metadata fields

The initial implementation should focus on:

1. Continuous logging
2. Named recordings
3. TSV export
4. YAML metadata
5. SMB file access
6. Automatic low-space cleanup
