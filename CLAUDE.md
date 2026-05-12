# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

NeuroKairos: A GPS-disciplined IRIG-H timecode system for synchronizing neuroscience data streams (electrophysiology, cameras, behavioral data) to UTC time. Runs on Raspberry Pi 4B with a GPS timing receiver.

IRIG-H encodes UTC time as pulse-width modulated TTL signals: 60 bits/frame at 1 Hz, using 0.2s (binary 0), 0.5s (binary 1), and 0.8s (position marker P) pulse widths in BCD format.

## Build Commands

```bash
# Compile the C sender (runs on Raspberry Pi, requires root for /dev/mem)
cd raspberry_pi/sender && make

# Install as systemd service (default pins)
./raspberry_pi/scripts/install.sh

# Install with custom pins
./raspberry_pi/scripts/install.sh -p 17 -n 27

# Install with custom LED warning threshold
./raspberry_pi/scripts/install.sh -p 17 -w 2.0

# Uninstall systemd service
./raspberry_pi/scripts/uninstall.sh

# Run with default pins (BCM GPIO 9, inverted disabled)
./raspberry_pi/sender/irig_sender

# Run with custom pins
./raspberry_pi/sender/irig_sender -p 17 -n 27

# Run with custom LED warning threshold (blink when root dispersion > 2ms)
./raspberry_pi/sender/irig_sender -w 2.0

# Run with per-pulse latency logging
./raspberry_pi/sender/irig_sender --latency-log /tmp/latency.csv

# Install Python package (editable/development mode)
pip install -e ".[test]"

# Run tests
pytest tests/ -v
```

Prebuilt Pi OS images are produced by pi-gen — see `image/README.md` and `.github/workflows/build-image.yml`. The workflow cross-compiles `raspberry_pi/sender/irig_sender.c` for arm64 on the runner (using `gcc-aarch64-linux-gnu`) and drops the binary plus `raspberry_pi/systemd/irig-sender.service` into the pi-gen stage's `files/` at build time, so those source locations remain the single source of truth. We deliberately avoid compiling inside pi-gen's chroot because qemu/binfmt reliability across stage boundaries is fragile.

## Architecture

Three-component system: **Encoder** (Raspberry Pi, generates IRIG-H), **Decoder** (Python, extracts timing from recordings), and **Synchronizer** (`ClockTable`, bridges clock domains).

### Encoder (C sender)
- `raspberry_pi/sender/irig_sender.c` — Production sender. Uses direct `/dev/mem` GPIO register access with hybrid sleep/busy-wait for nanosecond-level timing precision. Default output on BCM GPIO 9 (normal), inverted disabled. Both pins configurable via CLI flags (`-p`/`-n`). Polls chrony every ~60 seconds and encodes sync status (stratum, root dispersion) in unused IRIG-H frame bits 43-44 and 46-48. Controls the RPi ACT LED to indicate sync quality. RT hardening: SCHED_FIFO priority 80, `mlockall` to prevent page faults, `CLOCK_MONOTONIC` for busy-wait loops (immune to chrony step adjustments). No CPU affinity pinning (avoids scheduler-attraction effect). `--latency-log FILE` records per-pulse onset latency for benchmarking. Runs as systemd service at Nice -20.

### Chrony Integration
- `raspberry_pi/scripts/install_chrony_server.sh` — Installs chrony + gpsd on the RPi with GPS. Configures PPS-disciplined stratum 1 NTP server.
- `raspberry_pi/scripts/install_chrony_client.sh` — Installs chrony as NTP client (no GPS). Supports custom server (`--server`).
- `raspberry_pi/scripts/test_chrony.sh` — Diagnostic script for checking chrony/gpsd status.

### Core Python Library (`neurokairos/`)
- `clock_table.py` — `ClockTable` dataclass: sparse time mapping (source <-> reference) with bidirectional interpolation (linear extrapolation up to 1.5 s beyond boundaries; returns NaN beyond that), optional per-pulse sync arrays (`sync_stratum`, `sync_dispersion_upperbound_ms`), save/load to NPZ, JSON-serializable metadata.

### Decoder Subpackage (`neurokairos/decoders/`)
- `ttl.py` — Signal processing: `auto_threshold` (Otsu's method), `detect_edges`, `measure_pulse_widths`. NumPy only, no dependencies on other modules.
- `irig.py` — Complete IRIG-H decoder pipeline: pulse classification, BCD encode/decode, frame decoding (complete + partial), robust handling of missing/extra pulses and concatenated files, `build_clock_table` orchestrator (computes measured rate as source_span/reference_span, builds per-pulse sync arrays from frame anchors), plus top-level entry points `decode_dat_irig` and `decode_intervals_irig`.
- `sglx.py` — SpikeGLX `.meta` reader + `decode_sglx_irig` entry point.
- `video.py` — Video LED extraction + `decode_video_irig` entry point. OpenCV (`cv2`) is an optional dependency.
- `events.py` — Event log parsing for MedPC files and CSV/TSV. Extracts IRIG pulse events and behavioral events from timestamped event logs. Supports MedPC TIME.CODE format and generic CSV/TSV. Provides `extract_irig_pulses` (pair HIGH/LOW → onsets/offsets for `decode_intervals_irig`), `filter_non_pulse_events`, `convert_events_to_utc`, and `write_events_csv`.
- `decoder.py` — `IRIGDecoder` unified facade class with `from_dat`, `from_sglx`, `from_video`, `from_intervals`, and `from_events` classmethods. Delegates to the standalone `decode_*` functions. Event-based inputs support post-decode behavioral event extraction via `get_behavioral_events_utc` and `save_behavioral_events_csv`.
- `report.py` — Sync report PNG generation. `generate_sync_report` produces a multi-panel diagnostic figure (signal snippet, pulse histogram, full overview, clock jitter, sync status). `_try_generate_report` is the entry-point wrapper that catches errors so report failures never block decoding. matplotlib is lazy-imported (optional dependency via `pip install neurokairos[report]`). All `decode_*` entry points call `_try_generate_report` when `save=True`, producing a `.sync_report.png` alongside the `.clocktable.npz`. When `raw_signal` is None (intervals path), only jitter and sync panels are drawn. The jitter panel title and text header show the measured rate with units (Hz for samples, fps for frames, s/s for seconds).

### Public API (`neurokairos/__init__.py`)
- `ClockTable` — sparse time mapping
- `IRIGDecoder` — unified decoder facade (from `decoders/decoder.py`)
- `bcd_encode`, `bcd_decode` — BCD encoding/decoding
- `decode_dat_irig` — decode from interleaved int16 `.dat` files
- `decode_sglx_irig` — decode from SpikeGLX `.bin` + `.meta`
- `decode_video_irig` — decode from video files with IRIG LED
- `decode_intervals_irig` — decode from pre-extracted pulse intervals
- `parse_medpc_file` — parse MedPC data files
- `parse_csv_events` — parse CSV/TSV event files
- `extract_irig_pulses` — extract IRIG pulse onsets/offsets from event data
- `convert_events_to_utc` — convert event timestamps to UTC via ClockTable
- `ROOT_DISPERSION_UPPER_MS` — bucket upper bounds for root dispersion (ms)
- BCD weight constants: `SECONDS_WEIGHTS`, `MINUTES_WEIGHTS`, `HOURS_WEIGHTS`, `DAY_OF_YEAR_WEIGHTS`, `DECISECONDS_WEIGHTS`, `YEARS_WEIGHTS`

### MATLAB Interface (`matlab/`)
- `ClockTable.m` — MATLAB classdef that loads `.clocktable.npz` files and provides `source_to_reference`/`reference_to_source` interpolation matching Python behavior exactly (including 1.5 s extrapolation limit → NaN beyond).
- `decode_irig.m` — Function that shells out to Python to run decoding, then loads the resulting NPZ. Supports `'format'`, `'n_channels'`, `'irig_channel'`, and `'python'` name-value pairs.
- `+nk_internal/read_npy.m` — Internal helper to parse `.npy` files (float64 and unicode string dtypes).
- `+nk_internal/read_npz.m` — Internal helper to unzip `.npz` and dispatch to `read_npy`.

## Key Constants (neurokairos/decoders/irig.py)

Pulse-width fractions of 1 second: `PULSE_FRAC_ZERO` (0.2), `PULSE_FRAC_ONE` (0.5), `PULSE_FRAC_MARKER` (0.8). Classification boundaries are midpoints: 0.35 (0/1), 0.65 (1/marker). Min valid: 0.1, max: 0.95.

## Sync Status Encoding (NeuroKairos Extension)

The C sender polls chrony every ~60 seconds and encodes sync quality in previously unused IRIG-H frame bits. Bits 43-44 carry a 2-bit stratum code (1->0, 2->1, 3->2, >=4->3). Bits 46-48 carry a 3-bit root dispersion bucket on a doubling scale from <0.25ms (0) to >=16ms (7). Bits 42 and 45 remain zero (reserved). Old recordings with all-zero status bits are ambiguous with stratum 1 / best dispersion. See `docs/irig-h-standard.md` for the full encoding table.

The Python decoder extracts sync status from each complete frame and produces two per-pulse arrays on ClockTable: `sync_stratum` (float64, values 1-4) and `sync_dispersion_upperbound_ms` (float64, bucket upper bounds from `ROOT_DISPERSION_UPPER_MS`: 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, inf). Values are forward-filled from frame anchors and backward-filled for leading pulses before the first decoded frame. Session-level worst-case scalars remain in metadata (`stratum`, `UTC_sync_precision`).

## Known Bug History

- **Day-of-year off-by-one (C sender, fixed in `6def02b`):** C's `tm_yday` is 0-indexed (0-365) but IRIG-H expects 1-indexed (1-366). The original C sender omitted the `+1`, causing every transmitted day to be one too low. Python was never affected (`timetuple().tm_yday` is already 1-indexed). Old branches `less-cpu` and `charlie-irig` still have the unfixed code.
- **Local time used instead of UTC in C sender encoding (fixed in `981834e`):** `precalculate_next_frame()` called `localtime()` instead of `gmtime()` to decompose the Unix timestamp into BCD fields. On a Pi configured with a non-UTC timezone, the encoded IRIG time would be offset by the UTC offset — and would shift by an additional hour at DST transitions.
- **Local time used for century reconstruction in Python decoder (fixed in `f7aabfb`):** `decode_frame()` and `decode_partial_frame()` called `datetime.now().year` (no timezone) to derive the century from the 2-digit IRIG year. On a system whose local year differs from UTC year (e.g., Dec 31 local time while UTC is already Jan 1), this would produce a timestamp in the wrong century. Fixed by passing `timezone.utc` to `datetime.now()`.
- **Frames must start on minute boundaries:** The C sender waits for the next :00 second before transmitting its first frame. Each frame is 60 bits (60 seconds), so subsequent frames naturally align to minute boundaries. The BCD seconds field is always 0.
