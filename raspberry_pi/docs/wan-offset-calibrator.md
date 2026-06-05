# WAN NTP Offset Calibration Daemon

## Motivation

WAN NTP servers (Google, Apple, Cloudflare) each have a small but consistent systematic
offset (~0.1–0.3 ms) relative to GPS truth. When chrony switches between servers due to
packet loss or reachability changes, it slews to correct the inter-server offset
difference, causing a transient spike. The fix is a daemon that continuously measures
each WAN server's offset relative to a truth source and writes `offset` correction terms
to a chrony drop-in config file. With corrections applied, all servers look identical to
chrony and source switches cause near-zero correction.

The approach is Option 1 from the design discussion: dynamic include file +
`chronyc reload sources`. No chrony restart is ever needed.

---

## State Machine

The daemon determines its operating mode at the start of each calibration cycle by
inspecting `chronyc tracking`.

| State | Condition | Action |
|---|---|---|
| `TRUTH_ACTIVE` | Selected refid matches `truth_refids` (e.g. GPS, PPS) or `truth_ips` (e.g. local stratum-1 server) | Measure each WAN server against truth via `chronyd -Q`; update per-server EMAs; rewrite include file; reload chrony if any EMA shifted enough |
| `FROZEN` | `truth_ever_seen = True` in persisted state, but truth is no longer selected | Keep last written include file unchanged; log "frozen since \<timestamp\>" each cycle |
| `WAN_ONLY` | Truth has never been seen since first run | Measure inter-server offsets; calibrate relative to chrony's currently-selected WAN server (pseudo-truth = offset 0) |

`truth_ever_seen` is sticky: once set True in the JSON state file, the daemon never
reverts to `WAN_ONLY`. Transition `FROZEN → TRUTH_ACTIVE` occurs when truth reappears.

**Rationale for FROZEN vs. fallback to WAN_ONLY:** When truth temporarily disappears,
the last GPS-calibrated offsets are better than re-deriving relative offsets from the
now-uncalibrated WAN servers. Frozen offsets remain valid for hours because WAN server
path characteristics are stable on that timescale.

---

## Files to Create / Modify

### New files

| File | Purpose |
|---|---|
| `raspberry_pi/scripts/chrony_wan_offset_calibrator.py` | Main daemon |
| `raspberry_pi/systemd/chrony-wan-offset-calibrator.service` | systemd unit |
| `raspberry_pi/scripts/install_wan_offset_calibrator.sh` | Installer script |
| `tests/test_chrony_wan_offset_calibrator.py` | Tests (write first — TDD) |

### Modified files

| File | Change |
|---|---|
| `image/stage-neurokairos-server/.../chrony.conf` | Add WAN server lines with `maxpoll`; the existing `confdir /etc/chrony/conf.d` already picks up the daemon's output file automatically — no `include` directive needed |
| `image/stage-neurokairos-client/.../chrony.conf` | Replace `pool pool.ntp.org` with explicit server lines; add `confdir /etc/chrony/conf.d` if not already present |

A default `wan_servers.conf` (server lines, no `offset` terms) must be staged into the
image alongside each `chrony.conf` so chrony does not fail on a fresh Pi before the
daemon has run for the first time.

---

## Constants and Defaults

```python
CALIBRATION_INTERVAL_S    = 300      # measure every 5 minutes
STARTUP_HOLD_S            = 300      # no reload_sources for the first 5 minutes
MIN_RELOAD_INTERVAL_S     = 3600     # at most one reload per hour
DEFAULT_EMA_ALPHA         = 0.1      # ~9-sample / ~45-minute time constant
DEFAULT_RELOAD_THRESHOLD  = 5e-5     # 0.05 ms minimum shift to trigger a reload
DEFAULT_MAXPOLL           = 10
DEFAULT_WAN_SERVERS       = ["time.google.com", "time.apple.com", "time.cloudflare.com"]
DEFAULT_TRUTH_REFIDS      = frozenset({"GPS", "PPS"})
DEFAULT_TRUTH_IPS         = frozenset({"10.49.98.251"})
```

---

## Main Loop (pseudocode)

```python
startup_mono   = time.monotonic()
last_reload_mono = startup_mono - MIN_RELOAD_INTERVAL_S   # allow reload after hold expires
ema_state      = load_state(config.state_file)

while True:
    ts       = utc_now()
    tracking = run_chronyc_tracking(...)
    mode     = detect_calibration_mode(tracking, ema_state, truth_refids, truth_ips)

    if mode == TRUTH_ACTIVE:
        measurements = measure_wan_servers_against_truth(wan_servers, ...)
        changed      = update_emas(ema_state, measurements, alpha, threshold)
        write_include_file(wan_servers, ema_state, include_path, maxpoll, mode, ts)
        if should_reload(changed, startup_mono, last_reload_mono, ...):
            reload_chrony_sources()
            last_reload_mono = time.monotonic()

    elif mode == FROZEN:
        log_frozen_status(ema_state, ts, config)

    elif mode == WAN_ONLY:
        selected     = get_chrony_selected_source()
        measurements = measure_wan_servers_against_truth(wan_servers, ...)
        changed      = update_emas(ema_state, measurements, alpha, threshold)
        offsets      = compute_wan_only_offsets(ema_state, wan_servers, selected)
        write_include_file(..., offsets_override=offsets)
        if should_reload(changed, ...):
            reload_chrony_sources()
            last_reload_mono = time.monotonic()

    append_tsv_row_calibrator(ts, mode, ema_state, measurements, reload_triggered, config)
    save_state(ema_state, config.state_file)
    _interruptible_sleep(config.calibration_interval_s)
```

`_interruptible_sleep` loops in 1-second increments checking a SIGTERM flag set by a
signal handler.

---

## Key Functions

### Truth detection

```python
def _extract_ref_id_canonical(raw_ref_id: str) -> tuple[str, str]:
    # Returns (name_upper, ip_dotted).
    # Handles: "PPS", "GPS", "C0A8012A (10.49.98.251)", raw IP strings.

def detect_calibration_mode(
    tracking: dict,
    ema_state: dict,
    truth_refids: frozenset[str],
    truth_ips: frozenset[str],
) -> CalibrationMode
```

Reuses `parse_chronyc_tracking()` and `run_chronyc_tracking()` from
`log_tm2000b_observer.py` without modification.

### Measurement

```python
def _parse_chronyd_q_output(output: str) -> float | None:
    # Parses "System clock wrong by X.XXXXXXX seconds (step not made)".
    # Also handles "offset X.XXXXXXX seconds" and scientific notation.
    # Returns None if no parseable offset found.

def measure_one_server_chronyd_q(
    host: str,
    *,
    maxpoll: int,
    timeout_s: float,
    chronyd_binary: str,
) -> dict:   # {ok, host, offset_s, error, duration_s}
    # Runs: chronyd -Q 'server <host> iburst maxpoll <N>'
    # offset_s sign convention: positive = local clock is ahead of server.

def measure_wan_servers_against_truth(
    wan_servers: list[str],
    *,
    maxpoll: int,
    timeout_s: float,
    chronyd_binary: str,
    inter_query_jitter_s: float = 1.0,
) -> dict[str, dict]   # {host: measurement_dict}
    # Measures servers serially with a short inter-query pause.
```

`chronyd -Q` is used rather than raw NTP socket queries because it sends multiple
exchanges and applies chrony's built-in clock filter, giving a much tighter per-sample
estimate (~0.1 ms vs ~0.5 × RTT for a single packet).

### EMA

```python
def update_ema(current_ema: float | None, new_sample: float, alpha: float) -> float
    # First sample (current_ema is None) returns new_sample directly.

def ema_changed_enough(
    old_ema: float | None, new_ema: float, threshold_s: float
) -> bool

def update_emas(
    ema_state: dict,
    measurements: dict[str, dict],
    alpha: float,
    threshold_s: float,
    timestamp: datetime,
) -> bool:
    # Updates ema_state["servers"] in place.
    # Returns True if any server's EMA shifted by >= threshold_s.
    # Skips servers with ok=False.
```

Per-server EMA state fields: `ema_offset_s`, `ema_n`, `last_updated_utc`,
`last_raw_offset_s`.

### Include file

Sign convention: `offset_directive = -ema_offset_s_vs_truth`. If a server's clock is
150 µs fast (its reported time is 150 µs ahead of GPS truth), chrony needs to subtract
150 µs, so `offset -0.000150`.

```python
def write_include_file(
    wan_servers: list[str],
    ema_state: dict,
    include_path: Path,
    maxpoll: int,
    mode: CalibrationMode,
    timestamp: datetime,
    offsets_override: dict[str, float] | None = None,
) -> None:
    # Writes atomically via os.replace(tmp, final).
    # offsets_override is used in WAN_ONLY mode to pass pre-computed relative offsets.
```

Output format:

```
# Generated by chrony-wan-offset-calibrator
# Last updated: 2026-06-05T12:34:56Z
# Mode: TRUTH_ACTIVE  truth_last_seen: 2026-06-05T12:34:56Z
server time.google.com iburst maxpoll 10 offset -0.000150
server time.apple.com iburst maxpoll 10 offset -0.000210
server time.cloudflare.com iburst maxpoll 10 offset -0.000185
```

### WAN-only pseudo-truth

```python
def compute_wan_only_offsets(
    ema_state: dict,
    wan_servers: list[str],
    selected_source: str | None,
) -> dict[str, float]:
    # selected_source gets 0.0.
    # Others get -(their_ema - selected_ema).
    # If selected_source is None or has no EMA yet, all return 0.0.
```

### Reload guard

```python
def should_reload(
    emas_changed: bool,
    startup_mono: float,
    last_reload_mono: float,
    startup_hold_s: float,
    min_reload_interval_s: float,
) -> bool

def reload_chrony_sources(
    *, chronyc_binary: str, timeout_s: float
) -> dict:   # {ok, returncode, output, error}
```

Only reloads when: `emas_changed=True` AND past the startup hold window AND past the
per-hour rate limit.

### State persistence

```python
def load_state(state_path: Path) -> dict   # fresh default if file missing or corrupt
def save_state(state: dict, state_path: Path) -> None   # atomic write
```

Top-level JSON keys: `truth_ever_seen`, `last_truth_seen_utc`,
`offsets_frozen_since_utc`, `servers` (per-server dicts), `mode`,
`include_file_written_utc`.

---

## Systemd Unit

```ini
[Unit]
Description=Chrony WAN NTP offset calibration daemon
After=chrony.service network-online.target
Wants=network-online.target
BindsTo=chrony.service

[Service]
Type=simple
User=root
ExecStart=/usr/bin/python3 /usr/local/sbin/chrony_wan_offset_calibrator.py
Restart=on-failure
RestartSec=30s
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

`User=root` is required to write `/etc/chrony/conf.d/wan_servers.conf` and to run
`chronyd -Q` (which binds to port 123).

---

## Installer (`install_wan_offset_calibrator.sh`)

Follows the pattern of `install_gnss_status_logger.sh`:

1. `set -euo pipefail`; verify running as root
2. `install -m 0755` the script to `/usr/local/sbin/`
3. `install -m 0644` the service unit to `/etc/systemd/system/`
4. `mkdir -p /etc/chrony/conf.d /home/pi/calibrator_logs`
5. If `/etc/chrony/conf.d/wan_servers.conf` does not exist, write a default (server
   lines with no `offset` terms) so chrony starts cleanly before the daemon's first run
6. `systemctl daemon-reload && systemctl enable chrony-wan-offset-calibrator && systemctl restart ...`
7. Print a reminder: "Ensure WAN server lines are NOT duplicated in chrony.conf"

---

## CLI Arguments

| Argument | Default |
|---|---|
| `--wan-servers` | `time.google.com time.apple.com time.cloudflare.com` |
| `--truth-refids` | `GPS PPS` |
| `--truth-ips` | `10.49.98.251` |
| `--include-file` | `/etc/chrony/conf.d/wan_servers.conf` |
| `--state-file` | `/var/lib/chrony/wan_offset_calibrator_state.json` |
| `--output-dir` | `/home/pi/calibrator_logs` |
| `--maxpoll` | `10` |
| `--ema-alpha` | `0.1` |
| `--reload-threshold-s` | `5e-5` |
| `--calibration-interval-s` | `300` |
| `--startup-hold-s` | `300` |
| `--min-reload-interval-s` | `3600` |
| `--chronyc-binary` | `chronyc` |
| `--chronyd-binary` | `chronyd` |
| `--timeout-s` | `30.0` |

---

## TSV Log Schema

Daily files: `/home/pi/calibrator_logs/wan_offset_calibrator_YYYY-MM-DD.tsv`

Fixed columns: `timestamp_utc`, `mode`, `truth_ref_id`, `truth_ever_seen`,
`offsets_frozen_since_utc`, `reload_triggered`.

Per WAN server (dynamic based on `--wan-servers`): `<host>_meas_ok`,
`<host>_meas_offset_s`, `<host>_ema_offset_s`, `<host>_chrony_correction_s`,
`<host>_ema_n`.

Uses `append_tsv_row()` from `log_tm2000b_observer.py`.

---

## Reused Functions

From `raspberry_pi/scripts/log_tm2000b_observer.py`:
- `parse_chronyc_tracking(output)` — parses reference_id, stratum, leap_status
- `run_chronyc_tracking(*, chronyc_binary, timeout_s)` — subprocess wrapper
- `append_tsv_row(path, fieldnames, row)` — TSV logging with auto-header
- `utc_now()`, `iso_utc()`, `_parse_float_field()`

From `raspberry_pi/scripts/log_gnss_status.py`:
- `_load_previous_state()` / `_write_state_file()` pattern for JSON state persistence

---

## Tests (TDD — write before implementation)

File: `tests/test_chrony_wan_offset_calibrator.py`

Module loaded via `importlib.util.spec_from_file_location` following the pattern in
`tests/test_tm2000b_observer_logger.py`.

### Group A — `_parse_chronyd_q_output`
- Parses positive offset from "System clock wrong by X.XXX seconds (step not made)"
- Parses negative offset
- Returns None on unparseable input
- Parses scientific notation (e.g. `3.5e-05 seconds`)

### Group B — `_extract_ref_id_canonical` / `detect_calibration_mode`
- Truth active via refid ("PPS" in truth_refids)
- Truth active via IP in hex format "C0A8012A (10.49.98.251)"
- FROZEN when truth was previously seen but WAN is now selected
- WAN_ONLY on fresh start (truth never seen)
- chronyc failure → FROZEN if truth_ever_seen, WAN_ONLY otherwise
- `truth_ever_seen` is set True in ema_state when TRUTH_ACTIVE is detected

### Group C — `update_ema` / `ema_changed_enough`
- First sample (None) returns new_sample directly
- EMA converges toward a constant input
- `ema_changed_enough`: None old → True; above threshold → True; below → False; exact threshold → True

### Group D — `update_emas`
- Returns True on first measurement (no prior EMA)
- Returns False when EMA is already stable within threshold
- Skips servers with `ok=False`
- Increments `ema_n` and sets `last_updated_utc`

### Group E — `write_include_file`
- File is created at the expected path
- Each WAN server appears with an `offset` directive
- Sign is correct: `ema_offset_s=+0.000150` → `offset -0.000150` in the file
- Atomic: no `.tmp` file left after success
- Mode appears in the comment header

### Group F — `should_reload`
- False when `emas_changed=False`
- False within the startup hold window
- False within the rate-limit window
- True when all conditions are met

### Group G — `load_state` / `save_state`
- Missing file → fresh default state
- Corrupt JSON → fresh default state
- Roundtrip save/load preserves all fields
- Atomic write: no tmp file left on success

### Group H — `compute_wan_only_offsets`
- Selected source gets `0.0`
- Other servers get `-(their_ema - selected_ema)`
- `selected_source=None` → all `0.0`

### Group I — `measure_one_server_chronyd_q` (subprocess mocked)
- Success: `ok=True`, `offset_s` correct
- `subprocess.TimeoutExpired` → `ok=False`, error mentions timeout
- Non-zero exit with no parseable output → `ok=False`

### Group J — `reload_chrony_sources` (subprocess mocked)
- rc=0 → `ok=True`
- rc=1 → `ok=False`

---

## Verification

1. **Tests**: `uv run pytest tests/test_chrony_wan_offset_calibrator.py -v` — all groups
   pass (confirm RED before implementing each group)
2. **Dry-run on Pi**: Run with `--calibration-interval-s 30`; inspect
   `/etc/chrony/conf.d/wan_servers.conf` to confirm offset terms appear; inspect
   `/home/pi/calibrator_logs/` for TSV rows
3. **FROZEN test**: Pass a bogus `--truth-refids` value so no source matches; confirm
   daemon enters FROZEN state and stops updating the include file
4. **Reload confirmation**: `journalctl -u chrony-wan-offset-calibrator -f` shows reload
   events; `chronyc sources` shows updated offset values; verify no large correction spike
   in `tm2000b_internet_observer_logs` around the reload time
