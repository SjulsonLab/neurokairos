# Event Logger Handoff

## Scope

This handoff is for continuing the **event logger control service / temporary web UI**
work on the Raspberry Pi via Codex CLI.

Repository:
`/home/pi/neurokairos`

Branch:
`event-logger`

Pi target:
`pi@192.168.1.204`

Primary URL:
`http://neurokairos.local`

## Current local state

Modified files in the local worktree:

- `raspberry_pi/eventlogger/eventlogger_control.py`
- `raspberry_pi/eventlogger/web/index.html`
- `tests/test_eventlogger_control.py`
- `raspberry_pi/eventlogger/README.md`
- `raspberry_pi/eventlogger/event-logger.md`

Unrelated local untracked file:

- `raspberry_pi/sender/README.md`

Important: do not treat `raspberry_pi/sender/README.md` as part of the current
event logger UI task unless the user explicitly asks for it.

## Test status

Latest completed test runs on the local machine:

- `uv run pytest tests/test_eventlogger_control.py -q` -> `11 passed`
- `uv run pytest -q` -> `272 passed`

The previously interrupted full-suite run actually completed successfully.

## Pi deployment status

### Already deployed to the Pi

The Pi is already running the earlier dark-theme / compact-input version with:

- sans serif font
- dark theme
- reserved filename shown during recording
- blinking record light
- compact input dots

Services previously verified active:

- `neurokairos-eventlogger`
- `neurokairos-eventlogger-control`
- `neurokairos-local-alias`
- `smbd`
- `avahi-daemon`

The Pi serves:

- `http://127.0.0.1/`
- `http://127.0.0.1/v1/status`

### Not yet deployed to the Pi

The most recent local-only UI/backend fixes below have **not** been deployed yet:

1. `Pulses recorded` should show **recording-local counts**, not lifetime
   `rising_count`.
2. After stop, pulse boxes should stop incrementing.
3. Remove the extra bottom generic result/status line from the recording panel.
4. Remove the bottom `Enabled: DIN1 GPIO 5 ...` line.
5. Tighten horizontal alignment of `Pulses recorded` relative to the counters.
6. Make DIN numbers larger.
7. Use more purely graphical icon-style record/stop buttons.

Those changes exist locally in:

- `raspberry_pi/eventlogger/eventlogger_control.py`
- `raspberry_pi/eventlogger/web/index.html`
- `tests/test_eventlogger_control.py`

## What changed locally but is not yet on the Pi

### Backend change

`eventlogger_control.py` now snapshots per-input rising-edge baselines at
recording start:

- `recording["input_rising_baselines"] = {input_name: rising_count_at_start}`

`build_status()` now exposes:

- `active_recording["input_rising_baselines"]`

This lets the frontend compute:

- `recording-local pulse count = current rising_count - baseline`

instead of showing lifetime edge totals.

### Frontend change

`web/index.html` now:

- renders separate indicator and pulse-count rows
- removes the bottom enabled-input metadata line
- removes the duplicate result-status line
- uses icon-style record/stop buttons
- enlarges DIN numbers
- tightens spacing between pulse boxes and their label
- shows pulse boxes as recording-local counts only while recording
- shows `0` when idle

## Why the pulse-count bug happened

The previous UI displayed raw `input.rising_count`, which is the daemon's
continuous lifetime counter. That is correct for the raw status feed, but wrong
for a recording-oriented display. The browser needed either:

- journal slicing on every poll, which would be wasteful and wrong for a live UI
- or a recording-start baseline, which is the correct solution

The baseline approach is now implemented locally and tested.

## Next steps on the Pi

From the Pi:

```bash
cd /home/pi/neurokairos
git status --short
```

If this worktree already has the same local edits, continue there. If not,
either copy the local changes over or commit/push them from the Mac first.

Once the updated files are present on the Pi, redeploy with:

```bash
cd /home/pi/neurokairos
sudo ./raspberry_pi/eventlogger/install_eventlogger.sh
```

Then verify:

```bash
systemctl is-active neurokairos-eventlogger neurokairos-eventlogger-control
curl -fsS http://127.0.0.1/v1/status
curl -fsS http://127.0.0.1/ | grep -E 'control-button|Pulses recorded|Digital inputs:'
```

Then test from the Mac at:

`http://neurokairos.local`

## Expected behavior after redeploy

- During recording:
  - record button looks pressed
  - blinking red light is active
  - filename is stable and matches the final exported TSV name
  - duration increments continuously
  - pulse boxes increment from zero for that recording only

- After stop:
  - record button returns to idle state
  - blinking light stops
  - pulse boxes return to zero and no longer increment
  - no extra duplicate filename/status line remains under the recording panel

## Files most worth opening first on the Pi

- `raspberry_pi/eventlogger/eventlogger_control.py`
- `raspberry_pi/eventlogger/web/index.html`
- `tests/test_eventlogger_control.py`

