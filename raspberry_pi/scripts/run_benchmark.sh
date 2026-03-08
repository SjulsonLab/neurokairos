#!/bin/bash

# Run the IRIG latency benchmark with progressive CPU stress.
# Usage: ./run_benchmark.sh <label> [sender_path]
#
# Produces ~/irig_latency_<label>_<hostname>.csv
# Five 20-minute phases: idle, 1-core, 2-core, 3-core, 4-core stress.
# Uses stress-ng if available, otherwise falls back to shell-based CPU burner.

set -e

LABEL="${1:?Usage: $0 <label> [sender_path]}"
SENDER="${2:-./irig_sender}"
HOSTNAME=$(hostname)
LATENCY_LOG="$HOME/irig_latency_${LABEL}_${HOSTNAME}.csv"
PHASE_MINUTES=20
TOTAL_FRAMES=$((PHASE_MINUTES * 5))  # 100 frames = 100 minutes

echo "=== IRIG Latency Benchmark ==="
echo "Label:    $LABEL"
echo "Sender:   $SENDER"
echo "Log:      $LATENCY_LOG"
echo "Frames:   $TOTAL_FRAMES (${PHASE_MINUTES}min x 5 phases)"
echo ""

# CPU stress function: uses stress-ng if available, otherwise shell busy-loop
run_cpu_stress() {
    local N_CORES=$1
    local DURATION_SEC=$2

    if command -v stress-ng &>/dev/null; then
        stress-ng --cpu "$N_CORES" --timeout "${DURATION_SEC}s"
    else
        # Shell-based CPU burner fallback
        local PIDS=()
        for i in $(seq 1 "$N_CORES"); do
            ( while true; do :; done ) &
            PIDS+=($!)
        done
        sleep "$DURATION_SEC"
        for pid in "${PIDS[@]}"; do
            kill "$pid" 2>/dev/null || true
        done
        wait "${PIDS[@]}" 2>/dev/null || true
    fi
}

PHASE_SECONDS=$((PHASE_MINUTES * 60))

# Start IRIG sender in background
sudo "$SENDER" --frames $TOTAL_FRAMES --latency-log "$LATENCY_LOG" &
IRIG_PID=$!
echo "[$(date)] Started IRIG sender (PID $IRIG_PID)"

# Phase 1: idle
echo "[$(date)] Phase 1/5: idle (${PHASE_MINUTES} min)"
sleep $PHASE_SECONDS

# Phase 2: 1 core stress
echo "[$(date)] Phase 2/5: 1 core stress (${PHASE_MINUTES} min)"
run_cpu_stress 1 $PHASE_SECONDS
echo "[$(date)] Phase 2 complete"

# Phase 3: 2 cores stress
echo "[$(date)] Phase 3/5: 2 cores stress (${PHASE_MINUTES} min)"
run_cpu_stress 2 $PHASE_SECONDS
echo "[$(date)] Phase 3 complete"

# Phase 4: 3 cores stress
echo "[$(date)] Phase 4/5: 3 cores stress (${PHASE_MINUTES} min)"
run_cpu_stress 3 $PHASE_SECONDS
echo "[$(date)] Phase 4 complete"

# Phase 5: 4 cores stress
echo "[$(date)] Phase 5/5: 4 cores stress (${PHASE_MINUTES} min)"
run_cpu_stress 4 $PHASE_SECONDS
echo "[$(date)] Phase 5 complete"

# Wait for IRIG sender to finish
echo "[$(date)] Waiting for IRIG sender to finish..."
wait $IRIG_PID
echo "[$(date)] Benchmark complete: $LATENCY_LOG"
