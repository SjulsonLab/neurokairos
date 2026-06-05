#!/bin/bash

# Diagnostic script for checking chrony and gpsd status.
#
# Checks:
#   - chrony service status
#   - chronyc tracking output (stratum, root dispersion, sync status)
#   - chronyc sources output (which sources are reachable)
#   - chrony/gpsd/gnss-pps journal entries for an optional time window
#   - gpsd status (if installed, i.e., server mode)
#   - PPS device availability
#
# Usage:
#   ./test_chrony.sh [--since "YYYY-MM-DD HH:MM"] [--until "YYYY-MM-DD HH:MM"] [--lines N]

set -uo pipefail

SINCE_ARG=""
UNTIL_ARG=""
JOURNAL_LINES=100

print_usage() {
    echo "Usage: $0 [--since \"YYYY-MM-DD HH:MM\"] [--until \"YYYY-MM-DD HH:MM\"] [--lines N]"
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --since)
            shift
            if [ "$#" -eq 0 ]; then
                echo "Error: --since requires a value"
                print_usage
                exit 1
            fi
            SINCE_ARG="$1"
            ;;
        --until)
            shift
            if [ "$#" -eq 0 ]; then
                echo "Error: --until requires a value"
                print_usage
                exit 1
            fi
            UNTIL_ARG="$1"
            ;;
        --lines)
            shift
            if [ "$#" -eq 0 ]; then
                echo "Error: --lines requires a value"
                print_usage
                exit 1
            fi
            JOURNAL_LINES="$1"
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            echo "Error: unknown argument '$1'"
            print_usage
            exit 1
            ;;
    esac
    shift
done

echo "=== NeuroKairos Time Sync Diagnostics ==="
echo "Date: $(date -u '+%Y-%m-%d %H:%M:%S UTC')"
echo ""

# --- chrony service status ---
echo "--- chrony service status ---"
if systemctl is-active --quiet chrony 2>/dev/null; then
    echo "chrony: RUNNING"
else
    echo "chrony: NOT RUNNING"
    systemctl status chrony --no-pager 2>/dev/null || echo "(chrony not installed)"
fi
echo ""

# --- chronyc tracking ---
echo "--- chronyc tracking ---"
if command -v chronyc >/dev/null 2>&1; then
    chronyc tracking 2>/dev/null || echo "(chronyc tracking failed)"
else
    echo "(chronyc not installed)"
fi
echo ""

# --- chronyc sources ---
echo "--- chronyc sources ---"
if command -v chronyc >/dev/null 2>&1; then
    chronyc sources -v 2>/dev/null || echo "(chronyc sources failed)"
else
    echo "(chronyc not installed)"
fi
echo ""

# --- chronyc sourcestats ---
echo "--- chronyc sourcestats ---"
if command -v chronyc >/dev/null 2>&1; then
    chronyc sourcestats 2>/dev/null || echo "(chronyc sourcestats failed)"
fi
echo ""

# --- chrony/gpsd/gnss-pps journal window ---
echo "--- journalctl diagnostics ---"
if command -v journalctl >/dev/null 2>&1; then
    if [ -n "${SINCE_ARG}" ] || [ -n "${UNTIL_ARG}" ]; then
        echo "Window: since='${SINCE_ARG:-<unset>}' until='${UNTIL_ARG:-<unset>}'"
    else
        echo "Window: last ${JOURNAL_LINES} lines"
    fi

    journal_cmd=(journalctl -u chrony -u gpsd -u gnss-pps --no-pager -n "${JOURNAL_LINES}")
    if [ -n "${SINCE_ARG}" ]; then
        journal_cmd+=(--since "${SINCE_ARG}")
    fi
    if [ -n "${UNTIL_ARG}" ]; then
        journal_cmd+=(--until "${UNTIL_ARG}")
    fi

    "${journal_cmd[@]}" 2>/dev/null || echo "(journalctl diagnostics failed)"
else
    echo "(journalctl not installed)"
fi
echo ""

# --- PPS device ---
echo "--- PPS device ---"
if [ -e /dev/pps0 ]; then
    echo "/dev/pps0: PRESENT"
    if command -v ppstest >/dev/null 2>&1; then
        echo "Running ppstest (3 second timeout)..."
        timeout 3 ppstest /dev/pps0 2>/dev/null || echo "(no PPS pulses detected in 3 seconds)"
    else
        echo "(ppstest not installed — install pps-tools for PPS verification)"
    fi
else
    echo "/dev/pps0: NOT FOUND (GPS PPS may not be configured)"
fi
echo ""

# --- gpsd status (server only) ---
echo "--- gpsd status ---"
if command -v gpsd >/dev/null 2>&1; then
    if systemctl is-active --quiet gpsd 2>/dev/null; then
        echo "gpsd: RUNNING"
    else
        echo "gpsd: NOT RUNNING"
    fi

    if command -v gpspipe >/dev/null 2>&1; then
        echo "GPS data (5 second sample):"
        timeout 5 gpspipe -w 2>/dev/null | head -5 || echo "(no GPS data received)"
    fi
else
    echo "gpsd: NOT INSTALLED (this is expected on client-only machines)"
fi
echo ""

echo "=== Diagnostics complete ==="
