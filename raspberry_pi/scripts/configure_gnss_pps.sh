#!/bin/bash

# Program the u-blox M8T receiver so PPS stays active in both lock and holdover.
#
# This script is designed to run at boot after gpsd is up. It uses ubxtool
# through gpsd so the Pi does not need to talk to the serial port directly.
#
# The receiver is configured to:
#   - use the stationary dynamic model for a fixed installation
#   - output a 1 Hz PPS on TIMEPULSE 0
#   - output a 1 Hz, 100 ms PPS in both locked and unlocked TP5 states
#   - prefer binary output over NMEA so gpsd sees less serial chatter
#   - save the configuration so the settings survive power cycles when possible

set -euo pipefail

GNSS_DEVICE="${GNSS_DEVICE:-/dev/ttyAMA0}"
GNSS_UBX_TARGET="${GNSS_UBX_TARGET:-localhost:2947:${GNSS_DEVICE}}"
UBX_PROTOCOL_VERSION="${UBX_PROTOCOL_VERSION:-22}"
UBX_DYNAMIC_MODEL="${UBX_DYNAMIC_MODEL:-2}"
# gpsd's `ubxtool -e PPS` leaves the unlocked TP5 pulse length at zero.
# Use an explicit CFG-TP5 payload so the receiver keeps pulsing in holdover.
UBX_TP5_PPS_COMMAND="${UBX_TP5_PPS_COMMAND:-06,31,00,01,00,00,02,00,00,00,40,42,0f,00,40,42,0f,00,a0,86,01,00,a0,86,01,00,00,00,00,00,77,00,00,00}"
# This fixed-position timing profile was derived from the last known good
# locked GNSS solution on this test install. If the antenna moves, update
# these coordinates before rerunning the script.
UBX_TMODE2_FIXED_COMMAND="${UBX_TMODE2_FIXED_COMMAND:-06,3d,02,00,00,00,ef,cf,02,08,c8,4a,57,e4,29,d5,bc,18,40,42,0f,00,00,00,00,00,00,00,00,00}"

export UBXOPTS="-P ${UBX_PROTOCOL_VERSION} -v 0"

log() {
    logger -t configure_gnss_pps "$*"
    echo "$*"
}

wait_for_receiver() {
    local attempt
    for attempt in $(seq 1 30); do
        if ubxtool ${GNSS_UBX_TARGET} -p MON-VER -w 2 >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
    done
    return 1
}

log "waiting for receiver on ${GNSS_DEVICE}"
if ! wait_for_receiver; then
    log "receiver did not respond"
    exit 1
fi

log "configuring M8T PPS for lock and holdover"
ubxtool ${GNSS_UBX_TARGET} -p "MODEL,${UBX_DYNAMIC_MODEL}" -w 4 >/dev/null
ubxtool ${GNSS_UBX_TARGET} -c "${UBX_TP5_PPS_COMMAND}" -w 4 >/dev/null
ubxtool ${GNSS_UBX_TARGET} -c "${UBX_TMODE2_FIXED_COMMAND}" -w 4 >/dev/null
ubxtool ${GNSS_UBX_TARGET} -e BINARY -w 4 >/dev/null
ubxtool ${GNSS_UBX_TARGET} -d NMEA -w 4 >/dev/null

log "saving receiver configuration"
ubxtool ${GNSS_UBX_TARGET} -p SAVE -w 4 >/dev/null || true

log "GNSS PPS configuration complete"
