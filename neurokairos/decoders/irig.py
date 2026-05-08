"""IRIG-H timecode decoding and ClockTable assembly.

All pulse-width constants are expressed as fractions of 1 second so that the
decoder is sampling-rate-agnostic.

Includes support for the NeuroKairos sync status extension: chrony NTP
synchronization quality encoded in IRIG-H frame bits 43-44 (stratum) and
46-48 (root dispersion bucket). The session-level worst-case values are
stored in the ClockTable's metadata dict as ``stratum`` (int, 1-4) and
``UTC_sync_precision`` (human-readable string, e.g. ``"< 0.25 ms"``).
"""

import datetime as _dt
import logging

import numpy as np

logger = logging.getLogger(__name__)

from ..clock_table import ClockTable
from .ttl import auto_threshold, detect_edges, measure_pulse_widths

# -- Pulse-width fractions (of 1 second) --------------------------------------

PULSE_FRAC_ZERO = 0.2
PULSE_FRAC_ONE = 0.5
PULSE_FRAC_MARKER = 0.8

# Classification boundaries (midpoints between adjacent nominal widths)
MIN_VALID_FRAC = 0.1
BOUNDARY_ZERO_ONE = (PULSE_FRAC_ZERO + PULSE_FRAC_ONE) / 2      # 0.35
BOUNDARY_ONE_MARKER = (PULSE_FRAC_ONE + PULSE_FRAC_MARKER) / 2  # 0.65
MAX_VALID_FRAC = 0.95

# Pulse type codes
PULSE_INVALID = -1
PULSE_ZERO = 0
PULSE_ONE = 1
PULSE_MARKER = 2

# -- BCD weight arrays (IRIG-H standard) --------------------------------------

SECONDS_WEIGHTS = np.array([1, 2, 4, 8, 10, 20, 40])
MINUTES_WEIGHTS = np.array([1, 2, 4, 8, 10, 20, 40])
HOURS_WEIGHTS = np.array([1, 2, 4, 8, 10, 20])
DAY_OF_YEAR_WEIGHTS = np.array([1, 2, 4, 8, 10, 20, 40, 80, 100, 200])
DECISECONDS_WEIGHTS = np.array([1, 2, 4, 8])
YEARS_WEIGHTS = np.array([1, 2, 4, 8, 10, 20, 40, 80])

# Marker positions within a 60-bit IRIG frame
MARKER_POSITIONS = {0, 9, 19, 29, 39, 49, 59}

# -- NeuroKairos sync status extension ----------------------------------------
# Bits 43-44: stratum (2 bits), bits 46-48: root dispersion bucket (3 bits).
# See docs/irig-h-standard.md in irig_unix_timecodes for full specification.

STRATUM_BITS = (43, 44)
ROOT_DISPERSION_BITS = (46, 47, 48)

# Upper-bound values (ms) for each root dispersion bucket (index = encoded value)
ROOT_DISPERSION_UPPER_MS = [0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, float("inf")]

# Human-readable labels for root dispersion buckets (index = encoded value)
ROOT_DISPERSION_LABELS = [
    "< 0.25 ms",   # 0
    "< 0.5 ms",    # 1
    "< 1 ms",      # 2
    "< 2 ms",      # 3
    "< 4 ms",      # 4
    "< 8 ms",      # 5
    "< 16 ms",     # 6
    ">= 16 ms",    # 7
]


# -- Core functions ------------------------------------------------------------

def bcd_encode(value, weights):
    """Encode *value* into BCD bits using *weights* (greedy, MSB-first).

    Returns a list of bools the same length as *weights*.
    """
    bits = [False] * len(weights)
    for i in reversed(range(len(weights))):
        if weights[i] <= value:
            bits[i] = True
            value -= weights[i]
    return bits


def estimate_samples_per_second(pulse_onsets):
    """Median inter-pulse-onset interval, with 2x outlier filter."""
    intervals = np.diff(pulse_onsets.astype(np.float64))
    median = np.median(intervals)
    valid = (intervals > median * 0.5) & (intervals < median * 2.0)
    return float(np.median(intervals[valid]))


def classify_pulses(pulse_widths, samples_per_second):
    """Classify pulse widths into type codes.

    Returns int8 array of pulse type codes.
    """
    fracs = pulse_widths.astype(np.float64) / samples_per_second

    types = np.full(len(pulse_widths), PULSE_INVALID, dtype=np.int8)

    m0 = (fracs >= MIN_VALID_FRAC) & (fracs < BOUNDARY_ZERO_ONE)
    types[m0] = PULSE_ZERO

    m1 = (fracs >= BOUNDARY_ZERO_ONE) & (fracs < BOUNDARY_ONE_MARKER)
    types[m1] = PULSE_ONE

    mm = (fracs >= BOUNDARY_ONE_MARKER) & (fracs <= MAX_VALID_FRAC)
    types[mm] = PULSE_MARKER

    return types


def find_frame_boundaries(pulse_types):
    """Return indices where a new frame starts (consecutive marker pulses).

    Two adjacent markers at positions *i* and *i+1* indicate that *i+1* is
    bit 0 (Pr) of a new frame.
    """
    is_m = pulse_types == PULSE_MARKER
    consecutive = is_m[:-1] & is_m[1:]
    return np.where(consecutive)[0] + 1


def bcd_decode(bits, weights):
    """Dot-product BCD decode: ``sum(bit * weight)``."""
    return int(np.dot(bits, weights))


def decode_frame(pulse_types_60):
    """Decode a 60-pulse IRIG frame into a UTC unix timestamp.

    Returns the timestamp (float) or *None* if the frame is invalid.
    """
    if len(pulse_types_60) != 60:
        return None

    # Validate marker positions
    for pos in MARKER_POSITIONS:
        if pulse_types_60[pos] != PULSE_MARKER:
            return None

    # Reject frames with any invalid pulse
    if np.any(pulse_types_60 == PULSE_INVALID):
        return None

    bits = pulse_types_60.astype(np.float64)

    seconds = (
        bcd_decode(bits[1:5], SECONDS_WEIGHTS[0:4])
        + bcd_decode(bits[6:9], SECONDS_WEIGHTS[4:7])
    )
    minutes = (
        bcd_decode(bits[10:14], MINUTES_WEIGHTS[0:4])
        + bcd_decode(bits[15:18], MINUTES_WEIGHTS[4:7])
    )
    hours = (
        bcd_decode(bits[20:24], HOURS_WEIGHTS[0:4])
        + bcd_decode(bits[25:27], HOURS_WEIGHTS[4:6])
    )
    day_of_year = (
        bcd_decode(bits[30:34], DAY_OF_YEAR_WEIGHTS[0:4])
        + bcd_decode(bits[35:39], DAY_OF_YEAR_WEIGHTS[4:8])
        + bcd_decode(bits[40:42], DAY_OF_YEAR_WEIGHTS[8:10])
    )
    year_2digit = (
        bcd_decode(bits[50:54], YEARS_WEIGHTS[0:4])
        + bcd_decode(bits[55:59], YEARS_WEIGHTS[4:8])
    )

    year = year_2digit + (_dt.datetime.now(_dt.timezone.utc).year // 100) * 100

    try:
        dt = _dt.datetime(year, 1, 1, hours, minutes, seconds,
                          tzinfo=_dt.timezone.utc)
        dt += _dt.timedelta(days=day_of_year - 1)
        return dt.timestamp()
    except (ValueError, OverflowError):
        return None


def decode_sync_status(pulse_types_60):
    """Decode NeuroKairos chrony sync status from a 60-pulse IRIG frame.

    Reads bits 43-44 (stratum, 2 bits) and bits 46-48 (root dispersion
    bucket, 3 bits). These are a NeuroKairos extension that encode chrony
    NTP synchronization quality in previously unused IRIG-H frame bits.

    Note on backward compatibility: old recordings without sync status have
    all-zero status bits, which maps to stratum=1 and root_dispersion_bucket=0
    (``< 0.25 ms``). This is ambiguous with a genuinely excellent GPS-locked
    source.

    Parameters
    ----------
    pulse_types_60 : ndarray (int8, length 60)
        Classified pulse types for one complete frame (0=zero, 1=one, 2=marker).

    Returns
    -------
    dict
        ``'stratum'``: int (1-4, where 4 means stratum 4+ or not synchronized)
        ``'root_dispersion_bucket'``: int (0-7, index into ROOT_DISPERSION_LABELS)
        Returns ``{'stratum': -1, 'root_dispersion_bucket': -1}`` if the frame
        is too short or status bits are markers/invalid.
    """
    if len(pulse_types_60) != 60:
        return {"stratum": -1, "root_dispersion_bucket": -1}

    # Status bits must be data bits (0 or 1), not markers or invalid
    status_positions = [43, 44, 46, 47, 48]
    for pos in status_positions:
        val = int(pulse_types_60[pos])
        if val not in (PULSE_ZERO, PULSE_ONE):
            return {"stratum": -1, "root_dispersion_bucket": -1}

    # Decode 2-bit stratum: bit 43 = LSB, bit 44 = MSB
    stratum_enc = int(pulse_types_60[43]) | (int(pulse_types_60[44]) << 1)
    stratum = stratum_enc + 1  # 0->1, 1->2, 2->3, 3->4

    # Decode 3-bit root dispersion: bit 46 = LSB, 47, 48 = MSB
    disp_enc = (int(pulse_types_60[46])
                | (int(pulse_types_60[47]) << 1)
                | (int(pulse_types_60[48]) << 2))

    return {"stratum": stratum, "root_dispersion_bucket": disp_enc}


def decode_partial_frame(pulse_types, boundary_idx):
    """Decode time from partial frames around a frame boundary.

    Extracts seconds/minutes/hours from the *new* frame (these change every
    frame, so they must come from the frame being decoded).  Day-of-year and
    year are taken from the new frame when available, falling back to the
    *previous* frame's tail (these fields change at most once per day/year).

    Requires at least ~27 pulses after the boundary (through the hours field)
    and ~10 pulses before it (for year, if not available in the new frame).

    Returns the timestamp (float) or *None* if not enough data.
    """
    n = len(pulse_types)
    b = boundary_idx

    def _bits(bit_positions, frame_offset):
        """Read pulse types at *bit_positions* relative to a frame start.

        *frame_offset* is 0 for the new frame, -60 for the previous frame.
        Returns a float array or None if any bit is out of range or invalid.
        """
        vals = []
        for pos in bit_positions:
            p = b + frame_offset + pos
            if not (0 <= p < n):
                return None
            if pulse_types[p] == PULSE_INVALID or pulse_types[p] == PULSE_MARKER:
                return None
            vals.append(float(pulse_types[p]))
        return np.array(vals)

    def _field_new(positions, weights):
        bits = _bits(positions, 0)
        return bcd_decode(bits, weights) if bits is not None else None

    def _field_either(positions, weights):
        bits = _bits(positions, 0)
        if bits is not None:
            return bcd_decode(bits, weights)
        bits = _bits(positions, -60)
        return bcd_decode(bits, weights) if bits is not None else None

    # Seconds, minutes, hours — must come from the new frame
    sec = _combine(
        _field_new([1, 2, 3, 4], SECONDS_WEIGHTS[0:4]),
        _field_new([6, 7, 8], SECONDS_WEIGHTS[4:7]),
    )
    mins = _combine(
        _field_new([10, 11, 12, 13], MINUTES_WEIGHTS[0:4]),
        _field_new([15, 16, 17], MINUTES_WEIGHTS[4:7]),
    )
    hrs = _combine(
        _field_new([20, 21, 22, 23], HOURS_WEIGHTS[0:4]),
        _field_new([25, 26], HOURS_WEIGHTS[4:6]),
    )
    if sec is None or mins is None or hrs is None:
        return None

    # Day-of-year and year — prefer new frame, fall back to previous
    doy = _combine(
        _field_either([30, 31, 32, 33], DAY_OF_YEAR_WEIGHTS[0:4]),
        _field_either([35, 36, 37, 38], DAY_OF_YEAR_WEIGHTS[4:8]),
        _field_either([40, 41], DAY_OF_YEAR_WEIGHTS[8:10]),
    )
    year_2d = _combine(
        _field_either([50, 51, 52, 53], YEARS_WEIGHTS[0:4]),
        _field_either([55, 56, 57, 58], YEARS_WEIGHTS[4:8]),
    )
    if doy is None or year_2d is None:
        return None

    year = year_2d + (_dt.datetime.now(_dt.timezone.utc).year // 100) * 100

    try:
        dt = _dt.datetime(year, 1, 1, hrs, mins, sec, tzinfo=_dt.timezone.utc)
        dt += _dt.timedelta(days=doy - 1)
        return dt.timestamp()
    except (ValueError, OverflowError):
        return None


def _combine(*parts):
    """Sum BCD sub-field results; return None if any part is None."""
    if any(p is None for p in parts):
        return None
    return sum(parts)


# -- Robust pulse handling -----------------------------------------------------

def compute_elapsed_seconds(pulse_onsets, sps):
    """Compute real seconds elapsed between consecutive pulses.

    Returns int64 array of length ``len(pulse_onsets) - 1``.
    Values: 0 = extra pulse (noise spike), 1 = normal, 2+ = missing pulse(s).
    """
    intervals = np.diff(pulse_onsets.astype(np.float64))
    return np.rint(intervals / sps).astype(np.int64)


def remove_extra_pulses(pulse_onsets, pulse_widths, pulse_types,
                        elapsed_seconds, sps):
    """Remove extra pulses where ``elapsed_seconds == 0`` (noise spikes).

    For each cluster of zero-elapsed pulses, groups them by rounded elapsed
    second from the cluster start.  Within each group, keeps the pulse with
    a valid IRIG classification (preferring the one closest to the expected
    onset position).  This correctly handles noise spikes near the midpoint
    between two real pulses (where both surrounding intervals round to 0).

    Returns filtered copies of all arrays plus updated elapsed_seconds.
    """
    keep = np.ones(len(pulse_onsets), dtype=bool)
    i = 0
    while i < len(elapsed_seconds):
        if elapsed_seconds[i] == 0:
            # Collect the cluster of consecutive zero-elapsed entries
            cluster_start = i
            while i < len(elapsed_seconds) and elapsed_seconds[i] == 0:
                i += 1
            cluster_indices = list(range(cluster_start, i + 1))

            # Group pulses by their rounded cumulative second from cluster start
            origin = pulse_onsets[cluster_indices[0]]
            groups = {}
            for ci in cluster_indices:
                cs = int(np.rint((pulse_onsets[ci] - origin) / sps))
                groups.setdefault(cs, []).append(ci)

            # From each group, keep the best pulse
            keepers = set()
            for cs, members in groups.items():
                expected_onset = origin + cs * sps
                best = max(members, key=lambda p: (
                    pulse_types[p] != PULSE_INVALID,
                    -abs(pulse_onsets[p] - expected_onset)))
                keepers.add(best)

            for ci in cluster_indices:
                if ci not in keepers:
                    keep[ci] = False
        else:
            i += 1

    n_removed = int(np.sum(~keep))
    if n_removed > 0:
        logger.info("Removed %d extra pulse(s)", n_removed)

    onsets_f = pulse_onsets[keep]
    widths_f = pulse_widths[keep]
    types_f = pulse_types[keep]
    elapsed_f = compute_elapsed_seconds(onsets_f, sps)

    return onsets_f, widths_f, types_f, elapsed_f


def assign_timestamps_from_anchors(pulse_onsets, elapsed_seconds, decoded,
                                   sps):
    """Assign UTC timestamps to every pulse using decoded-frame anchors.

    Uses cumulative elapsed seconds (from inter-onset rounding) rather than
    pulse index counting, so missing and extra pulses don't corrupt timestamps.

    Detects concatenation boundaries where the decoded time gap between anchors
    exceeds the interval-based elapsed time by more than 1 second.

    Parameters
    ----------
    pulse_onsets : ndarray
        Sample indices of pulse onsets.
    elapsed_seconds : ndarray
        Integer seconds between consecutive pulses (length n_pulses - 1).
    decoded : list of (int, float)
        Decoded anchors as ``(pulse_index, unix_timestamp)`` pairs.
    sps : float
        Samples per second.

    Returns
    -------
    reference : ndarray
        UTC timestamp for every pulse (float64).
    """
    n = len(pulse_onsets)
    reference = np.empty(n, dtype=np.float64)

    # Cumulative elapsed seconds relative to pulse 0
    cum_sec = np.empty(n, dtype=np.float64)
    cum_sec[0] = 0.0
    cum_sec[1:] = np.cumsum(elapsed_seconds).astype(np.float64)

    if len(decoded) == 1:
        # Single anchor — propagate using cumulative seconds
        aidx, atime = decoded[0]
        reference[:] = atime + (cum_sec - cum_sec[aidx])
        return reference

    # Multiple anchors — process segments between consecutive anchors
    # Start with all NaN, fill segment by segment
    reference[:] = np.nan

    for k in range(len(decoded) - 1):
        a_idx, a_time = decoded[k]
        b_idx, b_time = decoded[k + 1]

        interval_sec = cum_sec[b_idx] - cum_sec[a_idx]
        decoded_sec = b_time - a_time

        if abs(interval_sec - decoded_sec) <= 1:
            # Normal segment — propagate from anchor A
            pass  # handled below in the fill pass
        else:
            # Concatenation boundary — find where forward and backward
            # propagations diverge
            logger.info(
                "Concatenation boundary between anchors at pulse %d and %d: "
                "interval %.0f s vs decoded %.0f s (gap %.0f s)",
                a_idx, b_idx, interval_sec, decoded_sec,
                decoded_sec - interval_sec,
            )

    # Fill from each anchor outward, letting later anchors override earlier
    # ones for their local region. Process anchors in order; each anchor
    # controls from mid-gap-to-previous through mid-gap-to-next.
    for k, (aidx, atime) in enumerate(decoded):
        # Determine region this anchor controls
        if k == 0:
            lo = 0
        else:
            prev_idx = decoded[k - 1][0]
            prev_time = decoded[k - 1][1]
            interval_sec = cum_sec[aidx] - cum_sec[prev_idx]
            decoded_sec = atime - prev_time
            if abs(interval_sec - decoded_sec) <= 1:
                # Normal — split at midpoint
                lo = (prev_idx + aidx) // 2
            else:
                # Concatenation — find the boundary pulse
                lo = _find_concat_boundary(pulse_onsets, prev_idx, aidx)

        if k == len(decoded) - 1:
            hi = n
        else:
            next_idx = decoded[k + 1][0]
            next_time = decoded[k + 1][1]
            interval_sec = cum_sec[next_idx] - cum_sec[aidx]
            decoded_sec = next_time - atime
            if abs(interval_sec - decoded_sec) <= 1:
                hi = (aidx + next_idx) // 2
            else:
                hi = _find_concat_boundary(pulse_onsets, aidx, next_idx)

        reference[lo:hi] = atime + (cum_sec[lo:hi] - cum_sec[aidx])

    # Fill any remaining NaN at the edges (shouldn't happen, but be safe)
    if np.isnan(reference[0]):
        first_idx, first_time = decoded[0]
        reference[:first_idx] = first_time + (
            cum_sec[:first_idx] - cum_sec[first_idx])
    if np.isnan(reference[-1]):
        last_idx, last_time = decoded[-1]
        reference[last_idx:] = last_time + (
            cum_sec[last_idx:] - cum_sec[last_idx])

    return reference


def _find_concat_boundary(pulse_onsets, a_idx, b_idx):
    """Find the pulse index where a concatenation boundary lies.

    Looks for the largest inter-onset gap between the two anchors (a physical
    break where files were stitched together).  Falls back to the midpoint
    if all gaps are similar.
    """
    if b_idx - a_idx <= 1:
        return b_idx

    intervals = np.diff(pulse_onsets[a_idx:b_idx + 1].astype(np.float64))
    median_interval = np.median(intervals)

    max_rel_idx = int(np.argmax(intervals))
    if intervals[max_rel_idx] > median_interval * 1.5:
        return a_idx + max_rel_idx + 1

    # No clear physical break — split at the midpoint
    return (a_idx + b_idx + 1) // 2


def _build_sync_arrays(n_pulses, sync_anchors):
    """Build per-pulse sync status arrays from frame-level anchor points.

    Sets values at each anchor's pulse index, forward-fills through the
    array, then backward-fills leading NaNs from the first valid reading.

    Parameters
    ----------
    n_pulses : int
        Total number of pulses (length of output arrays).
    sync_anchors : list of (int, dict)
        ``(pulse_index, sync_dict)`` pairs from decoded frames.
        Each dict has ``'stratum'`` (1-4) and ``'root_dispersion_bucket'``
        (0-7).

    Returns
    -------
    (sync_stratum, sync_dispersion_upperbound_ms) : tuple of ndarray or
        (None, None) if no anchors.
    """
    if not sync_anchors:
        return None, None

    stratum = np.full(n_pulses, np.nan, dtype=np.float64)
    disp = np.full(n_pulses, np.nan, dtype=np.float64)

    # Set values at each frame's start pulse
    for idx, ss in sync_anchors:
        stratum[idx] = float(ss["stratum"])
        disp[idx] = ROOT_DISPERSION_UPPER_MS[ss["root_dispersion_bucket"]]

    # Forward-fill: propagate last valid value to subsequent NaN positions
    last_s = np.nan
    last_d = np.nan
    for i in range(n_pulses):
        if not np.isnan(stratum[i]):
            last_s = stratum[i]
            last_d = disp[i]
        else:
            stratum[i] = last_s
            disp[i] = last_d

    # Backward-fill leading NaNs from first valid reading
    first_valid = np.argmax(~np.isnan(stratum))
    if first_valid > 0:
        stratum[:first_valid] = stratum[first_valid]
        disp[:first_valid] = disp[first_valid]

    return stratum, disp


def build_clock_table(pulse_onsets, pulse_widths):
    """Build a :class:`ClockTable` from detected IRIG pulses.

    Every surviving pulse gets an entry — extra pulses (noise spikes) are
    removed, and missing pulses produce gaps in the timestamp sequence that
    are correctly bridged via inter-onset timing.  Concatenated files are
    detected by comparing decoded-frame anchors against elapsed-time intervals.
    """
    n_pulses = len(pulse_onsets)
    if n_pulses < 2:
        raise ValueError("Need at least 2 pulses to build a clock table")

    n_raw_pulses = n_pulses

    sps = estimate_samples_per_second(pulse_onsets)
    pulse_types = classify_pulses(pulse_widths, sps)

    # Compute elapsed seconds and remove extra (noise) pulses
    elapsed = compute_elapsed_seconds(pulse_onsets, sps)
    pulse_onsets, pulse_widths, pulse_types, elapsed = (
        remove_extra_pulses(
            pulse_onsets, pulse_widths, pulse_types, elapsed, sps))

    n_extra_removed = n_raw_pulses - len(pulse_onsets)
    n_missing_gaps = int(np.sum(elapsed > 1))

    n_outliers = int(np.sum((elapsed != 1) & (elapsed != 0)))
    if len(elapsed) > 0 and n_outliers / len(elapsed) > 0.1:
        logger.warning(
            "%.0f%% of pulse intervals are outliers (%d/%d) — heavy pulse loss",
            100 * n_outliers / len(elapsed), n_outliers, len(elapsed))

    # Find frame boundaries and decode all possible frames
    frame_starts = find_frame_boundaries(pulse_types)
    n_clean = len(pulse_onsets)

    decoded = []  # (pulse_index, unix_timestamp)
    sync_statuses = []  # per-frame sync status dicts (complete frames only)
    sync_anchors = []   # (pulse_index, sync_dict) for per-pulse arrays
    for start in frame_starts:
        end = start + 60
        if end <= n_clean:
            ts = decode_frame(pulse_types[start:end])
            if ts is not None:
                decoded.append((int(start), ts))
                ss = decode_sync_status(pulse_types[start:end])
                if ss["stratum"] >= 0:
                    sync_statuses.append(ss)
                    sync_anchors.append((int(start), ss))
                continue
        # Fall back to partial decoding for each boundary
        ts = decode_partial_frame(pulse_types, start)
        if ts is not None:
            decoded.append((int(start), ts))

    if not decoded:
        raise ValueError("No valid IRIG frames could be decoded")

    # Validate anchors: discard any with non-monotonic timestamps.
    # Partial-frame decoding near missing pulses can produce shifted BCD reads
    # that yield wildly wrong timestamps; this filter removes them.
    decoded.sort()
    filtered = [decoded[0]]
    for k in range(1, len(decoded)):
        if decoded[k][1] > filtered[-1][1]:
            filtered.append(decoded[k])
        else:
            logger.warning(
                "Discarding inconsistent anchor at pulse %d "
                "(time %.0f <= previous %.0f)",
                decoded[k][0], decoded[k][1], filtered[-1][1])
    decoded = filtered

    # Assign timestamps using all anchors and elapsed-second intervals
    reference = assign_timestamps_from_anchors(
        pulse_onsets, elapsed, decoded, sps)

    # Count concatenation boundaries: jumps in reference that exceed the
    # expected 1-second spacing by more than 2 seconds
    ref_diffs = np.diff(reference)
    n_concat_boundaries = int(np.sum(ref_diffs > 3.0))

    # Human-readable recording start/stop datetimes
    try:
        rec_start = _dt.datetime.fromtimestamp(
            reference[0], tz=_dt.timezone.utc
        ).strftime("%Y-%m-%dT%H:%M:%SZ")
    except (OSError, OverflowError, ValueError):
        rec_start = None
    try:
        rec_stop = _dt.datetime.fromtimestamp(
            reference[-1], tz=_dt.timezone.utc
        ).strftime("%Y-%m-%dT%H:%M:%SZ")
    except (OSError, OverflowError, ValueError):
        rec_stop = None

    # Compute session-level worst-case sync status from all decoded frames.
    # "Worst" = highest stratum and highest root dispersion bucket.
    worst_stratum = None
    worst_disp_bucket = None
    if sync_statuses:
        worst_stratum = max(ss["stratum"] for ss in sync_statuses)
        worst_disp_bucket = max(
            ss["root_dispersion_bucket"] for ss in sync_statuses)

    metadata = {
        "recording_start": rec_start,
        "recording_stop": rec_stop,
        "n_raw_pulses": n_raw_pulses,
        "n_extra_removed": n_extra_removed,
        "n_missing_gaps": n_missing_gaps,
        "n_frames_decoded": len(decoded),
        "n_concat_boundaries": n_concat_boundaries,
        "stratum": worst_stratum,
        "UTC_sync_precision": (
            ROOT_DISPERSION_LABELS[worst_disp_bucket]
            if worst_disp_bucket is not None
            else None
        ),
    }

    # Build per-pulse sync arrays from frame anchors
    sync_stratum, sync_disp = _build_sync_arrays(
        n_clean, sync_anchors
    )

    # Compute measured rate from the full source/reference mapping.
    # This is more accurate than the median inter-onset interval (sps)
    # because it recovers fractional rates that integer-interval medians
    # cannot represent.
    measured_rate = (
        (pulse_onsets[-1] - pulse_onsets[0])
        / (reference[-1] - reference[0])
    )

    return ClockTable(
        source=pulse_onsets.astype(np.float64),
        reference=reference,
        nominal_rate=measured_rate,
        source_units="samples",
        metadata=metadata,
        sync_stratum=sync_stratum,
        sync_dispersion_upperbound_ms=sync_disp,
    )


def decode_dat_irig(dat_path, n_channels, irig_channel, save=True):
    """Decode IRIG timecodes from an interleaved int16 ``.dat`` file.

    Parameters
    ----------
    dat_path : str or path-like
        Path to the binary file (interleaved int16).
    n_channels : int
        Number of interleaved channels.
    irig_channel : int
        Zero-based index of the IRIG channel.
    save : bool
        If True (default), save the ClockTable to
        ``<dat_path>.clocktable.npz`` alongside the data file.

    Returns
    -------
    ClockTable
        Sparse sample-index-to-UTC-time mapping with one entry per IRIG
        pulse (~1 Hz).
    """
    from pathlib import Path

    dat_path = Path(dat_path)
    raw = np.memmap(dat_path, dtype=np.int16, mode="r")
    data = raw.reshape(-1, n_channels)
    irig_signal = data[:, irig_channel]

    threshold = auto_threshold(irig_signal)
    rising, falling = detect_edges(irig_signal, threshold)
    onsets, widths = measure_pulse_widths(rising, falling)

    ct = build_clock_table(onsets, widths)

    ct.metadata["source_file"] = dat_path.name
    ct.metadata["source_path"] = str(dat_path.resolve())
    ct.metadata["n_channels"] = n_channels
    ct.metadata["irig_channel"] = irig_channel

    if save:
        ct_path = dat_path.parent / (dat_path.name + ".clocktable.npz")
        ct.save(ct_path)
        logger.info("Saved clock table to %s", ct_path)

        # Generate visual sync report alongside the NPZ
        png_path = dat_path.parent / (dat_path.name + ".sync_report.png")
        from .report import _try_generate_report
        _try_generate_report(
            ct, png_path,
            raw_signal=irig_signal,
            threshold=threshold,
            pulse_widths=widths,
        )

    return ct


def decode_intervals_irig(intervals, offsets=None, source_units="seconds",
                          save=None):
    """Decode IRIG timecodes from pre-extracted pulse intervals.

    Accepts pulse onset/offset times directly (e.g. from Open Ephys, TTL
    loggers, camera event files) without requiring access to raw binary data.

    Parameters
    ----------
    intervals : array-like or IntervalSet-like
        Either a 1-D array of pulse onset times, or an object with ``.start``
        and ``.end`` attributes (duck-typed IntervalSet).
    offsets : array-like, optional
        1-D array of pulse offset times.  Required when *intervals* is an
        array; ignored when it has ``.start`` / ``.end`` attributes.
    source_units : str
        Label for the ClockTable source axis (default ``"seconds"``).
    save : str or path-like, optional
        If provided, save the ClockTable to this path.

    Returns
    -------
    ClockTable
        Sparse source-time-to-UTC mapping with one entry per IRIG pulse.
    """
    from pathlib import Path

    if hasattr(intervals, "start") and hasattr(intervals, "end"):
        onsets = np.asarray(intervals.start, dtype=np.float64)
        offs = np.asarray(intervals.end, dtype=np.float64)
    else:
        onsets = np.asarray(intervals, dtype=np.float64)
        if offsets is None:
            raise ValueError(
                "offsets is required when intervals is an array "
                "(not an IntervalSet-like object)")
        offs = np.asarray(offsets, dtype=np.float64)
        if len(onsets) != len(offs):
            raise ValueError(
                f"intervals and offsets must have the same length "
                f"(got {len(onsets)} and {len(offs)})")

    widths = offs - onsets
    ct = build_clock_table(onsets, widths)

    # Override source_units, carrying over metadata and sync arrays
    metadata = dict(ct.metadata) if ct.metadata else {}
    ct = ClockTable(
        source=ct.source,
        reference=ct.reference,
        nominal_rate=ct.nominal_rate,
        source_units=source_units,
        metadata=metadata,
        sync_stratum=ct.sync_stratum,
        sync_dispersion_upperbound_ms=ct.sync_dispersion_upperbound_ms,
    )

    if save is not None:
        save_path = Path(save)
        ct.save(save_path)
        logger.info("Saved clock table to %s", save_path)

        # Generate visual sync report (no raw signal for intervals path)
        png_path = save_path.parent / (
            save_path.name.replace(".clocktable.npz", "") + ".sync_report.png"
        )
        from .report import _try_generate_report
        _try_generate_report(ct, png_path)

    return ct
