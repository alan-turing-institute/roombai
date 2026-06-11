"""
Obstacle-aware move planner using OpenCV texture and darkness analysis.

Analyses a JPEG snapshot to estimate forward clearance in three zones
(left, straight, right) and returns the best MoveAction.

Algorithm
---------
1. Decode the JPEG; convert to grayscale once for both analyses below.

Texture layer (smooth vs rough):
2. Compute a per-pixel local standard-deviation texture map.
   High score → rough surface (carpet) → navigable floor.
   Low score  → smooth surface (plywood / wall) → obstacle.
3. Ignore the top HORIZON_FRAC of the image (ceiling / far wall).
4. Split the remaining ROI into left / centre / right thirds.
5. For each zone scan row-by-row from the top downward; the first row
   whose mean texture exceeds FLOOR_TEXTURE_THRESHOLD is the boundary
   between obstacle face and navigable carpet.
6. Map that boundary's y-position to metres:
       distance ≈ DISTANCE_K / (y_norm - HORIZON_FRAC)

Darkness layer (black metal chair/table legs):
7. For each zone scan row-by-ray from the top down to DARK_SCAN_FRAC (to
   avoid triggering on the car's own chassis at the bottom of the image).
   The first row where more than DARK_ROW_THRESHOLD of pixels are below
   DARKNESS_LEVEL intensity is a dense dark-obstacle row (chair legs, metal
   frames).  Convert its y-position to metres with the same formula.

Combined clearance:
8. Zone clearance = min(texture clearance, dark-obstacle clearance).
9. Apply a 0.7 safety factor, cap at max_distance.
10. If every zone's probed clearance is below REVERSE_THRESHOLD, return
    direction="reverse" with distance REVERSE_DISTANCE_M.

Door detection:
11. detect_door() analyses the upper DOOR_UPPER_ROI of the image for a
    prominently bright vertical band — the signature of a lit room visible
    through a glass partition or open doorway.  A prominence (peak vs.
    flanking mean) above DOOR_PROMINENCE identifies the door zone.
12. Pass target_zone to plan_move() to bias the chosen direction toward the
    detected door when that zone has at least TARGET_ZONE_MIN_CLEARANCE of
    space (ensures the car does not drive into an obstacle to reach the door).

Calibration
-----------
FLOOR_TEXTURE_THRESHOLD: derived from frame_0003.jpg (plywood bench vs
carpet).  plywood std-dev ≈ 0.69, carpet ≈ 14.67; threshold = 5.0.

DARKNESS_LEVEL / DARK_ROW_THRESHOLD: calibrated from frame_0006.jpg
(chair legs, black metal bars).  Chair-leg rows reach dark_frac ≈ 0.90+;
threshold = 0.40 catches dense chair rows while ignoring sparse distant
legs (dark_frac < 0.35) and the car chassis (excluded by DARK_SCAN_FRAC).

DOOR_PROMINENCE: calibrated from frame_0008_door.jpg (glass partition with
lit room beyond).  Door prominence ≈ 45; nearest non-door frame ≈ 30.
Threshold 40 gives clear separation.

HORIZON_FRAC and DISTANCE_K should be tuned empirically on your floor.
"""

from __future__ import annotations

import base64
from dataclasses import dataclass
from typing import Literal, Optional

import cv2
import numpy as np

# ── Calibration constants ─────────────────────────────────────────────────────

HORIZON_FRAC: float = 0.35    # rows above this y-fraction are beyond max_distance
DISTANCE_K:   float = 0.35    # perspective scale constant (metres)

TEXTURE_WINDOW:          int   = 15   # local std-dev window size (pixels)
FLOOR_TEXTURE_THRESHOLD: float = 5.0  # min texture score to classify as carpet
SMOOTH_WINDOW:           int   = 5    # row-score smoothing window

# Dark-obstacle detection (chair legs, metal bars)
DARKNESS_LEVEL:     int   = 60    # pixel intensity below which a pixel is "dark"
DARK_ROW_THRESHOLD: float = 0.40  # min dark-pixel fraction in a row to flag it
DARK_SCAN_FRAC:     float = 0.85  # only scan for dark obstacles above this y-norm
                                   # (excludes the car's own chassis at the bottom)

SAFETY_FACTOR: float = 0.7

# Reverse: triggered when every zone's probed clearance is below this threshold.
REVERSE_THRESHOLD:  float = 1.0  # metres — all zones below this → reverse
REVERSE_DISTANCE_M: float = 0.5  # metres to travel when reversing

_PROBE_DISTANCE: float = 100.0   # sentinel max_distance used for reverse check

# Zone-consistency check: when two adjacent zones are both close-blocked, a
# lateral zone whose clearance is ZONE_JUMP_RATIO× larger than the adjacent
# minimum is treated as an optical illusion (carpet visible through a gap/
# doorway beyond the obstacle, not a physically navigable path).
ZONE_CONSISTENCY_THRESHOLD: float = 0.75   # metres — both neighbours must be below this
ZONE_JUMP_RATIO:            float = 1.5    # lateral zone is suspect if it exceeds this multiple

# Door detection: look for a prominently bright vertical band in the upper image.
# A lit room visible through a glass partition creates a column-brightness peak
# that stands well above the flanking regions.
DOOR_UPPER_ROI:   float = 0.55   # fraction of image height to scan for door
DOOR_PROMINENCE:  float = 40.0   # min brightness prominence vs. flanking means
DOOR_MIN_PEAK:    int   = 130    # peak must be at least this bright (0–255)
DOOR_EDGE_MARGIN: float = 0.15   # peak must be this far from each side edge

# When plan_move is called with a target_zone, the car will favour that zone
# if its clearance is at least this value (ensures we do not head straight into
# a barely-visible opening).
TARGET_ZONE_MIN_CLEARANCE: float = 0.5

# Steered-reverse thresholds: when reversing, use a steering lock if one lateral
# zone is significantly more open than the other (suggesting a one-sided obstacle).
# Uses the original (pre-consistency-check) probe values so optical artefacts do
# not bias the direction.
#
# REVERSE_STEER_MIN_CLEARANCE: the clearer lateral zone must reach this value
#   before we steer.  Set just above the frame_0001 corner artefact (≈0.69 m) so
#   a fully-walled corner always reverses straight.
# REVERSE_STEER_MARGIN: the two lateral zones must differ by at least this much.
REVERSE_STEER_MIN_CLEARANCE: float = 0.7
REVERSE_STEER_MARGIN:        float = 0.15


# ── Public interface ──────────────────────────────────────────────────────────

@dataclass
class MoveAction:
    direction: Literal["left", "straight", "right", "reverse", "reverse+left", "reverse+right"]
    distance_metres: float
    left_clearance_m: float
    center_clearance_m: float
    right_clearance_m: float
    reasoning: str


def plan_move(
    image_b64: str,
    max_distance: float = 5.0,
    target_zone: Optional[Literal["left", "straight", "right"]] = None,
) -> MoveAction:
    """Return the best move action given a base64-encoded JPEG snapshot.

    target_zone: when set (e.g. from detect_door()), the car will steer toward
    that zone provided its clearance is at least TARGET_ZONE_MIN_CLEARANCE and
    no reverse is required.  Without it, the zone with the most clearance wins.
    """
    img  = _decode(image_b64)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    tex  = _texture_map(gray)
    H, W = tex.shape

    roi_top  = int(H * HORIZON_FRAC)
    tex_roi  = tex[roi_top:, :]
    gray_roi = gray[roi_top:, :]

    w3 = W // 3
    col_slices = {
        "left":     slice(0,   w3),
        "straight": slice(w3,  2 * w3),
        "right":    slice(2 * w3, None),
    }

    # Probe both layers at a large distance for the reverse check.
    probe = {
        name: min(
            _zone_texture_clearance(tex_roi[:, s],  roi_top, H, _PROBE_DISTANCE),
            _zone_dark_clearance(   gray_roi[:, s], roi_top, H, _PROBE_DISTANCE),
        )
        for name, s in col_slices.items()
    }

    # Zone-consistency check: a lateral zone that is many times larger than
    # both of its close-blocked neighbours is seeing carpet through a gap/
    # doorway that the car cannot physically reach without first reversing.
    # Use original probe values so the two checks do not cascade.
    # The lower bound (> 0.1) excludes the floor sentinel (no detection) so
    # that a fully-blocked adjacent zone does not inflate the ratio.
    _ps, _pl, _pr = probe["straight"], probe["left"], probe["right"]
    if (0.1 < _ps < ZONE_CONSISTENCY_THRESHOLD
            and 0.1 < _pr < ZONE_CONSISTENCY_THRESHOLD
            and _pl > min(_ps, _pr) * ZONE_JUMP_RATIO):
        probe["left"] = 0.1
    elif (0.1 < _ps < ZONE_CONSISTENCY_THRESHOLD
            and 0.1 < _pl < ZONE_CONSISTENCY_THRESHOLD
            and _pr > min(_ps, _pl) * ZONE_JUMP_RATIO):
        probe["right"] = 0.1

    # Reported clearances are capped at the caller-supplied max_distance.
    clearances = {k: min(v, max_distance) for k, v in probe.items()}

    reasoning_prefix = (
        f"L={clearances['left']:.2f}m "
        f"S={clearances['straight']:.2f}m "
        f"R={clearances['right']:.2f}m → "
    )

    if max(probe.values()) < REVERSE_THRESHOLD:
        # Determine steering direction from original (pre-consistency-check) lateral
        # probe values.  If one side is clearly more open, the obstacle is on the
        # opposite side; steer the reverse arc so the car's front swings toward the
        # clearer side.  Only steer if the clearer zone exceeds REVERSE_STEER_MIN_CLEARANCE
        # (prevents corner artefacts from triggering a steered reverse).
        max_lateral = max(_pl, _pr)
        if max_lateral >= REVERSE_STEER_MIN_CLEARANCE:
            if _pl > _pr + REVERSE_STEER_MARGIN:
                rev_dir = "reverse+right"   # left clearer → obstacle on right
            elif _pr > _pl + REVERSE_STEER_MARGIN:
                rev_dir = "reverse+left"    # right clearer → obstacle on left
            else:
                rev_dir = "reverse"
        else:
            rev_dir = "reverse"
        return MoveAction(
            direction=rev_dir,
            distance_metres=min(REVERSE_DISTANCE_M, max_distance),
            left_clearance_m=clearances["left"],
            center_clearance_m=clearances["straight"],
            right_clearance_m=clearances["right"],
            reasoning=reasoning_prefix + rev_dir,
        )

    if (target_zone is not None
            and clearances.get(target_zone, 0) >= TARGET_ZONE_MIN_CLEARANCE):
        best = target_zone
    else:
        best = max(clearances, key=clearances.get)

    distance = round(
        min(max(clearances[best] * SAFETY_FACTOR, 0.1), max_distance), 2
    )

    return MoveAction(
        direction=best,
        distance_metres=distance,
        left_clearance_m=clearances["left"],
        center_clearance_m=clearances["straight"],
        right_clearance_m=clearances["right"],
        reasoning=reasoning_prefix + best,
    )


def detect_door(image_b64: str) -> Optional[str]:
    """Detect a lit doorway and return which zone it occupies.

    A glass partition with a lit room behind it produces a per-column
    brightness peak whose prominence (peak value minus the mean of the
    flanking bands) exceeds that of any other feature in the image.

    Returns 'left', 'straight', or 'right', or None if no door is found.
    """
    img  = _decode(image_b64)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    H, W = gray.shape

    upper_h = int(H * DOOR_UPPER_ROI)
    col_profile = gray[:upper_h, :].mean(axis=0).astype(np.float32)

    # Smooth at ~1 % of image width to suppress single-column noise.
    k          = max(1, W // 100)
    col_smooth = np.convolve(col_profile, np.ones(k) / k, mode="same")

    peak_col = int(np.argmax(col_smooth))
    peak_val = float(col_smooth[peak_col])

    # Peaks too close to the edge have unreliable flank estimates.
    edge = int(DOOR_EDGE_MARGIN * W)
    if peak_col < edge or peak_col > W - edge:
        return None

    if peak_val < DOOR_MIN_PEAK:
        return None

    # Prominence: how much the peak rises above the mean of the flanking bands
    # (5 %–25 % of image width to each side of the peak).
    inner = int(0.05 * W)
    outer = int(0.25 * W)
    l_start = max(0,     peak_col - outer)
    l_end   = max(0,     peak_col - inner)
    r_start = min(W - 1, peak_col + inner)
    r_end   = min(W - 1, peak_col + outer)
    l_mean  = float(col_smooth[l_start:l_end].mean()) if l_end > l_start else peak_val
    r_mean  = float(col_smooth[r_start:r_end].mean()) if r_end > r_start else peak_val
    prominence = peak_val - max(l_mean, r_mean)

    if prominence < DOOR_PROMINENCE:
        return None

    w3 = W // 3
    if peak_col < w3:
        return "left"
    elif peak_col < 2 * w3:
        return "straight"
    return "right"


# ── Internal helpers ──────────────────────────────────────────────────────────

def _decode(image_b64: str) -> np.ndarray:
    raw = base64.b64decode(image_b64)
    buf = np.frombuffer(raw, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("cv2.imdecode failed — check that the bytes are a valid JPEG")
    return img


def _texture_map(gray: np.ndarray) -> np.ndarray:
    """Per-pixel local standard deviation of grayscale intensity."""
    f = gray.astype(np.float32)
    k = (TEXTURE_WINDOW, TEXTURE_WINDOW)
    mean    = cv2.boxFilter(f,    -1, k)
    mean_sq = cv2.boxFilter(f**2, -1, k)
    return np.sqrt(np.maximum(mean_sq - mean**2, 0))


def _zone_texture_clearance(
    zone: np.ndarray,
    roi_top: int,
    H: int,
    max_distance: float,
) -> float:
    """Distance to first carpet row scanning from horizon downward.

    Returns max_distance when floor is visible at the horizon.
    Returns 0.1 m when no carpet row is found (zone fully blocked).
    """
    if zone.shape[0] == 0:
        return 0.1

    row_scores = zone.mean(axis=1)
    kernel     = np.ones(SMOOTH_WINDOW) / SMOOTH_WINDOW
    smoothed   = np.convolve(row_scores, kernel, mode="same")

    for row in range(zone.shape[0]):
        if smoothed[row] > FLOOR_TEXTURE_THRESHOLD:
            y_norm = (roi_top + row) / H
            if y_norm <= HORIZON_FRAC + 1e-9:
                return max_distance
            return min(DISTANCE_K / (y_norm - HORIZON_FRAC), max_distance)

    return 0.1


def _zone_dark_clearance(
    gray_zone: np.ndarray,
    roi_top: int,
    H: int,
    max_distance: float,
) -> float:
    """Distance to first dense dark row (chair/table legs, metal bars).

    Scans from the horizon down to DARK_SCAN_FRAC to avoid triggering on
    the car's own chassis at the very bottom of the image.
    Returns max_distance when no dense dark row is found.
    """
    dark_limit = max(0, int(DARK_SCAN_FRAC * H) - roi_top)
    zone = gray_zone[:dark_limit]
    if zone.shape[0] == 0:
        return max_distance

    dark_frac = (zone < DARKNESS_LEVEL).mean(axis=1).astype(np.float32)
    kernel    = np.ones(SMOOTH_WINDOW) / SMOOTH_WINDOW
    smoothed  = np.convolve(dark_frac, kernel, mode="same")

    for row in range(zone.shape[0]):
        if smoothed[row] > DARK_ROW_THRESHOLD:
            y_norm = (roi_top + row) / H
            if y_norm <= HORIZON_FRAC + 1e-9:
                return max_distance
            return min(DISTANCE_K / (y_norm - HORIZON_FRAC), max_distance)

    return max_distance
