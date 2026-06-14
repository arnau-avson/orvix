"""Classify the active state of a traffic light from a tight crop.

Approach: convert to HSV and count saturated, bright pixels in the canonical
red / yellow / green hue ranges. The dominant color wins, but we require:
  - a minimum number of matching pixels (so a dark/off light returns 'unknown')
  - a minimum brightness contrast across the crop — a real traffic light has
    a bright lit bulb against a dark housing (high V std dev), while a flat
    painted sign (e.g. a 'no entry' disc) has uniform brightness. This kills
    the most common false positive — solid red traffic signs.
"""
from typing import Tuple

import cv2
import numpy as np


# OpenCV HSV ranges: H in [0,179], S/V in [0,255].
# Red wraps around the hue circle, so we need two ranges.
_RED_RANGES: Tuple[Tuple[Tuple[int, int, int], Tuple[int, int, int]], ...] = (
    ((0, 120, 120), (10, 255, 255)),
    ((170, 120, 120), (179, 255, 255)),
)
_YELLOW_RANGE = ((18, 120, 150), (35, 255, 255))
_GREEN_RANGE = ((40, 80, 100), (90, 255, 255))

# A lit bulb should occupy at least this fraction of the cropped area.
_MIN_FRACTION = 0.02
# Minimum std dev of the V (brightness) channel. Flat painted signs have
# uniform brightness (low std dev); lit traffic lights have a bright bulb
# against a dark housing (high std dev). Empirically ~25 separates them.
_MIN_V_STDDEV = 25.0
# Maximum fraction of bright-white pixels — only applied when the dominant
# colored signal is RED. This targets the most common false positive for a
# red traffic light: the 'no entry' sign (red disc with a thick white
# horizontal bar). Empirically: no-entry ~9% white, real red semáforo ~3%.
# We can't apply this filter unconditionally, because yellow-housed
# semáforos with bright (whitewashed) buildings behind them can hit ~8%
# white legitimately while showing a green or yellow bulb.
_MAX_WHITE_FRACTION_FOR_RED = 0.06
_WHITE_RANGE = ((0, 0, 200), (179, 50, 255))


def _mask_count(hsv: np.ndarray, lo: Tuple[int, int, int], hi: Tuple[int, int, int]) -> int:
    mask = cv2.inRange(hsv, np.array(lo, dtype=np.uint8), np.array(hi, dtype=np.uint8))
    return int(cv2.countNonZero(mask))


def classify_state(crop_bgr: np.ndarray) -> str:
    """Return 'red', 'yellow', 'green', or 'unknown'."""
    if crop_bgr.size == 0 or crop_bgr.shape[0] < 5 or crop_bgr.shape[1] < 5:
        return "unknown"

    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    v_channel = hsv[:, :, 2]

    # Reject low-contrast crops (flat signs, uniform regions).
    if float(np.std(v_channel)) < _MIN_V_STDDEV:
        return "unknown"

    total = crop_bgr.shape[0] * crop_bgr.shape[1]
    red_count = sum(_mask_count(hsv, lo, hi) for lo, hi in _RED_RANGES)
    yellow_count = _mask_count(hsv, *_YELLOW_RANGE)
    green_count = _mask_count(hsv, *_GREEN_RANGE)

    scores = {"red": red_count, "yellow": yellow_count, "green": green_count}
    best = max(scores, key=scores.get)

    if scores[best] < total * _MIN_FRACTION:
        return "unknown"

    # No-entry sign defense: a red-dominant crop with a wide white band is
    # almost certainly a 'no entry' road sign, not a lit red bulb.
    if best == "red":
        white_count = _mask_count(hsv, *_WHITE_RANGE)
        if white_count > total * _MAX_WHITE_FRACTION_FOR_RED:
            return "unknown"

    return best
