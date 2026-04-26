"""Helpers to mask out fixed UI overlays in test imagery.

Real robot camera frames don't have these — they only show up in screenshots
of map services like Google Street View, where the minimap and compass
widgets are sources of detector false positives.
"""
from typing import List, Tuple

BBox = Tuple[int, int, int, int]


def streetview_overlay_regions(image_shape: Tuple[int, int, int]) -> List[BBox]:
    """Standard Google Street View UI overlay rectangles, in pixel coords.

    Returns regions for the bottom-left minimap, the bottom-right
    navigation compass, and the bottom Google attribution strip.
    """
    h, w = image_shape[:2]
    return [
        # Bottom-left minimap (with "Ampliar" button)
        (0, int(h * 0.74), int(w * 0.20), h),
        # Bottom-right zoom + compass widgets
        (int(w * 0.94), int(h * 0.66), w, h),
        # Top-right (occasionally Google logo / report-issue link)
        (int(w * 0.95), 0, w, int(h * 0.06)),
    ]
