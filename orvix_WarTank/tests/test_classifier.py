"""Traffic-light state classifier tests with synthesized crops."""
import numpy as np
import pytest

from delivery_robot.perception.classifier import classify_state


def _crop(width=40, height=80, fill=(0, 0, 0)):
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = fill  # BGR
    return img


def _add_bulb(img, color_bgr, center_y_frac, radius_px=8):
    h, w = img.shape[:2]
    cy = int(h * center_y_frac)
    cx = w // 2
    yy, xx = np.ogrid[:h, :w]
    mask = (yy - cy) ** 2 + (xx - cx) ** 2 <= radius_px ** 2
    img[mask] = color_bgr
    return img


class TestClassifier:
    def test_red_bulb_dark_housing(self):
        # Dark gray housing + red bulb in upper third
        img = _crop(fill=(40, 40, 40))
        img = _add_bulb(img, (0, 0, 255), 0.25, radius_px=10)
        assert classify_state(img) == "red"

    def test_green_bulb_dark_housing(self):
        img = _crop(fill=(30, 30, 30))
        img = _add_bulb(img, (0, 255, 0), 0.75, radius_px=10)
        assert classify_state(img) == "green"

    def test_yellow_bulb_dark_housing(self):
        img = _crop(fill=(30, 30, 30))
        img = _add_bulb(img, (0, 255, 255), 0.5, radius_px=10)
        assert classify_state(img) == "yellow"

    def test_uniform_red_returns_unknown(self):
        # No contrast — solid red. v_std too low.
        img = _crop(fill=(0, 0, 200))
        assert classify_state(img) == "unknown"

    def test_no_entry_sign_pattern_returns_unknown(self):
        # Red disc + thick white horizontal bar — the classic false positive.
        img = _crop(width=30, height=50, fill=(0, 0, 200))  # red dominant
        img[20:30, :] = (255, 255, 255)  # white bar in middle
        # Add some background variation to pass v_std.
        img[0:5, :] = (0, 0, 0)
        assert classify_state(img) == "unknown"

    def test_green_with_bright_background_still_green(self):
        # Yellow-housed green semáforo against a white wall — must NOT be
        # rejected by the white_frac filter (it only kicks in for red).
        # Realistic V variation: bright wall (V≈220), small housing edge
        # (V≈180), prominent green bulb, shadow at the bottom (V≈40).
        img = _crop(fill=(220, 220, 220))   # bright wall
        img[0:5, 5:35] = (0, 180, 180)      # thin housing top edge
        img[60:80, :] = (40, 40, 40)        # shadow / pole base
        img = _add_bulb(img, (0, 255, 0), 0.5, radius_px=15)
        assert classify_state(img) == "green"

    def test_too_small_returns_unknown(self):
        img = np.zeros((3, 3, 3), dtype=np.uint8)
        assert classify_state(img) == "unknown"

    def test_empty_returns_unknown(self):
        img = np.zeros((0, 0, 3), dtype=np.uint8)
        assert classify_state(img) == "unknown"
