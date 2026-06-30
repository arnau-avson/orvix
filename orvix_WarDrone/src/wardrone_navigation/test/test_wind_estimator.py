"""Tests for wind_estimator_node pure functions.

Pure functions and RunningAverageFilter are replicated inline to avoid
importing wind_estimator_node.py which requires rclpy at module level.
"""

import math
import pytest
from collections import deque


# --- Replicated pure functions from wind_estimator_node.py ---

def body_to_ned(vx_body: float, vy_body: float, vz_body: float, yaw_rad: float):
    cos_y = math.cos(yaw_rad)
    sin_y = math.sin(yaw_rad)
    v_north = vx_body * cos_y - vy_body * sin_y
    v_east = vx_body * sin_y + vy_body * cos_y
    v_down = vz_body
    return v_north, v_east, v_down


def estimate_wind(actual_ned: tuple, commanded_ned: tuple) -> tuple:
    return (
        actual_ned[0] - commanded_ned[0],
        actual_ned[1] - commanded_ned[1],
        actual_ned[2] - commanded_ned[2],
    )


def wind_speed_and_direction(wind_n: float, wind_e: float) -> tuple:
    speed = math.sqrt(wind_n ** 2 + wind_e ** 2)
    if speed < 0.01:
        return 0.0, 0.0
    to_dir = math.degrees(math.atan2(wind_e, wind_n))
    from_dir = (to_dir + 180.0) % 360.0
    return speed, from_dir


class RunningAverageFilter:
    def __init__(self, window_s: float):
        self._window_s = window_s
        self._samples = deque()

    def add_sample(self, timestamp: float, values: tuple):
        self._samples.append((timestamp, values))
        self._prune(timestamp)

    def get_average(self, now: float) -> tuple:
        self._prune(now)
        if not self._samples:
            return (0.0, 0.0, 0.0)
        n = len(self._samples)
        dim = len(self._samples[0][1])
        sums = [0.0] * dim
        for _, vals in self._samples:
            for i in range(dim):
                sums[i] += vals[i]
        return tuple(s / n for s in sums)

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def _prune(self, now: float):
        cutoff = now - self._window_s
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()


# --- Tests ---

class TestBodyToNed:
    def test_yaw_zero_identity(self):
        """With yaw=0, body frame aligns with NED: forward=north."""
        vn, ve, vd = body_to_ned(1.0, 0.0, 0.0, 0.0)
        assert abs(vn - 1.0) < 1e-9
        assert abs(ve - 0.0) < 1e-9
        assert abs(vd - 0.0) < 1e-9

    def test_yaw_90_rotated(self):
        """With yaw=90 deg, forward (vx) becomes east."""
        yaw = math.radians(90.0)
        vn, ve, vd = body_to_ned(1.0, 0.0, 0.0, yaw)
        assert abs(vn - 0.0) < 1e-9
        assert abs(ve - 1.0) < 1e-9
        assert abs(vd - 0.0) < 1e-9

    def test_yaw_180_reversed(self):
        """With yaw=180 deg, forward becomes south (negative north)."""
        yaw = math.radians(180.0)
        vn, ve, vd = body_to_ned(1.0, 0.0, 0.0, yaw)
        assert abs(vn - (-1.0)) < 1e-9
        assert abs(ve - 0.0) < 1e-9

    def test_lateral_velocity_yaw_zero(self):
        """Lateral body velocity (vy) at yaw=0: vy positive -> east positive."""
        vn, ve, vd = body_to_ned(0.0, 1.0, 0.0, 0.0)
        assert abs(vn - 0.0) < 1e-9
        assert abs(ve - 1.0) < 1e-9

    def test_vertical_passthrough(self):
        """Vertical velocity passes through unchanged."""
        vn, ve, vd = body_to_ned(0.0, 0.0, 3.5, 0.0)
        assert abs(vd - 3.5) < 1e-9


class TestEstimateWind:
    def test_no_wind(self):
        wn, we, wd = estimate_wind((5.0, 3.0, -1.0), (5.0, 3.0, -1.0))
        assert abs(wn) < 1e-9
        assert abs(we) < 1e-9
        assert abs(wd) < 1e-9

    def test_north_wind(self):
        wn, we, wd = estimate_wind((10.0, 0.0, 0.0), (5.0, 0.0, 0.0))
        assert abs(wn - 5.0) < 1e-9
        assert abs(we) < 1e-9
        assert abs(wd) < 1e-9

    def test_mixed_wind(self):
        wn, we, wd = estimate_wind((3.0, 2.0, 1.0), (1.0, 1.0, 0.5))
        assert abs(wn - 2.0) < 1e-9
        assert abs(we - 1.0) < 1e-9
        assert abs(wd - 0.5) < 1e-9


class TestWindSpeedAndDirection:
    def test_zero_wind(self):
        speed, direction = wind_speed_and_direction(0.0, 0.0)
        assert speed == 0.0
        assert direction == 0.0

    def test_pure_north_wind(self):
        """Wind blowing to the north -> comes from south (180 deg)."""
        speed, direction = wind_speed_and_direction(5.0, 0.0)
        assert abs(speed - 5.0) < 1e-9
        assert abs(direction - 180.0) < 1e-6

    def test_pure_east_wind(self):
        """Wind blowing to the east -> comes from west (270 deg)."""
        speed, direction = wind_speed_and_direction(0.0, 5.0)
        assert abs(speed - 5.0) < 1e-9
        assert abs(direction - 270.0) < 1e-6

    def test_pure_south_wind(self):
        """Wind blowing to the south -> comes from north (0 deg)."""
        speed, direction = wind_speed_and_direction(-5.0, 0.0)
        assert abs(speed - 5.0) < 1e-9
        assert abs(direction - 0.0) < 1e-6

    def test_diagonal_wind_speed(self):
        speed, _ = wind_speed_and_direction(3.0, 4.0)
        assert abs(speed - 5.0) < 1e-9

    def test_near_zero_threshold(self):
        speed, direction = wind_speed_and_direction(0.001, 0.001)
        assert speed == 0.0


class TestRunningAverageFilter:
    def test_single_sample(self):
        f = RunningAverageFilter(10.0)
        f.add_sample(100.0, (3.0, 4.0, 5.0))
        avg = f.get_average(100.0)
        assert abs(avg[0] - 3.0) < 1e-9
        assert abs(avg[1] - 4.0) < 1e-9
        assert abs(avg[2] - 5.0) < 1e-9

    def test_multiple_samples(self):
        f = RunningAverageFilter(10.0)
        f.add_sample(100.0, (2.0, 4.0, 6.0))
        f.add_sample(101.0, (4.0, 6.0, 8.0))
        avg = f.get_average(101.0)
        assert abs(avg[0] - 3.0) < 1e-9
        assert abs(avg[1] - 5.0) < 1e-9
        assert abs(avg[2] - 7.0) < 1e-9

    def test_window_expiry(self):
        f = RunningAverageFilter(5.0)
        f.add_sample(100.0, (10.0, 10.0, 10.0))
        f.add_sample(104.0, (20.0, 20.0, 20.0))
        avg = f.get_average(106.0)
        assert abs(avg[0] - 20.0) < 1e-9
        assert f.sample_count == 1

    def test_empty_filter(self):
        f = RunningAverageFilter(10.0)
        avg = f.get_average(100.0)
        assert avg == (0.0, 0.0, 0.0)

    def test_sample_count(self):
        f = RunningAverageFilter(10.0)
        assert f.sample_count == 0
        f.add_sample(100.0, (1.0, 2.0, 3.0))
        assert f.sample_count == 1
        f.add_sample(101.0, (4.0, 5.0, 6.0))
        assert f.sample_count == 2

    def test_all_samples_expired(self):
        f = RunningAverageFilter(2.0)
        f.add_sample(100.0, (5.0, 5.0, 5.0))
        avg = f.get_average(200.0)
        assert avg == (0.0, 0.0, 0.0)
        assert f.sample_count == 0
