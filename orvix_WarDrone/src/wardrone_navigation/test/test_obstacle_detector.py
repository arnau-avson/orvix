"""Tests for obstacle_detector_node module.

These tests exercise the obstacle detection logic (IoU matching,
approach velocity estimation, threat level computation, distance estimation)
as pure Python -- no ROS 2 runtime required.

Logic is replicated from the source to allow testing outside the Docker
container (where rclpy is not available), following the same pattern as
test_safety_monitor.py.
"""

import math
import pytest
import numpy as np


# ---------------------------------------------------------------------------
# Constants (replicated from obstacle_detector_node.py)
# ---------------------------------------------------------------------------

SECTORS = [
    'front', 'front_right', 'right', 'rear_right',
    'rear', 'rear_left', 'left', 'front_left',
]

SECTOR_BEARINGS = {
    'front': 0.0,
    'front_right': 45.0,
    'right': 90.0,
    'rear_right': 135.0,
    'rear': 180.0,
    'rear_left': -135.0,
    'left': -90.0,
    'front_left': -45.0,
}

SECTOR_LABELS = {
    'front': 'FRONT',
    'front_right': 'FRONT_RIGHT',
    'right': 'RIGHT',
    'rear_right': 'REAR_RIGHT',
    'rear': 'REAR',
    'rear_left': 'REAR_LEFT',
    'left': 'LEFT',
    'front_left': 'FRONT_LEFT',
}

THREAT_NONE = 0
THREAT_MONITOR = 1
THREAT_CAUTION = 2
THREAT_WARNING = 3
THREAT_CRITICAL = 4
THREAT_EMERGENCY = 5


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestSectorConstants:
    """Verify sector bearing angles and labels."""

    def test_eight_sectors(self):
        assert len(SECTORS) == 8

    def test_front_bearing_is_zero(self):
        assert SECTOR_BEARINGS['front'] == 0.0

    def test_rear_bearing_is_180(self):
        assert SECTOR_BEARINGS['rear'] == 180.0

    def test_right_bearing_is_90(self):
        assert SECTOR_BEARINGS['right'] == 90.0

    def test_left_bearing_is_neg90(self):
        assert SECTOR_BEARINGS['left'] == -90.0

    def test_all_sectors_have_labels(self):
        for sector in SECTORS:
            assert sector in SECTOR_LABELS

    def test_all_sectors_have_bearings(self):
        for sector in SECTORS:
            assert sector in SECTOR_BEARINGS


class TestIoUComputation:
    """Test the IoU calculation between two bounding boxes.

    Replicates the static _compute_iou method from ObstacleDetectorNode.
    """

    @staticmethod
    def _compute_iou(box_a, box_b):
        """IoU between two (x1, y1, x2, y2) boxes."""
        x1 = max(box_a[0], box_b[0])
        y1 = max(box_a[1], box_b[1])
        x2 = min(box_a[2], box_b[2])
        y2 = min(box_a[3], box_b[3])

        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
        area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
        union = area_a + area_b - inter

        return inter / union if union > 0 else 0.0

    def test_identical_boxes(self):
        box = (10, 10, 50, 50)
        assert self._compute_iou(box, box) == pytest.approx(1.0)

    def test_no_overlap(self):
        box_a = (0, 0, 10, 10)
        box_b = (20, 20, 30, 30)
        assert self._compute_iou(box_a, box_b) == pytest.approx(0.0)

    def test_partial_overlap(self):
        box_a = (0, 0, 20, 20)
        box_b = (10, 10, 30, 30)
        expected = 100.0 / 700.0
        assert self._compute_iou(box_a, box_b) == pytest.approx(expected, abs=0.01)

    def test_box_inside_another(self):
        box_a = (0, 0, 100, 100)
        box_b = (20, 20, 40, 40)
        expected = 400.0 / 10000.0
        assert self._compute_iou(box_a, box_b) == pytest.approx(expected, abs=0.01)

    def test_zero_area_box(self):
        box_a = (10, 10, 10, 10)
        box_b = (10, 10, 20, 20)
        assert self._compute_iou(box_a, box_b) == pytest.approx(0.0)

    def test_adjacent_boxes_no_overlap(self):
        box_a = (0, 0, 10, 10)
        box_b = (10, 0, 20, 10)
        assert self._compute_iou(box_a, box_b) == pytest.approx(0.0)

    def test_symmetry(self):
        box_a = (0, 0, 20, 20)
        box_b = (10, 10, 30, 30)
        assert self._compute_iou(box_a, box_b) == pytest.approx(
            self._compute_iou(box_b, box_a)
        )


class TestThreatLevelComputation:
    """Test threat level computation based on distance and TTC.

    Replicates _compute_threat_level from ObstacleDetectorNode.
    """

    @staticmethod
    def _compute_threat(distance, ttc, approach_vel,
                        dist_emergency=3.0, dist_critical=6.0,
                        dist_warning=12.0, dist_caution=25.0,
                        ttc_emergency=2.0, ttc_critical=4.0,
                        ttc_warning=6.0, ttc_caution=10.0):
        threat = THREAT_NONE

        if distance <= dist_emergency:
            threat = max(threat, THREAT_EMERGENCY)
        elif distance <= dist_critical:
            threat = max(threat, THREAT_CRITICAL)
        elif distance <= dist_warning:
            threat = max(threat, THREAT_WARNING)
        elif distance <= dist_caution:
            threat = max(threat, THREAT_CAUTION)
        else:
            threat = max(threat, THREAT_MONITOR)

        if ttc > 0:
            if ttc <= ttc_emergency:
                threat = max(threat, THREAT_EMERGENCY)
            elif ttc <= ttc_critical:
                threat = max(threat, THREAT_CRITICAL)
            elif ttc <= ttc_warning:
                threat = max(threat, THREAT_WARNING)
            elif ttc <= ttc_caution:
                threat = max(threat, THREAT_CAUTION)

        if approach_vel > 25.0:
            threat = max(threat, THREAT_CRITICAL)
        elif approach_vel > 15.0:
            threat = max(threat, THREAT_WARNING)

        return threat

    def test_far_away_stationary_is_monitor(self):
        assert self._compute_threat(50.0, -1.0, 0.0) == THREAT_MONITOR

    def test_close_distance_is_emergency(self):
        assert self._compute_threat(2.0, -1.0, 0.0) == THREAT_EMERGENCY

    def test_critical_distance(self):
        assert self._compute_threat(5.0, -1.0, 0.0) == THREAT_CRITICAL

    def test_warning_distance(self):
        assert self._compute_threat(10.0, -1.0, 0.0) == THREAT_WARNING

    def test_caution_distance(self):
        assert self._compute_threat(20.0, -1.0, 0.0) == THREAT_CAUTION

    def test_low_ttc_escalates_threat(self):
        assert self._compute_threat(20.0, 1.5, 13.0) == THREAT_EMERGENCY

    def test_medium_ttc_escalates_to_critical(self):
        assert self._compute_threat(20.0, 3.5, 5.7) == THREAT_CRITICAL

    def test_ttc_warning_escalation(self):
        assert self._compute_threat(30.0, 5.0, 6.0) == THREAT_WARNING

    def test_fast_approach_escalates_to_warning(self):
        assert self._compute_threat(50.0, -1.0, 16.0) == THREAT_WARNING

    def test_very_fast_approach_escalates_to_critical(self):
        assert self._compute_threat(50.0, -1.0, 30.0) == THREAT_CRITICAL

    def test_no_ttc_no_escalation(self):
        assert self._compute_threat(50.0, -1.0, 0.0) == THREAT_MONITOR

    def test_boundary_distance_emergency(self):
        assert self._compute_threat(3.0, -1.0, 0.0) == THREAT_EMERGENCY

    def test_boundary_distance_critical(self):
        assert self._compute_threat(6.0, -1.0, 0.0) == THREAT_CRITICAL


class TestMonocularDistanceEstimation:
    """Test the monocular distance estimation formula.

    distance = (known_size * focal_px) / bbox_height_px
    focal_px = (cam_w / 2) / tan(hfov / 2)
    """

    @staticmethod
    def _estimate_distance(known_size_m, bbox_h_px, cam_w=640, hfov_deg=62.0):
        focal_px = (cam_w / 2.0) / math.tan(math.radians(hfov_deg / 2.0))
        if bbox_h_px > 0:
            return (known_size_m * focal_px) / bbox_h_px
        return 100.0

    def test_known_bird_close(self):
        dist = self._estimate_distance(0.3, 150)
        assert 0.5 < dist < 2.0

    def test_known_car_far(self):
        dist = self._estimate_distance(2.0, 20)
        assert dist > 30.0

    def test_zero_bbox_returns_100(self):
        dist = self._estimate_distance(0.5, 0)
        assert dist == 100.0

    def test_distance_decreases_with_larger_bbox(self):
        dist_small = self._estimate_distance(0.5, 30)
        dist_large = self._estimate_distance(0.5, 100)
        assert dist_large < dist_small

    def test_larger_object_farther_at_same_bbox(self):
        dist_small_obj = self._estimate_distance(0.3, 50)
        dist_large_obj = self._estimate_distance(2.0, 50)
        assert dist_large_obj > dist_small_obj

    def test_wider_fov_reduces_focal_length(self):
        dist_narrow = self._estimate_distance(0.5, 50, hfov_deg=62.0)
        dist_wide = self._estimate_distance(0.5, 50, hfov_deg=120.0)
        assert dist_wide < dist_narrow


class TestApproachVelocityEstimation:
    """Test the approach velocity estimation logic.

    The algorithm uses linear regression on sqrt(area) vs time.
    """

    @staticmethod
    def _compute_slope(times, areas):
        sqrt_areas = np.array([math.sqrt(a) for a in areas])
        t = np.array(times)

        n = len(t)
        sum_t = np.sum(t)
        sum_a = np.sum(sqrt_areas)
        sum_ta = np.sum(t * sqrt_areas)
        sum_t2 = np.sum(t * t)

        denom = n * sum_t2 - sum_t * sum_t
        if abs(denom) < 1e-9:
            return 0.0

        slope = (n * sum_ta - sum_t * sum_a) / denom
        return slope

    def test_stationary_object_zero_slope(self):
        times = [i * 0.1 for i in range(10)]
        areas = [2500] * 10
        slope = self._compute_slope(times, areas)
        assert abs(slope) < 0.01

    def test_approaching_object_positive_slope(self):
        times = [i * 0.1 for i in range(10)]
        areas = [500 + i * 150 for i in range(10)]
        slope = self._compute_slope(times, areas)
        assert slope > 0

    def test_receding_object_negative_slope(self):
        times = [i * 0.1 for i in range(10)]
        areas = [2000 - i * 150 for i in range(10)]
        slope = self._compute_slope(times, areas)
        assert slope < 0

    def test_fast_approach_large_slope(self):
        times = [i * 0.1 for i in range(10)]
        slow_areas = [500 + i * 50 for i in range(10)]
        fast_areas = [500 + i * 300 for i in range(10)]
        slow_slope = self._compute_slope(times, slow_areas)
        fast_slope = self._compute_slope(times, fast_areas)
        assert fast_slope > slow_slope

    def test_single_point_returns_zero(self):
        slope = self._compute_slope([0.0], [1000])
        assert slope == 0.0


class TestBearingCalculation:
    """Test bearing angle computation from image pixel offset."""

    @staticmethod
    def _compute_bearing(sector, bbox_cx, cam_w=640, hfov_deg=62.0):
        focal_px = (cam_w / 2.0) / math.tan(math.radians(hfov_deg / 2.0))
        px_offset = bbox_cx - (cam_w / 2.0)
        angle_offset = math.degrees(math.atan2(px_offset, focal_px))
        return SECTOR_BEARINGS[sector] + angle_offset

    def test_center_of_front_camera_is_zero(self):
        bearing = self._compute_bearing('front', 320)
        assert abs(bearing) < 0.1

    def test_right_of_front_camera_positive(self):
        bearing = self._compute_bearing('front', 600)
        assert bearing > 0

    def test_left_of_front_camera_negative(self):
        bearing = self._compute_bearing('front', 40)
        assert bearing < 0

    def test_center_of_rear_camera_is_180(self):
        bearing = self._compute_bearing('rear', 320)
        assert abs(bearing - 180.0) < 0.1

    def test_center_of_right_camera_is_90(self):
        bearing = self._compute_bearing('right', 320)
        assert abs(bearing - 90.0) < 0.1
