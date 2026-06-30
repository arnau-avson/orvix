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
    'top', 'bottom',
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
    'top': 0.0,
    'bottom': 0.0,
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
    'top': 'TOP',
    'bottom': 'BOTTOM',
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

    def test_ten_sectors(self):
        assert len(SECTORS) == 10

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

    def test_top_sector_exists(self):
        assert 'top' in SECTORS
        assert SECTOR_LABELS['top'] == 'TOP'
        assert 'top' in SECTOR_BEARINGS

    def test_bottom_sector_exists(self):
        assert 'bottom' in SECTORS
        assert SECTOR_LABELS['bottom'] == 'BOTTOM'
        assert 'bottom' in SECTOR_BEARINGS


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


# ---------------------------------------------------------------------------
# Distance confidence (replicated from obstacle_detector_node.py)
# ---------------------------------------------------------------------------

class TestDistanceConfidence:
    """Test the compute_distance_confidence pure function."""

    @staticmethod
    def _compute_confidence(frames_tracked, class_conf,
                             innovation_area, predicted_area,
                             min_frames=10):
        maturity = min(1.0, frames_tracked / min_frames)
        if predicted_area > 1.0:
            stability = 1.0 - min(1.0, abs(innovation_area) / predicted_area)
        else:
            stability = 0.0
        conf = maturity * max(class_conf, 0.1) * stability
        return max(0.0, min(1.0, conf))

    def test_new_track_low_confidence(self):
        """Track with only 1 frame should have low confidence."""
        conf = self._compute_confidence(1, 0.9, 10, 1000)
        assert conf < 0.15

    def test_mature_track_high_confidence(self):
        """Track with 20 frames and good classification → high confidence."""
        conf = self._compute_confidence(20, 0.9, 10, 10000)
        assert conf > 0.7

    def test_unknown_class_lower_confidence(self):
        """Unknown classification (conf=0.0) uses floor of 0.1."""
        conf = self._compute_confidence(20, 0.0, 10, 10000)
        assert conf < 0.15  # maturity=1.0 * 0.1 * stability ≈ 0.1

    def test_high_innovation_low_stability(self):
        """Large innovation relative to area → low stability."""
        conf = self._compute_confidence(20, 0.9, 900, 1000)
        assert conf < 0.15  # stability = 0.1

    def test_zero_predicted_area(self):
        """Zero predicted area → stability = 0 → confidence = 0."""
        conf = self._compute_confidence(20, 0.9, 0, 0)
        assert conf == 0.0

    def test_confidence_bounds(self):
        """Confidence is always in [0, 1]."""
        conf = self._compute_confidence(100, 1.0, 0, 10000)
        assert 0.0 <= conf <= 1.0
        conf2 = self._compute_confidence(0, 0.0, 99999, 1)
        assert 0.0 <= conf2 <= 1.0

    def test_exact_min_frames(self):
        """At exactly min_confident_frames, maturity should be 1.0."""
        conf = self._compute_confidence(10, 1.0, 0, 10000, min_frames=10)
        assert conf == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# Smoothed distance estimation
# ---------------------------------------------------------------------------

class TestSmoothedDistanceEstimation:
    """Test Kalman-smoothed area → distance formula."""

    CAM_W = 640
    CAM_HFOV = 62.0
    FOCAL_PX = (CAM_W / 2.0) / math.tan(math.radians(CAM_HFOV / 2.0))

    @classmethod
    def _estimate_distance(cls, known_size, area, aspect_ratio):
        """Distance from smoothed area and aspect ratio."""
        if aspect_ratio > 0.01 and area > 1.0:
            h = np.sqrt(area / aspect_ratio)
            if h > 1.0:
                return (known_size * cls.FOCAL_PX) / h
        return 100.0

    def test_valid_distance(self):
        """Reasonable area should give reasonable distance."""
        dist = self._estimate_distance(0.5, 2500, 1.0)
        assert 1.0 < dist < 50.0

    def test_larger_area_closer(self):
        """Bigger area means closer object."""
        dist_small = self._estimate_distance(0.5, 1000, 1.0)
        dist_large = self._estimate_distance(0.5, 4000, 1.0)
        assert dist_large < dist_small

    def test_larger_known_size_farther(self):
        """Bigger known size → same area means farther distance."""
        dist_small_obj = self._estimate_distance(0.3, 2500, 1.0)
        dist_large_obj = self._estimate_distance(2.0, 2500, 1.0)
        assert dist_large_obj > dist_small_obj

    def test_tiny_area_returns_far(self):
        """Very tiny area (< 1) returns 100m (far away)."""
        dist = self._estimate_distance(0.5, 0.5, 1.0)
        assert dist == 100.0

    def test_zero_aspect_ratio_returns_far(self):
        """Zero aspect ratio (degenerate) returns 100m."""
        dist = self._estimate_distance(0.5, 2500, 0.0)
        assert dist == 100.0


# ---------------------------------------------------------------------------
# Conservative unknown distance
# ---------------------------------------------------------------------------

class TestConservativeUnknownDistance:
    """Unknown objects should be estimated closer for safety."""

    CAM_W = 640
    CAM_HFOV = 62.0
    FOCAL_PX = (CAM_W / 2.0) / math.tan(math.radians(CAM_HFOV / 2.0))

    def test_unknown_uses_smaller_size(self):
        """Unknown classification should produce closer distance than 'person'."""
        bbox_h = 50.0
        dist_person = (0.5 * self.FOCAL_PX) / bbox_h
        dist_conservative = (0.3 * self.FOCAL_PX) / bbox_h  # bird size
        assert dist_conservative < dist_person

    def test_conservative_takes_minimum(self):
        """min(default_dist, conservative_dist) should always be the conservative one."""
        bbox_h = 80.0
        dist_default = (0.5 * self.FOCAL_PX) / bbox_h
        dist_conservative = (0.3 * self.FOCAL_PX) / bbox_h
        result = min(dist_default, dist_conservative)
        assert result == dist_conservative


# ---------------------------------------------------------------------------
# Confidence-based threat dampening
# ---------------------------------------------------------------------------

class TestThreatDampening:
    """Test that low confidence dampens CRITICAL to WARNING."""

    THREAT_WARNING = 3
    THREAT_CRITICAL = 4
    THREAT_EMERGENCY = 5

    @staticmethod
    def _dampen(threat, confidence):
        """Replicate confidence dampening logic."""
        if confidence < 0.3 and threat == 4:  # CRITICAL
            return 3  # WARNING
        return threat

    def test_low_confidence_dampens_critical(self):
        assert self._dampen(self.THREAT_CRITICAL, 0.1) == self.THREAT_WARNING

    def test_high_confidence_keeps_critical(self):
        assert self._dampen(self.THREAT_CRITICAL, 0.9) == self.THREAT_CRITICAL

    def test_emergency_never_dampened(self):
        assert self._dampen(self.THREAT_EMERGENCY, 0.1) == self.THREAT_EMERGENCY

    def test_warning_not_affected(self):
        assert self._dampen(self.THREAT_WARNING, 0.1) == self.THREAT_WARNING
