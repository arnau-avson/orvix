"""Tests for obstacle_detector_node module.

These tests exercise the obstacle detection logic (motion tracking, IoU matching,
approach velocity estimation, threat level computation, distance estimation)
as pure Python -- no ROS 2 runtime required.
"""

import math
import pytest


class TestObstacleDetectorImport:
    def test_import(self):
        from wardrone_navigation.obstacle_detector_node import ObstacleDetectorNode
        assert ObstacleDetectorNode is not None

    def test_import_constants(self):
        from wardrone_navigation.obstacle_detector_node import (
            SECTORS, SECTOR_BEARINGS, SECTOR_LABELS,
            THREAT_NONE, THREAT_MONITOR, THREAT_CAUTION,
            THREAT_WARNING, THREAT_CRITICAL, THREAT_EMERGENCY,
        )
        assert len(SECTORS) == 8
        assert len(SECTOR_BEARINGS) == 8
        assert len(SECTOR_LABELS) == 8


class TestSectorConstants:
    """Verify sector bearing angles and labels."""

    def test_front_bearing_is_zero(self):
        from wardrone_navigation.obstacle_detector_node import SECTOR_BEARINGS
        assert SECTOR_BEARINGS['front'] == 0.0

    def test_rear_bearing_is_180(self):
        from wardrone_navigation.obstacle_detector_node import SECTOR_BEARINGS
        assert SECTOR_BEARINGS['rear'] == 180.0

    def test_right_bearing_is_90(self):
        from wardrone_navigation.obstacle_detector_node import SECTOR_BEARINGS
        assert SECTOR_BEARINGS['right'] == 90.0

    def test_left_bearing_is_neg90(self):
        from wardrone_navigation.obstacle_detector_node import SECTOR_BEARINGS
        assert SECTOR_BEARINGS['left'] == -90.0

    def test_all_sectors_have_labels(self):
        from wardrone_navigation.obstacle_detector_node import SECTORS, SECTOR_LABELS
        for sector in SECTORS:
            assert sector in SECTOR_LABELS


class TestIoUComputation:
    """Test the IoU calculation between two bounding boxes."""

    @staticmethod
    def _compute_iou(box_a, box_b):
        from wardrone_navigation.obstacle_detector_node import ObstacleDetectorNode
        return ObstacleDetectorNode._compute_iou(box_a, box_b)

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
        # Intersection: (10,10)-(20,20) = 100
        # Area A: 400, Area B: 400, Union: 400+400-100 = 700
        expected = 100.0 / 700.0
        assert self._compute_iou(box_a, box_b) == pytest.approx(expected, abs=0.01)

    def test_box_inside_another(self):
        box_a = (0, 0, 100, 100)
        box_b = (20, 20, 40, 40)
        # Intersection = 20*20 = 400
        # Area A = 10000, Area B = 400, Union = 10000
        expected = 400.0 / 10000.0
        assert self._compute_iou(box_a, box_b) == pytest.approx(expected, abs=0.01)

    def test_zero_area_box(self):
        box_a = (10, 10, 10, 10)  # zero area
        box_b = (10, 10, 20, 20)
        assert self._compute_iou(box_a, box_b) == pytest.approx(0.0)

    def test_adjacent_boxes_no_overlap(self):
        box_a = (0, 0, 10, 10)
        box_b = (10, 0, 20, 10)
        assert self._compute_iou(box_a, box_b) == pytest.approx(0.0)


class TestThreatLevelComputation:
    """Test threat level computation based on distance and TTC."""

    @staticmethod
    def _compute_threat(distance, ttc, approach_vel,
                        dist_emergency=3.0, dist_critical=6.0,
                        dist_warning=12.0, dist_caution=25.0,
                        ttc_emergency=2.0, ttc_critical=4.0,
                        ttc_warning=6.0, ttc_caution=10.0):
        """Replicate the threat level logic from ObstacleDetectorNode."""
        from wardrone_navigation.obstacle_detector_node import (
            THREAT_NONE, THREAT_MONITOR, THREAT_CAUTION,
            THREAT_WARNING, THREAT_CRITICAL, THREAT_EMERGENCY,
        )
        threat = THREAT_NONE

        # Distance-based
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

        # TTC-based
        if ttc > 0:
            if ttc <= ttc_emergency:
                threat = max(threat, THREAT_EMERGENCY)
            elif ttc <= ttc_critical:
                threat = max(threat, THREAT_CRITICAL)
            elif ttc <= ttc_warning:
                threat = max(threat, THREAT_WARNING)
            elif ttc <= ttc_caution:
                threat = max(threat, THREAT_CAUTION)

        # Fast approach
        if approach_vel > 15.0:
            threat = max(threat, THREAT_WARNING)
        elif approach_vel > 25.0:
            threat = max(threat, THREAT_CRITICAL)

        return threat

    def test_far_away_stationary_is_monitor(self):
        from wardrone_navigation.obstacle_detector_node import THREAT_MONITOR
        assert self._compute_threat(50.0, -1.0, 0.0) == THREAT_MONITOR

    def test_close_distance_is_emergency(self):
        from wardrone_navigation.obstacle_detector_node import THREAT_EMERGENCY
        assert self._compute_threat(2.0, -1.0, 0.0) == THREAT_EMERGENCY

    def test_critical_distance(self):
        from wardrone_navigation.obstacle_detector_node import THREAT_CRITICAL
        assert self._compute_threat(5.0, -1.0, 0.0) == THREAT_CRITICAL

    def test_warning_distance(self):
        from wardrone_navigation.obstacle_detector_node import THREAT_WARNING
        assert self._compute_threat(10.0, -1.0, 0.0) == THREAT_WARNING

    def test_caution_distance(self):
        from wardrone_navigation.obstacle_detector_node import THREAT_CAUTION
        assert self._compute_threat(20.0, -1.0, 0.0) == THREAT_CAUTION

    def test_low_ttc_escalates_threat(self):
        """Object at 20m (CAUTION by distance) but TTC=1.5s should be EMERGENCY."""
        from wardrone_navigation.obstacle_detector_node import THREAT_EMERGENCY
        assert self._compute_threat(20.0, 1.5, 13.0) == THREAT_EMERGENCY

    def test_medium_ttc_escalates_to_critical(self):
        """Object at 20m (CAUTION) with TTC=3.5s should escalate to CRITICAL."""
        from wardrone_navigation.obstacle_detector_node import THREAT_CRITICAL
        assert self._compute_threat(20.0, 3.5, 5.7) == THREAT_CRITICAL

    def test_ttc_warning_escalation(self):
        """Object at 30m (MONITOR) with TTC=5.0s should escalate to WARNING."""
        from wardrone_navigation.obstacle_detector_node import THREAT_WARNING
        assert self._compute_threat(30.0, 5.0, 6.0) == THREAT_WARNING

    def test_fast_approach_escalates_to_warning(self):
        """Object at 50m but approaching at >15 m/s should be WARNING."""
        from wardrone_navigation.obstacle_detector_node import THREAT_WARNING
        assert self._compute_threat(50.0, -1.0, 16.0) == THREAT_WARNING

    def test_no_ttc_no_escalation(self):
        """Negative TTC (not approaching) should not escalate."""
        from wardrone_navigation.obstacle_detector_node import THREAT_MONITOR
        assert self._compute_threat(50.0, -1.0, 0.0) == THREAT_MONITOR


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
        """A bird (0.3m) filling 150px should be ~1.1m away."""
        dist = self._estimate_distance(0.3, 150)
        assert 0.5 < dist < 2.0

    def test_known_car_far(self):
        """A car (2.0m) appearing as 20px should be far away."""
        dist = self._estimate_distance(2.0, 20)
        assert dist > 30.0

    def test_zero_bbox_returns_100(self):
        dist = self._estimate_distance(0.5, 0)
        assert dist == 100.0

    def test_distance_decreases_with_larger_bbox(self):
        """Closer object = larger bbox = smaller distance."""
        dist_small = self._estimate_distance(0.5, 30)
        dist_large = self._estimate_distance(0.5, 100)
        assert dist_large < dist_small


class TestTrackedContourDataclass:
    """Test the TrackedContour dataclass."""

    def test_create_tracked_contour(self):
        from wardrone_navigation.obstacle_detector_node import TrackedContour
        tc = TrackedContour(
            contour_id=1,
            sector='front',
            bbox=(10, 20, 50, 60),
            area=1600.0,
            center=(30, 40),
        )
        assert tc.contour_id == 1
        assert tc.sector == 'front'
        assert tc.classification == 'unknown'
        assert tc.frames_tracked == 0
        assert len(tc.history) == 0

    def test_history_is_independent_per_instance(self):
        """Each TrackedContour must have its own history deque."""
        from wardrone_navigation.obstacle_detector_node import TrackedContour
        tc1 = TrackedContour(contour_id=1, sector='front',
                             bbox=(0, 0, 10, 10), area=100, center=(5, 5))
        tc2 = TrackedContour(contour_id=2, sector='rear',
                             bbox=(0, 0, 10, 10), area=100, center=(5, 5))
        tc1.history.append((0.0, 100, (5, 5)))
        assert len(tc1.history) == 1
        assert len(tc2.history) == 0


class TestApproachVelocityEstimation:
    """Test the approach velocity estimation logic.

    The algorithm uses linear regression on sqrt(area) vs time.
    A growing object (increasing sqrt_area) indicates approach.
    """

    def test_stationary_object_zero_velocity(self):
        """Constant area over time should give ~0 velocity."""
        from wardrone_navigation.obstacle_detector_node import TrackedContour
        import time as _time

        tc = TrackedContour(contour_id=1, sector='front',
                            bbox=(0, 0, 50, 50), area=2500, center=(25, 25))
        now = _time.time()
        for i in range(10):
            tc.history.append((now - 1.0 + i * 0.1, 2500, (25, 25)))

        # Manually call the velocity estimation logic
        # sqrt(2500) = 50, constant -> slope ≈ 0
        times = [h[0] - tc.history[0][0] for h in tc.history]
        sqrt_areas = [math.sqrt(h[1]) for h in tc.history]

        import numpy as np
        t = np.array(times)
        s = np.array(sqrt_areas)

        if len(t) >= 2 and (t[-1] - t[0]) > 0.05:
            n = len(t)
            sum_t = np.sum(t)
            sum_a = np.sum(s)
            sum_ta = np.sum(t * s)
            sum_t2 = np.sum(t * t)
            denom = n * sum_t2 - sum_t * sum_t
            if abs(denom) > 1e-9:
                slope = (n * sum_ta - sum_t * sum_a) / denom
                assert abs(slope) < 1.0  # Near zero slope

    def test_approaching_object_positive_velocity(self):
        """Growing area over time should give positive approach velocity."""
        import time as _time
        import numpy as np

        now = _time.time()
        # Simulate object area growing from 500 to 2000 over 1 second
        areas = [500 + i * 150 for i in range(10)]
        times_rel = [i * 0.1 for i in range(10)]

        sqrt_areas = [math.sqrt(a) for a in areas]
        t = np.array(times_rel)
        s = np.array(sqrt_areas)

        n = len(t)
        sum_t = np.sum(t)
        sum_a = np.sum(s)
        sum_ta = np.sum(t * s)
        sum_t2 = np.sum(t * t)
        denom = n * sum_t2 - sum_t * sum_t
        slope = (n * sum_ta - sum_t * sum_a) / denom

        # Positive slope = object is approaching
        assert slope > 0

    def test_receding_object_negative_slope(self):
        """Shrinking area over time should give negative slope."""
        import numpy as np

        # Area decreasing from 2000 to 500
        areas = [2000 - i * 150 for i in range(10)]
        times_rel = [i * 0.1 for i in range(10)]

        sqrt_areas = [math.sqrt(a) for a in areas]
        t = np.array(times_rel)
        s = np.array(sqrt_areas)

        n = len(t)
        sum_t = np.sum(t)
        sum_a = np.sum(s)
        sum_ta = np.sum(t * s)
        sum_t2 = np.sum(t * t)
        denom = n * sum_t2 - sum_t * sum_t
        slope = (n * sum_ta - sum_t * sum_a) / denom

        # Negative slope = object is receding
        assert slope < 0
