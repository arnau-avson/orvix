"""Tests for kalman_tracker module.

Since kalman_tracker.py is a pure Python+numpy module with no rclpy
dependency, we import it directly -- no need to replicate functions inline.
"""

import math

import numpy as np
import pytest

from wardrone_navigation.kalman_tracker import (
    KalmanBoxTracker,
    associate_detections,
    bbox_to_z,
    compute_iou,
    x_to_bbox,
)


# ---------------------------------------------------------------------------
# Bbox conversions
# ---------------------------------------------------------------------------

class TestBboxConversions:
    def test_roundtrip_square(self):
        """A square bbox should survive a bbox->z->bbox roundtrip."""
        bbox = (100, 100, 200, 200)
        z = bbox_to_z(bbox)
        recovered = x_to_bbox(z)
        assert abs(recovered[0] - bbox[0]) < 1.0
        assert abs(recovered[1] - bbox[1]) < 1.0
        assert abs(recovered[2] - bbox[2]) < 1.0
        assert abs(recovered[3] - bbox[3]) < 1.0

    def test_roundtrip_wide(self):
        """Wide rectangle: w=200, h=50."""
        bbox = (50, 100, 250, 150)
        z = bbox_to_z(bbox)
        recovered = x_to_bbox(z)
        assert abs(recovered[0] - bbox[0]) < 1.0
        assert abs(recovered[1] - bbox[1]) < 1.0
        assert abs(recovered[2] - bbox[2]) < 1.0
        assert abs(recovered[3] - bbox[3]) < 1.0

    def test_roundtrip_tall(self):
        """Tall rectangle: w=30, h=120."""
        bbox = (200, 50, 230, 170)
        z = bbox_to_z(bbox)
        recovered = x_to_bbox(z)
        assert abs(recovered[0] - bbox[0]) < 1.0
        assert abs(recovered[1] - bbox[1]) < 1.0
        assert abs(recovered[2] - bbox[2]) < 1.0
        assert abs(recovered[3] - bbox[3]) < 1.0

    def test_bbox_to_z_values(self):
        """Check cx, cy, area, aspect_ratio explicitly."""
        bbox = (100, 200, 300, 400)  # w=200, h=200
        z = bbox_to_z(bbox)
        assert z[0, 0] == pytest.approx(200.0)  # cx
        assert z[1, 0] == pytest.approx(300.0)  # cy
        assert z[2, 0] == pytest.approx(40000.0)  # area
        assert z[3, 0] == pytest.approx(1.0)  # aspect ratio

    def test_aspect_ratio_wide(self):
        bbox = (0, 0, 200, 100)  # w=200, h=100
        z = bbox_to_z(bbox)
        assert z[3, 0] == pytest.approx(2.0)

    def test_aspect_ratio_tall(self):
        bbox = (0, 0, 50, 200)  # w=50, h=200
        z = bbox_to_z(bbox)
        assert z[3, 0] == pytest.approx(0.25)

    def test_x_to_bbox_clamps_area(self):
        """Area below 1 gets clamped to 1."""
        x = np.array([[100], [100], [0.1], [1.0], [0], [0], [0]])
        bb = x_to_bbox(x)
        # Should not crash; area forced to 1.0
        assert bb[2] > bb[0]
        assert bb[3] > bb[1]


# ---------------------------------------------------------------------------
# IoU
# ---------------------------------------------------------------------------

class TestComputeIou:
    def test_identical_boxes(self):
        box = (100, 100, 200, 200)
        assert compute_iou(box, box) == pytest.approx(1.0)

    def test_no_overlap(self):
        a = (0, 0, 50, 50)
        b = (100, 100, 150, 150)
        assert compute_iou(a, b) == pytest.approx(0.0)

    def test_partial_overlap(self):
        a = (0, 0, 100, 100)
        b = (50, 50, 150, 150)
        # intersection = 50*50 = 2500
        # union = 10000 + 10000 - 2500 = 17500
        assert compute_iou(a, b) == pytest.approx(2500 / 17500)

    def test_symmetry(self):
        a = (10, 20, 80, 90)
        b = (30, 40, 120, 130)
        assert compute_iou(a, b) == pytest.approx(compute_iou(b, a))

    def test_contained(self):
        outer = (0, 0, 200, 200)
        inner = (50, 50, 150, 150)
        # intersection = 100*100 = 10000
        # union = 40000 + 10000 - 10000 = 40000
        assert compute_iou(outer, inner) == pytest.approx(10000 / 40000)

    def test_zero_area_box(self):
        a = (100, 100, 100, 100)  # zero area
        b = (50, 50, 150, 150)
        assert compute_iou(a, b) == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# Associate detections
# ---------------------------------------------------------------------------

class TestAssociateDetections:
    def test_empty_trackers(self):
        dets = [(0, 0, 50, 50), (100, 100, 150, 150)]
        matches, unm_d, unm_t = associate_detections(dets, [])
        assert len(matches) == 0
        assert unm_d == [0, 1]
        assert unm_t == []

    def test_empty_detections(self):
        trks = [(0, 0, 50, 50)]
        matches, unm_d, unm_t = associate_detections([], trks)
        assert len(matches) == 0
        assert unm_d == []
        assert unm_t == [0]

    def test_both_empty(self):
        matches, unm_d, unm_t = associate_detections([], [])
        assert matches == []
        assert unm_d == []
        assert unm_t == []

    def test_perfect_match(self):
        box = (100, 100, 200, 200)
        matches, unm_d, unm_t = associate_detections([box], [box])
        assert len(matches) == 1
        assert matches[0] == (0, 0)
        assert unm_d == []
        assert unm_t == []

    def test_below_threshold(self):
        det = (0, 0, 50, 50)
        trk = (200, 200, 250, 250)  # no overlap
        matches, unm_d, unm_t = associate_detections([det], [trk], 0.3)
        assert len(matches) == 0
        assert unm_d == [0]
        assert unm_t == [0]

    def test_two_dets_two_trks(self):
        det_a = (10, 10, 60, 60)
        det_b = (200, 200, 260, 260)
        trk_a = (12, 12, 62, 62)  # close to det_a
        trk_b = (198, 198, 258, 258)  # close to det_b
        matches, unm_d, unm_t = associate_detections(
            [det_a, det_b], [trk_a, trk_b], 0.3)
        assert len(matches) == 2
        match_set = {(m[0], m[1]) for m in matches}
        assert (0, 0) in match_set  # det_a -> trk_a
        assert (1, 1) in match_set  # det_b -> trk_b

    def test_more_dets_than_trks(self):
        dets = [(0, 0, 50, 50), (100, 100, 150, 150), (300, 300, 350, 350)]
        trks = [(2, 2, 52, 52)]  # matches first det
        matches, unm_d, unm_t = associate_detections(dets, trks, 0.3)
        assert len(matches) == 1
        assert matches[0] == (0, 0)
        assert set(unm_d) == {1, 2}

    def test_more_trks_than_dets(self):
        dets = [(0, 0, 50, 50)]
        trks = [(2, 2, 52, 52), (200, 200, 250, 250)]
        matches, unm_d, unm_t = associate_detections(dets, trks, 0.3)
        assert len(matches) == 1
        assert matches[0] == (0, 0)
        assert unm_t == [1]

    def test_greedy_picks_best_first(self):
        """When two dets overlap two trks, greedy should pick highest IoU first."""
        det_a = (0, 0, 100, 100)
        det_b = (10, 10, 110, 110)  # overlaps with both trks
        trk_a = (0, 0, 100, 100)   # perfect match for det_a
        trk_b = (12, 12, 112, 112)  # best match for det_b
        matches, _, _ = associate_detections([det_a, det_b], [trk_a, trk_b], 0.3)
        assert len(matches) == 2
        # det_a (idx 0) should match trk_a (idx 0) because IoU=1.0 is highest
        match_dict = {m[0]: m[1] for m in matches}
        assert match_dict[0] == 0


# ---------------------------------------------------------------------------
# KalmanBoxTracker
# ---------------------------------------------------------------------------

class TestKalmanBoxTracker:
    def test_init_state_matches_bbox(self):
        bbox = (100, 100, 200, 200)
        trk = KalmanBoxTracker(bbox)
        state_bbox = trk.get_state()
        assert abs(state_bbox[0] - 100) < 1.0
        assert abs(state_bbox[1] - 100) < 1.0
        assert abs(state_bbox[2] - 200) < 1.0
        assert abs(state_bbox[3] - 200) < 1.0

    def test_initial_area(self):
        bbox = (0, 0, 100, 50)  # w=100, h=50, area=5000
        trk = KalmanBoxTracker(bbox)
        assert trk.get_area() == pytest.approx(5000.0)

    def test_initial_velocity_zero(self):
        trk = KalmanBoxTracker((0, 0, 50, 50))
        dx, dy = trk.get_velocity()
        assert dx == pytest.approx(0.0)
        assert dy == pytest.approx(0.0)

    def test_predict_with_zero_velocity(self):
        """With no velocity, prediction should be nearly identical to init."""
        trk = KalmanBoxTracker((100, 100, 200, 200))
        pred = trk.predict()
        assert abs(pred[0] - 100) < 5.0
        assert abs(pred[1] - 100) < 5.0
        assert abs(pred[2] - 200) < 5.0
        assert abs(pred[3] - 200) < 5.0

    def test_predict_learns_velocity(self):
        """After several updates with rightward motion, predict should extrapolate."""
        trk = KalmanBoxTracker((100, 100, 150, 150))
        for i in range(1, 15):
            trk.predict()
            trk.update((100 + i * 10, 100, 150 + i * 10, 150))
        # After learning rightward motion, predict should move right
        pred = trk.predict()
        # Centre should be past the last update position
        last_cx = 100 + 14 * 10 + 25  # last centre x
        pred_cx = (pred[0] + pred[2]) / 2.0
        assert pred_cx > last_cx - 5  # should extrapolate forward

    def test_update_resets_time_since_update(self):
        trk = KalmanBoxTracker((0, 0, 50, 50))
        trk.predict()
        assert trk.time_since_update == 1
        trk.update((0, 0, 50, 50))
        assert trk.time_since_update == 0

    def test_hit_streak_increments(self):
        trk = KalmanBoxTracker((0, 0, 50, 50))
        assert trk.hit_streak == 0
        trk.predict()
        trk.update((0, 0, 50, 50))
        assert trk.hit_streak == 1
        trk.predict()
        trk.update((0, 0, 50, 50))
        assert trk.hit_streak == 2

    def test_area_stays_positive(self):
        """Even with noisy shrinking measurements, area should stay positive."""
        trk = KalmanBoxTracker((100, 100, 110, 110))  # small bbox
        for _ in range(20):
            trk.predict()
            # Feed a very small bbox
            trk.update((100, 100, 101, 101))
        assert trk.get_area() > 0

    def test_area_velocity_positive_when_growing(self):
        """Area velocity should become positive when object is growing."""
        trk = KalmanBoxTracker((100, 100, 150, 150))
        for i in range(10):
            trk.predict()
            size = 50 + i * 5
            trk.update((100, 100, 100 + size, 100 + size))
        assert trk.get_area_velocity() > 0

    def test_area_velocity_negative_when_shrinking(self):
        """Area velocity should become negative when object is shrinking."""
        trk = KalmanBoxTracker((100, 100, 200, 200))
        for i in range(10):
            trk.predict()
            size = 100 - i * 5
            trk.update((100, 100, 100 + size, 100 + size))
        assert trk.get_area_velocity() < 0

    def test_innovation_after_update(self):
        trk = KalmanBoxTracker((100, 100, 200, 200))
        trk.predict()
        trk.update((110, 110, 210, 210))  # shifted
        inn = trk.get_innovation()
        assert inn.shape == (4, 1)
        # cx innovation should be positive (observed > predicted)
        assert inn[0, 0] > 0

    def test_unique_ids(self):
        trk1 = KalmanBoxTracker((0, 0, 50, 50))
        trk2 = KalmanBoxTracker((100, 100, 150, 150))
        assert trk1.id != trk2.id

    def test_predict_without_update_increases_time(self):
        trk = KalmanBoxTracker((0, 0, 50, 50))
        trk.predict()
        assert trk.time_since_update == 1
        trk.predict()
        assert trk.time_since_update == 2
        trk.predict()
        assert trk.time_since_update == 3


# ---------------------------------------------------------------------------
# Kalman smoothing (integration test)
# ---------------------------------------------------------------------------

class TestKalmanSmoothing:
    def test_noisy_area_smoothed(self):
        """Kalman-filtered area should have lower variance than raw noisy input."""
        np.random.seed(42)
        true_area = 10000.0
        trk = KalmanBoxTracker((50, 50, 150, 150))  # area = 10000

        raw_areas = []
        kalman_areas = []

        for _ in range(30):
            noise = np.random.normal(0, 30)  # noisy size
            side = 100 + noise
            bbox = (50, 50, 50 + side, 50 + side)
            trk.predict()
            trk.update(bbox)
            raw_areas.append(side * side)
            kalman_areas.append(trk.get_area())

        raw_std = np.std(raw_areas)
        kalman_std = np.std(kalman_areas)
        assert kalman_std < raw_std, (
            f"Kalman std ({kalman_std:.1f}) should be < raw std ({raw_std:.1f})"
        )

    def test_stationary_object_converges(self):
        """Repeated same-bbox updates should converge area to true value."""
        bbox = (100, 100, 200, 200)  # area = 10000
        trk = KalmanBoxTracker(bbox)
        for _ in range(20):
            trk.predict()
            trk.update(bbox)
        assert abs(trk.get_area() - 10000.0) < 100.0

    def test_moving_object_tracks_centre(self):
        """A linearly moving object should be tracked accurately."""
        trk = KalmanBoxTracker((0, 0, 50, 50))
        for i in range(1, 20):
            trk.predict()
            trk.update((i * 5, 0, i * 5 + 50, 50))
        # Predicted centre should be near the last observed centre
        state = trk.get_state()
        last_cx = 19 * 5 + 25
        pred_cx = (state[0] + state[2]) / 2.0
        assert abs(pred_cx - last_cx) < 15.0
