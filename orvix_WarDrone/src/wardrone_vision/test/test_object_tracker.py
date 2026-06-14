"""Tests for object_tracker module."""

import pytest
import numpy as np
from wardrone_vision.object_tracker import (
    BoundingBox, Track, ObjectTracker, compute_iou, build_iou_matrix, greedy_assignment
)


class TestBoundingBox:
    def test_center(self):
        bb = BoundingBox(0, 0, 100, 100)
        assert bb.cx == 50.0
        assert bb.cy == 50.0

    def test_area(self):
        bb = BoundingBox(10, 20, 110, 120)
        assert bb.area == 10000

    def test_zero_area(self):
        bb = BoundingBox(50, 50, 50, 50)
        assert bb.area == 0


class TestIoU:
    def test_identical_boxes(self):
        a = BoundingBox(0, 0, 100, 100)
        b = BoundingBox(0, 0, 100, 100)
        assert compute_iou(a, b) == pytest.approx(1.0)

    def test_no_overlap(self):
        a = BoundingBox(0, 0, 50, 50)
        b = BoundingBox(100, 100, 200, 200)
        assert compute_iou(a, b) == pytest.approx(0.0)

    def test_partial_overlap(self):
        a = BoundingBox(0, 0, 100, 100)
        b = BoundingBox(50, 50, 150, 150)
        iou = compute_iou(a, b)
        assert 0.0 < iou < 1.0
        # Overlap: 50x50 = 2500, Union: 10000+10000-2500 = 17500
        assert iou == pytest.approx(2500.0 / 17500.0, abs=0.01)

    def test_contained(self):
        a = BoundingBox(0, 0, 200, 200)
        b = BoundingBox(50, 50, 150, 150)
        iou = compute_iou(a, b)
        # Overlap: 100x100=10000, Union: 40000+10000-10000=40000
        assert iou == pytest.approx(10000.0 / 40000.0, abs=0.01)


class TestGreedyAssignment:
    def test_perfect_match(self):
        iou_matrix = np.array([[1.0, 0.0], [0.0, 1.0]])
        matches, unmatched_t, unmatched_d = greedy_assignment(iou_matrix, 0.3)
        assert len(matches) == 2
        assert len(unmatched_t) == 0
        assert len(unmatched_d) == 0

    def test_no_match(self):
        iou_matrix = np.array([[0.1, 0.1], [0.1, 0.1]])
        matches, unmatched_t, unmatched_d = greedy_assignment(iou_matrix, 0.3)
        assert len(matches) == 0
        assert len(unmatched_t) == 2
        assert len(unmatched_d) == 2

    def test_partial_match(self):
        iou_matrix = np.array([[0.8, 0.1], [0.1, 0.1]])
        matches, unmatched_t, unmatched_d = greedy_assignment(iou_matrix, 0.3)
        assert len(matches) == 1
        assert matches[0] == (0, 0)
        assert 1 in unmatched_t
        assert 1 in unmatched_d


class TestObjectTracker:
    def _make_detection(self, x1, y1, x2, y2, class_id=2):
        return {
            'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
            'class_id': class_id, 'class_name': 'car',
            'confidence': 0.9,
        }

    def test_first_detection_creates_track(self):
        tracker = ObjectTracker()
        dets = [self._make_detection(100, 100, 200, 200)]
        tracks = tracker.update(dets)
        assert len(tracks) == 1
        assert tracks[0].track_id == 1

    def test_consistent_tracking(self):
        tracker = ObjectTracker()
        # Frame 1
        tracker.update([self._make_detection(100, 100, 200, 200)])
        # Frame 2 - slight movement
        tracks = tracker.update([self._make_detection(105, 105, 205, 205)])
        assert len(tracks) == 1
        assert tracks[0].track_id == 1  # Same ID

    def test_track_lost(self):
        tracker = ObjectTracker(max_lost_frames=2)
        tracker.update([self._make_detection(100, 100, 200, 200)])
        # 3 empty frames
        tracker.update([])
        tracker.update([])
        tracks = tracker.update([])
        assert len(tracks) == 0

    def test_multiple_objects(self):
        tracker = ObjectTracker()
        dets = [
            self._make_detection(0, 0, 50, 50),
            self._make_detection(200, 200, 300, 300),
        ]
        tracks = tracker.update(dets)
        assert len(tracks) == 2
        assert tracks[0].track_id != tracks[1].track_id

    def test_get_best_track(self):
        tracker = ObjectTracker()
        tracker.update([
            {'x1': 0, 'y1': 0, 'x2': 50, 'y2': 50, 'class_id': 0,
             'class_name': 'person', 'confidence': 0.7},
            {'x1': 200, 'y1': 200, 'x2': 300, 'y2': 300, 'class_id': 2,
             'class_name': 'car', 'confidence': 0.95},
        ])
        best_car = tracker.get_best_track(class_id=2)
        assert best_car is not None
        assert best_car.class_id == 2

    def test_reset(self):
        tracker = ObjectTracker()
        tracker.update([self._make_detection(100, 100, 200, 200)])
        assert len(tracker.tracks) == 1
        tracker.reset()
        assert len(tracker.tracks) == 0
