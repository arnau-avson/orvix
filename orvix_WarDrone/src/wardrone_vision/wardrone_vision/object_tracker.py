"""Simple multi-object tracker using IoU-based assignment.

Implements a simplified SORT (Simple Online and Realtime Tracking) algorithm:
1. Match new detections to existing tracks using IoU
2. Update matched tracks
3. Create new tracks for unmatched detections
4. Age out tracks that haven't been matched
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import numpy as np


@dataclass
class BoundingBox:
    x1: int
    y1: int
    x2: int
    y2: int

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) / 2.0

    @property
    def cy(self) -> float:
        return (self.y1 + self.y2) / 2.0

    @property
    def area(self) -> float:
        return max(0, self.x2 - self.x1) * max(0, self.y2 - self.y1)


@dataclass
class Track:
    track_id: int
    bbox: BoundingBox
    class_id: int
    class_name: str
    confidence: float
    frames_since_detection: int = 0
    total_frames: int = 0
    velocity_x: float = 0.0
    velocity_y: float = 0.0


def compute_iou(box_a: BoundingBox, box_b: BoundingBox) -> float:
    """Compute Intersection over Union between two bounding boxes."""
    x1 = max(box_a.x1, box_b.x1)
    y1 = max(box_a.y1, box_b.y1)
    x2 = min(box_a.x2, box_b.x2)
    y2 = min(box_a.y2, box_b.y2)

    inter_area = max(0, x2 - x1) * max(0, y2 - y1)
    if inter_area == 0:
        return 0.0

    union_area = box_a.area + box_b.area - inter_area
    if union_area == 0:
        return 0.0

    return inter_area / union_area


def build_iou_matrix(tracks: List[Track], detections: List[BoundingBox]) -> np.ndarray:
    """Build IoU cost matrix between existing tracks and new detections."""
    n_tracks = len(tracks)
    n_dets = len(detections)
    iou_matrix = np.zeros((n_tracks, n_dets))

    for i, track in enumerate(tracks):
        for j, det in enumerate(detections):
            iou_matrix[i, j] = compute_iou(track.bbox, det)

    return iou_matrix


def greedy_assignment(iou_matrix: np.ndarray, threshold: float = 0.3) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
    """Greedy assignment of detections to tracks based on IoU.

    Returns:
        matches: List of (track_idx, detection_idx) pairs
        unmatched_tracks: List of track indices without a detection
        unmatched_detections: List of detection indices without a track
    """
    n_tracks, n_dets = iou_matrix.shape
    matches = []
    used_tracks = set()
    used_dets = set()

    # Flatten and sort by IoU descending
    indices = []
    for i in range(n_tracks):
        for j in range(n_dets):
            if iou_matrix[i, j] >= threshold:
                indices.append((iou_matrix[i, j], i, j))

    indices.sort(key=lambda x: x[0], reverse=True)

    for _, track_idx, det_idx in indices:
        if track_idx not in used_tracks and det_idx not in used_dets:
            matches.append((track_idx, det_idx))
            used_tracks.add(track_idx)
            used_dets.add(det_idx)

    unmatched_tracks = [i for i in range(n_tracks) if i not in used_tracks]
    unmatched_detections = [j for j in range(n_dets) if j not in used_dets]

    return matches, unmatched_tracks, unmatched_detections


class ObjectTracker:
    """SORT-like multi-object tracker."""

    def __init__(
        self,
        iou_threshold: float = 0.3,
        max_lost_frames: int = 30,
    ):
        self._iou_threshold = iou_threshold
        self._max_lost_frames = max_lost_frames
        self._tracks: List[Track] = []
        self._next_id = 1

    @property
    def tracks(self) -> List[Track]:
        return self._tracks

    @property
    def active_tracks(self) -> List[Track]:
        return [t for t in self._tracks if t.frames_since_detection == 0]

    def update(self, detections: List[dict]) -> List[Track]:
        """Update tracker with new detections.

        Args:
            detections: List of dicts with keys: x1, y1, x2, y2, class_id, class_name, confidence

        Returns:
            Updated list of all active tracks.
        """
        det_boxes = [BoundingBox(d['x1'], d['y1'], d['x2'], d['y2']) for d in detections]

        if len(self._tracks) == 0:
            # No existing tracks, create new ones
            for det, bbox in zip(detections, det_boxes):
                self._tracks.append(Track(
                    track_id=self._next_id,
                    bbox=bbox,
                    class_id=det['class_id'],
                    class_name=det['class_name'],
                    confidence=det['confidence'],
                ))
                self._next_id += 1
            return self._tracks

        if len(detections) == 0:
            # No new detections, age all tracks
            for track in self._tracks:
                track.frames_since_detection += 1
            self._tracks = [t for t in self._tracks if t.frames_since_detection <= self._max_lost_frames]
            return self._tracks

        # Build IoU matrix and assign
        iou_matrix = build_iou_matrix(self._tracks, det_boxes)
        matches, unmatched_tracks, unmatched_dets = greedy_assignment(
            iou_matrix, self._iou_threshold
        )

        # Update matched tracks
        for track_idx, det_idx in matches:
            track = self._tracks[track_idx]
            new_bbox = det_boxes[det_idx]
            det = detections[det_idx]

            # Compute velocity (pixel shift)
            track.velocity_x = new_bbox.cx - track.bbox.cx
            track.velocity_y = new_bbox.cy - track.bbox.cy

            track.bbox = new_bbox
            track.confidence = det['confidence']
            track.frames_since_detection = 0
            track.total_frames += 1

        # Age unmatched tracks
        for track_idx in unmatched_tracks:
            self._tracks[track_idx].frames_since_detection += 1

        # Create new tracks for unmatched detections
        for det_idx in unmatched_dets:
            det = detections[det_idx]
            self._tracks.append(Track(
                track_id=self._next_id,
                bbox=det_boxes[det_idx],
                class_id=det['class_id'],
                class_name=det['class_name'],
                confidence=det['confidence'],
            ))
            self._next_id += 1

        # Remove dead tracks
        self._tracks = [t for t in self._tracks if t.frames_since_detection <= self._max_lost_frames]

        return self._tracks

    def get_track_by_id(self, track_id: int) -> Optional[Track]:
        for t in self._tracks:
            if t.track_id == track_id:
                return t
        return None

    def get_best_track(self, class_id: Optional[int] = None) -> Optional[Track]:
        """Get the best active track (highest confidence), optionally filtered by class."""
        candidates = self.active_tracks
        if class_id is not None:
            candidates = [t for t in candidates if t.class_id == class_id]
        if not candidates:
            return None
        return max(candidates, key=lambda t: t.confidence)

    def reset(self):
        self._tracks.clear()
        self._next_id = 1
