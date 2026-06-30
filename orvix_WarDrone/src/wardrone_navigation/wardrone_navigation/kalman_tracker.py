"""Kalman filter tracker for bounding boxes -- numpy-only, no rclpy.

Implements the SORT tracker's state model for tracking obstacles across
video frames using a linear Kalman filter with constant-velocity assumption.

State vector:  x = [cx, cy, s, r, dx, dy, ds]  (7 dimensions)
  cx, cy : bounding-box centre
  s      : bounding-box area
  r      : aspect ratio  w / h  (assumed quasi-constant)
  dx, dy : centre velocity
  ds     : area velocity

Measurement:   z = [cx, cy, s, r]  (4 dimensions)

Reference: Bewley et al., "Simple Online and Realtime Tracking" (SORT), 2016.
"""

import numpy as np


# ---------------------------------------------------------------------------
# Coordinate conversions
# ---------------------------------------------------------------------------

def bbox_to_z(bbox):
    """Convert bounding box (x1, y1, x2, y2) to measurement [cx, cy, s, r].

    Args:
        bbox: tuple/list of (x1, y1, x2, y2).

    Returns:
        np.ndarray of shape (4, 1).
    """
    x1, y1, x2, y2 = bbox
    w = x2 - x1
    h = y2 - y1
    cx = x1 + w / 2.0
    cy = y1 + h / 2.0
    s = w * h          # area
    r = w / h if h > 0 else 1.0  # aspect ratio
    return np.array([[cx], [cy], [s], [r]], dtype=np.float64)


def x_to_bbox(x):
    """Convert state vector (or its first 4 elements) to (x1, y1, x2, y2).

    Args:
        x: np.ndarray of shape (7, 1) or (7,) or (4, 1) or (4,).

    Returns:
        Tuple (x1, y1, x2, y2) as floats.
    """
    arr = np.asarray(x).flatten()
    cx, cy, s, r = arr[0], arr[1], arr[2], arr[3]
    s = max(s, 1.0)       # area must be positive
    r = max(r, 0.01)      # avoid division by zero
    w = np.sqrt(s * r)
    h = s / w if w > 0 else np.sqrt(s)
    return (cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0)


# ---------------------------------------------------------------------------
# IoU computation
# ---------------------------------------------------------------------------

def compute_iou(box_a, box_b):
    """Intersection-over-Union between two (x1, y1, x2, y2) bounding boxes.

    Args:
        box_a: tuple (x1, y1, x2, y2).
        box_b: tuple (x1, y1, x2, y2).

    Returns:
        float in [0, 1].
    """
    x1 = max(box_a[0], box_b[0])
    y1 = max(box_a[1], box_b[1])
    x2 = min(box_a[2], box_b[2])
    y2 = min(box_a[3], box_b[3])

    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    union = area_a + area_b - inter

    return inter / union if union > 0 else 0.0


# ---------------------------------------------------------------------------
# Greedy detection-to-tracker association
# ---------------------------------------------------------------------------

def associate_detections(detections, trackers, iou_threshold=0.3):
    """Greedy best-IoU matching between detections and predicted tracker bboxes.

    Builds an IoU matrix and iteratively picks the highest-IoU pair until
    no pair exceeds the threshold.  This avoids the scipy dependency that
    a Hungarian algorithm would require.

    Args:
        detections: list of (x1, y1, x2, y2) bboxes.
        trackers:   list of (x1, y1, x2, y2) predicted bboxes.
        iou_threshold: minimum IoU to accept a match.

    Returns:
        matches:              list of (det_idx, trk_idx) pairs.
        unmatched_detections: list of det_idx.
        unmatched_trackers:   list of trk_idx.
    """
    nd = len(detections)
    nt = len(trackers)

    if nt == 0:
        return [], list(range(nd)), []
    if nd == 0:
        return [], [], list(range(nt))

    # Build IoU matrix (nd × nt)
    iou_matrix = np.zeros((nd, nt), dtype=np.float64)
    for d in range(nd):
        for t in range(nt):
            iou_matrix[d, t] = compute_iou(detections[d], trackers[t])

    matched_dets = set()
    matched_trks = set()
    matches = []

    while True:
        best = np.unravel_index(np.argmax(iou_matrix), iou_matrix.shape)
        if iou_matrix[best] < iou_threshold:
            break
        d_idx, t_idx = int(best[0]), int(best[1])
        matches.append((d_idx, t_idx))
        matched_dets.add(d_idx)
        matched_trks.add(t_idx)
        iou_matrix[d_idx, :] = 0.0
        iou_matrix[:, t_idx] = 0.0

    unmatched_dets = [d for d in range(nd) if d not in matched_dets]
    unmatched_trks = [t for t in range(nt) if t not in matched_trks]

    return matches, unmatched_dets, unmatched_trks


# ---------------------------------------------------------------------------
# Kalman box tracker
# ---------------------------------------------------------------------------

class KalmanBoxTracker:
    """Tracks a single bounding box using a 7-state linear Kalman filter.

    Attributes:
        x: np.ndarray (7, 1) -- state [cx, cy, s, r, dx, dy, ds].
        P: np.ndarray (7, 7) -- state covariance.
        hit_streak: int      -- consecutive frames with successful update.
        time_since_update: int -- frames since last measurement update.
        id: int              -- unique tracker id (class-level counter).
    """

    _count = 0

    def __init__(self, bbox):
        """Initialise tracker from first detection bbox (x1, y1, x2, y2).

        Args:
            bbox: tuple (x1, y1, x2, y2).
        """
        # State transition (constant velocity)
        self.F = np.eye(7, dtype=np.float64)
        self.F[0, 4] = 1.0   # cx += dx
        self.F[1, 5] = 1.0   # cy += dy
        self.F[2, 6] = 1.0   # s  += ds

        # Measurement function
        self.H = np.zeros((4, 7), dtype=np.float64)
        self.H[0, 0] = 1.0   # observe cx
        self.H[1, 1] = 1.0   # observe cy
        self.H[2, 2] = 1.0   # observe s
        self.H[3, 3] = 1.0   # observe r

        # Measurement noise
        self.R = np.diag([1.0, 1.0, 10.0, 0.01]).astype(np.float64)

        # Process noise
        self.Q = np.eye(7, dtype=np.float64)
        self.Q[4, 4] = 0.01
        self.Q[5, 5] = 0.01
        self.Q[6, 6] = 0.0001
        # Cross-terms for velocity/position coupling
        self.Q[0, 4] = self.Q[4, 0] = 0.01
        self.Q[1, 5] = self.Q[5, 1] = 0.01
        self.Q[2, 6] = self.Q[6, 2] = 0.0001

        # Initial state from measurement
        z = bbox_to_z(bbox)
        self.x = np.zeros((7, 1), dtype=np.float64)
        self.x[:4] = z  # [cx, cy, s, r]
        # velocities initialised to zero

        # Initial covariance -- high uncertainty on velocities
        self.P = np.eye(7, dtype=np.float64) * 10.0
        self.P[4, 4] = 1000.0
        self.P[5, 5] = 1000.0
        self.P[6, 6] = 1000.0

        # Bookkeeping
        self.time_since_update = 0
        self.hit_streak = 0
        self._last_innovation = np.zeros((4, 1), dtype=np.float64)

        KalmanBoxTracker._count += 1
        self.id = KalmanBoxTracker._count

    # ----- Kalman predict / update ------------------------------------------

    def predict(self):
        """Advance state one step.  Returns predicted bbox (x1, y1, x2, y2)."""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        # Clamp area to positive
        if self.x[2, 0] < 1.0:
            self.x[2, 0] = 1.0

        self.time_since_update += 1
        return x_to_bbox(self.x)

    def update(self, bbox):
        """Correct state with a new measurement bbox (x1, y1, x2, y2)."""
        z = bbox_to_z(bbox)

        # Innovation
        y = z - self.H @ self.x
        self._last_innovation = y.copy()

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # State update
        self.x = self.x + K @ y

        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(7) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

        self.time_since_update = 0
        self.hit_streak += 1

    # ----- Accessors --------------------------------------------------------

    def get_state(self):
        """Current bbox (x1, y1, x2, y2) from state estimate."""
        return x_to_bbox(self.x)

    def get_area(self):
        """Kalman-smoothed bounding box area."""
        return max(self.x[2, 0], 0.0)

    def get_area_velocity(self):
        """Rate of change of area (ds / frame)."""
        return self.x[6, 0]

    def get_velocity(self):
        """Centre velocity (dx, dy) in pixels/frame."""
        return self.x[4, 0], self.x[5, 0]

    def get_innovation(self):
        """Last measurement innovation vector (4, 1)."""
        return self._last_innovation.copy()

    def get_innovation_area(self):
        """Scalar innovation on area component."""
        return self._last_innovation[2, 0]

    def get_predicted_area(self):
        """Area component of the predicted state (before last update)."""
        # This is the area from the current state, which after update
        # equals the corrected value.  For confidence we use the smoothed
        # area, which is a good proxy.
        return max(self.x[2, 0], 1.0)
