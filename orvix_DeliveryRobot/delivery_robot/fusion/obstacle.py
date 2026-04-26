"""Multi-frame fusion for the obstacle gate.

Asymmetric design — safety bias:
  - Stop quickly: a blocker observed in `min_blocker_frames` of the last
    `window_size` frames triggers STOP. Default 2 of 3.
  - Resume slowly is *not* enforced here (we let the orchestrator hold for
    a tick or two before re-reading), because being too slow to GO can be
    just as bad as missing a STOP — pedestrians behind us, etc.

The gate also tracks the "best blocker" (highest severity, then largest
area) seen in the recent window so the orchestrator can report *why* it
stopped, not just *that* it stopped.
"""
from collections import deque
from typing import Deque, List, Optional, Tuple

import numpy as np

from ..perception.obstacles import Obstacle, ObstacleDetector, should_stop


class FusedObstacleGate:
    def __init__(
        self,
        detector: ObstacleDetector,
        window_size: int = 3,
        min_blocker_frames: int = 2,
    ):
        if min_blocker_frames > window_size:
            raise ValueError("min_blocker_frames cannot exceed window_size")
        self.detector = detector
        self.window_size = window_size
        self.min_blocker_frames = min_blocker_frames
        self._window: Deque[Optional[Obstacle]] = deque(maxlen=window_size)
        self._latest_obstacles: List[Obstacle] = []

    def observe(self, frame: np.ndarray) -> Tuple[List[Obstacle], Optional[Obstacle]]:
        """Run detection on one frame.

        Returns (all_detected_obstacles, raw_blocker_this_frame). The raw
        blocker is what `should_stop()` returned for *this* frame alone —
        the smoothed decision is exposed via `current_blocker()`.
        """
        obstacles = self.detector.detect(frame)
        blocker = should_stop(obstacles, frame.shape)
        self._window.append(blocker)
        self._latest_obstacles = obstacles
        return obstacles, blocker

    def current_blocker(self) -> Optional[Obstacle]:
        """Smoothed verdict: the most relevant blocker only when sustained."""
        candidates = [b for b in self._window if b is not None]
        if len(candidates) < self.min_blocker_frames:
            return None
        # Pick highest severity, tiebreak by area (closer = bigger).
        return max(candidates, key=lambda o: (o.severity, o.area))

    @property
    def latest_obstacles(self) -> List[Obstacle]:
        return list(self._latest_obstacles)

    def reset(self) -> None:
        self._window.clear()
        self._latest_obstacles.clear()
