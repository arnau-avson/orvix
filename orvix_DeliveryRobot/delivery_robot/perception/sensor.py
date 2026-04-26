"""Single-frame implementation of TrafficLightSensor."""
from typing import List, Optional, Tuple

import numpy as np

from ..traffic_lights import TrafficLight, TrafficLightSensor
from .classifier import classify_state
from .detector import Detection, YOLOTrafficLightDetector


class ImageSensor(TrafficLightSensor):
    """Decide go/wait from a single camera frame.

    Selection policy: pick the most prominent visible light (largest area,
    tiebreak by detection confidence). Rationale: on approach to a crossing
    the closest physical light is the one governing it.

    Once the robot has a real camera with a known heading, this should be
    replaced by a sensor that uses `light.crossing_bearing` together with
    the camera's bearing to pick the correct light.
    """

    def __init__(
        self,
        frame: np.ndarray,
        detector: Optional[YOLOTrafficLightDetector] = None,
    ):
        self.frame = frame
        self.detector = detector or YOLOTrafficLightDetector()
        self._cache: Optional[Tuple[List[Detection], str]] = None

    def detect_and_classify(self) -> Tuple[List[Detection], str]:
        """Return (validated_detections, primary_state).

        A detection is 'validated' when the classifier returns a known state
        (red/yellow/green), not 'unknown'. This filters out detector false
        positives like flat red signs that lack a dark housing.
        """
        if self._cache is not None:
            return self._cache

        candidates = self.detector.detect(self.frame)
        validated: List[Tuple[Detection, str]] = []
        for d in candidates:
            state = classify_state(d.crop(self.frame))
            if state != "unknown":
                validated.append((d, state))

        if not validated:
            self._cache = ([], "unknown")
            return self._cache

        primary_det, primary_state = max(
            validated, key=lambda ds: (ds[0].area, ds[0].confidence)
        )
        self._cache = ([d for d, _ in validated], primary_state)
        return self._cache

    def is_green(self, light: TrafficLight) -> bool:  # noqa: ARG002
        _, state = self.detect_and_classify()
        return state == "green"
