"""Multi-frame fusion for the traffic-light sensor.

Wraps a `YOLOTrafficLightDetector` and exposes a `TrafficLightSensor` whose
`is_green()` only returns True once a green state has been observed in the
majority of recent frames. Smooths over single-frame flicker and false
positives that survived the per-frame classifier.
"""
from typing import Optional

import numpy as np

from ..perception.detector import YOLOTrafficLightDetector
from ..perception.sensor import ImageSensor
from ..traffic_lights import TrafficLight, TrafficLightSensor
from .temporal import TemporalStateVoter


class FusedTrafficLightSensor(TrafficLightSensor):
    """Frames in, smoothed state out.

    Usage from the orchestrator:
        fused.observe(frame)               # called every tick
        if fused.is_green(light): ...      # consulted at decision time
    """

    def __init__(
        self,
        detector: YOLOTrafficLightDetector,
        window_size: int = 5,
        min_agreement: int = 3,
    ):
        self.detector = detector
        self.voter = TemporalStateVoter(window_size, min_agreement)

    def observe(self, frame: np.ndarray) -> str:
        """Process one frame; returns the raw (unsmoothed) per-frame state."""
        sensor = ImageSensor(frame, detector=self.detector)
        _, raw_state = sensor.detect_and_classify()
        self.voter.observe(raw_state)
        return raw_state

    def fused_state(self) -> str:
        return self.voter.fused()

    def is_green(self, light: TrafficLight) -> bool:  # noqa: ARG002
        return self.fused_state() == "green"

    def reset(self) -> None:
        """Clear the rolling window — call when a new crossing begins."""
        self.voter.reset()
