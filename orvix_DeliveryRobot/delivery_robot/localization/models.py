from dataclasses import dataclass
from typing import Optional

from ..models import Point


@dataclass
class Pose:
    """The robot's estimated pose at a moment in time.

    Sources combined here (GPS, IMU, wheel odometry, visual odometry) are
    abstracted away — this is the fused output the navigation layer consumes.
    """
    point: Point
    heading_deg: Optional[float]   # Compass bearing 0=N, 90=E. None if unknown.
    speed_mps: Optional[float]     # None if unknown / stationary.
    accuracy_m: float              # 1-σ horizontal position error.
    timestamp_s: float             # Seconds since session start.
