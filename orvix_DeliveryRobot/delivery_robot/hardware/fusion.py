"""GPS + IMU sensor fusion.

GPS gives accurate position but its heading (course-over-ground) is unusable
when the robot is barely moving — it becomes noisy or undefined below ~1 m/s.
The IMU gives smooth heading at any speed but drifts over time when used
alone (gyro integration error).

Strategy: complementary filter on heading.
- At ground speed ≥ `gps_heading_speed_threshold_mps`: trust GPS heading.
- At zero speed: trust IMU heading.
- In between: linear blend, weight = speed / threshold.

Position itself is taken straight from GPS (cheaper modules don't justify
the complexity of full EKF until you add wheel odometry).
"""
import math
from typing import Optional

from ..localization.models import Pose
from ..localization.provider import LocalizationProvider
from .imu import IMUReader


class GPSIMULocalizer(LocalizationProvider):
    def __init__(
        self,
        gps: LocalizationProvider,
        imu: IMUReader,
        gps_heading_speed_threshold_mps: float = 1.0,
    ):
        self.gps = gps
        self.imu = imu
        self._threshold = gps_heading_speed_threshold_mps
        self._last_heading: Optional[float] = None

    def get_pose(self) -> Optional[Pose]:
        gps_pose = self.gps.get_pose()
        imu_sample = self.imu.read()
        if gps_pose is None:
            return None

        heading = self._fuse_heading(
            gps_heading=gps_pose.heading_deg,
            gps_speed=gps_pose.speed_mps or 0.0,
            imu_heading=imu_sample.heading_deg if imu_sample else None,
        )
        if heading is not None:
            self._last_heading = heading

        return Pose(
            point=gps_pose.point,
            heading_deg=heading,
            speed_mps=gps_pose.speed_mps,
            accuracy_m=gps_pose.accuracy_m,
            timestamp_s=gps_pose.timestamp_s,
        )

    def _fuse_heading(
        self,
        gps_heading: Optional[float],
        gps_speed: float,
        imu_heading: Optional[float],
    ) -> Optional[float]:
        if gps_heading is None and imu_heading is None:
            return self._last_heading
        if gps_heading is None:
            return imu_heading
        if imu_heading is None:
            return gps_heading

        if gps_speed >= self._threshold:
            weight_gps = 1.0
        elif gps_speed <= 0:
            weight_gps = 0.0
        else:
            weight_gps = gps_speed / self._threshold

        # Circular mean — handles wrap around 360°.
        rg = math.radians(gps_heading)
        ri = math.radians(imu_heading)
        x = weight_gps * math.cos(rg) + (1 - weight_gps) * math.cos(ri)
        y = weight_gps * math.sin(rg) + (1 - weight_gps) * math.sin(ri)
        return math.degrees(math.atan2(y, x)) % 360.0
