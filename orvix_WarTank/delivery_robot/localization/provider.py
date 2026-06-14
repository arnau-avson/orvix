"""Sources of pose data.

`LocalizationProvider` is the abstract interface — production will fuse
GPS + IMU + wheel/visual odometry behind it. For now we ship two concrete
implementations useful for testing routing logic without hardware:

- `MockLocalization`: replays a pre-recorded list of poses.
- `RouteSimulator`: walks the robot along a `Route`'s polyline at a given
  speed, useful to validate the tracker and approach detection end-to-end.
"""
from abc import ABC, abstractmethod
from typing import List, Optional

from ..geometry import bearing_deg, haversine_m
from ..models import Point, Route
from .models import Pose


class LocalizationProvider(ABC):
    @abstractmethod
    def get_pose(self) -> Optional[Pose]:
        """Return the latest pose, or None when the source is exhausted."""


class MockLocalization(LocalizationProvider):
    def __init__(self, poses: List[Pose]):
        self._iter = iter(poses)

    def get_pose(self) -> Optional[Pose]:
        return next(self._iter, None)


class RouteSimulator(LocalizationProvider):
    """Pretend the robot is walking the route polyline at constant speed.

    Useful for end-to-end tests of the navigation stack without a camera or
    GPS receiver.
    """

    def __init__(
        self,
        route: Route,
        speed_mps: float = 1.4,
        timestep_s: float = 1.0,
        accuracy_m: float = 2.0,
    ):
        self.polyline: List[Point] = route.full_polyline
        if len(self.polyline) < 2:
            raise ValueError("Route polyline must contain at least two points.")
        self.speed_mps = speed_mps
        self.timestep_s = timestep_s
        self.accuracy_m = accuracy_m

        self._cum_dist = [0.0]
        for i in range(1, len(self.polyline)):
            d = haversine_m(
                self.polyline[i - 1].lat, self.polyline[i - 1].lon,
                self.polyline[i].lat, self.polyline[i].lon,
            )
            self._cum_dist.append(self._cum_dist[-1] + d)
        self.total_m = self._cum_dist[-1]

        self._t = 0.0
        self._done = False

    def get_pose(self) -> Optional[Pose]:
        if self._done:
            return None

        target_d = self._t * self.speed_mps
        if target_d >= self.total_m:
            self._done = True
            last = self.polyline[-1]
            prev = self.polyline[-2]
            return Pose(
                point=last,
                heading_deg=bearing_deg(prev.lat, prev.lon, last.lat, last.lon),
                speed_mps=0.0,
                accuracy_m=self.accuracy_m,
                timestamp_s=self._t,
            )

        i = 1
        while i < len(self._cum_dist) and self._cum_dist[i] < target_d:
            i += 1

        seg_start_d = self._cum_dist[i - 1]
        seg_end_d = self._cum_dist[i]
        seg_len = seg_end_d - seg_start_d
        t = 0.0 if seg_len < 1e-9 else (target_d - seg_start_d) / seg_len

        a = self.polyline[i - 1]
        b = self.polyline[i]
        pose = Pose(
            point=Point(
                lat=a.lat + t * (b.lat - a.lat),
                lon=a.lon + t * (b.lon - a.lon),
            ),
            heading_deg=bearing_deg(a.lat, a.lon, b.lat, b.lon),
            speed_mps=self.speed_mps,
            accuracy_m=self.accuracy_m,
            timestamp_s=self._t,
        )
        self._t += self.timestep_s
        return pose
