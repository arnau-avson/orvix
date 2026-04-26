"""Project a stream of poses onto a route — the link between localization
and the routing/perception stack.
"""
from dataclasses import dataclass, field
from typing import List, Optional

from ..geometry import haversine_m, project_onto_segment
from ..models import Point, Route
from ..traffic_lights import TrafficLight
from .models import Pose


@dataclass
class TrackerState:
    pose: Pose
    progress_m: float                    # Distance traveled along the route polyline.
    remaining_m: float                   # Distance left to destination.
    off_route_distance_m: float          # Perpendicular distance from current pose to the route.
    is_off_route: bool                   # True when off_route_distance_m > threshold.
    nearest_segment_index: int           # Index of the closest polyline segment.
    approaching_lights: List[TrafficLight] = field(default_factory=list)


class RouteTracker:
    """Stateful projection of poses onto a route.

    Pass in a `Route` once, optionally attach traffic-light annotations from
    `plan_with_signals`, then feed each new `Pose` to `update()`. The
    returned `TrackerState` tells the navigation layer:
      - how far along the route the robot is
      - whether it has drifted off (e.g. user picked it up, GPS jumped)
      - which traffic lights are within `approach_radius_m` and need a
        sensor reading right now
    """

    def __init__(
        self,
        route: Route,
        off_route_threshold_m: float = 15.0,
        approach_radius_m: float = 15.0,
    ):
        if len(route.full_polyline) < 2:
            raise ValueError("Route polyline must have at least two points.")
        self.route = route
        self.polyline: List[Point] = route.full_polyline
        self.off_route_threshold_m = off_route_threshold_m
        self.approach_radius_m = approach_radius_m
        self.lights: List[TrafficLight] = []

        self._cum_dist = [0.0]
        for i in range(1, len(self.polyline)):
            d = haversine_m(
                self.polyline[i - 1].lat, self.polyline[i - 1].lon,
                self.polyline[i].lat, self.polyline[i].lon,
            )
            self._cum_dist.append(self._cum_dist[-1] + d)
        self.total_m = self._cum_dist[-1]

    def attach_lights(self, lights: List[TrafficLight]) -> None:
        self.lights = list(lights)

    def update(self, pose: Pose) -> TrackerState:
        best_i = 0
        best_dist = float("inf")
        best_t = 0.0
        for i in range(len(self.polyline) - 1):
            a = self.polyline[i]
            b = self.polyline[i + 1]
            _, _, t, dist = project_onto_segment(
                pose.point.lat, pose.point.lon,
                a.lat, a.lon, b.lat, b.lon,
            )
            if dist < best_dist:
                best_dist = dist
                best_i = i
                best_t = t

        seg_len = self._cum_dist[best_i + 1] - self._cum_dist[best_i]
        progress = self._cum_dist[best_i] + best_t * seg_len
        remaining = max(0.0, self.total_m - progress)

        approaching = [
            l for l in self.lights
            if haversine_m(pose.point.lat, pose.point.lon,
                           l.point.lat, l.point.lon) <= self.approach_radius_m
        ]

        return TrackerState(
            pose=pose,
            progress_m=progress,
            remaining_m=remaining,
            off_route_distance_m=best_dist,
            is_off_route=best_dist > self.off_route_threshold_m,
            nearest_segment_index=best_i,
            approaching_lights=approaching,
        )
