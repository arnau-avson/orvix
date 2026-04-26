"""Mission lifecycle: geocode → plan → execute → (replan if drift) → complete.

The manager owns one `Mission` and an `orchestrator` for the active leg. On
each tick it forwards the call to the orchestrator and reacts to terminal
states (ARRIVED → COMPLETED, OFF_ROUTE → REPLANNING).

Replan limit: a hard cap on automatic replans (default 3) — beyond that the
mission fails. Prevents pathological loops if the robot keeps drifting (bad
GPS, blocked sidewalk).
"""
import logging
from typing import Callable, Optional

import networkx as nx
import numpy as np

from ..fusion.obstacle import FusedObstacleGate
from ..fusion.traffic_light import FusedTrafficLightSensor
from ..geocoder import GeocodingError, geocode
from ..localization.models import Pose
from ..localization.tracker import RouteTracker
from ..models import Point, Route
from ..navigation.decision import NavigationDecision
from ..navigation.orchestrator import NavigationOrchestrator
from ..navigation.recovery import RecoveryMonitor, RecoveryPolicy
from ..navigation.states import NavigationState
from ..router import RoutingError, compute_route
from ..traffic_lights import plan_with_signals
from .models import Mission, MissionStatus

_log = logging.getLogger(__name__)


GraphLoader = Callable[[Point, Point], nx.MultiDiGraph]


class MissionManager:
    def __init__(
        self,
        mission: Mission,
        graph_loader: GraphLoader,
        light_sensor: FusedTrafficLightSensor,
        obstacle_gate: FusedObstacleGate,
        max_replans: int = 3,
        approach_radius_m: float = 15.0,
        arrived_radius_m: float = 5.0,
        recovery_policy: Optional[RecoveryPolicy] = None,
    ):
        self.mission = mission
        self.graph_loader = graph_loader
        self.light_sensor = light_sensor
        self.obstacle_gate = obstacle_gate
        self.max_replans = max_replans
        self.approach_radius_m = approach_radius_m
        self.arrived_radius_m = arrived_radius_m
        self.recovery_policy = recovery_policy or RecoveryPolicy()
        self.orchestrator: Optional[NavigationOrchestrator] = None
        self._replan_count = 0

    def start(self) -> None:
        if self.mission.status != MissionStatus.PENDING:
            raise ValueError(f"Mission already in state {self.mission.status}")
        self.mission.status = MissionStatus.PLANNING
        _log.info(
            "mission start",
            extra={"event": {
                "mission_id": self.mission.mission_id,
                "origin": self.mission.origin_address,
                "destination": self.mission.destination_address,
            }},
        )
        try:
            self.mission.origin_point = geocode(self.mission.origin_address)
            self.mission.destination_point = geocode(self.mission.destination_address)
        except GeocodingError as e:
            self._fail(f"Geocoding error: {e}")
            return
        if not self._plan_leg(self.mission.origin_point):
            return
        self.mission.status = MissionStatus.EN_ROUTE
        _log.info(
            "mission en_route",
            extra={"event": {
                "mission_id": self.mission.mission_id,
                "distance_m": round(self.mission.current_route.total_distance_m, 1),
            }},
        )

    def tick(self, pose: Pose, frame: np.ndarray) -> Optional[NavigationDecision]:
        if not self.is_active:
            return None

        decision = self.orchestrator.tick(pose, frame)

        if decision.state == NavigationState.ARRIVED:
            self.mission.status = MissionStatus.COMPLETED
            self.orchestrator = None
            _log.info(
                "mission completed",
                extra={"event": {"mission_id": self.mission.mission_id}},
            )
        elif decision.state == NavigationState.ERROR:
            self._fail(f"Orchestrator entered ERROR: {decision.reason}")
        elif decision.state == NavigationState.OFF_ROUTE:
            self._on_off_route(pose)

        return decision

    def _on_off_route(self, pose: Pose) -> None:
        if self._replan_count >= self.max_replans:
            self._fail(
                f"Off route after {self._replan_count} replans — giving up"
            )
            return
        self._replan_count += 1
        self.mission.status = MissionStatus.REPLANNING
        _log.warning(
            "mission replanning",
            extra={"event": {
                "mission_id": self.mission.mission_id,
                "attempt": self._replan_count,
                "from_lat": pose.point.lat,
                "from_lon": pose.point.lon,
            }},
        )
        if self._plan_leg(pose.point):
            self.mission.status = MissionStatus.EN_ROUTE

    def _plan_leg(self, origin: Point) -> bool:
        try:
            graph = self.graph_loader(origin, self.mission.destination_point)
            route = compute_route(graph, origin, self.mission.destination_point)
        except RoutingError as e:
            self._fail(f"Routing error: {e}")
            return False

        annotated = plan_with_signals(route, graph)
        self.mission.routes_history.append(route)

        tracker = RouteTracker(
            route,
            approach_radius_m=self.approach_radius_m,
        )
        tracker.attach_lights(annotated.lights)

        # Fresh sensor windows for the new leg.
        self.light_sensor.reset()
        self.obstacle_gate.reset()

        self.orchestrator = NavigationOrchestrator(
            route=route,
            tracker=tracker,
            light_sensor=self.light_sensor,
            obstacle_gate=self.obstacle_gate,
            arrived_radius_m=self.arrived_radius_m,
            recovery=RecoveryMonitor(policy=self.recovery_policy),
        )
        return True

    def _fail(self, reason: str) -> None:
        self.mission.status = MissionStatus.FAILED
        self.mission.failure_reason = reason
        self.orchestrator = None
        _log.error(
            "mission failed",
            extra={"event": {
                "mission_id": self.mission.mission_id,
                "reason": reason,
            }},
        )

    @property
    def is_active(self) -> bool:
        return self.orchestrator is not None
