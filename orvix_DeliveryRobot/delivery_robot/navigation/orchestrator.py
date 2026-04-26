"""The navigation state machine.

One `tick(pose, frame)` per loop iteration:
  1. Feed perception (lights only when approaching, obstacles always).
  2. Update the route tracker with the pose.
  3. Resolve the highest-priority condition into a NavigationDecision.

Priority order (top wins):
  ARRIVED  >  OFF_ROUTE  >  STOPPED_FOR_OBSTACLE  >  light handling  >  WALKING

This ordering encodes the safety/operational hierarchy: don't overshoot the
goal, don't proceed when lost, don't move when something blocks the path.
"""
import logging
from typing import Optional

import numpy as np

from ..fusion.obstacle import FusedObstacleGate
from ..fusion.traffic_light import FusedTrafficLightSensor
from ..localization.models import Pose
from ..localization.tracker import RouteTracker
from ..models import Route
from .decision import NavigationDecision
from .recovery import RecoveryMonitor
from .states import NavigationAction, NavigationState

_log = logging.getLogger(__name__)


class NavigationOrchestrator:
    def __init__(
        self,
        route: Route,
        tracker: RouteTracker,
        light_sensor: FusedTrafficLightSensor,
        obstacle_gate: FusedObstacleGate,
        arrived_radius_m: float = 5.0,
        recovery: Optional[RecoveryMonitor] = None,
    ):
        self.route = route
        self.tracker = tracker
        self.light_sensor = light_sensor
        self.obstacle_gate = obstacle_gate
        self.arrived_radius_m = arrived_radius_m
        self.recovery = recovery
        self.last_decision: Optional[NavigationDecision] = None
        # Set of (lat, lon) keys for lights we have already cleared, so we
        # don't re-trigger APPROACHING for the same light after we cross.
        self._cleared_lights: set = set()

    def tick(self, pose: Pose, frame: np.ndarray) -> NavigationDecision:
        # --- Perception ---
        # Obstacles: always observe — a peatón may step into the path at any
        # moment, regardless of route state.
        self.obstacle_gate.observe(frame)
        smoothed_blocker = self.obstacle_gate.current_blocker()

        # --- Localization ---
        tracker_state = self.tracker.update(pose)
        # Filter out lights we've already cleared (we crossed past them).
        approaching = [
            l for l in tracker_state.approaching_lights
            if self._light_key(l) not in self._cleared_lights
        ]

        # Lights: only run the (expensive) detector when there is actually a
        # light to potentially cross. Saves CPU and keeps the fused window
        # populated with relevant frames.
        if approaching:
            self.light_sensor.observe(frame)

        # --- Decide ---
        # ARRIVED takes precedence over everything (we're done).
        if tracker_state.remaining_m <= self.arrived_radius_m:
            return self._record(NavigationDecision(
                state=NavigationState.ARRIVED,
                action=NavigationAction.STOP,
                reason="Destination reached",
                tracker=tracker_state,
            ))

        # OFF_ROUTE: cannot trust the rest of the pipeline; surface for replan.
        if tracker_state.is_off_route:
            return self._record(NavigationDecision(
                state=NavigationState.OFF_ROUTE,
                action=NavigationAction.STOP,
                reason=f"Drifted {tracker_state.off_route_distance_m:.1f}m off route — replan needed",
                tracker=tracker_state,
            ))

        # STOPPED_FOR_OBSTACLE: don't move while the path is blocked, even
        # at a green light.
        if smoothed_blocker is not None:
            return self._record(NavigationDecision(
                state=NavigationState.STOPPED_FOR_OBSTACLE,
                action=NavigationAction.STOP,
                reason=f"Path blocked by {smoothed_blocker.class_name}",
                tracker=tracker_state,
                blocker=smoothed_blocker,
            ))

        # Crossing handling.
        if approaching:
            light = approaching[0]
            light_state = self.light_sensor.fused_state()

            if light_state == "green":
                # Mark this light cleared once we've decided to cross — we'll
                # walk past it and shouldn't re-trigger on the next tick.
                self._cleared_lights.add(self._light_key(light))
                self.light_sensor.reset()
                return self._record(NavigationDecision(
                    state=NavigationState.CROSSING,
                    action=NavigationAction.GO,
                    reason="Crossing on green",
                    tracker=tracker_state,
                    light=light,
                    light_state=light_state,
                ))
            elif light_state == "red":
                return self._record(NavigationDecision(
                    state=NavigationState.WAITING_AT_CROSSING,
                    action=NavigationAction.WAIT,
                    reason="Holding for red light",
                    tracker=tracker_state,
                    light=light,
                    light_state=light_state,
                ))
            else:
                # Yellow or unknown — be conservative and hold.
                return self._record(NavigationDecision(
                    state=NavigationState.APPROACHING_CROSSING,
                    action=NavigationAction.WAIT,
                    reason=f"Approaching crossing — light state: {light_state}",
                    tracker=tracker_state,
                    light=light,
                    light_state=light_state,
                ))

        # Default: just walk.
        return self._record(NavigationDecision(
            state=NavigationState.WALKING,
            action=NavigationAction.GO,
            reason="Walking on route",
            tracker=tracker_state,
        ))

    def _record(self, decision: NavigationDecision) -> NavigationDecision:
        if self.recovery is not None:
            decision = self.recovery.review(decision, decision.tracker.pose.timestamp_s)

        prev_state = self.last_decision.state if self.last_decision else None
        if decision.state != prev_state:
            _log.info(
                "state transition",
                extra={"event": {
                    "state": decision.state.value,
                    "action": decision.action.value,
                    "reason": decision.reason,
                    "progress_m": round(decision.tracker.progress_m, 1),
                    "remaining_m": round(decision.tracker.remaining_m, 1),
                    "light_state": decision.light_state,
                    "blocker": decision.blocker.class_name if decision.blocker else None,
                }},
            )
        if decision.recovery_warning:
            _log.warning(
                "recovery",
                extra={"event": {
                    "warning": decision.recovery_warning,
                    "state": decision.state.value,
                }},
            )

        self.last_decision = decision
        return decision

    @staticmethod
    def _light_key(light) -> tuple:
        return (round(light.point.lat, 6), round(light.point.lon, 6))
