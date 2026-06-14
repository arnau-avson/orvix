"""End-to-end simulation of a delivery mission through the full software stack.

Pipeline exercised:
    Mission(addresses)
      -> MissionManager (geocode + plan + replan + lifecycle)
        -> NavigationOrchestrator (state machine)
          -> RouteTracker (localization → progress + approaching lights)
          -> FusedTrafficLightSensor (perception, smoothed)
          -> FusedObstacleGate (perception, smoothed)

Two modes:
  default   : Mocked perception with a scripted scenario. Fast (~2 s),
              deterministic, exercises every NavigationState including
              STOPPED_FOR_OBSTACLE, WAITING_AT_CROSSING, CROSSING, etc.

  --real    : Real YOLOv8 perception with cycling test images. Slow (depends
              on YOLO inference speed, several minutes). Validates that the
              real perception modules wire correctly into the orchestrator.

The demo mission is short (Plaça de Catalunya -> Mercat de la Boqueria, ~700 m)
to keep wall-clock manageable. Increase `timestep_s` in the simulator to
fast-forward; decrease for finer state-transition resolution.
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from itertools import cycle
from pathlib import Path
from typing import Callable, List, Optional, Tuple

import cv2
import numpy as np

from delivery_robot import (
    Point,
    TrafficLight,
    geocode,
    load_walk_graph_for_trip,
)
from delivery_robot.localization import RouteSimulator
from delivery_robot.mission import Mission, MissionManager, MissionStatus
from delivery_robot.navigation import NavigationAction, NavigationDecision, NavigationState
from delivery_robot.observability import setup_logging
from delivery_robot.perception.obstacles import Obstacle


# ---------------------------------------------------------------------------
# Scripted (mocked) perception. Implements the same protocol the orchestrator
# expects from the real Fused* classes: observe(frame), fused_state() /
# current_blocker(), reset().
# ---------------------------------------------------------------------------

@dataclass
class Scenario:
    """Drives mocked perception: lookup of (light_state, blocker) by tick."""
    light_schedule: List[Tuple[int, str]]
    blocker_schedule: List[Tuple[int, Optional[Obstacle]]]
    tick: int = 0

    def light_now(self) -> str:
        current = "unknown"
        for t, state in self.light_schedule:
            if self.tick >= t:
                current = state
        return current

    def blocker_now(self) -> Optional[Obstacle]:
        current: Optional[Obstacle] = None
        for t, blocker in self.blocker_schedule:
            if self.tick >= t:
                current = blocker
        return current


class ScriptedLightSensor:
    def __init__(self, scenario: Scenario):
        self.scenario = scenario

    def observe(self, frame: np.ndarray) -> str:  # noqa: ARG002
        return self.scenario.light_now()

    def fused_state(self) -> str:
        return self.scenario.light_now()

    def is_green(self, light: TrafficLight) -> bool:  # noqa: ARG002
        return self.scenario.light_now() == "green"

    def reset(self) -> None:
        pass


class ScriptedObstacleGate:
    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.latest_obstacles: List[Obstacle] = []

    def observe(
        self, frame: np.ndarray  # noqa: ARG002
    ) -> Tuple[List[Obstacle], Optional[Obstacle]]:
        b = self.scenario.blocker_now()
        return ([b] if b else []), b

    def current_blocker(self) -> Optional[Obstacle]:
        return self.scenario.blocker_now()

    def reset(self) -> None:
        pass


# ---------------------------------------------------------------------------
# Demo orchestration
# ---------------------------------------------------------------------------

def _inject_demo_light(route, lights, fraction: float = 0.5) -> List[TrafficLight]:
    """Inject a synthetic pedestrian light at `fraction` of the route polyline.

    The Plaça Catalunya -> Boqueria route has no traffic-signal nodes in OSM
    (it goes through pedestrian-only zones), but for a state-machine demo we
    need at least one crossing event. So we plant one halfway.
    """
    if lights:
        return lights
    polyline = route.full_polyline
    idx = max(1, min(len(polyline) - 2, int(len(polyline) * fraction)))
    return [
        TrafficLight(
            node_id=-1,
            point=polyline[idx],
            kind="pedestrian",
            step_index=idx,
            approach_bearing=0.0,
            exit_bearing=90.0,
            crossing_bearing=90.0,
        )
    ]


def _print_transition(
    decision: NavigationDecision,
    prev: Optional[NavigationState],
    wall_clock_s: float,
) -> None:
    if decision.state == prev:
        return
    icon = {
        NavigationState.WALKING: ">",
        NavigationState.APPROACHING_CROSSING: "?",
        NavigationState.WAITING_AT_CROSSING: "|",
        NavigationState.CROSSING: ">>",
        NavigationState.STOPPED_FOR_OBSTACLE: "X",
        NavigationState.OFF_ROUTE: "!",
        NavigationState.ARRIVED: "*",
        NavigationState.ERROR: "!!",
    }.get(decision.state, "-")
    print(
        f"  t={wall_clock_s:5.0f}s  progress={decision.tracker.progress_m:6.0f}m  "
        f"[{icon}] {decision.state.value:24}  ({decision.action.value:4})  "
        f"-- {decision.reason}"
    )


def run_mocked() -> int:
    print("Mode: MOCKED perception (deterministic scripted scenario)\n")

    mission = Mission(
        mission_id="DEMO-001",
        origin_address="Plaça de Catalunya, Barcelona",
        destination_address="Mercat de la Boqueria, Barcelona",
    )

    def graph_loader(origin, destination):
        return load_walk_graph_for_trip(origin, destination, margin_m=400)

    scenario = Scenario(light_schedule=[], blocker_schedule=[])
    light_sensor = ScriptedLightSensor(scenario)
    obstacle_gate = ScriptedObstacleGate(scenario)

    manager = MissionManager(
        mission=mission,
        graph_loader=graph_loader,
        light_sensor=light_sensor,  # type: ignore[arg-type]
        obstacle_gate=obstacle_gate,  # type: ignore[arg-type]
        approach_radius_m=20.0,
    )
    manager.start()

    if mission.status == MissionStatus.FAILED:
        print(f"Mission failed during planning: {mission.failure_reason}")
        return 1

    # Inject a synthetic pedestrian light at the route midpoint so the demo
    # exercises crossing handling.
    route = mission.current_route
    demo_lights = _inject_demo_light(route, [])
    manager.orchestrator.tracker.attach_lights(demo_lights)

    # Script: a fake pedestrian appears for ~30 s mid-walk, then clears.
    # When the robot reaches the crossing area the light starts red, then
    # turns green after ~30 s.
    fake_pedestrian = Obstacle(
        class_name="person", cls_id=0,
        x1=400, y1=200, x2=600, y2=600, confidence=0.9,
    )
    # The robot at 1.4 m/s with 2 s ticks covers 2.8 m/tick. The 693 m route
    # therefore runs ~250 ticks; the synthetic light sits at the midpoint
    # (~tick 115). Schedule events relative to that.
    scenario.blocker_schedule = [
        (12, fake_pedestrian),  # pedestrian steps in front near tick 12
        (20, None),             # walks away by tick 20
    ]
    scenario.light_schedule = [
        (0, "unknown"),
        (100, "red"),    # light is red just before the robot reaches the crossing
        (130, "green"),  # turns green ~30 s later
    ]

    print(f"Route: {route.total_distance_m:.0f} m  "
          f"({len(demo_lights)} lights, including 1 synthetic)\n")

    sim = RouteSimulator(route, speed_mps=1.4, timestep_s=2.0)
    blank_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    _run_loop(
        sim=sim,
        manager=manager,
        frame_provider=lambda: blank_frame,
        scenario=scenario,
        max_ticks=2_000,
    )
    print(f"\nMission status: {mission.status.value}")
    return 0


def _run_loop(
    sim: RouteSimulator,
    manager: MissionManager,
    frame_provider: Callable[[], np.ndarray],
    scenario: Optional[Scenario] = None,
    max_ticks: int = 5_000,
) -> None:
    """Drive the manager tick-by-tick, only advancing the simulator on GO.

    Wall-clock time advances every tick (the world keeps turning even when
    the robot is paused at a red light). The simulator's pose only advances
    on GO actions, so WAIT/STOP correctly hold the robot in place.
    """
    pose = sim.get_pose()
    if pose is None:
        return

    prev_state: Optional[NavigationState] = None
    tick_count = 0
    wall_clock_s = 0.0
    while pose is not None and tick_count < max_ticks:
        if scenario is not None:
            scenario.tick = tick_count
        decision = manager.tick(pose, frame_provider())
        if decision is None:
            break
        _print_transition(decision, prev_state, wall_clock_s)
        prev_state = decision.state
        if decision.is_terminal:
            break
        wall_clock_s += sim.timestep_s
        tick_count += 1
        if decision.action == NavigationAction.GO:
            pose = sim.get_pose()
        # else: WAIT/STOP — keep current pose, world ticks on.


def run_real(image_dir: Path) -> int:
    print(f"Mode: REAL perception (YOLOv8, cycling images from {image_dir})\n")

    image_paths = sorted(image_dir.glob("*.png")) + sorted(image_dir.glob("*.jpg"))
    if not image_paths:
        print(f"ERROR: no images in {image_dir}")
        return 1
    frames = [cv2.imread(str(p)) for p in image_paths]
    frames = [f for f in frames if f is not None]
    if not frames:
        print(f"ERROR: no readable images in {image_dir}")
        return 1
    frame_iter = cycle(frames)
    print(f"Loaded {len(frames)} test frames\n")

    # Lazy import — only pull torch when --real is used.
    from delivery_robot.fusion import FusedObstacleGate, FusedTrafficLightSensor
    from delivery_robot.perception import ObstacleDetector, YOLOTrafficLightDetector

    light_detector = YOLOTrafficLightDetector(model_size="m")
    obstacle_detector = ObstacleDetector(model_size="m")
    light_sensor = FusedTrafficLightSensor(light_detector, window_size=3, min_agreement=2)
    obstacle_gate = FusedObstacleGate(obstacle_detector, window_size=3, min_blocker_frames=2)

    mission = Mission(
        mission_id="DEMO-REAL-001",
        origin_address="Plaça de Catalunya, Barcelona",
        destination_address="Mercat de la Boqueria, Barcelona",
    )

    def graph_loader(origin, destination):
        return load_walk_graph_for_trip(origin, destination, margin_m=400)

    manager = MissionManager(
        mission=mission,
        graph_loader=graph_loader,
        light_sensor=light_sensor,
        obstacle_gate=obstacle_gate,
        approach_radius_m=20.0,
    )
    manager.start()

    if mission.status == MissionStatus.FAILED:
        print(f"Mission failed during planning: {mission.failure_reason}")
        return 1

    route = mission.current_route
    demo_lights = _inject_demo_light(route, [])
    manager.orchestrator.tracker.attach_lights(demo_lights)
    print(f"Route: {route.total_distance_m:.0f} m\n")

    # Larger timestep — real YOLO is the bottleneck, no need for fine ticks.
    sim = RouteSimulator(route, speed_mps=1.4, timestep_s=10.0)
    _run_loop(
        sim=sim,
        manager=manager,
        frame_provider=lambda: next(frame_iter),
        scenario=None,
        max_ticks=300,
    )
    print(f"\nMission status: {mission.status.value}")
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--real", action="store_true",
        help="Use real YOLOv8 perception with cycling test images.",
    )
    parser.add_argument(
        "--images", default="testing",
        help="Directory of test images (used only with --real).",
    )
    args = parser.parse_args()

    setup_logging(
        json_path="logs/run_simulation.jsonl",
        console_level="WARNING",   # human-readable progress already prints inline
    )
    if args.real:
        sys.exit(run_real(Path(args.images)))
    sys.exit(run_mocked())
