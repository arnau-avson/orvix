"""End-to-end demo of the hardware-adapter layer.

This file shows how to wire the hardware-adapter classes (`CameraSource`,
`LocalizationProvider`, `MotorController`) into the navigation stack. The
default invocation runs everything with **mocks** so it works offline on
any machine — no camera, no GPS, no motors required. Each mock has a
real-hardware sibling clearly labelled below.

Real-hardware swap (when you have the boards):

    camera = OpenCVCamera(0, width=1280, height=720)        # USB webcam
    # camera = OpenCVCamera("rtsp://192.168.1.50/live")     # IP cam

    gps = NMEASerialGPS("/dev/ttyACM0")                     # u-blox NEO module
    imu = SerialIMU("/dev/ttyACM1")                         # Arduino sensor hub
    localization = GPSIMULocalizer(gps, imu)                # complementary filter

    motors = SerialMotorController("/dev/ttyACM2")          # motor driver MCU

    Robot(camera, localization, motors, graph_loader).run_mission(mission)

Mock invocation (this file): `python run_robot.py`
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Callable, List, Optional, Tuple

import numpy as np

from delivery_robot import (
    Point,
    TrafficLight,
    compute_route,
    geocode,
    load_walk_graph_for_trip,
    plan_with_signals,
)
from delivery_robot.hardware import (
    BlankCamera,
    CameraSource,
    ImageSequenceCamera,
    MockMotorController,
    Robot,
)
from delivery_robot.localization import RouteSimulator
from delivery_robot.localization.models import Pose
from delivery_robot.localization.provider import LocalizationProvider
from delivery_robot.mission import Mission, MissionManager, MissionStatus
from delivery_robot.navigation import (
    NavigationAction,
    NavigationDecision,
    NavigationState,
)
from delivery_robot.perception.obstacles import Obstacle


# ---------------------------------------------------------------------------
# Mocks specific to this demo. Pluggable, drop-in replacements for the real
# Fused* / motor / camera adapters when there's no hardware to talk to.
# ---------------------------------------------------------------------------

class PausableRouteSimulator(LocalizationProvider):
    """RouteSimulator wrapper that respects WAIT/STOP from the orchestrator.

    Real GPS hardware reports the robot's position regardless of whether
    motors are moving; you don't 'pause' real GPS. But a simulator that
    advances on every poll would walk the robot through red lights —
    exactly what we want to *not* happen. So in this demo we tell the
    simulator the robot's current motion command, and it only advances on GO.
    """

    def __init__(self, route, speed_mps: float = 1.4, timestep_s: float = 0.5):
        self._sim = RouteSimulator(route, speed_mps=speed_mps, timestep_s=timestep_s)
        self._last_pose: Optional[Pose] = None
        self._last_action: NavigationAction = NavigationAction.GO

    def set_last_action(self, action: NavigationAction) -> None:
        self._last_action = action

    def get_pose(self) -> Optional[Pose]:
        if self._last_action == NavigationAction.GO or self._last_pose is None:
            self._last_pose = self._sim.get_pose()
        return self._last_pose


class ScriptedFusedLightSensor:
    """Drop-in for FusedTrafficLightSensor using a tick-based schedule."""

    def __init__(self, schedule: List[Tuple[int, str]]):
        self.schedule = sorted(schedule)
        self.tick = 0

    def observe(self, frame: np.ndarray) -> str:  # noqa: ARG002
        return self._state()

    def fused_state(self) -> str:
        return self._state()

    def is_green(self, light: TrafficLight) -> bool:  # noqa: ARG002
        return self._state() == "green"

    def reset(self) -> None:
        return None

    def _state(self) -> str:
        s = "unknown"
        for t, state in self.schedule:
            if self.tick >= t:
                s = state
        return s


class ScriptedFusedObstacleGate:
    """Drop-in for FusedObstacleGate using a tick-based schedule."""

    def __init__(self, schedule: List[Tuple[int, Optional[Obstacle]]]):
        self.schedule = sorted(schedule)
        self.tick = 0
        self.latest_obstacles: List[Obstacle] = []

    def observe(self, frame: np.ndarray):  # noqa: ARG002
        b = self._blocker()
        return ([b] if b else []), b

    def current_blocker(self) -> Optional[Obstacle]:
        return self._blocker()

    def reset(self) -> None:
        return None

    def _blocker(self) -> Optional[Obstacle]:
        b: Optional[Obstacle] = None
        for t, blocker in self.schedule:
            if self.tick >= t:
                b = blocker
        return b


# ---------------------------------------------------------------------------
# Demo
# ---------------------------------------------------------------------------

def _inject_demo_light(route, fraction: float = 0.5) -> List[TrafficLight]:
    polyline = route.full_polyline
    idx = max(1, min(len(polyline) - 2, int(len(polyline) * fraction)))
    return [TrafficLight(
        node_id=-1, point=polyline[idx], kind="pedestrian",
        step_index=idx, approach_bearing=0.0, exit_bearing=90.0,
        crossing_bearing=90.0,
    )]


def _print_decision(decision: NavigationDecision, prev: Optional[NavigationState],
                    wall_clock_s: float) -> None:
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


def _build_camera(image_dir: Optional[Path]) -> CameraSource:
    if image_dir is None:
        return BlankCamera(width=640, height=480)
    paths = sorted(image_dir.glob("*.png")) + sorted(image_dir.glob("*.jpg"))
    if not paths:
        print(f"WARN: no images in {image_dir}, falling back to BlankCamera")
        return BlankCamera(width=640, height=480)
    return ImageSequenceCamera(paths, loop=True, fps=2.0)


def main(image_dir: Optional[Path]) -> int:
    print("Hardware-adapter demo (all mocks)\n")
    print("Building components:")
    print("  camera        = ImageSequenceCamera" if image_dir else
          "  camera        = BlankCamera")
    print("  localization  = PausableRouteSimulator (wraps RouteSimulator)")
    print("  motors        = MockMotorController (records command history)")
    print("  light_sensor  = ScriptedFusedLightSensor (red->green at tick 100)")
    print("  obstacle_gate = ScriptedFusedObstacleGate (pedestrian at tick 12)")
    print()

    mission = Mission(
        mission_id="HW-DEMO-001",
        origin_address="Plaça de Catalunya, Barcelona",
        destination_address="Mercat de la Boqueria, Barcelona",
    )

    def graph_loader(o: Point, d: Point):
        return load_walk_graph_for_trip(o, d, margin_m=400)

    # --- Hardware adapters (mocks) ---
    camera = _build_camera(image_dir)
    motors = MockMotorController()

    # We need the route to construct the RouteSimulator. The MissionManager
    # plans it during start(), so pre-plan here and feed both the simulator
    # and the manager. (In real hardware the GPS is independent of any route
    # so this dance isn't needed.)
    origin_pt = geocode(mission.origin_address)
    dest_pt = geocode(mission.destination_address)
    graph = graph_loader(origin_pt, dest_pt)
    route = compute_route(graph, origin_pt, dest_pt)
    annotated = plan_with_signals(route, graph)
    print(f"Route: {route.total_distance_m:.0f} m\n")

    localization = PausableRouteSimulator(route, speed_mps=1.4, timestep_s=2.0)

    # --- Scripted perception (so the demo runs in seconds, no YOLO download) ---
    fake_pedestrian = Obstacle(
        class_name="person", cls_id=0,
        x1=400, y1=200, x2=600, y2=600, confidence=0.9,
    )
    light_sensor = ScriptedFusedLightSensor([
        (0, "unknown"), (100, "red"), (130, "green"),
    ])
    obstacle_gate = ScriptedFusedObstacleGate([
        (12, fake_pedestrian), (20, None),
    ])

    # --- Wire into Robot — the same shape you'd use with real hardware ---
    robot = Robot(
        camera=camera,
        localization=localization,
        motors=motors,
        graph_loader=graph_loader,
        light_sensor=light_sensor,
        obstacle_gate=obstacle_gate,
    )
    # The Robot.run_mission() loop polls GPS every tick. With a real robot
    # that's correct (GPS keeps reporting). With our PausableRouteSimulator
    # we need to inform it of the orchestrator's last action so it doesn't
    # walk through red lights. We also want to inject scripted-tick state.
    # So we drive the loop manually here, in the exact same style as
    # Robot._loop, just with these two extra hooks.
    robot.manager = MissionManager(
        mission=mission,
        graph_loader=graph_loader,
        light_sensor=light_sensor,  # type: ignore[arg-type]
        obstacle_gate=obstacle_gate,  # type: ignore[arg-type]
    )
    robot.manager.start()
    robot.manager.orchestrator.tracker.attach_lights(
        annotated.lights or _inject_demo_light(route)
    )

    prev_state: Optional[NavigationState] = None
    tick = 0
    wall_clock_s = 0.0
    try:
        while robot.manager.is_active and tick < 5_000:
            light_sensor.tick = tick
            obstacle_gate.tick = tick

            frame = robot.camera.read()
            if frame is None:
                robot.motors.emergency_stop()
                raise RuntimeError("camera dropout")

            pose = robot.localization.get_pose()
            if pose is None:
                robot.motors.execute(NavigationAction.WAIT)
                time.sleep(robot.no_pose_wait_s)
                continue

            decision = robot.manager.tick(pose, frame)
            if decision is None:
                break

            robot.motors.execute(decision.action, pose.heading_deg)
            localization.set_last_action(decision.action)

            _print_decision(decision, prev_state, wall_clock_s)
            prev_state = decision.state
            if decision.is_terminal:
                break

            tick += 1
            wall_clock_s += 2.0
    finally:
        robot.motors.close()
        robot.camera.close()

    print(f"\nMission status: {mission.status.value}")
    summary = _summarize_motor_history(motors.history)
    print("\nMotor command summary:")
    for cmd, count in summary.items():
        print(f"  {cmd:6} : {count}")
    return 0


def _summarize_motor_history(history) -> dict:
    counts: dict = {}
    for cmd, _heading in history:
        counts[cmd] = counts.get(cmd, 0) + 1
    return counts


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--images", default=None,
        help="Directory of test images for the camera (e.g. testing/). "
             "Default: BlankCamera (no images)."
    )
    args = parser.parse_args()
    img_dir = Path(args.images) if args.images else None
    sys.exit(main(img_dir))
