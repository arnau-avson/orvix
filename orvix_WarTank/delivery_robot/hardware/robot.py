"""Top-level robot loop — wires hardware adapters to the navigation stack.

A `Robot` runs `MissionManager` to completion, sourcing frames from a
`CameraSource`, poses from a `LocalizationProvider`, and dispatching the
resulting `NavigationAction` to a `MotorController`.

Lifecycle:
    robot = Robot(camera, localization, motors, graph_loader)
    robot.run_mission(mission, on_decision=print)

Frame rate is determined by however fast the camera delivers and the
detectors process. On a Raspberry Pi 4 expect 5–10 Hz with YOLOv8n,
1–3 Hz with YOLOv8m. The orchestrator's logic is rate-independent.

Safety: any exception inside the loop triggers `motors.emergency_stop()`
before re-raising, so a Python crash leaves the robot stationary rather
than runaway.
"""
import logging
import time
from typing import Callable, Optional

import networkx as nx

from ..fusion import FusedObstacleGate, FusedTrafficLightSensor
from ..localization.provider import LocalizationProvider
from ..mission import Mission, MissionManager, MissionStatus
from ..models import Point
from ..navigation import NavigationAction, NavigationDecision
from ..perception import ObstacleDetector, YOLOTrafficLightDetector
from .camera import CameraSource
from .motors import MotorController

_log = logging.getLogger(__name__)


GraphLoader = Callable[[Point, Point], nx.MultiDiGraph]


class Robot:
    def __init__(
        self,
        camera: CameraSource,
        localization: LocalizationProvider,
        motors: MotorController,
        graph_loader: GraphLoader,
        light_sensor=None,                    # Inject for tests; default = real YOLO.
        obstacle_gate=None,                   # Inject for tests; default = real YOLO.
        light_window_size: int = 5,
        light_min_agreement: int = 3,
        obstacle_window_size: int = 3,
        obstacle_min_blocker_frames: int = 2,
        light_model_size: str = "m",
        obstacle_model_size: str = "m",
        no_pose_wait_s: float = 0.1,
        gps_dropout_estop_s: float = 30.0,
    ):
        self.camera = camera
        self.localization = localization
        self.motors = motors
        self.graph_loader = graph_loader
        self.no_pose_wait_s = no_pose_wait_s
        self.gps_dropout_estop_s = gps_dropout_estop_s

        if light_sensor is None:
            light_detector = YOLOTrafficLightDetector(model_size=light_model_size)
            light_sensor = FusedTrafficLightSensor(
                light_detector, light_window_size, light_min_agreement
            )
        if obstacle_gate is None:
            obstacle_detector = ObstacleDetector(model_size=obstacle_model_size)
            obstacle_gate = FusedObstacleGate(
                obstacle_detector, obstacle_window_size, obstacle_min_blocker_frames
            )
        self.light_sensor = light_sensor
        self.obstacle_gate = obstacle_gate
        self.manager: Optional[MissionManager] = None

    def run_mission(
        self,
        mission: Mission,
        on_decision: Optional[Callable[[NavigationDecision], None]] = None,
    ) -> Mission:
        """Block until the mission terminates (COMPLETED or FAILED).

        `on_decision` is invoked with each `NavigationDecision` for telemetry
        / display. It must not block.
        """
        self.manager = MissionManager(
            mission=mission,
            graph_loader=self.graph_loader,
            light_sensor=self.light_sensor,
            obstacle_gate=self.obstacle_gate,
        )
        self.manager.start()
        if mission.status == MissionStatus.FAILED:
            return mission

        try:
            self._loop(on_decision)
        except BaseException:
            # Safety: anything goes wrong, halt before propagating.
            self.motors.emergency_stop()
            raise
        finally:
            self.motors.close()
            self.camera.close()
        return mission

    def _loop(
        self,
        on_decision: Optional[Callable[[NavigationDecision], None]],
    ) -> None:
        no_pose_since: Optional[float] = None
        while self.manager and self.manager.is_active:
            frame = self.camera.read()
            if frame is None:
                self.motors.emergency_stop()
                _log.error("camera dropout — emergency stop")
                raise RuntimeError("Camera failed to deliver a frame")

            pose = self.localization.get_pose()
            if pose is None:
                now = time.monotonic()
                if no_pose_since is None:
                    no_pose_since = now
                    _log.warning("gps dropout started")
                elapsed = now - no_pose_since
                if elapsed >= self.gps_dropout_estop_s:
                    self.motors.emergency_stop()
                    _log.error(
                        "gps dropout escalation",
                        extra={"event": {"elapsed_s": round(elapsed, 1)}},
                    )
                    raise RuntimeError(
                        f"No GPS fix for {elapsed:.0f}s — emergency stop"
                    )
                self.motors.execute(NavigationAction.WAIT)
                time.sleep(self.no_pose_wait_s)
                continue

            if no_pose_since is not None:
                _log.info(
                    "gps recovered",
                    extra={"event": {"dropout_s": round(time.monotonic() - no_pose_since, 1)}},
                )
                no_pose_since = None

            decision = self.manager.tick(pose, frame)
            if decision is None:
                break

            self.motors.execute(decision.action, pose.heading_deg)
            if on_decision is not None:
                on_decision(decision)

            if decision.is_terminal:
                break
