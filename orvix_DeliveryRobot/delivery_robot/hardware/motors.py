"""Motor control adapter — translates `NavigationAction` into motion commands.

Two implementations:
- `SerialMotorController`: sends ASCII commands over USB-serial to a
  microcontroller (Arduino / ESP32) that handles the low-level PWM and
  closed-loop wheel control.
- `MockMotorController`: records calls in `.history`. For unit tests and
  software-only demos.

Why route motor control through a microcontroller? Same reasoning as the
IMU adapter: PWM and closed-loop wheel control need real-time guarantees
that a Linux user-space process can't provide while also running YOLO.

Wire protocol (ASCII, line-delimited, lowercase commands):
    go <speed_mmps>          — drive forward at given speed (mm/s)
    wait                     — coast: motors off, no active braking
    stop                     — active brake / hold position
    estop                    — emergency stop, latch — recover requires reset
    turn <heading_deg>       — request rotation to absolute compass heading
                               (the microcontroller closes the loop on
                               the IMU-reported heading)

Acknowledgement is optional — the Python side does not block on ACK in the
default implementation, so command latency stays low.
"""
from abc import ABC, abstractmethod
from typing import List, Optional, Tuple

from ..navigation import NavigationAction


class MotorController(ABC):
    @abstractmethod
    def execute(
        self,
        action: NavigationAction,
        target_heading_deg: Optional[float] = None,
    ) -> None:
        """Apply the high-level navigation action to the motors."""

    @abstractmethod
    def emergency_stop(self) -> None:
        """Halt immediately, ignoring queued commands. Latches until reset."""

    @abstractmethod
    def close(self) -> None: ...

    def __enter__(self) -> "MotorController":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()


class MockMotorController(MotorController):
    """Records every call. Use to assert on the orchestrator's commands in tests."""

    def __init__(self):
        self.history: List[Tuple[str, Optional[float]]] = []
        self.last_action: Optional[NavigationAction] = None

    def execute(self, action, target_heading_deg=None):
        self.history.append((action.value, target_heading_deg))
        self.last_action = action

    def emergency_stop(self):
        self.history.append(("estop", None))

    def close(self):
        return None


class SerialMotorController(MotorController):
    """Sends commands over USB-serial to the motor microcontroller."""

    DEFAULT_SPEED_MMPS = 1400  # 1.4 m/s ≈ 5 km/h, typical sidewalk delivery pace.

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        default_speed_mmps: int = DEFAULT_SPEED_MMPS,
        write_timeout_s: float = 0.5,
    ):
        try:
            import serial  # type: ignore
        except ImportError as e:
            raise ImportError(
                "pyserial is required for SerialMotorController — "
                "`pip install pyserial`"
            ) from e
        self._serial = serial.Serial(
            port, baud, timeout=0.5, write_timeout=write_timeout_s
        )
        self._default_speed = int(default_speed_mmps)
        self._estopped = False

    def execute(self, action, target_heading_deg=None):
        if self._estopped:
            return  # latched until close()/reset
        if action == NavigationAction.GO:
            cmd = f"go {self._default_speed}\n"
        elif action == NavigationAction.WAIT:
            cmd = "wait\n"
        elif action == NavigationAction.STOP:
            cmd = "stop\n"
        else:
            return
        self._serial.write(cmd.encode("ascii"))
        if target_heading_deg is not None and action == NavigationAction.GO:
            self._serial.write(f"turn {target_heading_deg:.1f}\n".encode("ascii"))

    def emergency_stop(self):
        try:
            self._serial.write(b"estop\n")
        finally:
            self._estopped = True

    def close(self):
        try:
            self.emergency_stop()
        finally:
            self._serial.close()
