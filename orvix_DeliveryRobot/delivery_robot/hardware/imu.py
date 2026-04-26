"""IMU input adapter.

The robot likely talks to a microcontroller (Arduino/ESP32) acting as a
sensor hub: that board reads the IMU chip directly (MPU6050, BNO055,
MPU9250 — over I2C) and forwards parsed values to the main computer over
a USB-serial link.

Why route through a microcontroller? Two reasons:
1. IMUs need to be sampled at high rate (≥100 Hz) and time-stamped tightly,
   which a real-time microcontroller does better than a Linux user-space
   loop competing with YOLO inference.
2. It abstracts the choice of chip — any IMU on the Arduino side, the same
   serial protocol on the Python side.

Expected line format (ASCII, line-delimited):
    $IMU,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<heading_or_empty>\n
where ax/ay/az are in g, gx/gy/gz in degrees/second, heading in degrees
(or empty when the IMU doesn't compute fusion itself).

Example: $IMU,0.01,-0.02,0.99,0.5,-0.3,0.1,247.5
"""
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class IMUSample:
    accel_xyz_g: Tuple[float, float, float]
    gyro_xyz_dps: Tuple[float, float, float]
    heading_deg: Optional[float]   # Magnetometer-fused yaw (chip-side), or None.
    timestamp_s: float


class IMUReader(ABC):
    @abstractmethod
    def read(self) -> Optional[IMUSample]:
        """Return latest sample, or None if the device hasn't reported yet."""

    @abstractmethod
    def close(self) -> None: ...


class MockIMU(IMUReader):
    """Fixed-pose IMU. Useful when you want to stub the IMU but still feed
    a heading into the GPS+IMU fusion layer.
    """

    def __init__(self, heading_deg: float = 0.0):
        self._heading = heading_deg
        self._t0 = time.monotonic()

    def read(self) -> Optional[IMUSample]:
        return IMUSample(
            accel_xyz_g=(0.0, 0.0, 1.0),
            gyro_xyz_dps=(0.0, 0.0, 0.0),
            heading_deg=self._heading,
            timestamp_s=time.monotonic() - self._t0,
        )

    def close(self) -> None:
        return None


class SerialIMU(IMUReader):
    """Reads $IMU lines from a serial sensor hub."""

    def __init__(self, port: str, baud: int = 115200, timeout_s: float = 0.2):
        try:
            import serial  # type: ignore
        except ImportError as e:
            raise ImportError(
                "pyserial is required for SerialIMU — `pip install pyserial`"
            ) from e
        self._serial = serial.Serial(port, baud, timeout=timeout_s)
        self._t0 = time.monotonic()
        self._latest: Optional[IMUSample] = None

    def read(self) -> Optional[IMUSample]:
        # Drain whatever has arrived; keep only the freshest valid sample.
        while self._serial.in_waiting:
            try:
                raw = self._serial.readline()
            except Exception:
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            if not line.startswith("$IMU,"):
                continue
            payload = line.split("*", 1)[0]  # strip optional checksum
            parts = payload.split(",")
            if len(parts) < 8:
                continue
            try:
                ax, ay, az = float(parts[1]), float(parts[2]), float(parts[3])
                gx, gy, gz = float(parts[4]), float(parts[5]), float(parts[6])
                heading = float(parts[7]) if parts[7] else None
            except ValueError:
                continue
            self._latest = IMUSample(
                accel_xyz_g=(ax, ay, az),
                gyro_xyz_dps=(gx, gy, gz),
                heading_deg=heading,
                timestamp_s=time.monotonic() - self._t0,
            )
        return self._latest

    def close(self) -> None:
        self._serial.close()
