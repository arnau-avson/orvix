"""GPS input adapter — reads NMEA-0183 sentences from a serial GPS module.

Tested against the protocol of u-blox NEO-6M / NEO-7M / NEO-M8N receivers
(the cheap consumer modules sold for hobby projects), which all default to
9600 baud and emit the standard sentence set.

We parse two sentences:
- $GPGGA / $GNGGA  — position + fix quality + HDOP (→ horizontal accuracy)
- $GPRMC / $GNRMC  — position + ground speed + course over ground (heading)

Position alone comes from GGA. Heading and speed come from RMC. We cache
the latest of each and emit a fused `Pose` from the most recent values.
"""
import time
from typing import Optional

from ..localization.models import Pose
from ..localization.provider import LocalizationProvider
from ..models import Point


class NMEAParseError(Exception):
    pass


def _parse_lat(value: str, hemi: str) -> float:
    if not value or len(value) < 4:
        raise NMEAParseError(f"bad latitude: {value!r}")
    deg = int(value[:2])
    minutes = float(value[2:])
    lat = deg + minutes / 60.0
    if hemi == "S":
        lat = -lat
    elif hemi != "N":
        raise NMEAParseError(f"bad lat hemisphere: {hemi!r}")
    return lat


def _parse_lon(value: str, hemi: str) -> float:
    if not value or len(value) < 5:
        raise NMEAParseError(f"bad longitude: {value!r}")
    deg = int(value[:3])
    minutes = float(value[3:])
    lon = deg + minutes / 60.0
    if hemi == "W":
        lon = -lon
    elif hemi != "E":
        raise NMEAParseError(f"bad lon hemisphere: {hemi!r}")
    return lon


def parse_gpgga(sentence: str) -> Optional[dict]:
    """Parse $GPGGA / $GNGGA. Returns dict or None for bad/no-fix lines."""
    parts = sentence.split(",")
    if len(parts) < 10 or parts[0] not in ("$GPGGA", "$GNGGA"):
        return None
    if parts[6] == "0":  # fix quality 0 = no fix
        return None
    try:
        lat = _parse_lat(parts[2], parts[3])
        lon = _parse_lon(parts[4], parts[5])
        hdop = float(parts[8]) if parts[8] else 99.0
    except (ValueError, NMEAParseError):
        return None
    # Rough horizontal accuracy from HDOP (×4 m is a common approximation
    # for consumer-grade chipsets in open-sky conditions).
    return {"lat": lat, "lon": lon, "accuracy_m": hdop * 4.0}


def parse_gprmc(sentence: str) -> Optional[dict]:
    """Parse $GPRMC / $GNRMC. Returns dict or None for invalid lines."""
    parts = sentence.split(",")
    if len(parts) < 12 or parts[0] not in ("$GPRMC", "$GNRMC"):
        return None
    if parts[2] != "A":  # status A = active fix
        return None
    try:
        lat = _parse_lat(parts[3], parts[4])
        lon = _parse_lon(parts[5], parts[6])
        speed_knots = float(parts[7]) if parts[7] else 0.0
        heading_deg = float(parts[8]) if parts[8] else None
    except (ValueError, NMEAParseError):
        return None
    return {
        "lat": lat,
        "lon": lon,
        "speed_mps": speed_knots * 0.5144,  # 1 knot = 0.5144 m/s
        "heading_deg": heading_deg,
    }


class NMEASerialGPS(LocalizationProvider):
    """Polling-style GPS reader.

    `get_pose()` drains whatever has arrived since the last call, parses any
    valid GGA/RMC sentences, and returns a Pose from the latest data. If
    nothing has been received yet (cold start), returns None.
    """

    def __init__(self, port: str, baud: int = 9600, timeout_s: float = 0.5):
        try:
            import serial  # type: ignore
        except ImportError as e:
            raise ImportError(
                "pyserial is required for NMEASerialGPS — `pip install pyserial`"
            ) from e
        self._serial = serial.Serial(port, baud, timeout=timeout_s)
        self._t0 = time.monotonic()
        self._pos: Optional[dict] = None      # latest GGA payload
        self._motion: Optional[dict] = None   # latest RMC payload

    def get_pose(self) -> Optional[Pose]:
        # Drain whatever has accumulated. Don't block on read.
        while self._serial.in_waiting:
            try:
                raw = self._serial.readline()
            except Exception:
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            if not line.startswith("$"):
                continue
            gga = parse_gpgga(line)
            if gga is not None:
                self._pos = gga
                continue
            rmc = parse_gprmc(line)
            if rmc is not None:
                self._motion = rmc

        if self._pos is None:
            return None

        return Pose(
            point=Point(lat=self._pos["lat"], lon=self._pos["lon"]),
            heading_deg=self._motion["heading_deg"] if self._motion else None,
            speed_mps=self._motion["speed_mps"] if self._motion else None,
            accuracy_m=self._pos["accuracy_m"],
            timestamp_s=time.monotonic() - self._t0,
        )

    def close(self) -> None:
        self._serial.close()


class MockGPS(LocalizationProvider):
    """Returns a fixed pose. For wiring up the loop without GPS hardware.

    For a moving mock that follows a route, use `RouteSimulator` from
    `delivery_robot.localization` instead — same interface.
    """

    def __init__(
        self,
        point: Point,
        heading_deg: Optional[float] = None,
        speed_mps: float = 0.0,
        accuracy_m: float = 5.0,
    ):
        self._point = point
        self._heading = heading_deg
        self._speed = speed_mps
        self._accuracy = accuracy_m
        self._t0 = time.monotonic()

    def get_pose(self) -> Optional[Pose]:
        return Pose(
            point=self._point,
            heading_deg=self._heading,
            speed_mps=self._speed,
            accuracy_m=self._accuracy,
            timestamp_s=time.monotonic() - self._t0,
        )
