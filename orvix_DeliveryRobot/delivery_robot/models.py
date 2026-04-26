from dataclasses import dataclass, field
from typing import List, Optional


@dataclass(frozen=True)
class Point:
    lat: float
    lon: float

    def as_tuple(self) -> tuple[float, float]:
        return (self.lat, self.lon)


@dataclass
class Step:
    """A single edge along the route.

    `geometry` holds the full polyline (sidewalk curve) of the edge in the
    direction of travel. It always starts at `start` and ends at `end`. When
    the underlying OSM edge has no explicit geometry, it falls back to
    `[start, end]`.
    """
    start: Point
    end: Point
    length_m: float
    street_name: Optional[str] = None
    highway_type: Optional[str] = None
    geometry: List[Point] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not self.geometry:
            self.geometry = [self.start, self.end]


@dataclass
class Route:
    origin: Point
    destination: Point
    steps: List[Step] = field(default_factory=list)

    @property
    def total_distance_m(self) -> float:
        return sum(s.length_m for s in self.steps)

    @property
    def waypoints(self) -> List[Point]:
        """One point per node along the route (decision points only)."""
        if not self.steps:
            return []
        pts = [self.steps[0].start]
        for s in self.steps:
            pts.append(s.end)
        return pts

    @property
    def full_polyline(self) -> List[Point]:
        """Every point of the actual sidewalk curve, deduplicated at edge joins."""
        if not self.steps:
            return []
        pts: List[Point] = list(self.steps[0].geometry)
        for s in self.steps[1:]:
            # Skip the first point of each subsequent step — it equals the
            # last point of the previous step.
            pts.extend(s.geometry[1:])
        return pts

    def estimated_time_s(self, speed_mps: float = 1.4) -> float:
        """Default speed ≈ 5 km/h, typical for a delivery robot on sidewalk."""
        return self.total_distance_m / speed_mps
