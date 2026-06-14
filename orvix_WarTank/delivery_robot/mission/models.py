from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

from ..models import Point, Route


class MissionStatus(Enum):
    PENDING = "pending"          # Mission accepted, not yet started.
    PLANNING = "planning"        # Geocoding addresses, computing route.
    EN_ROUTE = "en_route"        # Walking the planned route.
    REPLANNING = "replanning"    # Off-route or blocked persistently — recomputing.
    COMPLETED = "completed"      # Reached destination.
    FAILED = "failed"            # Geocoding/routing failed; see failure_reason.


@dataclass
class Mission:
    mission_id: str
    origin_address: str
    destination_address: str
    status: MissionStatus = MissionStatus.PENDING
    origin_point: Optional[Point] = None
    destination_point: Optional[Point] = None
    routes_history: List[Route] = field(default_factory=list)  # Each replan appends.
    failure_reason: Optional[str] = None

    @property
    def current_route(self) -> Optional[Route]:
        return self.routes_history[-1] if self.routes_history else None
