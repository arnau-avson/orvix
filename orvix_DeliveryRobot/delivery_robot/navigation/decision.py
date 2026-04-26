from dataclasses import dataclass
from typing import Optional

from ..localization.tracker import TrackerState
from ..perception.obstacles import Obstacle
from ..traffic_lights import TrafficLight
from .states import NavigationAction, NavigationState


@dataclass
class NavigationDecision:
    """One tick of the orchestrator: state, action, and the context that
    justified them. The supervisor / motion layer consumes `action`; the
    operator-facing UI consumes `state` + `reason` for display.
    """
    state: NavigationState
    action: NavigationAction
    reason: str
    tracker: TrackerState
    blocker: Optional[Obstacle] = None
    light: Optional[TrafficLight] = None
    light_state: Optional[str] = None

    @property
    def is_terminal(self) -> bool:
        return self.state in (NavigationState.ARRIVED, NavigationState.ERROR)
