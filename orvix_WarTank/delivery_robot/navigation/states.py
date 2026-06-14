from enum import Enum


class NavigationState(Enum):
    """Where the robot is in its navigation lifecycle."""
    IDLE = "idle"
    WALKING = "walking"
    APPROACHING_CROSSING = "approaching_crossing"
    WAITING_AT_CROSSING = "waiting_at_crossing"
    CROSSING = "crossing"
    STOPPED_FOR_OBSTACLE = "stopped_for_obstacle"
    OFF_ROUTE = "off_route"
    ARRIVED = "arrived"
    ERROR = "error"


class NavigationAction(Enum):
    """The intent emitted to the (future) motion controller."""
    GO = "go"      # Continue forward at planned speed.
    WAIT = "wait"  # Hold position, keep observing (transient stop).
    STOP = "stop"  # Halt and surface a condition to the supervisor.
