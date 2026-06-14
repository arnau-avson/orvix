"""Mission state machine state and event definitions."""

from enum import Enum


class MissionState(Enum):
    """All possible states of the mission controller."""
    IDLE = "IDLE"
    PREFLIGHT = "PREFLIGHT"
    TAKEOFF = "TAKEOFF"
    NAVIGATE = "NAVIGATE"
    SEARCH = "SEARCH"
    TRACK = "TRACK"
    RTL = "RTL"
    LAND = "LAND"
    EMERGENCY = "EMERGENCY"
    DONE = "DONE"


class MissionEvent(Enum):
    """All possible events that trigger state transitions."""
    CMD_START = "CMD_START"
    PREFLIGHT_OK = "PREFLIGHT_OK"
    PREFLIGHT_FAIL = "PREFLIGHT_FAIL"
    TAKEOFF_COMPLETE = "TAKEOFF_COMPLETE"
    WAYPOINT_REACHED = "WAYPOINT_REACHED"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    TARGET_DETECTED = "TARGET_DETECTED"
    TARGET_LOCKED = "TARGET_LOCKED"
    TARGET_LOST = "TARGET_LOST"
    SEARCH_TIMEOUT = "SEARCH_TIMEOUT"
    SAFETY_WARNING = "SAFETY_WARNING"
    SAFETY_CRITICAL = "SAFETY_CRITICAL"
    CMD_RTL = "CMD_RTL"
    CMD_LAND = "CMD_LAND"
    CMD_ABORT = "CMD_ABORT"
    LANDED = "LANDED"
    HOME_REACHED = "HOME_REACHED"


class MissionType(Enum):
    """Types of missions the controller can execute."""
    NAVIGATE_ONLY = "navigate_only"
    TRACK_ONLY = "track_only"
    NAVIGATE_AND_TRACK = "navigate_and_track"
