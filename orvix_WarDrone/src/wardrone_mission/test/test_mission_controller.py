"""Tests for mission_controller_node module.

These tests exercise the MissionState, MissionEvent, and MissionType enums
from wardrone_mission.states as pure Python -- no ROS 2 runtime required.
"""

import pytest


class TestMissionControllerImport:
    def test_import(self):
        from wardrone_mission.mission_controller_node import MissionControllerNode
        assert MissionControllerNode is not None

    def test_import_states(self):
        from wardrone_mission.states import MissionState, MissionEvent, MissionType
        assert MissionState.IDLE.value == "IDLE"
        assert MissionEvent.CMD_START.value == "CMD_START"
        assert MissionType.NAVIGATE_AND_TRACK.value == "navigate_and_track"


# ---------------------------------------------------------------------------
# MissionState enum tests
# ---------------------------------------------------------------------------

class TestMissionState:
    """Comprehensive tests for the MissionState enum."""

    EXPECTED_STATES = [
        'IDLE', 'PREFLIGHT', 'TAKEOFF', 'NAVIGATE', 'SEARCH',
        'TRACK', 'RTL', 'LAND', 'EMERGENCY', 'DONE',
    ]

    def test_all_states_exist(self):
        from wardrone_mission.states import MissionState
        for name in self.EXPECTED_STATES:
            assert hasattr(MissionState, name), f"MissionState.{name} is missing"

    def test_state_count(self):
        from wardrone_mission.states import MissionState
        assert len(MissionState) == 10

    def test_state_values_match_names(self):
        """Each MissionState value must equal its member name."""
        from wardrone_mission.states import MissionState
        for state in MissionState:
            assert state.value == state.name, (
                f"MissionState.{state.name}.value is '{state.value}', expected '{state.name}'"
            )

    @pytest.mark.parametrize("name,expected_value", [
        ("IDLE", "IDLE"),
        ("PREFLIGHT", "PREFLIGHT"),
        ("TAKEOFF", "TAKEOFF"),
        ("NAVIGATE", "NAVIGATE"),
        ("SEARCH", "SEARCH"),
        ("TRACK", "TRACK"),
        ("RTL", "RTL"),
        ("LAND", "LAND"),
        ("EMERGENCY", "EMERGENCY"),
        ("DONE", "DONE"),
    ])
    def test_individual_state_value(self, name, expected_value):
        from wardrone_mission.states import MissionState
        assert MissionState[name].value == expected_value

    def test_str_representation(self):
        """String representation must contain the state name."""
        from wardrone_mission.states import MissionState
        for state in MissionState:
            s = str(state)
            assert state.name in s, f"str({state}) = '{s}' does not contain '{state.name}'"

    def test_repr_representation(self):
        """repr must contain both the class name and the member name."""
        from wardrone_mission.states import MissionState
        for state in MissionState:
            r = repr(state)
            assert 'MissionState' in r
            assert state.name in r

    def test_lookup_by_value(self):
        """States must be constructible from their string value."""
        from wardrone_mission.states import MissionState
        for name in self.EXPECTED_STATES:
            assert MissionState(name) == MissionState[name]

    def test_invalid_state_raises(self):
        from wardrone_mission.states import MissionState
        with pytest.raises(ValueError):
            MissionState("NONEXISTENT")

    def test_states_are_unique(self):
        """All state values must be unique."""
        from wardrone_mission.states import MissionState
        values = [s.value for s in MissionState]
        assert len(values) == len(set(values))

    def test_equality_and_identity(self):
        from wardrone_mission.states import MissionState
        assert MissionState.IDLE == MissionState.IDLE
        assert MissionState.IDLE is MissionState.IDLE
        assert MissionState.IDLE != MissionState.TAKEOFF

    def test_state_is_hashable(self):
        """Enum members must be usable as dict keys and in sets."""
        from wardrone_mission.states import MissionState
        d = {MissionState.IDLE: "idle", MissionState.TAKEOFF: "takeoff"}
        assert d[MissionState.IDLE] == "idle"
        s = {MissionState.IDLE, MissionState.IDLE, MissionState.DONE}
        assert len(s) == 2

    def test_iteration_order(self):
        """Iteration order must match declaration order."""
        from wardrone_mission.states import MissionState
        names = [s.name for s in MissionState]
        assert names == self.EXPECTED_STATES


# ---------------------------------------------------------------------------
# MissionEvent enum tests
# ---------------------------------------------------------------------------

class TestMissionEvent:
    """Comprehensive tests for the MissionEvent enum."""

    EXPECTED_EVENTS = [
        'CMD_START', 'PREFLIGHT_OK', 'PREFLIGHT_FAIL',
        'TAKEOFF_COMPLETE', 'WAYPOINT_REACHED', 'MISSION_COMPLETE',
        'TARGET_DETECTED', 'TARGET_LOCKED', 'TARGET_LOST',
        'SEARCH_TIMEOUT', 'SAFETY_WARNING', 'SAFETY_CRITICAL',
        'CMD_RTL', 'CMD_LAND', 'CMD_ABORT',
        'LANDED', 'HOME_REACHED',
    ]

    def test_all_events_exist(self):
        from wardrone_mission.states import MissionEvent
        for name in self.EXPECTED_EVENTS:
            assert hasattr(MissionEvent, name), f"MissionEvent.{name} is missing"

    def test_event_count(self):
        from wardrone_mission.states import MissionEvent
        assert len(MissionEvent) == 17

    def test_event_values_match_names(self):
        from wardrone_mission.states import MissionEvent
        for event in MissionEvent:
            assert event.value == event.name

    @pytest.mark.parametrize("name", [
        "CMD_START", "PREFLIGHT_OK", "PREFLIGHT_FAIL",
        "TAKEOFF_COMPLETE", "WAYPOINT_REACHED", "MISSION_COMPLETE",
        "TARGET_DETECTED", "TARGET_LOCKED", "TARGET_LOST",
        "SEARCH_TIMEOUT", "SAFETY_WARNING", "SAFETY_CRITICAL",
        "CMD_RTL", "CMD_LAND", "CMD_ABORT",
        "LANDED", "HOME_REACHED",
    ])
    def test_individual_event_value(self, name):
        from wardrone_mission.states import MissionEvent
        assert MissionEvent[name].value == name

    def test_str_representation(self):
        from wardrone_mission.states import MissionEvent
        for event in MissionEvent:
            assert event.name in str(event)

    def test_lookup_by_value(self):
        from wardrone_mission.states import MissionEvent
        for name in self.EXPECTED_EVENTS:
            assert MissionEvent(name) == MissionEvent[name]

    def test_invalid_event_raises(self):
        from wardrone_mission.states import MissionEvent
        with pytest.raises(ValueError):
            MissionEvent("NONEXISTENT")

    def test_events_are_unique(self):
        from wardrone_mission.states import MissionEvent
        values = [e.value for e in MissionEvent]
        assert len(values) == len(set(values))

    def test_command_events_subset(self):
        """Verify the command-type events form a recognizable subset."""
        from wardrone_mission.states import MissionEvent
        cmd_events = [e for e in MissionEvent if e.name.startswith('CMD_')]
        cmd_names = {e.name for e in cmd_events}
        assert cmd_names == {'CMD_START', 'CMD_RTL', 'CMD_LAND', 'CMD_ABORT'}

    def test_safety_events_subset(self):
        """Verify the safety-type events form a recognizable subset."""
        from wardrone_mission.states import MissionEvent
        safety_events = [e for e in MissionEvent if e.name.startswith('SAFETY_')]
        safety_names = {e.name for e in safety_events}
        assert safety_names == {'SAFETY_WARNING', 'SAFETY_CRITICAL'}

    def test_iteration_order(self):
        from wardrone_mission.states import MissionEvent
        names = [e.name for e in MissionEvent]
        assert names == self.EXPECTED_EVENTS


# ---------------------------------------------------------------------------
# MissionType enum tests
# ---------------------------------------------------------------------------

class TestMissionType:
    """Comprehensive tests for the MissionType enum."""

    EXPECTED_TYPES = [
        ('NAVIGATE_ONLY', 'navigate_only'),
        ('TRACK_ONLY', 'track_only'),
        ('NAVIGATE_AND_TRACK', 'navigate_and_track'),
    ]

    def test_all_types_exist(self):
        from wardrone_mission.states import MissionType
        for name, _ in self.EXPECTED_TYPES:
            assert hasattr(MissionType, name), f"MissionType.{name} is missing"

    def test_type_count(self):
        from wardrone_mission.states import MissionType
        assert len(MissionType) == 3

    @pytest.mark.parametrize("name,expected_value", [
        ("NAVIGATE_ONLY", "navigate_only"),
        ("TRACK_ONLY", "track_only"),
        ("NAVIGATE_AND_TRACK", "navigate_and_track"),
    ])
    def test_individual_type_value(self, name, expected_value):
        from wardrone_mission.states import MissionType
        assert MissionType[name].value == expected_value

    def test_type_values_are_lowercase(self):
        """MissionType values must be lowercase snake_case (unlike State/Event)."""
        from wardrone_mission.states import MissionType
        for mt in MissionType:
            assert mt.value == mt.value.lower(), f"{mt.name}.value is not lowercase"
            assert '_' in mt.value, f"{mt.name}.value has no underscore"

    def test_str_representation(self):
        from wardrone_mission.states import MissionType
        for mt in MissionType:
            assert mt.name in str(mt)

    def test_lookup_by_value(self):
        from wardrone_mission.states import MissionType
        for name, value in self.EXPECTED_TYPES:
            assert MissionType(value) == MissionType[name]

    def test_invalid_type_raises(self):
        from wardrone_mission.states import MissionType
        with pytest.raises(ValueError):
            MissionType("nonexistent")

    def test_types_are_unique(self):
        from wardrone_mission.states import MissionType
        values = [t.value for t in MissionType]
        assert len(values) == len(set(values))

    def test_iteration_order(self):
        from wardrone_mission.states import MissionType
        names = [t.name for t in MissionType]
        assert names == [n for n, _ in self.EXPECTED_TYPES]

    def test_types_are_distinct_from_states(self):
        """MissionType values must not collide with MissionState values."""
        from wardrone_mission.states import MissionState, MissionType
        state_values = {s.value for s in MissionState}
        type_values = {t.value for t in MissionType}
        assert state_values.isdisjoint(type_values)
