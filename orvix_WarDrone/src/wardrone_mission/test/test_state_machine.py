"""Tests for state_machine module."""

import pytest
from wardrone_mission.states import MissionState, MissionEvent
from wardrone_mission.state_machine import StateMachine, TransitionRecord


class TestStateMachine:
    def _build_basic_sm(self) -> StateMachine:
        sm = StateMachine(initial_state=MissionState.IDLE)
        sm.set_terminal_states(MissionState.DONE)
        sm.add_transition(MissionState.IDLE, MissionEvent.CMD_START, MissionState.PREFLIGHT)
        sm.add_transition(MissionState.PREFLIGHT, MissionEvent.PREFLIGHT_OK, MissionState.TAKEOFF)
        sm.add_transition(MissionState.PREFLIGHT, MissionEvent.PREFLIGHT_FAIL, MissionState.IDLE)
        sm.add_transition(MissionState.TAKEOFF, MissionEvent.TAKEOFF_COMPLETE, MissionState.NAVIGATE)
        sm.add_transition(MissionState.NAVIGATE, MissionEvent.MISSION_COMPLETE, MissionState.RTL)
        sm.add_transition(MissionState.RTL, MissionEvent.HOME_REACHED, MissionState.LAND)
        sm.add_transition(MissionState.LAND, MissionEvent.LANDED, MissionState.DONE)
        return sm

    def test_initial_state(self):
        sm = self._build_basic_sm()
        assert sm.state == MissionState.IDLE

    def test_valid_transition(self):
        sm = self._build_basic_sm()
        result = sm.handle_event(MissionEvent.CMD_START)
        assert result is True
        assert sm.state == MissionState.PREFLIGHT

    def test_invalid_event(self):
        sm = self._build_basic_sm()
        result = sm.handle_event(MissionEvent.TAKEOFF_COMPLETE)
        assert result is False
        assert sm.state == MissionState.IDLE

    def test_full_mission_flow(self):
        sm = self._build_basic_sm()
        sm.handle_event(MissionEvent.CMD_START)
        sm.handle_event(MissionEvent.PREFLIGHT_OK)
        sm.handle_event(MissionEvent.TAKEOFF_COMPLETE)
        sm.handle_event(MissionEvent.MISSION_COMPLETE)
        sm.handle_event(MissionEvent.HOME_REACHED)
        sm.handle_event(MissionEvent.LANDED)
        assert sm.state == MissionState.DONE

    def test_terminal_state_blocks_events(self):
        sm = self._build_basic_sm()
        sm.handle_event(MissionEvent.CMD_START)
        sm.handle_event(MissionEvent.PREFLIGHT_OK)
        sm.handle_event(MissionEvent.TAKEOFF_COMPLETE)
        sm.handle_event(MissionEvent.MISSION_COMPLETE)
        sm.handle_event(MissionEvent.HOME_REACHED)
        sm.handle_event(MissionEvent.LANDED)
        assert sm.state == MissionState.DONE
        result = sm.handle_event(MissionEvent.CMD_START)
        assert result is False
        assert sm.state == MissionState.DONE

    def test_preflight_fail_returns_to_idle(self):
        sm = self._build_basic_sm()
        sm.handle_event(MissionEvent.CMD_START)
        sm.handle_event(MissionEvent.PREFLIGHT_FAIL)
        assert sm.state == MissionState.IDLE

    def test_history(self):
        sm = self._build_basic_sm()
        sm.handle_event(MissionEvent.CMD_START, reason="test")
        assert len(sm.history) == 1
        record = sm.history[0]
        assert record.from_state == MissionState.IDLE
        assert record.event == MissionEvent.CMD_START
        assert record.to_state == MissionState.PREFLIGHT
        assert record.reason == "test"

    def test_guard_condition_blocks(self):
        sm = StateMachine(initial_state=MissionState.IDLE)
        sm.add_transition(
            MissionState.IDLE, MissionEvent.CMD_START, MissionState.PREFLIGHT,
            guard=lambda: False  # Always block
        )
        result = sm.handle_event(MissionEvent.CMD_START)
        assert result is False
        assert sm.state == MissionState.IDLE

    def test_guard_condition_allows(self):
        sm = StateMachine(initial_state=MissionState.IDLE)
        sm.add_transition(
            MissionState.IDLE, MissionEvent.CMD_START, MissionState.PREFLIGHT,
            guard=lambda: True
        )
        result = sm.handle_event(MissionEvent.CMD_START)
        assert result is True
        assert sm.state == MissionState.PREFLIGHT

    def test_transition_action(self):
        actions_called = []
        sm = StateMachine(initial_state=MissionState.IDLE)
        sm.add_transition(
            MissionState.IDLE, MissionEvent.CMD_START, MissionState.PREFLIGHT,
            action=lambda: actions_called.append('start')
        )
        sm.handle_event(MissionEvent.CMD_START)
        assert actions_called == ['start']

    def test_enter_exit_callbacks(self):
        log = []
        sm = StateMachine(initial_state=MissionState.IDLE)
        sm.add_transition(MissionState.IDLE, MissionEvent.CMD_START, MissionState.PREFLIGHT)
        sm.on_exit(MissionState.IDLE, lambda: log.append('exit_idle'))
        sm.on_enter(MissionState.PREFLIGHT, lambda: log.append('enter_preflight'))
        sm.handle_event(MissionEvent.CMD_START)
        assert log == ['exit_idle', 'enter_preflight']

    def test_global_transition(self):
        sm = StateMachine(initial_state=MissionState.IDLE)
        sm.add_transition(MissionState.IDLE, MissionEvent.CMD_START, MissionState.NAVIGATE)
        sm.add_global_transition(
            MissionEvent.SAFETY_CRITICAL, MissionState.EMERGENCY,
            exclude_states={MissionState.DONE, MissionState.EMERGENCY}
        )
        sm.handle_event(MissionEvent.CMD_START)
        assert sm.state == MissionState.NAVIGATE
        sm.handle_event(MissionEvent.SAFETY_CRITICAL)
        assert sm.state == MissionState.EMERGENCY

    def test_global_transition_from_idle(self):
        sm = StateMachine(initial_state=MissionState.IDLE)
        sm.add_global_transition(
            MissionEvent.SAFETY_CRITICAL, MissionState.EMERGENCY,
            exclude_states={MissionState.DONE, MissionState.EMERGENCY}
        )
        sm.handle_event(MissionEvent.SAFETY_CRITICAL)
        assert sm.state == MissionState.EMERGENCY

    def test_reset(self):
        sm = self._build_basic_sm()
        sm.handle_event(MissionEvent.CMD_START)
        sm.handle_event(MissionEvent.PREFLIGHT_OK)
        assert sm.state == MissionState.TAKEOFF
        sm.reset()
        assert sm.state == MissionState.IDLE
        assert len(sm.history) == 0

    def test_get_valid_events(self):
        sm = self._build_basic_sm()
        events = sm.get_valid_events()
        assert MissionEvent.CMD_START in events
        assert MissionEvent.TAKEOFF_COMPLETE not in events

    def test_on_any_transition_callback(self):
        records = []
        sm = self._build_basic_sm()
        sm.on_any_transition(lambda r: records.append(r))
        sm.handle_event(MissionEvent.CMD_START)
        assert len(records) == 1
        assert isinstance(records[0], TransitionRecord)

    def test_is_terminal(self):
        sm = self._build_basic_sm()
        assert sm.is_terminal is False
        # Fast forward to DONE
        sm.handle_event(MissionEvent.CMD_START)
        sm.handle_event(MissionEvent.PREFLIGHT_OK)
        sm.handle_event(MissionEvent.TAKEOFF_COMPLETE)
        sm.handle_event(MissionEvent.MISSION_COMPLETE)
        sm.handle_event(MissionEvent.HOME_REACHED)
        sm.handle_event(MissionEvent.LANDED)
        assert sm.is_terminal is True
