"""RecoveryMonitor tests — escalation thresholds and warnings."""
import pytest

from delivery_robot.localization.models import Pose
from delivery_robot.localization.tracker import TrackerState
from delivery_robot.models import Point
from delivery_robot.navigation import (
    NavigationAction,
    NavigationDecision,
    NavigationState,
    RecoveryMonitor,
    RecoveryPolicy,
)


def _pose(t: float):
    return Pose(point=Point(0.0, 0.0), heading_deg=0, speed_mps=0,
                accuracy_m=2.0, timestamp_s=t)


def _tracker_state(t: float):
    return TrackerState(
        pose=_pose(t),
        progress_m=10.0, remaining_m=90.0,
        off_route_distance_m=0.0, is_off_route=False,
        nearest_segment_index=0, approaching_lights=[],
    )


def _decision(state: NavigationState, t: float, action=NavigationAction.WAIT):
    return NavigationDecision(
        state=state,
        action=action,
        reason="test",
        tracker=_tracker_state(t),
    )


class TestRecoveryMonitor:
    def test_no_action_under_threshold(self):
        mon = RecoveryMonitor(RecoveryPolicy(obstacle_warn_after_s=60.0))
        d = mon.review(_decision(NavigationState.STOPPED_FOR_OBSTACLE, 0.0), 0.0)
        assert d.recovery_warning is None
        assert d.state == NavigationState.STOPPED_FOR_OBSTACLE

        d = mon.review(_decision(NavigationState.STOPPED_FOR_OBSTACLE, 30.0), 30.0)
        assert d.recovery_warning is None

    def test_warning_at_threshold(self):
        mon = RecoveryMonitor(RecoveryPolicy(
            obstacle_warn_after_s=60.0, obstacle_escalate_after_s=300.0,
        ))
        mon.review(_decision(NavigationState.STOPPED_FOR_OBSTACLE, 0.0), 0.0)
        d = mon.review(_decision(NavigationState.STOPPED_FOR_OBSTACLE, 65.0), 65.0)
        assert d.recovery_warning is not None
        assert d.state == NavigationState.STOPPED_FOR_OBSTACLE  # not escalated yet

    def test_escalation_to_error(self):
        mon = RecoveryMonitor(RecoveryPolicy(
            obstacle_warn_after_s=60.0, obstacle_escalate_after_s=120.0,
        ))
        mon.review(_decision(NavigationState.STOPPED_FOR_OBSTACLE, 0.0), 0.0)
        d = mon.review(_decision(NavigationState.STOPPED_FOR_OBSTACLE, 130.0), 130.0)
        assert d.state == NavigationState.ERROR
        assert d.action == NavigationAction.STOP
        assert "Recovery timeout" in d.reason

    def test_state_change_resets_clock(self):
        mon = RecoveryMonitor(RecoveryPolicy(
            obstacle_warn_after_s=10.0, obstacle_escalate_after_s=20.0,
        ))
        mon.review(_decision(NavigationState.STOPPED_FOR_OBSTACLE, 0.0), 0.0)
        mon.review(_decision(NavigationState.WALKING, 5.0, action=NavigationAction.GO), 5.0)
        # Back to obstacle — clock starts over
        d = mon.review(_decision(NavigationState.STOPPED_FOR_OBSTACLE, 12.0), 12.0)
        assert d.recovery_warning is None
        assert d.state == NavigationState.STOPPED_FOR_OBSTACLE

    def test_walking_never_escalates(self):
        mon = RecoveryMonitor(RecoveryPolicy())
        d = mon.review(_decision(NavigationState.WALKING, 0.0,
                                 action=NavigationAction.GO), 0.0)
        d = mon.review(_decision(NavigationState.WALKING, 10_000.0,
                                 action=NavigationAction.GO), 10_000.0)
        assert d.state == NavigationState.WALKING

    def test_unknown_light_escalates_fast(self):
        mon = RecoveryMonitor(RecoveryPolicy(
            approaching_unknown_warn_after_s=15.0,
            approaching_unknown_escalate_after_s=45.0,
        ))
        mon.review(_decision(NavigationState.APPROACHING_CROSSING, 0.0), 0.0)
        d = mon.review(_decision(NavigationState.APPROACHING_CROSSING, 50.0), 50.0)
        assert d.state == NavigationState.ERROR

    def test_red_light_escalates_after_long_wait(self):
        mon = RecoveryMonitor(RecoveryPolicy(
            waiting_red_warn_after_s=90.0,
            waiting_red_escalate_after_s=240.0,
        ))
        mon.review(_decision(NavigationState.WAITING_AT_CROSSING, 0.0), 0.0)
        d = mon.review(_decision(NavigationState.WAITING_AT_CROSSING, 250.0), 250.0)
        assert d.state == NavigationState.ERROR
