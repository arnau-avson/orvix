"""Tests for obstacle_avoidance_node module.

These tests exercise the avoidance decision logic (maneuver selection,
state transitions, classification-based decisions, blocked direction analysis)
as pure Python -- no ROS 2 runtime required.

Logic is replicated from the source to allow testing outside the Docker
container (where rclpy is not available), following the same pattern as
test_safety_monitor.py.
"""

import pytest
from enum import Enum, auto


# ---------------------------------------------------------------------------
# Enums (replicated from obstacle_avoidance_node.py)
# ---------------------------------------------------------------------------

class AvoidanceState(Enum):
    CLEAR = auto()
    MONITORING = auto()
    AVOIDING = auto()
    RESUMING = auto()


class AvoidanceManeuver(Enum):
    NONE = auto()
    EMERGENCY_STOP = auto()
    CLIMB_OVER = auto()
    LATERAL_SLIDE = auto()
    DECELERATE = auto()
    DIAGONAL_ESCAPE = auto()


THREAT_NONE = 0
THREAT_MONITOR = 1
THREAT_CAUTION = 2
THREAT_WARNING = 3
THREAT_CRITICAL = 4
THREAT_EMERGENCY = 5

# Classification-based preferred maneuver (replicated)
CLASSIFICATION_PREFERRED = {
    'building':  AvoidanceManeuver.LATERAL_SLIDE,
    'tree':      AvoidanceManeuver.LATERAL_SLIDE,
    'unknown':   None,
    'bird':      AvoidanceManeuver.CLIMB_OVER,
    'animal':    AvoidanceManeuver.CLIMB_OVER,
    'drone':     AvoidanceManeuver.LATERAL_SLIDE,
    'vehicle':   AvoidanceManeuver.CLIMB_OVER,
    'person':    AvoidanceManeuver.CLIMB_OVER,
}


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestAvoidanceStateEnum:
    """Verify the state machine has the expected states."""

    def test_all_states_exist(self):
        states = [AvoidanceState.CLEAR, AvoidanceState.MONITORING,
                  AvoidanceState.AVOIDING, AvoidanceState.RESUMING]
        assert len(states) == 4

    def test_states_are_distinct(self):
        all_states = list(AvoidanceState)
        assert len(all_states) == len(set(all_states))


class TestAvoidanceManeuverEnum:
    """Verify all avoidance maneuvers are available."""

    def test_all_maneuvers_exist(self):
        maneuvers = [
            AvoidanceManeuver.NONE,
            AvoidanceManeuver.EMERGENCY_STOP,
            AvoidanceManeuver.CLIMB_OVER,
            AvoidanceManeuver.LATERAL_SLIDE,
            AvoidanceManeuver.DECELERATE,
            AvoidanceManeuver.DIAGONAL_ESCAPE,
        ]
        assert len(maneuvers) == 6


class TestClassificationPreferredManeuver:
    """Test the classification-based maneuver preference table.

    Decision logic:
    - building/tree -> LATERAL_SLIDE (too tall to climb)
    - bird/animal   -> CLIMB_OVER (they fly at similar altitude)
    - drone         -> LATERAL_SLIDE (agile, get out of path)
    - vehicle       -> CLIMB_OVER (ground-bound)
    - person        -> CLIMB_OVER (ground-bound)
    - unknown       -> None (use geometric fallback)
    """

    def test_building_prefers_lateral(self):
        assert CLASSIFICATION_PREFERRED['building'] == AvoidanceManeuver.LATERAL_SLIDE

    def test_tree_prefers_lateral(self):
        assert CLASSIFICATION_PREFERRED['tree'] == AvoidanceManeuver.LATERAL_SLIDE

    def test_bird_prefers_climb(self):
        assert CLASSIFICATION_PREFERRED['bird'] == AvoidanceManeuver.CLIMB_OVER

    def test_animal_prefers_climb(self):
        assert CLASSIFICATION_PREFERRED['animal'] == AvoidanceManeuver.CLIMB_OVER

    def test_drone_prefers_lateral(self):
        assert CLASSIFICATION_PREFERRED['drone'] == AvoidanceManeuver.LATERAL_SLIDE

    def test_vehicle_prefers_climb(self):
        assert CLASSIFICATION_PREFERRED['vehicle'] == AvoidanceManeuver.CLIMB_OVER

    def test_person_prefers_climb(self):
        assert CLASSIFICATION_PREFERRED['person'] == AvoidanceManeuver.CLIMB_OVER

    def test_unknown_returns_none(self):
        assert CLASSIFICATION_PREFERRED['unknown'] is None


class TestGeometricManeuverSelection:
    """Test the geometric fallback maneuver selection logic.

    When classification is unknown, decisions are based on:
    - Obstacle sector (FRONT, LEFT, RIGHT, REAR, etc.)
    - Available escape routes
    - Preferred escape direction
    """

    @staticmethod
    def _select_geometric(sector, approach_vel=0.0,
                          can_climb=True,
                          can_go_left=True, can_go_right=True,
                          preferred_escape='up'):
        """Replicate geometric logic from _select_geometric_maneuver."""
        if sector in ('FRONT', 'FRONT_LEFT', 'FRONT_RIGHT'):
            if preferred_escape == 'up' and can_climb:
                return AvoidanceManeuver.CLIMB_OVER
            if can_go_right and sector != 'FRONT_RIGHT':
                return AvoidanceManeuver.LATERAL_SLIDE
            if can_go_left and sector != 'FRONT_LEFT':
                return AvoidanceManeuver.LATERAL_SLIDE
            if can_climb:
                return AvoidanceManeuver.CLIMB_OVER
            return AvoidanceManeuver.DIAGONAL_ESCAPE

        if sector in ('LEFT', 'RIGHT'):
            if can_climb:
                return AvoidanceManeuver.CLIMB_OVER
            return AvoidanceManeuver.LATERAL_SLIDE

        if sector in ('REAR', 'REAR_LEFT', 'REAR_RIGHT'):
            if approach_vel > 5.0:
                if can_climb:
                    return AvoidanceManeuver.CLIMB_OVER
                return AvoidanceManeuver.LATERAL_SLIDE
            return AvoidanceManeuver.DECELERATE

        if sector == 'TOP':
            can_lateral = can_go_left or can_go_right
            if can_lateral:
                return AvoidanceManeuver.LATERAL_SLIDE
            return AvoidanceManeuver.EMERGENCY_STOP

        if sector == 'BOTTOM':
            can_lateral = can_go_left or can_go_right
            if can_climb:
                return AvoidanceManeuver.CLIMB_OVER
            if can_lateral:
                return AvoidanceManeuver.LATERAL_SLIDE
            return AvoidanceManeuver.EMERGENCY_STOP

        return AvoidanceManeuver.EMERGENCY_STOP

    # -- Frontal obstacles --

    def test_front_preferred_up_climbs(self):
        m = self._select_geometric('FRONT', preferred_escape='up')
        assert m == AvoidanceManeuver.CLIMB_OVER

    def test_front_cannot_climb_slides_right(self):
        m = self._select_geometric('FRONT', can_climb=False)
        assert m == AvoidanceManeuver.LATERAL_SLIDE

    def test_front_right_obstacle_slides_left(self):
        m = self._select_geometric('FRONT_RIGHT', preferred_escape='lateral',
                                    can_go_left=True, can_go_right=True)
        assert m == AvoidanceManeuver.LATERAL_SLIDE

    def test_front_everything_blocked_diagonal(self):
        m = self._select_geometric('FRONT', can_climb=False,
                                    can_go_left=False, can_go_right=False,
                                    preferred_escape='lateral')
        assert m == AvoidanceManeuver.DIAGONAL_ESCAPE

    # -- Side obstacles --

    def test_left_obstacle_climbs(self):
        m = self._select_geometric('LEFT')
        assert m == AvoidanceManeuver.CLIMB_OVER

    def test_right_obstacle_cannot_climb_slides(self):
        m = self._select_geometric('RIGHT', can_climb=False)
        assert m == AvoidanceManeuver.LATERAL_SLIDE

    # -- Rear obstacles --

    def test_rear_slow_approach_decelerates(self):
        m = self._select_geometric('REAR', approach_vel=2.0)
        assert m == AvoidanceManeuver.DECELERATE

    def test_rear_fast_approach_climbs(self):
        m = self._select_geometric('REAR', approach_vel=10.0)
        assert m == AvoidanceManeuver.CLIMB_OVER

    def test_rear_fast_approach_cannot_climb_slides(self):
        m = self._select_geometric('REAR', approach_vel=10.0, can_climb=False)
        assert m == AvoidanceManeuver.LATERAL_SLIDE

    # -- Top obstacles (from above) --

    def test_top_obstacle_slides_laterally(self):
        m = self._select_geometric('TOP')
        assert m == AvoidanceManeuver.LATERAL_SLIDE

    def test_top_obstacle_no_lateral_emergency(self):
        m = self._select_geometric('TOP', can_go_left=False, can_go_right=False)
        assert m == AvoidanceManeuver.EMERGENCY_STOP

    def test_top_obstacle_does_not_climb(self):
        """Even if can_climb is True, TOP obstacle should NOT climb (toward it)."""
        m = self._select_geometric('TOP', can_climb=True)
        assert m != AvoidanceManeuver.CLIMB_OVER

    # -- Bottom obstacles (from below) --

    def test_bottom_obstacle_climbs(self):
        m = self._select_geometric('BOTTOM')
        assert m == AvoidanceManeuver.CLIMB_OVER

    def test_bottom_obstacle_no_climb_slides(self):
        m = self._select_geometric('BOTTOM', can_climb=False)
        assert m == AvoidanceManeuver.LATERAL_SLIDE

    def test_bottom_obstacle_all_blocked_emergency(self):
        m = self._select_geometric('BOTTOM', can_climb=False,
                                    can_go_left=False, can_go_right=False)
        assert m == AvoidanceManeuver.EMERGENCY_STOP


class TestStateTransitions:
    """Test expected state machine transitions.

    Replicates the transition logic from _handle_clear, _handle_monitoring,
    _handle_avoiding, and _handle_resuming.
    """

    @staticmethod
    def _transition_from_clear(max_threat, min_threat=3, decelerate_threat=2):
        state = AvoidanceState.CLEAR
        if max_threat >= min_threat:
            state = AvoidanceState.AVOIDING
        elif max_threat >= decelerate_threat:
            state = AvoidanceState.MONITORING
        return state

    @staticmethod
    def _transition_from_monitoring(max_threat, min_threat=3, decelerate_threat=2):
        state = AvoidanceState.MONITORING
        if max_threat >= min_threat:
            state = AvoidanceState.AVOIDING
        elif max_threat < decelerate_threat:
            state = AvoidanceState.CLEAR
        return state

    def test_clear_to_avoiding_on_high_threat(self):
        assert self._transition_from_clear(3) == AvoidanceState.AVOIDING

    def test_clear_to_monitoring_on_caution(self):
        assert self._transition_from_clear(2) == AvoidanceState.MONITORING

    def test_clear_stays_clear_on_low_threat(self):
        assert self._transition_from_clear(1) == AvoidanceState.CLEAR

    def test_clear_stays_clear_on_no_threat(self):
        assert self._transition_from_clear(0) == AvoidanceState.CLEAR

    def test_monitoring_to_avoiding_on_high_threat(self):
        assert self._transition_from_monitoring(4) == AvoidanceState.AVOIDING

    def test_monitoring_to_clear_on_low_threat(self):
        assert self._transition_from_monitoring(1) == AvoidanceState.CLEAR

    def test_monitoring_stays_on_caution(self):
        assert self._transition_from_monitoring(2) == AvoidanceState.MONITORING

    def test_avoiding_timeout_to_resuming(self):
        state = AvoidanceState.AVOIDING
        elapsed = 16.0
        max_dur = 15.0
        if elapsed > max_dur:
            state = AvoidanceState.RESUMING
        assert state == AvoidanceState.RESUMING

    def test_resuming_to_clear_after_delay(self):
        state = AvoidanceState.RESUMING
        elapsed = 3.0
        resume_delay = 2.0
        max_threat = 0
        min_threat = 3
        if max_threat < min_threat and elapsed >= resume_delay:
            state = AvoidanceState.CLEAR
        assert state == AvoidanceState.CLEAR

    def test_resuming_back_to_avoiding_on_new_threat(self):
        state = AvoidanceState.RESUMING
        max_threat = 4
        min_threat = 3
        if max_threat >= min_threat:
            state = AvoidanceState.AVOIDING
        assert state == AvoidanceState.AVOIDING

    def test_resuming_stays_if_still_waiting(self):
        state = AvoidanceState.RESUMING
        elapsed = 1.0
        resume_delay = 2.0
        max_threat = 0
        min_threat = 3
        if max_threat >= min_threat:
            state = AvoidanceState.AVOIDING
        elif elapsed >= resume_delay:
            state = AvoidanceState.CLEAR
        assert state == AvoidanceState.RESUMING


class TestSlideDirectionLogic:
    """Test the lateral slide direction determination.

    Slide direction = -1 if bearing >= 0 (obstacle right, slide left)
    Slide direction = +1 if bearing < 0 (obstacle left, slide right)
    """

    @staticmethod
    def _get_slide_direction(bearing):
        return -1.0 if bearing >= 0 else 1.0

    def test_obstacle_right_slide_left(self):
        assert self._get_slide_direction(45.0) == -1.0

    def test_obstacle_left_slide_right(self):
        assert self._get_slide_direction(-45.0) == 1.0

    def test_obstacle_front_center_slides_left(self):
        assert self._get_slide_direction(0.0) == -1.0

    def test_obstacle_directly_behind_slides_left(self):
        assert self._get_slide_direction(180.0) == -1.0


class TestClassificationAwareDecision:
    """Integration test: verify that classification overrides geometric fallback.

    The full decision flow:
    1. Look up classification in CLASSIFICATION_PREFERRED
    2. If a preference exists AND is available, use it
    3. Otherwise fall to geometric fallback
    """

    @staticmethod
    def _decide(classification, sector, can_climb=True, can_lateral=True):
        """Simplified decision replicating _select_maneuver logic."""
        preferred = CLASSIFICATION_PREFERRED.get(classification)

        if preferred is not None:
            # Override preference for vertical sectors
            if sector == 'TOP' and preferred == AvoidanceManeuver.CLIMB_OVER:
                preferred = AvoidanceManeuver.LATERAL_SLIDE
            elif sector == 'BOTTOM' and preferred == AvoidanceManeuver.LATERAL_SLIDE:
                preferred = AvoidanceManeuver.CLIMB_OVER

            if preferred == AvoidanceManeuver.CLIMB_OVER and can_climb:
                return AvoidanceManeuver.CLIMB_OVER
            elif preferred == AvoidanceManeuver.LATERAL_SLIDE and can_lateral:
                return AvoidanceManeuver.LATERAL_SLIDE
            elif can_climb and can_lateral:
                return AvoidanceManeuver.DIAGONAL_ESCAPE
            elif can_climb:
                return AvoidanceManeuver.CLIMB_OVER
            elif can_lateral:
                return AvoidanceManeuver.LATERAL_SLIDE
            else:
                return AvoidanceManeuver.EMERGENCY_STOP
        else:
            # Geometric fallback (simplified)
            return AvoidanceManeuver.CLIMB_OVER  # Default

    def test_bird_from_front_climbs(self):
        assert self._decide('bird', 'FRONT') == AvoidanceManeuver.CLIMB_OVER

    def test_building_from_front_slides(self):
        assert self._decide('building', 'FRONT') == AvoidanceManeuver.LATERAL_SLIDE

    def test_drone_from_side_slides(self):
        assert self._decide('drone', 'RIGHT') == AvoidanceManeuver.LATERAL_SLIDE

    def test_vehicle_from_front_climbs(self):
        assert self._decide('vehicle', 'FRONT') == AvoidanceManeuver.CLIMB_OVER

    def test_bird_cannot_climb_slides(self):
        """Bird prefers climb, but if climb blocked, should slide."""
        assert self._decide('bird', 'FRONT', can_climb=False) == AvoidanceManeuver.LATERAL_SLIDE

    def test_building_cannot_lateral_climbs(self):
        """Building prefers lateral, but if blocked, should climb."""
        assert self._decide('building', 'FRONT', can_lateral=False) == AvoidanceManeuver.CLIMB_OVER

    def test_everything_blocked_emergency(self):
        """If both climb and lateral are blocked, emergency stop."""
        assert self._decide('bird', 'FRONT', can_climb=False, can_lateral=False) == \
               AvoidanceManeuver.EMERGENCY_STOP

    def test_unknown_falls_to_geometric(self):
        """Unknown classification should use geometric fallback."""
        result = self._decide('unknown', 'FRONT')
        assert result == AvoidanceManeuver.CLIMB_OVER  # Default geometric

    # -- TOP/BOTTOM classification overrides --

    def test_bird_from_top_slides(self):
        """Bird normally prefers CLIMB, but from TOP should slide (don't climb toward it)."""
        assert self._decide('bird', 'TOP') == AvoidanceManeuver.LATERAL_SLIDE

    def test_vehicle_from_top_slides(self):
        """Vehicle normally prefers CLIMB, but from TOP should slide."""
        assert self._decide('vehicle', 'TOP') == AvoidanceManeuver.LATERAL_SLIDE

    def test_bird_from_bottom_climbs(self):
        """Bird from below: climb away."""
        assert self._decide('bird', 'BOTTOM') == AvoidanceManeuver.CLIMB_OVER

    def test_drone_from_bottom_climbs(self):
        """Drone normally prefers LATERAL, but from BOTTOM should climb away."""
        assert self._decide('drone', 'BOTTOM') == AvoidanceManeuver.CLIMB_OVER

    def test_building_from_top_slides(self):
        """Building from above: slide (already prefers lateral, no change)."""
        assert self._decide('building', 'TOP') == AvoidanceManeuver.LATERAL_SLIDE
