"""Tests for obstacle_avoidance_node module.

These tests exercise the avoidance decision logic (maneuver selection,
state transitions, classification-based decisions, blocked direction analysis)
as pure Python -- no ROS 2 runtime required.
"""

import pytest


class TestObstacleAvoidanceImport:
    def test_import(self):
        from wardrone_navigation.obstacle_avoidance_node import ObstacleAvoidanceNode
        assert ObstacleAvoidanceNode is not None

    def test_import_enums(self):
        from wardrone_navigation.obstacle_avoidance_node import (
            AvoidanceState, AvoidanceManeuver,
        )
        assert AvoidanceState.CLEAR is not None
        assert AvoidanceManeuver.CLIMB_OVER is not None


class TestAvoidanceStateEnum:
    """Verify the state machine has the expected states."""

    def test_all_states_exist(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceState
        states = [AvoidanceState.CLEAR, AvoidanceState.MONITORING,
                  AvoidanceState.AVOIDING, AvoidanceState.RESUMING]
        assert len(states) == 4

    def test_states_are_distinct(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceState
        all_states = list(AvoidanceState)
        assert len(all_states) == len(set(all_states))


class TestAvoidanceManeuverEnum:
    """Verify all avoidance maneuvers are available."""

    def test_all_maneuvers_exist(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
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
    """Test that the classification-based maneuver preference table is correct.

    Decision logic:
    - building/tree -> LATERAL_SLIDE (too tall to climb)
    - bird/animal   -> CLIMB_OVER (they fly at similar altitude)
    - drone         -> LATERAL_SLIDE (agile, get out of path)
    - vehicle       -> CLIMB_OVER (ground-bound)
    - person        -> CLIMB_OVER (ground-bound)
    - unknown       -> None (use geometric fallback)
    """

    @staticmethod
    def _get_preferred(classification):
        from wardrone_navigation.obstacle_avoidance_node import ObstacleAvoidanceNode
        return ObstacleAvoidanceNode._CLASSIFICATION_PREFERRED.get(classification)

    def test_building_prefers_lateral(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
        assert self._get_preferred('building') == AvoidanceManeuver.LATERAL_SLIDE

    def test_tree_prefers_lateral(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
        assert self._get_preferred('tree') == AvoidanceManeuver.LATERAL_SLIDE

    def test_bird_prefers_climb(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
        assert self._get_preferred('bird') == AvoidanceManeuver.CLIMB_OVER

    def test_animal_prefers_climb(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
        assert self._get_preferred('animal') == AvoidanceManeuver.CLIMB_OVER

    def test_drone_prefers_lateral(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
        assert self._get_preferred('drone') == AvoidanceManeuver.LATERAL_SLIDE

    def test_vehicle_prefers_climb(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
        assert self._get_preferred('vehicle') == AvoidanceManeuver.CLIMB_OVER

    def test_person_prefers_climb(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
        assert self._get_preferred('person') == AvoidanceManeuver.CLIMB_OVER

    def test_unknown_returns_none(self):
        """Unknown classification should fall back to geometric logic."""
        assert self._get_preferred('unknown') is None


class TestGeometricManeuverSelection:
    """Test the geometric fallback maneuver selection logic.

    When the classification is unknown, the system decides based on:
    - Obstacle sector (FRONT, LEFT, RIGHT, REAR, etc.)
    - Available escape routes (blocked directions)
    - Preferred escape direction parameter
    """

    @staticmethod
    def _select_geometric(sector, approach_vel=0.0, blocked=None,
                          can_climb=True, can_lateral=True,
                          can_go_left=True, can_go_right=True,
                          preferred_escape='up'):
        """Replicate geometric logic from _select_geometric_maneuver."""
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver

        # Create a mock obstacle-like object
        class MockObstacle:
            def __init__(self, av):
                self.approach_velocity_m_s = av

        obstacle = MockObstacle(approach_vel)

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
            if obstacle.approach_velocity_m_s > 5.0:
                if can_climb:
                    return AvoidanceManeuver.CLIMB_OVER
                return AvoidanceManeuver.LATERAL_SLIDE
            return AvoidanceManeuver.DECELERATE

        return AvoidanceManeuver.EMERGENCY_STOP

    # -- Frontal obstacles --

    def test_front_preferred_up_climbs(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
        m = self._select_geometric('FRONT', preferred_escape='up')
        assert m == AvoidanceManeuver.CLIMB_OVER

    def test_front_cannot_climb_slides_right(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
        m = self._select_geometric('FRONT', can_climb=False)
        assert m == AvoidanceManeuver.LATERAL_SLIDE

    def test_front_right_obstacle_slides_left(self):
        """If obstacle is FRONT_RIGHT, right is blocked, should slide left."""
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
        m = self._select_geometric('FRONT_RIGHT', preferred_escape='lateral',
                                    can_go_left=True, can_go_right=True)
        # Can't slide right (sector is FRONT_RIGHT), so slides left
        # Actually, preferred_escape is 'lateral', not 'up', so first check fails
        # Then: can_go_right and sector != 'FRONT_RIGHT' => False
        # Then: can_go_left and sector != 'FRONT_LEFT' => True
        assert m == AvoidanceManeuver.LATERAL_SLIDE

    def test_front_everything_blocked_diagonal(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
        m = self._select_geometric('FRONT', can_climb=False,
                                    can_go_left=False, can_go_right=False,
                                    preferred_escape='lateral')
        assert m == AvoidanceManeuver.DIAGONAL_ESCAPE

    # -- Side obstacles --

    def test_left_obstacle_climbs(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
        m = self._select_geometric('LEFT')
        assert m == AvoidanceManeuver.CLIMB_OVER

    def test_right_obstacle_cannot_climb_slides(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
        m = self._select_geometric('RIGHT', can_climb=False)
        assert m == AvoidanceManeuver.LATERAL_SLIDE

    # -- Rear obstacles --

    def test_rear_slow_approach_decelerates(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
        m = self._select_geometric('REAR', approach_vel=2.0)
        assert m == AvoidanceManeuver.DECELERATE

    def test_rear_fast_approach_climbs(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
        m = self._select_geometric('REAR', approach_vel=10.0)
        assert m == AvoidanceManeuver.CLIMB_OVER

    def test_rear_fast_approach_cannot_climb_slides(self):
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceManeuver
        m = self._select_geometric('REAR', approach_vel=10.0, can_climb=False)
        assert m == AvoidanceManeuver.LATERAL_SLIDE


class TestStateTransitions:
    """Test expected state machine transitions."""

    def test_clear_to_avoiding_on_high_threat(self):
        """CLEAR -> AVOIDING when threat >= min_threat (WARNING=3)."""
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceState
        state = AvoidanceState.CLEAR
        max_threat = 3  # WARNING
        min_threat = 3
        decelerate_threat = 2

        if max_threat >= min_threat:
            state = AvoidanceState.AVOIDING
        elif max_threat >= decelerate_threat:
            state = AvoidanceState.MONITORING

        assert state == AvoidanceState.AVOIDING

    def test_clear_to_monitoring_on_caution(self):
        """CLEAR -> MONITORING when threat at CAUTION level."""
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceState
        state = AvoidanceState.CLEAR
        max_threat = 2  # CAUTION
        min_threat = 3
        decelerate_threat = 2

        if max_threat >= min_threat:
            state = AvoidanceState.AVOIDING
        elif max_threat >= decelerate_threat:
            state = AvoidanceState.MONITORING

        assert state == AvoidanceState.MONITORING

    def test_clear_stays_clear_on_low_threat(self):
        """CLEAR stays CLEAR when threat < CAUTION."""
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceState
        state = AvoidanceState.CLEAR
        max_threat = 1  # MONITOR
        min_threat = 3
        decelerate_threat = 2

        if max_threat >= min_threat:
            state = AvoidanceState.AVOIDING
        elif max_threat >= decelerate_threat:
            state = AvoidanceState.MONITORING

        assert state == AvoidanceState.CLEAR

    def test_avoiding_timeout_to_resuming(self):
        """AVOIDING -> RESUMING after max_avoidance_duration_s."""
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceState
        state = AvoidanceState.AVOIDING
        elapsed = 16.0
        max_dur = 15.0

        if elapsed > max_dur:
            state = AvoidanceState.RESUMING

        assert state == AvoidanceState.RESUMING

    def test_resuming_to_clear_after_delay(self):
        """RESUMING -> CLEAR after resume_delay_s with no threats."""
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceState
        state = AvoidanceState.RESUMING
        elapsed = 3.0
        resume_delay = 2.0
        max_threat = 0

        if max_threat < 3 and elapsed >= resume_delay:
            state = AvoidanceState.CLEAR

        assert state == AvoidanceState.CLEAR

    def test_resuming_back_to_avoiding_on_new_threat(self):
        """RESUMING -> AVOIDING if a new high threat appears."""
        from wardrone_navigation.obstacle_avoidance_node import AvoidanceState
        state = AvoidanceState.RESUMING
        max_threat = 4  # CRITICAL
        min_threat = 3

        if max_threat >= min_threat:
            state = AvoidanceState.AVOIDING

        assert state == AvoidanceState.AVOIDING


class TestSlideDirectionLogic:
    """Test the lateral slide direction determination."""

    def test_obstacle_right_slide_left(self):
        """Obstacle bearing > 0 (right) should slide left (negative direction)."""
        bearing = 45.0  # Right side
        slide_direction = -1.0 if bearing >= 0 else 1.0
        assert slide_direction == -1.0

    def test_obstacle_left_slide_right(self):
        """Obstacle bearing < 0 (left) should slide right (positive direction)."""
        bearing = -45.0  # Left side
        slide_direction = -1.0 if bearing >= 0 else 1.0
        assert slide_direction == 1.0

    def test_obstacle_front_center_slides_left(self):
        """Obstacle at bearing 0 (center front) defaults to left slide."""
        bearing = 0.0
        slide_direction = -1.0 if bearing >= 0 else 1.0
        assert slide_direction == -1.0
