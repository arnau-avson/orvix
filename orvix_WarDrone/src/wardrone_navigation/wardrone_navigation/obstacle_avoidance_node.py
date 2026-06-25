"""Obstacle avoidance node -- reactive obstacle evasion.

Receives obstacle detections from the obstacle detector and generates
avoidance commands that override normal waypoint navigation.

Avoidance strategies:
1. EMERGENCY_STOP  -- Immediate brake (hover), obstacle too close
2. CLIMB_OVER      -- Ascend to clear the obstacle, then resume altitude
3. LATERAL_SLIDE   -- Move perpendicular to the obstacle bearing
4. DECELERATE      -- Slow down while monitoring the situation
5. DIAGONAL_ESCAPE -- Combined climb + lateral when cornered

Priority system:
- The avoidance node publishes on /wardrone/cmd_velocity with a flag
  on /wardrone/obstacle_avoidance/active so the waypoint navigator
  knows to pause its position commands.
- The safety monitor can also listen to /wardrone/safety/event for
  OBSTACLE_* events.

State machine:
    CLEAR -> MONITORING -> AVOIDING -> RESUMING -> CLEAR
"""

import math
import time
from enum import Enum, auto
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from wardrone_interfaces.msg import Obstacle, ObstacleArray, Telemetry


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


# Threat level constants (mirror obstacle_detector_node)
THREAT_NONE = 0
THREAT_MONITOR = 1
THREAT_CAUTION = 2
THREAT_WARNING = 3
THREAT_CRITICAL = 4
THREAT_EMERGENCY = 5


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # --- Parameters ---
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('avoidance_speed_m_s', 2.0)
        self.declare_parameter('climb_speed_m_s', 2.5)
        self.declare_parameter('lateral_speed_m_s', 2.0)
        self.declare_parameter('climb_clearance_m', 5.0)
        self.declare_parameter('lateral_clearance_m', 4.0)
        self.declare_parameter('resume_delay_s', 2.0)
        self.declare_parameter('max_avoidance_duration_s', 15.0)
        self.declare_parameter('min_threat_to_avoid', 3)  # WARNING level
        self.declare_parameter('emergency_stop_threat', 5)  # EMERGENCY level
        self.declare_parameter('decelerate_threat', 2)  # CAUTION level
        self.declare_parameter('preferred_escape_direction', 'up')  # 'up', 'left', 'right'
        self.declare_parameter('max_altitude_m', 120.0)
        self.declare_parameter('min_altitude_m', 3.0)

        self._control_rate = self.get_parameter('control_rate_hz').value
        self._avoidance_speed = self.get_parameter('avoidance_speed_m_s').value
        self._climb_speed = self.get_parameter('climb_speed_m_s').value
        self._lateral_speed = self.get_parameter('lateral_speed_m_s').value
        self._climb_clearance = self.get_parameter('climb_clearance_m').value
        self._lateral_clearance = self.get_parameter('lateral_clearance_m').value
        self._resume_delay = self.get_parameter('resume_delay_s').value
        self._max_avoidance_dur = self.get_parameter('max_avoidance_duration_s').value
        self._min_threat = self.get_parameter('min_threat_to_avoid').value
        self._emergency_threat = self.get_parameter('emergency_stop_threat').value
        self._decelerate_threat = self.get_parameter('decelerate_threat').value
        self._preferred_escape = self.get_parameter('preferred_escape_direction').value
        self._max_alt = self.get_parameter('max_altitude_m').value
        self._min_alt = self.get_parameter('min_altitude_m').value

        # --- State ---
        self._state = AvoidanceState.CLEAR
        self._current_maneuver = AvoidanceManeuver.NONE
        self._latest_obstacles: List[Obstacle] = []
        self._avoidance_start_time = 0.0
        self._resume_start_time = 0.0
        self._maneuver_distance_traveled = 0.0
        self._maneuver_start_alt = 0.0
        self._is_armed = False
        self._is_in_air = False
        self._current_alt = 0.0
        self._current_yaw_deg = 0.0
        self._last_obstacle_time = 0.0

        cb_group = ReentrantCallbackGroup()

        # --- Publishers ---
        self._pub_cmd_vel = self.create_publisher(
            Twist, '/wardrone/cmd_velocity', 10
        )
        self._pub_active = self.create_publisher(
            Bool, '/wardrone/obstacle_avoidance/active', 10
        )
        self._pub_event = self.create_publisher(
            String, '/wardrone/safety/event', 10
        )
        self._pub_state = self.create_publisher(
            String, '/wardrone/obstacle_avoidance/state', 10
        )

        # --- Subscribers ---
        self.create_subscription(
            ObstacleArray, '/wardrone/obstacles',
            self._on_obstacles, 10,
            callback_group=cb_group,
        )
        self.create_subscription(
            Telemetry, '/wardrone/telemetry',
            self._on_telemetry, 10,
        )

        # --- Control loop ---
        self._control_timer = self.create_timer(
            1.0 / self._control_rate, self._control_tick
        )

        self.get_logger().info(
            f'Obstacle Avoidance ready: rate={self._control_rate}Hz, '
            f'min_threat={self._min_threat}, preferred_escape={self._preferred_escape}'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_telemetry(self, msg: Telemetry):
        self._is_armed = msg.is_armed
        self._is_in_air = msg.is_in_air
        self._current_alt = msg.relative_altitude_m
        self._current_yaw_deg = msg.yaw_deg

    def _on_obstacles(self, msg: ObstacleArray):
        self._latest_obstacles = list(msg.obstacles)
        if msg.obstacles:
            self._last_obstacle_time = time.time()

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------

    def _control_tick(self):
        """Main avoidance control loop."""
        if not self._is_armed or not self._is_in_air:
            if self._state != AvoidanceState.CLEAR:
                self._transition_to(AvoidanceState.CLEAR)
            return

        now = time.time()

        if self._state == AvoidanceState.CLEAR:
            self._handle_clear()

        elif self._state == AvoidanceState.MONITORING:
            self._handle_monitoring()

        elif self._state == AvoidanceState.AVOIDING:
            self._handle_avoiding(now)

        elif self._state == AvoidanceState.RESUMING:
            self._handle_resuming(now)

        # Publish active flag
        active_msg = Bool()
        active_msg.data = self._state in (
            AvoidanceState.AVOIDING, AvoidanceState.RESUMING
        )
        self._pub_active.publish(active_msg)

    def _handle_clear(self):
        """No obstacles detected. Check if new threats appear."""
        max_threat, worst_obstacle = self._get_max_threat()

        if max_threat >= self._min_threat:
            self._transition_to(AvoidanceState.AVOIDING)
            self._select_maneuver(worst_obstacle, max_threat)
        elif max_threat >= self._decelerate_threat:
            self._transition_to(AvoidanceState.MONITORING)

    def _handle_monitoring(self):
        """Low-level threats detected. Monitor and decelerate if needed."""
        max_threat, worst_obstacle = self._get_max_threat()

        if max_threat >= self._min_threat:
            self._transition_to(AvoidanceState.AVOIDING)
            self._select_maneuver(worst_obstacle, max_threat)
        elif max_threat < self._decelerate_threat:
            self._transition_to(AvoidanceState.CLEAR)
        else:
            # Publish deceleration command
            self._current_maneuver = AvoidanceManeuver.DECELERATE
            self._publish_decelerate_cmd()

    def _handle_avoiding(self, now: float):
        """Actively avoiding an obstacle."""
        elapsed = now - self._avoidance_start_time

        # Safety: max avoidance duration
        if elapsed > self._max_avoidance_dur:
            self.get_logger().warn('Avoidance timeout, transitioning to RESUMING')
            self._transition_to(AvoidanceState.RESUMING)
            return

        # Check if threat is still present
        max_threat, worst_obstacle = self._get_max_threat()

        if max_threat >= self._emergency_threat:
            # Escalate to emergency stop regardless of current maneuver
            if self._current_maneuver != AvoidanceManeuver.EMERGENCY_STOP:
                self._current_maneuver = AvoidanceManeuver.EMERGENCY_STOP
                self.get_logger().warn('EMERGENCY STOP: obstacle too close!')
                self._publish_event('OBSTACLE_EMERGENCY_STOP')

        # Execute current maneuver
        if self._current_maneuver == AvoidanceManeuver.EMERGENCY_STOP:
            self._execute_emergency_stop()
            # If threat reduces, try a different maneuver
            if max_threat < self._emergency_threat and max_threat >= self._min_threat:
                self._select_maneuver(worst_obstacle, max_threat)

        elif self._current_maneuver == AvoidanceManeuver.CLIMB_OVER:
            done = self._execute_climb_over()
            if done:
                self._transition_to(AvoidanceState.RESUMING)

        elif self._current_maneuver == AvoidanceManeuver.LATERAL_SLIDE:
            done = self._execute_lateral_slide(worst_obstacle)
            if done:
                self._transition_to(AvoidanceState.RESUMING)

        elif self._current_maneuver == AvoidanceManeuver.DIAGONAL_ESCAPE:
            done = self._execute_diagonal_escape(worst_obstacle)
            if done:
                self._transition_to(AvoidanceState.RESUMING)

        elif self._current_maneuver == AvoidanceManeuver.DECELERATE:
            self._publish_decelerate_cmd()
            if max_threat < self._decelerate_threat:
                self._transition_to(AvoidanceState.RESUMING)

        # If all threats gone during avoidance
        if max_threat < self._decelerate_threat:
            self._transition_to(AvoidanceState.RESUMING)

    def _handle_resuming(self, now: float):
        """Obstacle avoided, returning to normal navigation."""
        if self._resume_start_time == 0.0:
            self._resume_start_time = now

        elapsed = now - self._resume_start_time

        # Check for new threats
        max_threat, worst_obstacle = self._get_max_threat()
        if max_threat >= self._min_threat:
            self._transition_to(AvoidanceState.AVOIDING)
            self._select_maneuver(worst_obstacle, max_threat)
            return

        if elapsed >= self._resume_delay:
            # If we climbed, descend back to original altitude
            if self._current_maneuver == AvoidanceManeuver.CLIMB_OVER:
                alt_diff = self._current_alt - self._maneuver_start_alt
                if alt_diff > 1.0:
                    cmd = Twist()
                    cmd.linear.z = -self._climb_speed * 0.5  # Gentle descent
                    self._pub_cmd_vel.publish(cmd)
                    return

            self._transition_to(AvoidanceState.CLEAR)

    # ------------------------------------------------------------------
    # Maneuver selection
    # ------------------------------------------------------------------

    # Classification-based preferred maneuver:
    # - Buildings/structures: ALWAYS slide laterally (they are tall, climbing won't help)
    # - Birds/animals: Prefer climbing (they fly at a similar altitude, going up avoids them)
    # - Other drones: Fast lateral slide (drones are agile, get out of their path quickly)
    # - Vehicles: Climb over (vehicles are ground-bound)
    # - Persons: Climb over (ground-bound)
    # - Unknown: Use preferred_escape_direction parameter
    _CLASSIFICATION_PREFERRED = {
        'building':  AvoidanceManeuver.LATERAL_SLIDE,
        'tree':      AvoidanceManeuver.LATERAL_SLIDE,
        'unknown':   None,  # Use default logic
        'bird':      AvoidanceManeuver.CLIMB_OVER,
        'animal':    AvoidanceManeuver.CLIMB_OVER,
        'drone':     AvoidanceManeuver.LATERAL_SLIDE,
        'vehicle':   AvoidanceManeuver.CLIMB_OVER,
        'person':    AvoidanceManeuver.CLIMB_OVER,
    }

    def _select_maneuver(self, obstacle: Optional[Obstacle], threat: int):
        """Select the best avoidance maneuver based on obstacle type and location.

        Decision hierarchy:
        1. Emergency stop if obstacle is dangerously close (any type).
        2. Classification-based preference (bird → climb, building → lateral, etc.).
        3. Fallback to geometric analysis if classification is unknown or
           the preferred maneuver is blocked.
        4. Diagonal escape as last resort when both climb and lateral are blocked.
        """
        if obstacle is None:
            self._current_maneuver = AvoidanceManeuver.EMERGENCY_STOP
            return

        self._avoidance_start_time = time.time()
        self._maneuver_start_alt = self._current_alt
        self._maneuver_distance_traveled = 0.0

        # ---- Emergency: too close, just stop regardless of type ----
        if threat >= self._emergency_threat:
            self._current_maneuver = AvoidanceManeuver.EMERGENCY_STOP
            self._publish_event('OBSTACLE_EMERGENCY_STOP')
            self.get_logger().warn(
                f'EMERGENCY STOP: {obstacle.classification} at '
                f'{obstacle.estimated_distance_m:.1f}m, '
                f'sector={obstacle.sector}'
            )
            return

        sector = obstacle.sector
        classification = obstacle.classification

        # ---- Determine available escape routes ----
        blocked = self._get_blocked_directions()
        can_climb = self._current_alt < (self._max_alt - self._climb_clearance)
        can_go_left = 'LEFT' not in blocked and 'FRONT_LEFT' not in blocked
        can_go_right = 'RIGHT' not in blocked and 'FRONT_RIGHT' not in blocked
        can_lateral = can_go_left or can_go_right

        # ---- Classification-based decision ----
        preferred = self._CLASSIFICATION_PREFERRED.get(classification)

        if preferred is not None:
            # Try the classification-based preference first
            if preferred == AvoidanceManeuver.CLIMB_OVER and can_climb:
                maneuver = AvoidanceManeuver.CLIMB_OVER
            elif preferred == AvoidanceManeuver.LATERAL_SLIDE and can_lateral:
                maneuver = AvoidanceManeuver.LATERAL_SLIDE
            elif can_climb and can_lateral:
                # Preferred direction blocked, use diagonal
                maneuver = AvoidanceManeuver.DIAGONAL_ESCAPE
            elif can_climb:
                maneuver = AvoidanceManeuver.CLIMB_OVER
            elif can_lateral:
                maneuver = AvoidanceManeuver.LATERAL_SLIDE
            else:
                maneuver = AvoidanceManeuver.EMERGENCY_STOP
        else:
            # ---- Unknown classification: geometric fallback ----
            maneuver = self._select_geometric_maneuver(
                sector, obstacle, blocked, can_climb, can_lateral,
                can_go_left, can_go_right,
            )

        # ---- Special case: fast approach from behind ----
        # If something is chasing us from behind at high speed (another drone,
        # bird of prey), climbing sharply is the safest bet because it changes
        # our altitude plane immediately.
        if (sector in ('REAR', 'REAR_LEFT', 'REAR_RIGHT')
                and obstacle.approach_velocity_m_s > 5.0):
            if can_climb:
                maneuver = AvoidanceManeuver.CLIMB_OVER
            elif can_lateral:
                maneuver = AvoidanceManeuver.LATERAL_SLIDE

        self._current_maneuver = maneuver
        self._publish_event(f'OBSTACLE_AVOIDANCE_{maneuver.name}')
        self.get_logger().info(
            f'Avoidance: {maneuver.name} for {classification} at '
            f'{obstacle.estimated_distance_m:.1f}m, sector={sector}, '
            f'TTC={obstacle.time_to_collision_s:.1f}s'
        )

    def _select_geometric_maneuver(
        self, sector, obstacle, blocked, can_climb, can_lateral,
        can_go_left, can_go_right,
    ) -> AvoidanceManeuver:
        """Fallback maneuver selection using only obstacle geometry."""
        # Obstacle from front
        if sector in ('FRONT', 'FRONT_LEFT', 'FRONT_RIGHT'):
            if self._preferred_escape == 'up' and can_climb:
                return AvoidanceManeuver.CLIMB_OVER
            if can_go_right and sector != 'FRONT_RIGHT':
                return AvoidanceManeuver.LATERAL_SLIDE
            if can_go_left and sector != 'FRONT_LEFT':
                return AvoidanceManeuver.LATERAL_SLIDE
            if can_climb:
                return AvoidanceManeuver.CLIMB_OVER
            return AvoidanceManeuver.DIAGONAL_ESCAPE

        # Obstacle from sides
        if sector in ('LEFT', 'RIGHT'):
            if can_climb:
                return AvoidanceManeuver.CLIMB_OVER
            return AvoidanceManeuver.LATERAL_SLIDE

        # Obstacle from rear
        if sector in ('REAR', 'REAR_LEFT', 'REAR_RIGHT'):
            if obstacle.approach_velocity_m_s > 5.0:
                if can_climb:
                    return AvoidanceManeuver.CLIMB_OVER
                return AvoidanceManeuver.LATERAL_SLIDE
            return AvoidanceManeuver.DECELERATE

        return AvoidanceManeuver.EMERGENCY_STOP

    def _get_blocked_directions(self) -> set:
        """Return set of sectors with active threats >= WARNING."""
        blocked = set()
        for obs in self._latest_obstacles:
            if obs.threat_level >= THREAT_WARNING:
                blocked.add(obs.sector)
        return blocked

    # ------------------------------------------------------------------
    # Maneuver execution
    # ------------------------------------------------------------------

    def _execute_emergency_stop(self):
        """Hover in place immediately."""
        cmd = Twist()
        # All zeros = hover (offboard will maintain position)
        self._pub_cmd_vel.publish(cmd)

    def _execute_climb_over(self) -> bool:
        """Ascend by climb_clearance_m. Returns True when done."""
        alt_gained = self._current_alt - self._maneuver_start_alt

        if alt_gained >= self._climb_clearance:
            # Hold altitude briefly
            cmd = Twist()
            self._pub_cmd_vel.publish(cmd)
            return True

        # Clamp to max altitude
        if self._current_alt >= self._max_alt - 1.0:
            cmd = Twist()
            self._pub_cmd_vel.publish(cmd)
            return True

        cmd = Twist()
        cmd.linear.z = self._climb_speed  # Ascend (ENU: +z = up)
        self._pub_cmd_vel.publish(cmd)
        return False

    def _execute_lateral_slide(self, obstacle: Optional[Obstacle]) -> bool:
        """Slide perpendicular to the obstacle direction. Returns True when done."""
        # Determine slide direction based on obstacle bearing
        if obstacle is not None:
            bearing = obstacle.bearing_deg
        else:
            bearing = 0.0

        # Slide away from the obstacle
        # If obstacle is to the right (positive bearing), slide left (negative y in body)
        # If obstacle is to the left (negative bearing), slide right (positive y in body)
        slide_direction = -1.0 if bearing >= 0 else 1.0

        # Also check if the preferred side is blocked
        blocked = self._get_blocked_directions()
        if slide_direction > 0 and ('RIGHT' in blocked or 'FRONT_RIGHT' in blocked):
            slide_direction = -1.0  # Force left
        elif slide_direction < 0 and ('LEFT' in blocked or 'FRONT_LEFT' in blocked):
            slide_direction = 1.0  # Force right

        # Track distance via time * speed
        elapsed = time.time() - self._avoidance_start_time
        distance_slid = elapsed * self._lateral_speed

        if distance_slid >= self._lateral_clearance:
            cmd = Twist()
            self._pub_cmd_vel.publish(cmd)
            return True

        cmd = Twist()
        cmd.linear.y = slide_direction * self._lateral_speed
        # Also add slight climb for safety margin
        cmd.linear.z = 0.5
        self._pub_cmd_vel.publish(cmd)
        return False

    def _execute_diagonal_escape(self, obstacle: Optional[Obstacle]) -> bool:
        """Combined climb + lateral movement. Used when single direction blocked."""
        if obstacle is not None:
            bearing = obstacle.bearing_deg
        else:
            bearing = 0.0

        slide_direction = -1.0 if bearing >= 0 else 1.0

        elapsed = time.time() - self._avoidance_start_time
        alt_gained = self._current_alt - self._maneuver_start_alt

        done_lateral = (elapsed * self._lateral_speed) >= self._lateral_clearance
        done_climb = alt_gained >= (self._climb_clearance * 0.5)

        if done_lateral and done_climb:
            cmd = Twist()
            self._pub_cmd_vel.publish(cmd)
            return True

        cmd = Twist()
        if not done_lateral:
            cmd.linear.y = slide_direction * self._lateral_speed * 0.7
        if not done_climb:
            cmd.linear.z = self._climb_speed * 0.7
        self._pub_cmd_vel.publish(cmd)
        return False

    def _publish_decelerate_cmd(self):
        """Publish a reduced-speed forward command."""
        cmd = Twist()
        # Reduce forward speed to 30% of normal
        cmd.linear.x = self._avoidance_speed * 0.3
        self._pub_cmd_vel.publish(cmd)

    # ------------------------------------------------------------------
    # State transitions
    # ------------------------------------------------------------------

    def _transition_to(self, new_state: AvoidanceState):
        if new_state == self._state:
            return

        old_state = self._state
        self._state = new_state

        if new_state == AvoidanceState.CLEAR:
            self._current_maneuver = AvoidanceManeuver.NONE
            self._resume_start_time = 0.0

        elif new_state == AvoidanceState.RESUMING:
            self._resume_start_time = time.time()

        state_msg = String()
        state_msg.data = f'{old_state.name}->{new_state.name}'
        self._pub_state.publish(state_msg)

        self.get_logger().info(f'Avoidance state: {old_state.name} -> {new_state.name}')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_max_threat(self) -> Tuple[int, Optional[Obstacle]]:
        """Return the highest threat level and corresponding obstacle."""
        if not self._latest_obstacles:
            return THREAT_NONE, None

        worst = max(self._latest_obstacles, key=lambda o: o.threat_level)
        return worst.threat_level, worst

    def _publish_event(self, event_type: str):
        msg = String()
        msg.data = event_type
        self._pub_event.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
