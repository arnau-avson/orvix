"""Mission Controller Node.

Top-level state machine that orchestrates the drone's mission:
- Manages transitions between navigation, search, tracking, and safety modes
- Subscribes to vehicle state, tracking state, and safety events
- Calls action servers for takeoff, land, and mission execution
- Publishes mission state for monitoring
"""

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from wardrone_interfaces.msg import VehicleState, TrackingTarget, MissionState as MissionStateMsg
from wardrone_interfaces.srv import Arm, SetFlightMode
from wardrone_interfaces.action import Takeoff, Land, ExecuteMission

from wardrone_mission.states import MissionState, MissionEvent, MissionType
from wardrone_mission.state_machine import StateMachine, TransitionRecord


class MissionControllerNode(Node):

    def __init__(self):
        super().__init__('mission_controller')

        # Parameters
        self.declare_parameter('auto_arm', False)
        self.declare_parameter('mission_file', '')
        self.declare_parameter('mission_type', 'navigate_and_track')
        self.declare_parameter('search_pattern', 'expanding_square')
        self.declare_parameter('search_timeout_s', 60.0)
        self.declare_parameter('search_radius_m', 50.0)
        self.declare_parameter('takeoff_altitude_m', 10.0)
        self.declare_parameter('rtl_altitude_m', 15.0)

        self._mission_file = self.get_parameter('mission_file').value
        self._search_timeout = self.get_parameter('search_timeout_s').value
        self._search_pattern = self.get_parameter('search_pattern').value
        self._takeoff_alt = self.get_parameter('takeoff_altitude_m').value

        mission_type_str = self.get_parameter('mission_type').value
        try:
            self._mission_type = MissionType(mission_type_str)
        except ValueError:
            self._mission_type = MissionType.NAVIGATE_AND_TRACK

        # State machine
        self._sm = self._build_state_machine()
        self._mission_start_time = 0.0
        self._search_start_time = 0.0
        self._current_wp_index = 0
        self._total_waypoints = 0

        # Vehicle state cache
        self._vehicle_healthy = False
        self._vehicle_armed = False
        self._vehicle_in_air = False
        self._tracking_state = "SEARCHING"

        cb_group = ReentrantCallbackGroup()

        # Publishers
        self._pub_mission_state = self.create_publisher(MissionStateMsg, '/wardrone/mission/state', 10)
        self._pub_cmd_vel = self.create_publisher(Twist, '/wardrone/cmd_velocity', 10)

        # Subscribers
        self.create_subscription(VehicleState, '/wardrone/state', self._on_vehicle_state, 10)
        self.create_subscription(TrackingTarget, '/wardrone/tracking/target', self._on_tracking_target, 10)
        self.create_subscription(String, '/wardrone/tracking/state', self._on_tracking_state, 10)
        self.create_subscription(String, '/wardrone/safety/event', self._on_safety_event, 10)

        # Action clients
        self._takeoff_client = ActionClient(self, Takeoff, '/wardrone/takeoff', callback_group=cb_group)
        self._land_client = ActionClient(self, Land, '/wardrone/land', callback_group=cb_group)
        self._mission_client = ActionClient(self, ExecuteMission, '/wardrone/execute_mission', callback_group=cb_group)

        # Service clients
        self._arm_client = self.create_client(Arm, '/wardrone/arm', callback_group=cb_group)
        self._mode_client = self.create_client(SetFlightMode, '/wardrone/set_flight_mode', callback_group=cb_group)

        # Main tick timer (5 Hz)
        self.create_timer(0.2, self._tick)

        # State publisher timer (2 Hz)
        self.create_timer(0.5, self._publish_state)

        self.get_logger().info(f'Mission Controller ready (type={self._mission_type.value})')

    def _build_state_machine(self) -> StateMachine:
        sm = StateMachine(initial_state=MissionState.IDLE)
        sm.set_terminal_states(MissionState.DONE)

        # Normal flow
        sm.add_transition(MissionState.IDLE, MissionEvent.CMD_START, MissionState.PREFLIGHT)
        sm.add_transition(MissionState.PREFLIGHT, MissionEvent.PREFLIGHT_OK, MissionState.TAKEOFF)
        sm.add_transition(MissionState.PREFLIGHT, MissionEvent.PREFLIGHT_FAIL, MissionState.IDLE)
        sm.add_transition(MissionState.TAKEOFF, MissionEvent.TAKEOFF_COMPLETE, MissionState.NAVIGATE)

        # Navigation
        sm.add_transition(MissionState.NAVIGATE, MissionEvent.WAYPOINT_REACHED, MissionState.NAVIGATE)
        sm.add_transition(MissionState.NAVIGATE, MissionEvent.TARGET_DETECTED, MissionState.TRACK)

        # Mission complete → depends on mission type
        if self._mission_type == MissionType.NAVIGATE_ONLY:
            sm.add_transition(MissionState.NAVIGATE, MissionEvent.MISSION_COMPLETE, MissionState.RTL)
        else:
            sm.add_transition(MissionState.NAVIGATE, MissionEvent.MISSION_COMPLETE, MissionState.SEARCH)

        # Search
        sm.add_transition(MissionState.SEARCH, MissionEvent.TARGET_DETECTED, MissionState.TRACK)
        sm.add_transition(MissionState.SEARCH, MissionEvent.SEARCH_TIMEOUT, MissionState.RTL)

        # Track
        sm.add_transition(MissionState.TRACK, MissionEvent.TARGET_LOST, MissionState.SEARCH)
        sm.add_transition(MissionState.TRACK, MissionEvent.CMD_RTL, MissionState.RTL)

        # RTL and Land
        sm.add_transition(MissionState.RTL, MissionEvent.HOME_REACHED, MissionState.LAND)
        sm.add_transition(MissionState.LAND, MissionEvent.LANDED, MissionState.DONE)

        # Emergency
        sm.add_transition(MissionState.EMERGENCY, MissionEvent.LANDED, MissionState.DONE)

        # Global safety transitions
        sm.add_global_transition(
            MissionEvent.SAFETY_CRITICAL, MissionState.EMERGENCY,
            exclude_states={MissionState.DONE, MissionState.EMERGENCY}
        )
        sm.add_global_transition(
            MissionEvent.SAFETY_WARNING, MissionState.RTL,
            exclude_states={MissionState.DONE, MissionState.EMERGENCY, MissionState.RTL, MissionState.LAND}
        )
        sm.add_global_transition(
            MissionEvent.CMD_ABORT, MissionState.LAND,
            exclude_states={MissionState.DONE, MissionState.LAND}
        )

        # Track-only mode: takeoff goes directly to search
        if self._mission_type == MissionType.TRACK_ONLY:
            sm.add_transition(MissionState.TAKEOFF, MissionEvent.TAKEOFF_COMPLETE, MissionState.SEARCH)

        # Logging
        sm.on_any_transition(self._on_state_transition)

        return sm

    def _on_state_transition(self, record: TransitionRecord):
        self.get_logger().info(
            f'State: {record.from_state.value} --{record.event.value}--> {record.to_state.value}'
            + (f' ({record.reason})' if record.reason else '')
        )

    # --- Subscribers ---

    def _on_vehicle_state(self, msg: VehicleState):
        self._vehicle_healthy = msg.is_healthy
        self._vehicle_armed = msg.is_armed
        self._vehicle_in_air = msg.is_in_air

        # Detect landing
        if self._sm.state in (MissionState.LAND, MissionState.EMERGENCY):
            if not msg.is_in_air and self._vehicle_armed:
                self._sm.handle_event(MissionEvent.LANDED, "Vehicle landed")

    def _on_tracking_target(self, msg: TrackingTarget):
        if msg.is_tracking:
            if self._sm.state in (MissionState.SEARCH, MissionState.NAVIGATE):
                self._sm.handle_event(MissionEvent.TARGET_DETECTED, "Target acquired")
        else:
            if self._sm.state == MissionState.TRACK:
                self._sm.handle_event(MissionEvent.TARGET_LOST, "Target lost")

    def _on_tracking_state(self, msg: String):
        self._tracking_state = msg.data

    def _on_safety_event(self, msg: String):
        event_type = msg.data
        if event_type in ('CRITICAL_BATTERY', 'LINK_LOST'):
            self._sm.handle_event(MissionEvent.SAFETY_CRITICAL, event_type)
        elif event_type in ('LOW_BATTERY', 'GPS_DEGRADED'):
            self._sm.handle_event(MissionEvent.SAFETY_WARNING, event_type)

    # --- Main tick ---

    def _tick(self):
        """Main control loop, runs at 5 Hz."""
        state = self._sm.state

        if state == MissionState.IDLE:
            # Auto-start if configured
            if self.get_parameter('auto_arm').value and self._mission_file:
                self._sm.handle_event(MissionEvent.CMD_START, "Auto-start")

        elif state == MissionState.PREFLIGHT:
            self._do_preflight()

        elif state == MissionState.SEARCH:
            self._do_search()

    def _do_preflight(self):
        """Check if vehicle is ready for flight."""
        if self._vehicle_healthy:
            self._mission_start_time = time.time()
            self._sm.handle_event(MissionEvent.PREFLIGHT_OK, "All health checks passed")
        # Could add timeout logic here

    def _do_search(self):
        """Check search timeout."""
        if self._search_start_time == 0.0:
            self._search_start_time = time.time()

        elapsed = time.time() - self._search_start_time
        if elapsed > self._search_timeout:
            self._sm.handle_event(MissionEvent.SEARCH_TIMEOUT, f"Search timeout ({elapsed:.0f}s)")
            self._search_start_time = 0.0

    # --- State publisher ---

    def _publish_state(self):
        msg = MissionStateMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self._sm.state.value

        if len(self._sm.history) > 0:
            msg.previous_state = self._sm.history[-1].from_state.value
            msg.transition_reason = self._sm.history[-1].reason
        else:
            msg.previous_state = ""
            msg.transition_reason = ""

        msg.mission_elapsed_s = (
            time.time() - self._mission_start_time
            if self._mission_start_time > 0 else 0.0
        )
        msg.current_waypoint_index = self._current_wp_index
        msg.total_waypoints = self._total_waypoints

        self._pub_mission_state.publish(msg)

    # --- Public interface ---

    def start_mission(self):
        """External trigger to start a mission."""
        self._sm.handle_event(MissionEvent.CMD_START, "Manual start")

    def abort_mission(self):
        """External trigger to abort."""
        self._sm.handle_event(MissionEvent.CMD_ABORT, "Manual abort")

    def request_rtl(self):
        """External trigger for return to launch."""
        self._sm.handle_event(MissionEvent.CMD_RTL, "Manual RTL")


def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
