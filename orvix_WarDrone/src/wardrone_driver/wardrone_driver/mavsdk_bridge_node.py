import asyncio
import math
import threading
from queue import Queue, Empty

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Quaternion
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus

from wardrone_interfaces.msg import Telemetry, VehicleState
from wardrone_interfaces.srv import Arm, SetFlightMode
from wardrone_interfaces.action import Takeoff, Land

from wardrone_driver.mavsdk_client import MavsdkClient, TelemetryData


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles (radians) to quaternion."""
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class MavsdkBridgeNode(Node):
    def __init__(self):
        super().__init__('mavsdk_bridge')

        # Parameters
        self.declare_parameter('connection_url', 'udp://:14540')
        self.declare_parameter('system_id', 1)
        self.declare_parameter('telemetry_rate_hz', 10.0)
        self.declare_parameter('battery_rate_hz', 1.0)
        self.declare_parameter('state_rate_hz', 5.0)
        self.declare_parameter('offboard_rate_hz', 20.0)
        self.declare_parameter('connection_timeout_s', 30.0)
        self.declare_parameter('command_timeout_s', 10.0)

        conn_url = self.get_parameter('connection_url').value
        sys_id = self.get_parameter('system_id').value

        # MAVSDK client
        self._client = MavsdkClient(connection_url=conn_url, system_id=sys_id)
        self._telemetry_queue: Queue = Queue(maxsize=200)

        # Callback group for services/actions
        self._cb_group = ReentrantCallbackGroup()

        # --- Publishers ---
        self._pub_telemetry = self.create_publisher(Telemetry, '/wardrone/telemetry', 10)
        self._pub_state = self.create_publisher(VehicleState, '/wardrone/state', 10)
        self._pub_position = self.create_publisher(PoseStamped, '/wardrone/position', 10)
        self._pub_velocity = self.create_publisher(TwistStamped, '/wardrone/velocity', 10)
        self._pub_battery = self.create_publisher(BatteryState, '/wardrone/battery', 10)
        self._pub_gps = self.create_publisher(NavSatFix, '/wardrone/gps_info', 10)

        # --- Subscribers ---
        self.create_subscription(Twist, '/wardrone/cmd_velocity', self._on_cmd_velocity, 10)
        self.create_subscription(PoseStamped, '/wardrone/cmd_position', self._on_cmd_position, 10)
        self.create_subscription(PoseStamped, '/wardrone/vio/pose', self._on_vio_pose, 10)

        # --- Services ---
        self.create_service(Arm, '/wardrone/arm', self._handle_arm, callback_group=self._cb_group)
        self.create_service(SetFlightMode, '/wardrone/set_flight_mode', self._handle_set_flight_mode,
                           callback_group=self._cb_group)

        # --- Action Servers ---
        self._takeoff_action = ActionServer(
            self, Takeoff, '/wardrone/takeoff', self._handle_takeoff,
            callback_group=self._cb_group
        )
        self._land_action = ActionServer(
            self, Land, '/wardrone/land', self._handle_land,
            callback_group=self._cb_group
        )

        # Offboard state
        self._offboard_active = False

        # --- Asyncio loop in background thread ---
        self._loop = asyncio.new_event_loop()
        self._async_thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self._async_thread.start()

        # Start MAVSDK connection
        future = asyncio.run_coroutine_threadsafe(self._start_mavsdk(), self._loop)
        # Non-blocking - will log when connected

        # Timer to publish telemetry from queue
        telem_rate = self.get_parameter('telemetry_rate_hz').value
        self.create_timer(1.0 / telem_rate, self._publish_telemetry_tick)

        self.get_logger().info(f'MAVSDK Bridge starting, connecting to {conn_url}...')

    def _run_async_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    async def _start_mavsdk(self):
        try:
            connected = await self._client.connect()
            if connected:
                self.get_logger().info('Connected to PX4!')
                self._client.add_telemetry_callback(self._on_telemetry_update)
                await self._client.start_telemetry_tasks()
            else:
                self.get_logger().error('Failed to connect to PX4')
        except Exception as e:
            self.get_logger().error(f'MAVSDK connection error: {e}')

    def _on_telemetry_update(self, telem: TelemetryData):
        try:
            self._telemetry_queue.put_nowait(telem)
        except Exception:
            pass  # Queue full, drop oldest

    def _make_header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        return header

    def _publish_telemetry_tick(self):
        """Drain the telemetry queue and publish the latest data."""
        latest = None
        try:
            while True:
                latest = self._telemetry_queue.get_nowait()
        except Empty:
            pass

        if latest is None:
            return

        t = latest
        header = self._make_header()

        # Full telemetry
        msg = Telemetry()
        msg.header = header
        msg.latitude_deg = t.latitude_deg
        msg.longitude_deg = t.longitude_deg
        msg.absolute_altitude_m = t.absolute_altitude_m
        msg.relative_altitude_m = t.relative_altitude_m
        msg.velocity_north_m_s = t.velocity_north_m_s
        msg.velocity_east_m_s = t.velocity_east_m_s
        msg.velocity_down_m_s = t.velocity_down_m_s
        msg.roll_deg = t.roll_deg
        msg.pitch_deg = t.pitch_deg
        msg.yaw_deg = t.yaw_deg
        msg.battery_voltage_v = t.battery_voltage_v
        msg.battery_remaining_pct = t.battery_remaining_pct
        msg.gps_num_satellites = t.gps_num_satellites
        msg.gps_fix_type = t.gps_fix_type
        msg.flight_mode = t.flight_mode.value
        msg.is_armed = t.is_armed
        msg.is_in_air = t.is_in_air
        msg.is_gyrometer_calibration_ok = t.is_gyrometer_calibration_ok
        msg.is_accelerometer_calibration_ok = t.is_accelerometer_calibration_ok
        msg.is_magnetometer_calibration_ok = t.is_magnetometer_calibration_ok
        msg.is_local_position_ok = t.is_local_position_ok
        msg.is_global_position_ok = t.is_global_position_ok
        msg.is_home_position_ok = t.is_home_position_ok
        self._pub_telemetry.publish(msg)

        # Vehicle state
        state_msg = VehicleState()
        state_msg.header = header
        state_msg.flight_mode = t.flight_mode.value
        state_msg.is_armed = t.is_armed
        state_msg.is_in_air = t.is_in_air
        state_msg.is_healthy = (
            t.is_gyrometer_calibration_ok and
            t.is_accelerometer_calibration_ok and
            t.is_magnetometer_calibration_ok and
            t.is_local_position_ok and
            t.is_global_position_ok and
            t.is_home_position_ok
        )
        state_msg.battery_remaining_pct = t.battery_remaining_pct
        state_msg.gps_fix_type = t.gps_fix_type
        self._pub_state.publish(state_msg)

        # Position (ENU)
        pose_msg = PoseStamped()
        pose_msg.header = header
        enu = MavsdkClient.ned_to_enu(
            t.velocity_north_m_s, t.velocity_east_m_s, 0.0
        )
        # For position we use GPS lat/lon converted - simplified: use relative alt as z
        pose_msg.pose.position.x = 0.0  # Would need local frame conversion
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = float(t.relative_altitude_m)
        yaw_rad = math.radians(t.yaw_deg)
        roll_rad = math.radians(t.roll_deg)
        pitch_rad = math.radians(t.pitch_deg)
        pose_msg.pose.orientation = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
        self._pub_position.publish(pose_msg)

        # Velocity (ENU)
        vel_msg = TwistStamped()
        vel_msg.header = header
        vel_msg.twist.linear.x = t.velocity_east_m_s  # ENU x = East
        vel_msg.twist.linear.y = t.velocity_north_m_s  # ENU y = North
        vel_msg.twist.linear.z = -t.velocity_down_m_s  # ENU z = Up
        self._pub_velocity.publish(vel_msg)

        # Battery
        bat_msg = BatteryState()
        bat_msg.header = header
        bat_msg.voltage = t.battery_voltage_v
        bat_msg.percentage = t.battery_remaining_pct / 100.0
        self._pub_battery.publish(bat_msg)

        # GPS
        gps_msg = NavSatFix()
        gps_msg.header = header
        gps_msg.latitude = t.latitude_deg
        gps_msg.longitude = t.longitude_deg
        gps_msg.altitude = t.absolute_altitude_m
        gps_msg.status.status = NavSatStatus.STATUS_FIX if t.gps_fix_type >= 3 else NavSatStatus.STATUS_NO_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        self._pub_gps.publish(gps_msg)

    # --- Command Subscribers ---

    def _on_cmd_velocity(self, msg: Twist):
        """Receive velocity body commands (forward, right, down, yawspeed)."""
        async def _send():
            try:
                if not self._offboard_active:
                    await self._client.start_offboard()
                    self._offboard_active = True
                await self._client.send_velocity_body(
                    msg.linear.x,   # forward
                    msg.linear.y,   # right
                    msg.linear.z,   # down
                    math.degrees(msg.angular.z)  # yawspeed
                )
            except Exception as e:
                self.get_logger().warn(f'Velocity command failed: {e}')
        asyncio.run_coroutine_threadsafe(_send(), self._loop)

    def _on_cmd_position(self, msg: PoseStamped):
        """Receive position setpoint in ENU, convert to NED for PX4."""
        async def _send():
            try:
                if not self._offboard_active:
                    await self._client.start_offboard()
                    self._offboard_active = True
                north, east, down = MavsdkClient.enu_to_ned(
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z
                )
                # Extract yaw from quaternion (simplified)
                q = msg.pose.orientation
                yaw_enu = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                yaw_ned = math.degrees(MavsdkClient.yaw_enu_to_ned(yaw_enu))
                await self._client.send_position_ned(north, east, down, yaw_ned)
            except Exception as e:
                self.get_logger().warn(f'Position command failed: {e}')
        asyncio.run_coroutine_threadsafe(_send(), self._loop)

    def _on_vio_pose(self, msg: PoseStamped):
        """Receive VIO pose estimate in ENU, relay to PX4 as vision position estimate."""
        async def _send():
            try:
                # Convert ENU to NED for PX4
                north, east, down = MavsdkClient.enu_to_ned(
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z
                )
                q = msg.pose.orientation
                roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                                   1.0 - 2.0 * (q.x * q.x + q.y * q.y))
                pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
                yaw_enu = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                yaw_ned = MavsdkClient.yaw_enu_to_ned(yaw_enu)

                timestamp_us = int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1e3)
                await self._client.send_vision_position_estimate(
                    north, east, down, roll, pitch, yaw_ned, timestamp_us
                )
            except Exception as e:
                self.get_logger().warn(f'VIO pose relay failed: {e}')
        asyncio.run_coroutine_threadsafe(_send(), self._loop)

    # --- Service Handlers ---

    def _handle_arm(self, request: Arm.Request, response: Arm.Response):
        async def _do():
            try:
                if request.arm:
                    await self._client.arm()
                else:
                    await self._client.disarm()
                return True, 'OK'
            except Exception as e:
                return False, str(e)

        future = asyncio.run_coroutine_threadsafe(_do(), self._loop)
        success, message = future.result(timeout=10.0)
        response.success = success
        response.message = message
        return response

    def _handle_set_flight_mode(self, request: SetFlightMode.Request, response: SetFlightMode.Response):
        async def _do():
            try:
                mode = request.mode.upper()
                if mode == 'HOLD':
                    await self._client.set_flight_mode_hold()
                elif mode == 'RTL':
                    await self._client.return_to_launch()
                elif mode == 'LAND':
                    await self._client.land()
                elif mode == 'OFFBOARD':
                    await self._client.start_offboard()
                    self._offboard_active = True
                else:
                    return False, f'Unknown mode: {mode}'
                return True, 'OK'
            except Exception as e:
                return False, str(e)

        future = asyncio.run_coroutine_threadsafe(_do(), self._loop)
        success, message = future.result(timeout=10.0)
        response.success = success
        response.message = message
        return response

    # --- Action Handlers ---

    async def _handle_takeoff(self, goal_handle):
        """Handle Takeoff action: arm + takeoff to target altitude."""
        self.get_logger().info(f'Takeoff requested: {goal_handle.request.target_altitude_m}m')
        target_alt = goal_handle.request.target_altitude_m

        try:
            # Arm
            await asyncio.wrap_future(
                asyncio.run_coroutine_threadsafe(self._client.arm(), self._loop)
            )
            # Takeoff
            await asyncio.wrap_future(
                asyncio.run_coroutine_threadsafe(
                    self._client.takeoff(target_alt), self._loop
                )
            )

            # Monitor altitude until reached
            feedback = Takeoff.Feedback()
            while True:
                current_alt = self._client.telemetry.relative_altitude_m
                feedback.current_altitude_m = current_alt
                goal_handle.publish_feedback(feedback)

                if current_alt >= target_alt * 0.95:
                    break
                await asyncio.sleep(0.5)

            goal_handle.succeed()
            result = Takeoff.Result()
            result.success = True
            result.message = 'Takeoff complete'
            result.final_altitude_m = self._client.telemetry.relative_altitude_m
            return result

        except Exception as e:
            goal_handle.abort()
            result = Takeoff.Result()
            result.success = False
            result.message = str(e)
            result.final_altitude_m = self._client.telemetry.relative_altitude_m
            return result

    async def _handle_land(self, goal_handle):
        """Handle Land action."""
        self.get_logger().info('Land requested')

        try:
            if not goal_handle.request.land_in_place:
                await asyncio.wrap_future(
                    asyncio.run_coroutine_threadsafe(
                        self._client.return_to_launch(), self._loop
                    )
                )
            else:
                await asyncio.wrap_future(
                    asyncio.run_coroutine_threadsafe(self._client.land(), self._loop)
                )

            # Monitor descent
            feedback = Land.Feedback()
            while True:
                current_alt = self._client.telemetry.relative_altitude_m
                feedback.current_altitude_m = current_alt
                feedback.is_descending = current_alt > 0.3
                goal_handle.publish_feedback(feedback)

                if not self._client.telemetry.is_in_air:
                    break
                await asyncio.sleep(0.5)

            self._offboard_active = False
            goal_handle.succeed()
            result = Land.Result()
            result.success = True
            result.message = 'Landing complete'
            return result

        except Exception as e:
            goal_handle.abort()
            result = Land.Result()
            result.success = False
            result.message = str(e)
            return result

    def destroy_node(self):
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._async_thread.join(timeout=5.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MavsdkBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
