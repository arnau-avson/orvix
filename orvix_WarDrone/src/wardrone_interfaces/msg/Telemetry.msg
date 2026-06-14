# Full telemetry snapshot from PX4 via MAVSDK
std_msgs/Header header

# Position (geodetic)
float64 latitude_deg
float64 longitude_deg
float32 absolute_altitude_m
float32 relative_altitude_m

# Velocity (NED frame)
float32 velocity_north_m_s
float32 velocity_east_m_s
float32 velocity_down_m_s

# Attitude (Euler, degrees)
float32 roll_deg
float32 pitch_deg
float32 yaw_deg

# Battery
float32 battery_voltage_v
float32 battery_remaining_pct

# GPS
uint8 gps_num_satellites
uint8 gps_fix_type

# Status
string flight_mode
bool is_armed
bool is_in_air

# Health
bool is_gyrometer_calibration_ok
bool is_accelerometer_calibration_ok
bool is_magnetometer_calibration_ok
bool is_local_position_ok
bool is_global_position_ok
bool is_home_position_ok
