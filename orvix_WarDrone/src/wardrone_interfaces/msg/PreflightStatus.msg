# Preflight checklist status published by mission controller
std_msgs/Header header

# Overall result
bool passed
string failure_reason

# Individual checks
bool gps_ok
string gps_detail
bool battery_ok
string battery_detail
bool gyro_ok
bool accel_ok
bool mag_ok
bool local_position_ok
bool global_position_ok
bool home_position_ok
bool mission_loaded
string mission_detail
