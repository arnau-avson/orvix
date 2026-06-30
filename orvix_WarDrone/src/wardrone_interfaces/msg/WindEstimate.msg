# Estimated wind vector
std_msgs/Header header

# Wind speed components in NED frame (m/s)
float32 wind_north_m_s
float32 wind_east_m_s
float32 wind_down_m_s

# Wind speed magnitude (m/s)
float32 wind_speed_m_s

# Wind direction (degrees, 0=North, CW, meteorological: where wind comes FROM)
float32 wind_direction_deg

# Confidence (0.0-1.0, based on filter convergence)
float32 confidence

# Whether wind is above safe threshold
bool is_warning
