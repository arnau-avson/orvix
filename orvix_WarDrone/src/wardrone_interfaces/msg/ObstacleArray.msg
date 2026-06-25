# Array of obstacles detected across all camera sectors
std_msgs/Header header

# All obstacles detected in this cycle
wardrone_interfaces/Obstacle[] obstacles

# Number of active camera sectors
uint8 active_sectors

# Whether any sector has a critical/emergency obstacle
bool emergency_detected

# Highest threat level across all obstacles (0-5)
uint8 max_threat_level
