# A single detected obstacle
std_msgs/Header header

# Sector where the obstacle was detected
# Values: FRONT, FRONT_RIGHT, RIGHT, REAR_RIGHT, REAR, REAR_LEFT, LEFT, FRONT_LEFT
string sector

# Bearing angle in degrees relative to drone heading (0=front, 90=right, 180=rear, -90=left)
float32 bearing_deg

# Estimated distance in meters (from monocular size estimation or sensor)
float32 estimated_distance_m

# Approach velocity in m/s (positive = approaching, negative = receding)
float32 approach_velocity_m_s

# Time to collision in seconds (-1 if not approaching)
float32 time_to_collision_s

# Classification
string classification          # "bird", "drone", "tree", "building", "vehicle", "unknown"
float32 classification_confidence

# Bounding box in the camera image (pixels)
int32 bbox_x1
int32 bbox_y1
int32 bbox_x2
int32 bbox_y2

# Apparent size (fraction of image area, 0.0-1.0)
float32 apparent_size_ratio

# Threat level: 0=none, 1=monitor, 2=caution, 3=warning, 4=critical, 5=emergency
uint8 threat_level
