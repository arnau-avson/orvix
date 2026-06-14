# The target being tracked by the tracker node
std_msgs/Header header
bool is_tracking
int32 track_id
uint16 class_id
string class_name
float32 confidence

# Bounding box (pixels)
int32 x1
int32 y1
int32 x2
int32 y2

# Target offset from image center (normalized -1.0 to 1.0)
float32 offset_x
float32 offset_y

# Estimated target size ratio (bbox area / image area)
float32 size_ratio

# Frames since last detection
uint32 frames_since_detection
