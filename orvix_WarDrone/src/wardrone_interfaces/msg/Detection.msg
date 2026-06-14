# A single YOLO detection
std_msgs/Header header
uint16 class_id
string class_name
float32 confidence
# Bounding box (pixels)
int32 x1
int32 y1
int32 x2
int32 y2
# Center (pixels)
int32 cx
int32 cy
# Track ID assigned by tracker (-1 if untracked)
int32 track_id
