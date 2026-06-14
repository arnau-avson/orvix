# State machine state published by mission controller
std_msgs/Header header
string state
string previous_state
string transition_reason
float32 mission_elapsed_s
uint32 current_waypoint_index
uint32 total_waypoints
