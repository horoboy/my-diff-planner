uint8 WAIT_START=0
uint8 TAKEOFF=1
uint8 TRANSIT_TO_ROI=2
uint8 SEARCH=3
uint8 TARGET_CONFIRM=4
uint8 APPROACH=5
uint8 STABILIZE=6
uint8 DROP=7
uint8 VERIFY_RELEASE=8
uint8 RETURN=9
uint8 LAND=10
uint8 COMPLETE=11
uint8 ABORT=12

std_msgs/Header header
uint8 state
string state_name
string detail
geometry_msgs/PoseStamped active_goal
uint32 goal_sequence
float64 elapsed

