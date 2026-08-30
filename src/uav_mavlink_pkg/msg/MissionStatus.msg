std_msgs/Header header
uint8 state
# 0=INIT  1=READY  2=TAKING_OFF  3=PATROLING  4=DETECTING  5=DROPPING
# 6=RING_CROSSING  7=LANDING  8=FINISHED  9=ERROR  10=EMERGENCY
string current_cell
Waypoint current_waypoint
string message
float64 progress         # 0.0 ~ 1.0
