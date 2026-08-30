std_msgs/Header header
uav_mavlink_pkg/Waypoint[] waypoints  # 引导航点序列 (起=被引导目标, 末=出口)
int8 target_index                     # 被引导目标序号 (0..2)
uint8 path_length                     # 路径格数 (起止均计入)
