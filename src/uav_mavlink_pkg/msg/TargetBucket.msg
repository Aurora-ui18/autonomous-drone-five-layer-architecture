std_msgs/Header header
int8 grid_x          # 目标格 X (0..9)
int8 grid_y          # 目标格 Y (0..7)
int8 color           # 0=红 1=绿 2=蓝
float32 confidence   # 检测置信度 0..1
float64 world_x      # 世界坐标偏移 X (m)
float64 world_y      # 世界坐标偏移 Y (m)
bool confirmed       # 是否经多帧确认
int8 seq             # 发现序号 1..3 (由决策层填充)
