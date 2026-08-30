float64 x
float64 y
float64 z
string cell_code         # 关联场地方格，可为空
uint8 type               # 0=HOVER_DETECT  1=PATROL  4=LAND  6=FIRE_APPROACH  7=FIRE_DESCEND  8=FIRE_DROP
string task_params       # JSON 任务参数（如投放模式等）
float64 yaw              # 机头朝向（度）
