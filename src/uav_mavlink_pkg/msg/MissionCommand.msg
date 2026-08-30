uint8 cmd
# 0=NOP  1=ARM  2=TAKEOFF  3=PATROL  4=DETECT  5=DROP  6=RING  7=LAND  8=ABORT  9=RETURN
# 10=PRE_IDENTIFY（D题模式2预处理）
float64[] params         # 命令参数，如 [target_height, tolerance]
string reason            # 人工可读的原因
