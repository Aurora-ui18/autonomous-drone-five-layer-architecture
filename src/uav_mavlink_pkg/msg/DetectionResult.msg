std_msgs/Header header
string class_name        # 类别名，如 "QR"
string class_name_en     # 英文类别名，如 "QRCODE"
float64 confidence       # 置信度 0~1
int32 count              # 检出数量
geometry_msgs/Point box_center   # 图像坐标系中心 (px)
float64[4] box         # [x1,y1,x2,y2] 像素坐标
string cell_code         # 所在方格代码（定位层填充）
