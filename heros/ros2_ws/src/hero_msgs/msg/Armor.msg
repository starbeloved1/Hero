# 单个装甲板的二维检测结果。角点坐标以图像左上角为原点，单位为像素。
std_msgs/Header header
geometry_msgs/Point32[4] corners
uint8 id
uint8 color
float32 confidence
