#pragma once

#include <tf2/LinearMath/Quaternion.h>

namespace hero_tf
{
//输入欧拉角，返回四元数(相机->云台的旋转)
tf2::Quaternion makeCamera2Gimbal(
  double camera_yaw_deg, double camera_pitch_deg, double camera_roll_deg);

// FLU2RDF固定旋转函数
tf2::Quaternion makeFLU2RDF();

}  // hero_tf
