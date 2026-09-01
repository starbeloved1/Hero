#include "hero_tf/transform_utils.hpp"

#include <cmath>

namespace hero_tf
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double degree2Radian(double degree)
{
  return degree * kPi / 180.0;
}

}  // namespace

tf2::Quaternion makeCamera2Gimbal(
  double camera_yaw_deg, double camera_pitch_deg, double camera_roll_deg)
{
  tf2::Quaternion inverse_yaw;
  tf2::Quaternion inverse_pitch;
  tf2::Quaternion inverse_roll;
  inverse_yaw.setRPY(0.0, 0.0, -degree2Radian(camera_yaw_deg));
  inverse_pitch.setRPY(0.0, -degree2Radian(camera_pitch_deg), 0.0);
  inverse_roll.setRPY(-degree2Radian(camera_roll_deg), 0.0, 0.0);

  tf2::Quaternion result = inverse_roll * inverse_pitch * inverse_yaw;
  result.normalize();
  return result;
}

tf2::Quaternion makeFLU2RDF()
{
  // camera_link 使用FLU；camera_optical_frame 使用RDF。
  tf2::Quaternion result;
  result.setRPY(-kPi / 2.0, 0.0, -kPi / 2.0);
  return result;
}

}  // hero_tf
