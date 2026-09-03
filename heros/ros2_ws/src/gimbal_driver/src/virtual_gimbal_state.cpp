#include "gimbal_driver/virtual_gimbal_state.hpp"

namespace gimbal_driver
{

bool VirtualGimbalState::setMode(int mode)
{
  if (mode < 1 || mode > 4) {
    return false;
  }
  mode_ = static_cast<uint8_t>(mode);
  return true;
}

bool VirtualGimbalState::setRobotColor(int robot_color)
{
  if (robot_color != 0 && robot_color != 1) {
    return false;
  }
  robot_color_ = static_cast<uint8_t>(robot_color);
  return true;
}

LegacyReadFrame VirtualGimbalState::frame() const
{
  LegacyReadFrame frame;
  frame.mode_flag = mode_;
  frame.pitch_deg = 0.0F;
  frame.yaw_deg = 0.0F;
  frame.up = false;
  frame.down = false;
  frame.robot_color = robot_color_;
  frame.right_clicked = false;
  return frame;
}

uint8_t VirtualGimbalState::mode() const
{
  return mode_;
}

uint8_t VirtualGimbalState::robotColor() const
{
  return robot_color_;
}

}  // gimbal_driver
