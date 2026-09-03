#pragma once

#include <cstdint>

#include "gimbal_driver/serial_protocol.hpp"

namespace gimbal_driver
{

// 本地调试时使用的固定云台状态；角度单位仍保持旧串口协议的角度制。
class VirtualGimbalState
{
public:
  bool setMode(int mode);
  bool setRobotColor(int robot_color);
  LegacyReadFrame frame() const;

  uint8_t mode() const;
  uint8_t robotColor() const;

private:
  uint8_t mode_{1U};
  uint8_t robot_color_{0U};
};

}  // gimbal_driver
