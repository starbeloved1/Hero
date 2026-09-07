#pragma once

#include <cstdint>

namespace camera_driver
{

enum class CameraRole
{
  kAim8mm,
  kBase,
};

bool isCameraRoleActive(CameraRole role, std::uint8_t gimbal_mode);
bool shouldPublishFrame(CameraRole role, std::uint8_t mode_before_capture,
                        std::uint8_t mode_before_publish);

}  // namespace camera_driver
