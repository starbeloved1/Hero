#include "camera_driver/camera_mode.hpp"

#include "hero_msgs/msg/gimbal_state.hpp"

namespace camera_driver
{

bool isCameraRoleActive(CameraRole role, std::uint8_t gimbal_mode)
{
  if (role == CameraRole::kBase) {
    return gimbal_mode == hero_msgs::msg::GimbalState::MODE_ANTI_BASE;
  }
  return gimbal_mode != hero_msgs::msg::GimbalState::MODE_ANTI_BASE;
}

bool shouldPublishFrame(CameraRole role, std::uint8_t mode_before_capture,
                        std::uint8_t mode_before_publish)
{
  return isCameraRoleActive(role, mode_before_capture) &&
         isCameraRoleActive(role, mode_before_publish);
}

}  // namespace camera_driver
