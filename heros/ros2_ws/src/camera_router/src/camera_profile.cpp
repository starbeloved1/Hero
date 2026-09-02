#include "camera_router/camera_profile.hpp"

#include "hero_msgs/msg/gimbal_state.hpp"

namespace camera_router
{

//根据模式选择对应相机
CameraProfile selectProfile(std::uint8_t mode, bool base_camera_enabled)
{
  if (mode == hero_msgs::msg::GimbalState::MODE_ANTI_BASE && base_camera_enabled) {
    return CameraProfile::kBase;
  }
  return CameraProfile::kAim8mm;
}

const char * profileName(CameraProfile profile)
{
  switch (profile) {
    case CameraProfile::kBase:
      return "base";
    case CameraProfile::kAim8mm:
    default:
      return "aim8mm";
  }
}

}  // camera_router
