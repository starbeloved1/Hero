#include <gtest/gtest.h>

#include "camera_driver/camera_mode.hpp"
#include "hero_msgs/msg/gimbal_state.hpp"

namespace
{

using camera_driver::CameraRole;
using hero_msgs::msg::GimbalState;

TEST(CameraMode, SelectsAimForFirstThreeModes)
{
  for (const auto mode : {GimbalState::MODE_NORMAL, GimbalState::MODE_ANTI_TOP,
                          GimbalState::MODE_AUTO_AIM}) {
    EXPECT_TRUE(camera_driver::isCameraRoleActive(CameraRole::kAim8mm, mode));
    EXPECT_FALSE(camera_driver::isCameraRoleActive(CameraRole::kBase, mode));
  }
}

TEST(CameraMode, SelectsBaseOnlyForAntiBase)
{
  EXPECT_FALSE(camera_driver::isCameraRoleActive(
    CameraRole::kAim8mm, GimbalState::MODE_ANTI_BASE));
  EXPECT_TRUE(camera_driver::isCameraRoleActive(
    CameraRole::kBase, GimbalState::MODE_ANTI_BASE));
}

TEST(CameraMode, UnknownModeFallsBackToAim)
{
  EXPECT_TRUE(camera_driver::isCameraRoleActive(CameraRole::kAim8mm, 0U));
  EXPECT_FALSE(camera_driver::isCameraRoleActive(CameraRole::kBase, 0U));
}

TEST(CameraMode, DropsFrameAcrossModeBoundary)
{
  EXPECT_TRUE(camera_driver::shouldPublishFrame(
    CameraRole::kAim8mm, GimbalState::MODE_NORMAL, GimbalState::MODE_ANTI_TOP));
  EXPECT_FALSE(camera_driver::shouldPublishFrame(
    CameraRole::kAim8mm, GimbalState::MODE_NORMAL, GimbalState::MODE_ANTI_BASE));
  EXPECT_FALSE(camera_driver::shouldPublishFrame(
    CameraRole::kBase, GimbalState::MODE_ANTI_BASE, GimbalState::MODE_NORMAL));
}

}  // namespace
