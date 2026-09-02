#include <gtest/gtest.h>

#include "camera_router/camera_profile.hpp"
#include "hero_msgs/msg/gimbal_state.hpp"

namespace camera_router
{

TEST(CameraProfile, AntiBaseSelectsBaseWhenEnabled)
{
  EXPECT_EQ(
    selectProfile(hero_msgs::msg::GimbalState::MODE_ANTI_BASE, true), CameraProfile::kBase);
}

TEST(CameraProfile, AntiBaseFallsBackToAim8mmWhenBaseDisabled)
{
  EXPECT_EQ(
    selectProfile(hero_msgs::msg::GimbalState::MODE_ANTI_BASE, false), CameraProfile::kAim8mm);
}

TEST(CameraProfile, AllOtherModesSelectAim8mm)
{
  EXPECT_EQ(
    selectProfile(hero_msgs::msg::GimbalState::MODE_NORMAL, true), CameraProfile::kAim8mm);
  EXPECT_EQ(
    selectProfile(hero_msgs::msg::GimbalState::MODE_ANTI_TOP, true), CameraProfile::kAim8mm);
  EXPECT_EQ(
    selectProfile(hero_msgs::msg::GimbalState::MODE_AUTO_AIM, true), CameraProfile::kAim8mm);
}

}  // camera_router
