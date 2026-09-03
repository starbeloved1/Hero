#include <gtest/gtest.h>

#include "gimbal_driver/virtual_gimbal_state.hpp"

namespace gimbal_driver
{

namespace
{

TEST(VirtualGimbalState, UsesExpectedDefaultState)
{
  VirtualGimbalState state;
  const auto frame = state.frame();

  EXPECT_EQ(frame.mode_flag, 1U);
  EXPECT_EQ(frame.robot_color, 0U);
  EXPECT_FLOAT_EQ(frame.yaw_deg, 0.0F);
  EXPECT_FLOAT_EQ(frame.pitch_deg, 0.0F);
  EXPECT_FALSE(frame.right_clicked);
  EXPECT_FALSE(frame.up);
  EXPECT_FALSE(frame.down);
}

TEST(VirtualGimbalState, AcceptsAllValidModesAndColors)
{
  VirtualGimbalState state;
  for (int mode = 1; mode <= 4; ++mode) {
    EXPECT_TRUE(state.setMode(mode));
    EXPECT_EQ(state.frame().mode_flag, static_cast<uint8_t>(mode));
  }
  EXPECT_TRUE(state.setRobotColor(0));
  EXPECT_EQ(state.frame().robot_color, 0U);
  EXPECT_TRUE(state.setRobotColor(1));
  EXPECT_EQ(state.frame().robot_color, 1U);
}

TEST(VirtualGimbalState, RejectsInvalidModeAndColorWithoutChangingState)
{
  VirtualGimbalState state;
  ASSERT_TRUE(state.setMode(3));
  ASSERT_TRUE(state.setRobotColor(1));

  EXPECT_FALSE(state.setMode(0));
  EXPECT_FALSE(state.setMode(5));
  EXPECT_FALSE(state.setRobotColor(-1));
  EXPECT_FALSE(state.setRobotColor(2));
  EXPECT_EQ(state.mode(), 3U);
  EXPECT_EQ(state.robotColor(), 1U);
}

}  // namespace

}  // gimbal_driver
