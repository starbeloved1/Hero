#include <gtest/gtest.h>

#include <hero_msgs/msg/gimbal_state.hpp>

#include "command_mux/candidate_selector.hpp"

namespace command_mux
{

TEST(CandidateSelector, SelectsOnlyMatchingModeSource)
{
  EXPECT_EQ(sourceForMode(hero_msgs::msg::GimbalState::MODE_NORMAL), CommandSource::kNormalAim);
  EXPECT_EQ(sourceForMode(hero_msgs::msg::GimbalState::MODE_ANTI_TOP), CommandSource::kAntiTop);
  EXPECT_EQ(sourceForMode(hero_msgs::msg::GimbalState::MODE_AUTO_AIM), CommandSource::kAutoAim);
  EXPECT_EQ(sourceForMode(hero_msgs::msg::GimbalState::MODE_ANTI_BASE), CommandSource::kAntiBase);
  EXPECT_EQ(sourceForMode(0U), CommandSource::kNone);
}

TEST(CandidateSelector, AcceptsOnlyFreshPastOrCurrentCommands)
{
  EXPECT_TRUE(isFresh(10.0, 10.0, 0.1));
  EXPECT_TRUE(isFresh(9.9, 10.0, 0.1));
  EXPECT_FALSE(isFresh(9.899, 10.0, 0.1));
  EXPECT_FALSE(isFresh(10.001, 10.0, 0.1));
  EXPECT_FALSE(isFresh(10.0, 10.0, -0.1));
}

}  // command_mux
