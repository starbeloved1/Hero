#include <gtest/gtest.h>

#include "aim_normal/normal_aimer.hpp"

namespace aim_normal
{

TEST(NormalAimer, ProducesCommandForValidArmor)
{
  NormalAimer aimer(NormalAimerConfig{});
  const auto result = aimer.aim({{1U, 5.0, 0.0, 0.0, false}}, 0.0, 0.0);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->selected.observation.id, 1U);
  EXPECT_FALSE(result->selected.held_target);
  EXPECT_NEAR(result->raw_yaw_rad, 0.0, 1e-9);
  EXPECT_GT(result->raw_pitch_rad, 0.0);
}

TEST(NormalAimer, HeldTargetIsExplicitlyMarked)
{
  NormalAimerConfig config;
  config.max_lost_frames = 1U;
  NormalAimer aimer(config);
  ASSERT_TRUE(aimer.aim({{1U, 5.0, 0.0, 0.0, false}}, 0.0, 0.0).has_value());
  const auto held = aimer.aim({}, 0.0, 0.0);
  ASSERT_TRUE(held.has_value());
  EXPECT_TRUE(held->selected.held_target);
  EXPECT_EQ(held->selected.lost_frames, 1U);
}

}  // aim_normal
