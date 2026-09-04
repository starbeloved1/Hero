#include <gtest/gtest.h>

#include "aim_normal/normal_aimer.hpp"

namespace aim_normal
{

NormalAimerConfig makeTestConfig()
{
  NormalAimerConfig config;
  config.ballistic.bullet_speed_mps = 11.6;
  config.ballistic.drag_coefficient = 0.29;
  config.ballistic.gravity_mps2 = 9.794;
  config.ballistic.air_density_kgpm3 = 1.169;
  config.ballistic.bullet_mass_kg = 0.041;
  config.ballistic.bullet_radius_m = 0.02125;
  config.ballistic.muzzle_offset_m = 0.28;
  config.ballistic.iteration_count = 10;
  return config;
}

TEST(NormalAimer, ProducesCommandForValidArmor)
{
  NormalAimer aimer(makeTestConfig());
  const auto result = aimer.aim({{1U, 5.0, 0.0, 0.0, false}}, 0.0, 0.0);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->selected.observation.id, 1U);
  EXPECT_FALSE(result->selected.held_target);
  EXPECT_NEAR(result->raw_yaw_rad, 0.0, 1e-9);
  EXPECT_GT(result->raw_pitch_rad, 0.0);
}

TEST(NormalAimer, HeldTargetIsExplicitlyMarked)
{
  NormalAimerConfig config = makeTestConfig();
  config.max_lost_frames = 1U;
  NormalAimer aimer(config);
  ASSERT_TRUE(aimer.aim({{1U, 5.0, 0.0, 0.0, false}}, 0.0, 0.0).has_value());
  const auto held = aimer.aim({}, 0.0, 0.0);
  ASSERT_TRUE(held.has_value());
  EXPECT_TRUE(held->selected.held_target);
  EXPECT_EQ(held->selected.lost_frames, 1U);
}

}  // aim_normal
