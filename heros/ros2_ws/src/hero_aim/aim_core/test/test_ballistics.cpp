#include <gtest/gtest.h>

#include "aim_core/ballistics.hpp"

namespace aim_core
{

BallisticConfig makeTestConfig()
{
  BallisticConfig config;
  config.bullet_speed_mps = 11.6;
  config.drag_coefficient = 0.29;
  config.gravity_mps2 = 9.794;
  config.air_density_kgpm3 = 1.169;
  config.bullet_mass_kg = 0.041;
  config.bullet_radius_m = 0.02125;
  config.muzzle_offset_m = 0.28;
  config.iteration_count = 10;
  return config;
}

TEST(Ballistics, SolvesForwardTarget)
{
  const auto trajectory = solveBallistics(makeTestConfig(), 5.0, 0.0, 0.0);
  ASSERT_TRUE(trajectory.has_value());
  EXPECT_NEAR(trajectory->yaw_rad, 0.0, 1e-9);
  EXPECT_GT(trajectory->pitch_rad, 0.0);
  EXPECT_GT(trajectory->flight_time_sec, 0.0);
}

TEST(Ballistics, RejectsInvalidDistanceScale)
{
  EXPECT_FALSE(solveBallistics(makeTestConfig(), 5.0, 0.0, 0.0, 0.0).has_value());
}

TEST(Ballistics, RejectsUnconfiguredPhysicalParameters)
{
  EXPECT_FALSE(solveBallistics(BallisticConfig{}, 5.0, 0.0, 0.0).has_value());
}

}  // aim_core
