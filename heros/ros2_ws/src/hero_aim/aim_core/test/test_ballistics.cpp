#include <gtest/gtest.h>

#include "aim_core/ballistics.hpp"

namespace aim_core
{

TEST(Ballistics, SolvesForwardTarget)
{
  const auto trajectory = solveBallistics(BallisticConfig{}, 5.0, 0.0, 0.0);
  ASSERT_TRUE(trajectory.has_value());
  EXPECT_NEAR(trajectory->yaw_rad, 0.0, 1e-9);
  EXPECT_GT(trajectory->pitch_rad, 0.0);
  EXPECT_GT(trajectory->flight_time_sec, 0.0);
}

TEST(Ballistics, RejectsInvalidDistanceScale)
{
  EXPECT_FALSE(solveBallistics(BallisticConfig{}, 5.0, 0.0, 0.0, 0.0).has_value());
}

}  // aim_core
