#include <gtest/gtest.h>

#include <cmath>

#include "aim_core/angle_utils.hpp"

namespace aim_core
{

TEST(AngleUtils, UnwrapsAcrossPiBoundary)
{
  const double value = unwrapNear(-3.13, 3.13);
  EXPECT_NEAR(value, 3.1531853071795863, 1e-9);
}

TEST(AngleUtils, SmoothesOnlySmallContinuousChanges)
{
  AngleSmoother smoother;
  const auto first = smoother.filter(1.0, 0.2);
  EXPECT_DOUBLE_EQ(first.yaw_rad, 1.0);
  const auto second = smoother.filter(1.02, 0.21);
  EXPECT_NEAR(second.yaw_rad, 1.014, 1e-12);
  EXPECT_NEAR(second.pitch_rad, 0.203, 1e-12);
  const auto jump = smoother.filter(1.2, 0.21);
  EXPECT_NEAR(jump.yaw_rad, 1.2, 1e-12);
}

}  // aim_core
