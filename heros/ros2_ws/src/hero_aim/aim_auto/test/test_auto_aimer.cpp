#include <gtest/gtest.h>

#include "aim_auto/auto_aimer.hpp"

namespace
{

aim_auto::AutoAimConfig makeConfig()
{
  aim_auto::AutoAimConfig config;
  config.ballistics.bullet_speed_mps = 20.0;
  config.ballistics.drag_coefficient = 0.0;
  config.ballistics.gravity_mps2 = 9.794;
  config.ballistics.air_density_kgpm3 = 1.169;
  config.ballistics.bullet_mass_kg = 0.0032;
  config.ballistics.bullet_radius_m = 0.0084;
  config.ballistics.muzzle_offset_m = 0.0;
  config.ballistics.iteration_count = 10;
  config.fire_confirm_frames = 1;
  config.fast_fire_confirm_frames = 1;
  config.high_acceleration_threshold_mps2 = 100.0;
  return config;
}

aim_auto::AutoTarget makeTarget(std::uint8_t id, double distance)
{
  aim_auto::AutoTarget target;
  target.id = id;
  target.tracking = true;
  target.converged = true;
  target.center_m = Eigen::Vector3d(distance, 0.0, 0.0);
  target.radius_m = 0.2;
  target.radius_offset_m = 0.0;
  target.height_offset_m = 0.0;
  return target;
}

TEST(AutoAimer, LocksNearestValidTarget)
{
  aim_auto::AutoAimer aimer(makeConfig());
  const auto result = aimer.aim({makeTarget(2U, 5.0), makeTarget(1U, 3.0)}, 1.0, 1.0, 0.0, 0.0);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->target_id, 1U);

  const auto held = aimer.aim({makeTarget(1U, 6.0), makeTarget(2U, 2.0)}, 1.1, 1.1, 0.0, 0.0);
  ASSERT_TRUE(held.has_value());
  EXPECT_EQ(held->target_id, 1U);
}

TEST(AutoAimer, UsesSameArmorForYawAndPitch)
{
  aim_auto::AutoAimer aimer(makeConfig());
  const auto result = aimer.aim({makeTarget(1U, 4.0)}, 1.0, 1.0, 0.0, 0.0);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->raw_yaw_rad, std::atan2(result->aim_point_m.y(), result->aim_point_m.x()), 1e-6);
}

TEST(AutoAimer, HighAccelerationBlocksFire)
{
  auto config = makeConfig();
  config.high_acceleration_threshold_mps2 = 0.1;
  aim_auto::AutoAimer aimer(config);
  auto target = makeTarget(1U, 4.0);
  ASSERT_TRUE(aimer.aim({target}, 1.0, 1.0, 0.0, 0.0).has_value());
  target.velocity_mps.x() = 1.0;
  const auto result = aimer.aim({target}, 1.1, 1.1, 0.0, 0.0);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->acceleration_safe);
  EXPECT_FALSE(result->shoot_ready);
}

}  // namespace
