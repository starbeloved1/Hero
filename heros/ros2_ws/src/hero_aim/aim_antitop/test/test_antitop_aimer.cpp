#include <gtest/gtest.h>

#include "aim_antitop/antitop_aimer.hpp"

namespace aim_antitop
{

AntitopAimerConfig makeTestConfig()
{
  AntitopAimerConfig config;
  config.ballistics.bullet_speed_mps = 11.6;
  config.ballistics.drag_coefficient = 0.29;
  config.ballistics.gravity_mps2 = 9.794;
  config.ballistics.air_density_kgpm3 = 1.169;
  config.ballistics.bullet_mass_kg = 0.041;
  config.ballistics.bullet_radius_m = 0.02125;
  config.ballistics.muzzle_offset_m = 0.28;
  config.ballistics.iteration_count = 10;
  return config;
}

AntitopTrackerState makeTrackerState(bool calibrated)
{
  AntitopTrackerState state;
  state.tracked_armor = {7U, Eigen::Vector3d(5.0, 0.0, 1.0)};
  state.rotation_center_m = Eigen::Vector3d(5.0, 0.0, 0.0);
  state.z_layers_m = {0.9, 1.0, 1.1};
  state.center_valid = true;
  state.calibrated = calibrated;
  return state;
}

TEST(AntitopAimer, HoldsPitchBeforeCalibration)
{
  AntitopAimer aimer(makeTestConfig());
  const auto result = aimer.aim(makeTrackerState(false), 0.0, 0.3);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->pitch_locked);
  EXPECT_NEAR(result->command_yaw_rad, 0.0, 1e-9);
  EXPECT_NEAR(result->command_pitch_rad, 0.3, 1e-9);
  EXPECT_NEAR(result->target_z_m, 1.0, 1e-9);
}

TEST(AntitopAimer, LocksPitchToLowestZLayerAfterCalibration)
{
  AntitopAimer aimer(makeTestConfig());
  const auto result = aimer.aim(makeTrackerState(true), 0.0, 0.3);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->pitch_locked);
  EXPECT_NEAR(result->target_z_m, 0.9, 1e-9);
  EXPECT_NE(result->command_pitch_rad, 0.3);
}

}  // aim_antitop
