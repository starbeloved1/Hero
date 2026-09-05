#include <gtest/gtest.h>

#include "aim_antitop/antitop_controller.hpp"

namespace aim_antitop
{

AntitopControllerConfig makeTestConfig()
{
  AntitopControllerConfig config;
  config.minimum_center_samples_for_zone = 1U;
  config.recent_z_window_size = 1U;
  config.system_delay_sec = 0.05;
  return config;
}

AntitopTrackerState makeTrackerState(double z, double image_x_px)
{
  AntitopTrackerState state;
  state.tracked_armor = {7U, Eigen::Vector3d(5.0, 0.0, z), image_x_px};
  state.rotation_center_m = Eigen::Vector3d(5.0, 0.0, 0.0);
  state.z_layers_m = {0.9, 1.0, 1.1};
  state.center_valid = true;
  state.calibrated = true;
  state.center_sample_count = 80U;
  return state;
}

TEST(AntitopController, StartsCountdownAfterTwoValidPeriodsAtLowestLayer)
{
  AntitopController controller(makeTestConfig());
  const auto state = makeTrackerState(0.9, 100.0);
  controller.update(state, 100.0, 0.0, 0.0);
  controller.update(makeTrackerState(0.9, 140.0), 100.0, 0.0, 0.1);
  controller.update(state, 100.0, 0.0, 0.6);
  controller.update(makeTrackerState(0.9, 140.0), 100.0, 0.0, 0.7);
  const auto countdown = controller.update(state, 100.0, 0.0, 1.2);
  EXPECT_EQ(countdown.matched_z_layer, 0);
  EXPECT_TRUE(countdown.countdown_active);
  EXPECT_NEAR(countdown.average_period_sec, 0.6, 1e-9);
  // 进入最低层射击区时，近期 Z 中位数同时成为后续弹道的目标高度。
  EXPECT_NEAR(countdown.target_z_m, 0.9, 1e-9);

  const auto fire = controller.update(state, 100.0, 0.0, 3.0);
  EXPECT_TRUE(fire.shoot_ready);
  EXPECT_FALSE(fire.countdown_active);
}

TEST(AntitopController, DoesNotStartCountdownForNonLowestLayer)
{
  AntitopController controller(makeTestConfig());
  const auto state = makeTrackerState(1.0, 100.0);
  controller.update(state, 100.0, 0.0, 0.0);
  controller.update(makeTrackerState(1.0, 140.0), 100.0, 0.0, 0.1);
  controller.update(state, 100.0, 0.0, 0.6);
  controller.update(makeTrackerState(1.0, 140.0), 100.0, 0.0, 0.7);
  const auto result = controller.update(state, 100.0, 0.0, 1.2);
  EXPECT_EQ(result.matched_z_layer, 1);
  EXPECT_FALSE(result.countdown_active);
  EXPECT_FALSE(result.shoot_ready);
}

}  // aim_antitop
