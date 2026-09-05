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
  // Tcontrol 故意加入不一致的处理延迟；周期仍必须只由 T0 计算为 0.6 秒。
  controller.update(state, 100.0, 0.0, 10.0, 10.1);
  controller.update(makeTrackerState(0.9, 140.0), 100.0, 0.0, 10.1, 10.3);
  controller.update(state, 100.0, 0.0, 10.6, 11.2);
  controller.update(makeTrackerState(0.9, 140.0), 100.0, 0.0, 10.7, 11.3);
  const auto countdown = controller.update(state, 100.0, 0.0, 11.2, 12.0);
  EXPECT_EQ(countdown.matched_z_layer, 0);
  EXPECT_TRUE(countdown.countdown_active);
  EXPECT_NEAR(countdown.average_period_sec, 0.6, 1e-9);
  EXPECT_NEAR(countdown.zone_stamp_sec, 11.2, 1e-9);
  EXPECT_NEAR(countdown.permit_stamp_sec, 12.95, 1e-9);
  EXPECT_NEAR(countdown.countdown_remaining_sec, 0.95, 1e-9);
  // 进入最低层射击区时，近期 Z 中位数同时成为后续弹道的目标高度。
  EXPECT_NEAR(countdown.target_z_m, 0.9, 1e-9);

  const auto fire = controller.update(state, 100.0, 0.0, 12.1, 13.0);
  EXPECT_TRUE(fire.shoot_ready);
  EXPECT_FALSE(fire.countdown_active);
}

TEST(AntitopController, DoesNotStartCountdownForNonLowestLayer)
{
  AntitopController controller(makeTestConfig());
  const auto state = makeTrackerState(1.0, 100.0);
  controller.update(state, 100.0, 0.0, 0.0, 0.0);
  controller.update(makeTrackerState(1.0, 140.0), 100.0, 0.0, 0.1, 0.1);
  controller.update(state, 100.0, 0.0, 0.6, 0.6);
  controller.update(makeTrackerState(1.0, 140.0), 100.0, 0.0, 0.7, 0.7);
  const auto result = controller.update(state, 100.0, 0.0, 1.2, 1.2);
  EXPECT_EQ(result.matched_z_layer, 1);
  EXPECT_FALSE(result.countdown_active);
  EXPECT_FALSE(result.shoot_ready);
}

}  // aim_antitop
