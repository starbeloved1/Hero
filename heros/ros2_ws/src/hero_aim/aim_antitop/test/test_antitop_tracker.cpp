#include <gtest/gtest.h>

#include "aim_antitop/antitop_tracker.hpp"

namespace aim_antitop
{

AntitopTrackerConfig makeTestConfig()
{
  AntitopTrackerConfig config;
  config.center_window_size = 20U;
  config.minimum_center_samples = 3U;
  config.calibration_start_center_samples = 3U;
  config.calibration_min_samples = 12U;
  config.calibration_max_samples = 20U;
  config.minimum_layer_samples = 3U;
  return config;
}

TEST(AntitopTracker, IgnoresNonOutpostArmor)
{
  AntitopTracker tracker(makeTestConfig());
  EXPECT_FALSE(tracker.update({{1U, Eigen::Vector3d(4.0, 0.0, 1.0)}}).has_value());
}

TEST(AntitopTracker, SelectsObservationContinuousWithPreviousFrame)
{
  AntitopTracker tracker(makeTestConfig());
  ASSERT_TRUE(tracker.update({
    {7U, Eigen::Vector3d(5.0, 0.0, 1.0)},
    {7U, Eigen::Vector3d(2.0, 0.0, 1.0)}}).has_value());
  const auto state = tracker.update({
    {7U, Eigen::Vector3d(2.1, 0.0, 1.0)},
    {7U, Eigen::Vector3d(5.1, 0.0, 1.0)}});
  ASSERT_TRUE(state.has_value());
  EXPECT_NEAR(state->tracked_armor.position_m.x(), 5.1, 1e-9);
}

TEST(AntitopTracker, FitsCenterAndCalibratesThreeZLayers)
{
  AntitopTracker tracker(makeTestConfig());
  const std::array<double, 3> z_values{0.90, 1.00, 1.10};
  constexpr double kPi = 3.14159265358979323846;
  std::optional<AntitopTrackerState> state;
  for (int index = 0; index < 15; ++index) {
    const double angle = static_cast<double>(index) * 2.0 * kPi / 15.0;
    state = tracker.update({{7U, Eigen::Vector3d(
      5.0 + 0.2 * std::cos(angle), 0.2 * std::sin(angle), z_values[index % 3])}});
  }
  ASSERT_TRUE(state.has_value());
  EXPECT_TRUE(state->center_valid);
  EXPECT_TRUE(state->calibrated);
  EXPECT_NEAR(state->rotation_center_m.x(), 5.0, 0.08);
  EXPECT_NEAR(state->rotation_center_m.y(), 0.0, 0.08);
  EXPECT_NEAR(state->z_layers_m[0], 0.90, 0.01);
  EXPECT_NEAR(state->z_layers_m[1], 1.00, 0.01);
  EXPECT_NEAR(state->z_layers_m[2], 1.10, 0.01);
}

TEST(AntitopTracker, ResetClearsCalibration)
{
  AntitopTracker tracker(makeTestConfig());
  tracker.update({{7U, Eigen::Vector3d(5.0, 0.0, 1.0)}});
  tracker.reset();
  const auto state = tracker.update({{7U, Eigen::Vector3d(6.0, 0.0, 1.0)}});
  ASSERT_TRUE(state.has_value());
  EXPECT_FALSE(state->center_valid);
  EXPECT_FALSE(state->calibrated);
  EXPECT_EQ(state->center_sample_count, 1U);
}

TEST(AntitopTracker, DeterminesClockwiseDirectionFromImageX)
{
  auto config = makeTestConfig();
  config.direction_window_size = 2U;
  AntitopTracker tracker(config);
  ASSERT_TRUE(tracker.update({{7U, Eigen::Vector3d(5.0, 0.0, 1.0), 100.0}}).has_value());
  ASSERT_TRUE(tracker.update({{7U, Eigen::Vector3d(5.0, 0.0, 1.0), 99.0}}).has_value());
  const auto state = tracker.update({{7U, Eigen::Vector3d(5.0, 0.0, 1.0), 98.0}});
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->rotation_direction, 1);
}

}  // aim_antitop
