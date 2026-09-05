#include <gtest/gtest.h>

#include "aim_predictor/target_tracker.hpp"

namespace
{

aim_predictor::TrackerConfig makeConfig()
{
  aim_predictor::TrackerConfig config;
  config.min_consecutive_detections = 1;
  config.max_lost_frames = 2;
  return config;
}

TEST(TargetTracker, InitializesFourArmorModel)
{
  aim_predictor::TargetTracker tracker(makeConfig());
  tracker.initialize(3U, {Eigen::Vector3d(3.0, 0.0, 0.2), 0.0}, 10.0);

  const auto estimate = tracker.estimate(10.0);
  EXPECT_TRUE(estimate.tracking);
  EXPECT_TRUE(estimate.converged);
  EXPECT_EQ(estimate.id, 3U);
  EXPECT_NEAR(estimate.center_m.x(), 3.2, 1e-9);
  EXPECT_NEAR(estimate.armors[0].position_m.x(), 3.0, 1e-9);
  EXPECT_NEAR(estimate.armors[2].position_m.x(), 3.4, 1e-9);
}

TEST(TargetTracker, RejectsExpiredTarget)
{
  aim_predictor::TargetTracker tracker(makeConfig());
  tracker.initialize(1U, {Eigen::Vector3d(2.0, 0.0, 0.0), 0.0}, 1.0);
  tracker.markLost();
  tracker.markLost();
  tracker.markLost();

  EXPECT_FALSE(tracker.estimate(1.1).tracking);
}

TEST(TargetTracker, RejectsInvalidConfig)
{
  auto config = makeConfig();
  config.max_valid_radius_m = config.min_valid_radius_m;
  EXPECT_THROW(aim_predictor::TargetTracker tracker(config), std::invalid_argument);
}

}  // namespace
