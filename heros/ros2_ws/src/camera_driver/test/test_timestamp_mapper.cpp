#include <gtest/gtest.h>

#include "camera_driver/timestamp_mapper.hpp"

namespace
{

TEST(DeviceTimestampMapper, MapsDifferentTickFrequencies)
{
  camera_driver::DeviceTimestampMapper mapper;
  EXPECT_EQ(mapper.update(1000U, 1000U, 1000000000LL).update,
            camera_driver::TimestampUpdate::kInitialized);
  const auto result = mapper.update(1250U, 1000U, 1250000000LL);
  EXPECT_EQ(result.update, camera_driver::TimestampUpdate::kUpdated);
  EXPECT_EQ(result.mapped_host_steady_ns, 1250000000LL);
}

TEST(DeviceTimestampMapper, FiltersReceiveJitter)
{
  camera_driver::DeviceTimestampMapper mapper;
  mapper.update(0U, 1000000U, 1000000000LL);
  const auto result = mapper.update(10000U, 1000000U, 1020000000LL);
  EXPECT_EQ(result.mapped_host_steady_ns, 1010200000LL);
  EXPECT_NEAR(result.residual_offset_sec, 0.0002, 1e-9);
}

TEST(DeviceTimestampMapper, TracksSlowDrift)
{
  camera_driver::DeviceTimestampMapper mapper;
  mapper.update(0U, 1000U, 0LL);
  const auto first = mapper.update(1000U, 1000U, 1010000000LL);
  const auto second = mapper.update(2000U, 1000U, 2020000000LL);
  EXPECT_GT(second.residual_offset_sec, first.residual_offset_sec);
  EXPECT_LT(second.residual_offset_sec, 0.02);
}

TEST(DeviceTimestampMapper, ResetsOnTimestampRollback)
{
  camera_driver::DeviceTimestampMapper mapper;
  mapper.update(100U, 1000U, 0LL);
  const auto reset = mapper.update(100U, 1000U, 1000000LL);
  EXPECT_EQ(reset.update, camera_driver::TimestampUpdate::kReset);
  EXPECT_FALSE(mapper.initialized());
  EXPECT_EQ(mapper.resetCount(), 1U);
  EXPECT_EQ(mapper.update(1U, 1000U, 2000000LL).update,
            camera_driver::TimestampUpdate::kInitialized);
}

TEST(DeviceTimestampMapper, MaintainsIndependentCameraMappings)
{
  camera_driver::DeviceTimestampMapper first;
  camera_driver::DeviceTimestampMapper second;
  first.update(0U, 1000U, 1000000000LL);
  second.update(0U, 1000000U, 5000000000LL);
  EXPECT_EQ(first.update(100U, 1000U, 1100000000LL).mapped_host_steady_ns,
            1100000000LL);
  EXPECT_EQ(second.update(100000U, 1000000U, 5100000000LL).mapped_host_steady_ns,
            5100000000LL);
}

TEST(DeviceTimestampMapper, DetectsRosSteadyClockJumps)
{
  EXPECT_FALSE(camera_driver::hasRosSteadyClockJump(1000LL, 100001000LL, 100000000LL));
  EXPECT_TRUE(camera_driver::hasRosSteadyClockJump(1000LL, 100001001LL, 100000000LL));
  EXPECT_THROW(camera_driver::hasRosSteadyClockJump(0LL, 0LL, -1LL),
               std::invalid_argument);
}

TEST(DeviceTimestampMapper, RejectsInvalidConfiguration)
{
  EXPECT_THROW(camera_driver::DeviceTimestampMapper(0.0), std::invalid_argument);
  camera_driver::DeviceTimestampMapper mapper;
  EXPECT_THROW(mapper.update(1U, 0U, 0LL), std::invalid_argument);
}

TEST(DeviceTimestampMapper, ParsesTimestampModes)
{
  EXPECT_EQ(camera_driver::parseTimestampMode("host"),
            camera_driver::TimestampMode::kHost);
  EXPECT_EQ(camera_driver::parseTimestampMode("device"),
            camera_driver::TimestampMode::kDevice);
  EXPECT_THROW(camera_driver::parseTimestampMode("hardware"), std::invalid_argument);
}

}  // namespace
