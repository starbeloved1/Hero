#include <gtest/gtest.h>

#include <array>
#include <vector>

#include <opencv2/calib3d.hpp>

#include "armor_solver/armor_pnp.hpp"

namespace armor_solver
{

namespace
{

CameraIntrinsics testIntrinsics()
{
  CameraIntrinsics intrinsics;
  intrinsics.matrix = {1000.0, 0.0, 640.0, 0.0, 1000.0, 512.0, 0.0, 0.0, 1.0};
  intrinsics.distortion = {0.0, 0.0, 0.0, 0.0, 0.0};
  return intrinsics;
}

std::array<cv::Point2f, 4> projectedCorners(ArmorSize size, const cv::Vec3d & tvec)
{
  const auto intrinsics = testIntrinsics();
  const cv::Mat matrix = (cv::Mat_<double>(3, 3) <<
    intrinsics.matrix[0], intrinsics.matrix[1], intrinsics.matrix[2],
    intrinsics.matrix[3], intrinsics.matrix[4], intrinsics.matrix[5],
    intrinsics.matrix[6], intrinsics.matrix[7], intrinsics.matrix[8]);
  std::vector<cv::Point2f> projected;
  cv::projectPoints(
    armorObjectPoints(size), cv::Vec3d(0.08, -0.12, 0.03), tvec, matrix,
    intrinsics.distortion, projected);
  return {projected[0], projected[1], projected[2], projected[3]};
}

}  // namespace

TEST(ArmorPnp, SolvesSmallArmorWithPositiveDepth)
{
  const auto expected_tvec = cv::Vec3d(0.15, -0.08, 4.5);
  const auto result = solveArmorPnP(
    projectedCorners(ArmorSize::kSmall, expected_tvec), ArmorSize::kSmall, testIntrinsics());

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.tvec[0], expected_tvec[0], 1e-3);
  EXPECT_NEAR(result.tvec[1], expected_tvec[1], 1e-3);
  EXPECT_NEAR(result.tvec[2], expected_tvec[2], 1e-3);
  EXPECT_LT(result.reprojection_error, 1e-3);
}

TEST(ArmorPnp, SolvesLargeArmorWithPositiveDepth)
{
  const auto expected_tvec = cv::Vec3d(-0.25, 0.12, 6.0);
  const auto result = solveArmorPnP(
    projectedCorners(ArmorSize::kLarge, expected_tvec), ArmorSize::kLarge, testIntrinsics());

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.tvec[0], expected_tvec[0], 1e-3);
  EXPECT_NEAR(result.tvec[1], expected_tvec[1], 1e-3);
  EXPECT_NEAR(result.tvec[2], expected_tvec[2], 1e-3);
}

TEST(ArmorPnp, RejectsInvalidCameraIntrinsics)
{
  auto intrinsics = testIntrinsics();
  intrinsics.matrix[0] = 0.0;
  const auto result = solveArmorPnP(
    projectedCorners(ArmorSize::kSmall, cv::Vec3d(0.0, 0.0, 3.0)), ArmorSize::kSmall, intrinsics);

  EXPECT_FALSE(result.success);
}

TEST(ArmorPnp, KeepsOldHeroArmorSizeMapping)
{
  EXPECT_EQ(armorSizeForId(1U), ArmorSize::kLarge);
  EXPECT_EQ(armorSizeForId(3U), ArmorSize::kSmall);
}

}  // armor_solver
