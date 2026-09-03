#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace armor_solver
{

enum class ArmorSize
{
  kSmall,
  kLarge,
};

struct CameraIntrinsics
{
  std::array<double, 9> matrix{};
  std::vector<double> distortion;
};

struct ArmorPoseEstimate
{
  cv::Vec3d rvec{};
  cv::Vec3d tvec{};
  double reprojection_error{0.0};
  bool success{false};
};

// 保持旧 Hero 的约定：只有英雄（ID 1）使用大装甲板尺寸。
ArmorSize armorSizeForId(std::uint8_t id);

// 返回与旧 Solver 一致的装甲板物体角点顺序。
const std::vector<cv::Point3f> & armorObjectPoints(ArmorSize size);

// 使用 IPPE 求解平面装甲板的全部候选位姿，并选择正深度且重投影误差最小者。
ArmorPoseEstimate solveArmorPnP(
  const std::array<cv::Point2f, 4> & image_points,
  ArmorSize size,
  const CameraIntrinsics & intrinsics);

}  // armor_solver
