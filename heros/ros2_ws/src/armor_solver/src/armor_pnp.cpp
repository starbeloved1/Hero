#include "armor_solver/armor_pnp.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/calib3d.hpp>

namespace armor_solver
{

namespace
{

constexpr double kSmallArmorHalfWidth = 0.0675;
constexpr double kLargeArmorHalfWidth = 0.115;
constexpr double kArmorHalfHeight = 0.028;

const std::vector<cv::Point3f> kSmallArmorPoints{
  {-kSmallArmorHalfWidth, kArmorHalfHeight, 0.0},
  {kSmallArmorHalfWidth, kArmorHalfHeight, 0.0},
  {kSmallArmorHalfWidth, -kArmorHalfHeight, 0.0},
  {-kSmallArmorHalfWidth, -kArmorHalfHeight, 0.0},
};

const std::vector<cv::Point3f> kLargeArmorPoints{
  {-kLargeArmorHalfWidth, kArmorHalfHeight, 0.0},
  {kLargeArmorHalfWidth, kArmorHalfHeight, 0.0},
  {kLargeArmorHalfWidth, -kArmorHalfHeight, 0.0},
  {-kLargeArmorHalfWidth, -kArmorHalfHeight, 0.0},
};

bool isValidIntrinsics(const CameraIntrinsics & intrinsics)
{
  for (const auto value : intrinsics.matrix) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return intrinsics.matrix[0] > 0.0 && intrinsics.matrix[4] > 0.0;
}

bool isValidPoints(const std::array<cv::Point2f, 4> & image_points)
{
  for (const auto & point : image_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return false;
    }
  }
  return true;
}

cv::Vec3d vectorFromMat(const cv::Mat & matrix)
{
  cv::Mat matrix_64f;
  matrix.reshape(1, 3).convertTo(matrix_64f, CV_64F);
  return {
    matrix_64f.at<double>(0), matrix_64f.at<double>(1), matrix_64f.at<double>(2)};
}

double reprojectionError(
  const std::vector<cv::Point3f> & object_points,
  const std::array<cv::Point2f, 4> & image_points,
  const cv::Vec3d & rvec,
  const cv::Vec3d & tvec,
  const cv::Mat & camera_matrix,
  const cv::Mat & distortion)
{
  std::vector<cv::Point2f> projected;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix, distortion, projected);
  double error = 0.0;
  for (std::size_t index = 0; index < image_points.size(); ++index) {
    error += cv::norm(projected[index] - image_points[index]);
  }
  return error / static_cast<double>(image_points.size());
}

}  // namespace

ArmorSize armorSizeForId(std::uint8_t id)
{
  return id == 1U ? ArmorSize::kLarge : ArmorSize::kSmall;
}

const std::vector<cv::Point3f> & armorObjectPoints(ArmorSize size)
{
  return size == ArmorSize::kLarge ? kLargeArmorPoints : kSmallArmorPoints;
}

ArmorPoseEstimate solveArmorPnP(
  const std::array<cv::Point2f, 4> & image_points,
  ArmorSize size,
  const CameraIntrinsics & intrinsics)
{
  ArmorPoseEstimate estimate;
  if (!isValidIntrinsics(intrinsics) || !isValidPoints(image_points)) {
    return estimate;
  }

  const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    intrinsics.matrix[0], intrinsics.matrix[1], intrinsics.matrix[2],
    intrinsics.matrix[3], intrinsics.matrix[4], intrinsics.matrix[5],
    intrinsics.matrix[6], intrinsics.matrix[7], intrinsics.matrix[8]);
  const cv::Mat distortion = cv::Mat(intrinsics.distortion).clone().reshape(1, 1);
  const auto & object_points = armorObjectPoints(size);
  const std::vector<cv::Point2f> points(image_points.begin(), image_points.end());

  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  const auto solution_count = cv::solvePnPGeneric(
    object_points, points, camera_matrix, distortion, rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
  if (solution_count <= 0) {
    return estimate;
  }

  double best_error = std::numeric_limits<double>::max();
  for (std::size_t index = 0; index < rvecs.size() && index < tvecs.size(); ++index) {
    const auto rvec = vectorFromMat(rvecs[index]);
    const auto tvec = vectorFromMat(tvecs[index]);
    if (!std::isfinite(tvec[0]) || !std::isfinite(tvec[1]) || !std::isfinite(tvec[2]) || tvec[2] <= 0.0) {
      continue;
    }
    const auto error = reprojectionError(
      object_points, image_points, rvec, tvec, camera_matrix, distortion);
    if (std::isfinite(error) && error < best_error) {
      estimate.rvec = rvec;
      estimate.tvec = tvec;
      estimate.reprojection_error = error;
      estimate.success = true;
      best_error = error;
    }
  }
  return estimate;
}

}  // armor_solver
