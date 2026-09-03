#include "armor_detector/armor_visualizer.hpp"

#include <array>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "armor_detector/armor_detector.hpp"

namespace armor_detector
{

namespace
{

cv::Scalar colorForDetection(int color)
{
  // OpenCV 使用 BGR 顺序；0=蓝色、1=红色，其他模型颜色使用黄色提示。
  if (color == 0) {
    return cv::Scalar(255, 100, 0);
  }
  if (color == 1) {
    return cv::Scalar(0, 100, 255);
  }
  return cv::Scalar(0, 220, 255);
}

const char * colorName(int color)
{
  if (color == 0) {
    return "B";
  }
  if (color == 1) {
    return "R";
  }
  return "?";
}

}  // namespace

cv::Mat drawDetections(
  const cv::Mat & image, const std::vector<ArmorDetection> & detections)
{
  cv::Mat visualization = image.clone();
  for (const auto & detection : detections) {
    const auto color = colorForDetection(detection.color);
    std::vector<cv::Point> corners;
    corners.reserve(detection.corners.size());
    cv::Point center;
    for (const auto & corner : detection.corners) {
      const cv::Point point(cvRound(corner.x), cvRound(corner.y));
      corners.push_back(point);
      center += point;
    }
    center.x /= static_cast<int>(detection.corners.size());
    center.y /= static_cast<int>(detection.corners.size());

    cv::polylines(visualization, corners, true, color, 2, cv::LINE_AA);
    cv::circle(visualization, center, 3, color, cv::FILLED, cv::LINE_AA);
    const std::string label = "ID " + std::to_string(detection.id) + " " +
      colorName(detection.color) + " " + cv::format("%.2f", detection.confidence);
    cv::putText(
      visualization, label, center + cv::Point(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.55,
      color, 2, cv::LINE_AA);
  }
  return visualization;
}

}  // armor_detector
