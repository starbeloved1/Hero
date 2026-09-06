#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace armor_detector
{

struct ArmorDetection;

// 在输入 BGR 图像副本上绘制检测框、类别和置信度
cv::Mat drawDetections(
  const cv::Mat & image, const std::vector<ArmorDetection> & detections);

}  // armor_detector
