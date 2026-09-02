#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace armor_detector
{

struct ArmorDetection
{
  std::array<cv::Point2f, 4> corners;
  int id{-1};
  int color{-1};
  float confidence{0.0F};
};

struct ArmorDetectorConfig
{
  std::string model_path;
  std::string device_name{"CPU"};
  float confidence_threshold{0.75F};
  float nms_threshold{0.45F};
};

class ArmorDetector
{
public:
  explicit ArmorDetector(const ArmorDetectorConfig & config);
  ~ArmorDetector();
  ArmorDetector(const ArmorDetector &) = delete;
  ArmorDetector & operator=(const ArmorDetector &) = delete;

  std::vector<ArmorDetection> detect(const cv::Mat & image, int target_color);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // armor_detector
