#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "armor_detector/armor_detector.hpp"
#include "armor_detector/armor_visualizer.hpp"

namespace armor_detector
{

TEST(ArmorVisualizer, DrawsOnCopyWithoutChangingInput)
{
  const cv::Mat input = cv::Mat::zeros(160, 240, CV_8UC3);
  ArmorDetection detection;
  detection.corners = {
    cv::Point2f(40.0F, 50.0F), cv::Point2f(160.0F, 50.0F),
    cv::Point2f(160.0F, 100.0F), cv::Point2f(40.0F, 100.0F)};
  detection.id = 1;
  detection.color = 1;
  detection.confidence = 0.95F;

  const auto output = drawDetections(input, {detection});

  EXPECT_EQ(cv::countNonZero(input.reshape(1)), 0);
  EXPECT_GT(cv::countNonZero(output.reshape(1)), 0);
}

}  // armor_detector
