#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "hero_antibase/antibase_processor.hpp"

TEST(AntiBaseProcessor, FirstFrameProducesAStableVisualization) {
  hero_antibase::AntiBaseConfig config;
  config.output_fps = 1000;
  config.crop_size = 32;
  config.output_size = 16;
  config.min_packet_gap_ms = 22.0;
  hero_antibase::AntiBaseProcessor processor(config);
  const cv::Mat image(32, 48, CV_8UC3, cv::Scalar(10, 20, 30));
  const auto first = processor.process(image, {});
  EXPECT_EQ(first.visualization.cols, 16);
  EXPECT_EQ(first.visualization.rows, 16);
  EXPECT_FALSE(first.visualization.empty());
}

TEST(AntiBaseProcessor, ResetClearsImageHistory) {
  hero_antibase::AntiBaseConfig config;
  config.output_fps = 1000;
  config.crop_size = 32;
  config.output_size = 16;
  hero_antibase::AntiBaseProcessor processor(config);
  const cv::Mat image(32, 32, CV_8UC3, cv::Scalar(0, 0, 0));
  processor.process(image, {});
  processor.reset();
  const auto result = processor.process(image, {});
  EXPECT_FALSE(result.visualization.empty());
  EXPECT_FALSE(result.suppress_trail);
}
