#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "hero_msgs/msg/armor_array.hpp"
#include "hero_msgs/msg/gimbal_state.hpp"

namespace armor_detector
{

class ArmorDetector;

class DetectorNode : public rclcpp::Node
{
public:
  DetectorNode();
  ~DetectorNode() override;

private:
  void receiveImage(const sensor_msgs::msg::Image::ConstSharedPtr & image);
  void receiveGimbalState(const hero_msgs::msg::GimbalState::ConstSharedPtr & state);
  void processLoop();
  void publishDetections(
    const sensor_msgs::msg::Image::ConstSharedPtr & image,
    const std::vector<class ArmorDetection> & detections);
  void publishVisualization(
    const sensor_msgs::msg::Image::ConstSharedPtr & image,
    const std::vector<class ArmorDetection> & detections);
  bool shouldPublishVisualization();
  int resolveTargetColor() const;

  std::unique_ptr<ArmorDetector> armor_detector_;
  rclcpp::Publisher<hero_msgs::msg::ArmorArray>::SharedPtr armor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<hero_msgs::msg::GimbalState>::SharedPtr gimbal_state_sub_;
  std::mutex image_mutex_;
  std::condition_variable image_cv_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  std::thread worker_;
  std::atomic<bool> running_{false};
  std::atomic<int> robot_color_{-1};
  int target_color_{-1};
  std::chrono::nanoseconds visualization_period_{0};
  std::chrono::steady_clock::time_point last_visualization_time_{};
};

}  // armor_detector
