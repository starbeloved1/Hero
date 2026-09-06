#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "hero_antibase/antibase_processor.hpp"
#include "hero_msgs/msg/anti_base_debug.hpp"
#include "hero_msgs/msg/anti_base_packet.hpp"
#include "hero_msgs/msg/anti_base_tx_status.hpp"
#include "hero_msgs/msg/gimbal_state.hpp"

namespace hero_antibase {

class HeroAntiBaseNode : public rclcpp::Node {
public:
  HeroAntiBaseNode();
  ~HeroAntiBaseNode() override;

private:
  void receiveImage(const sensor_msgs::msg::Image::ConstSharedPtr &image);
  void
  receiveGimbalState(const hero_msgs::msg::GimbalState::ConstSharedPtr &state);
  void receiveTxStatus(
      const hero_msgs::msg::AntiBaseTxStatus::ConstSharedPtr &status);
  void processLoop();
  void publishResult(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                     const ProcessorResult &result);
  bool shouldPublishVisualization();

  std::unique_ptr<AntiBaseProcessor> processor_;
  rclcpp::Publisher<hero_msgs::msg::AntiBasePacket>::SharedPtr packet_pub_;
  rclcpp::Publisher<hero_msgs::msg::AntiBaseDebug>::SharedPtr debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<hero_msgs::msg::GimbalState>::SharedPtr
      gimbal_state_sub_;
  rclcpp::Subscription<hero_msgs::msg::AntiBaseTxStatus>::SharedPtr
      tx_status_sub_;

  std::mutex image_mutex_;
  std::condition_variable image_cv_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  std::mutex processor_mutex_;
  std::mutex feedback_mutex_;
  TxFeedback feedback_;
  std::thread worker_;
  std::atomic<bool> running_{false};
  std::atomic<uint8_t> mode_{hero_msgs::msg::GimbalState::MODE_NORMAL};
  std::atomic<bool> reset_requested_{false};
  std::chrono::nanoseconds visualization_period_{0};
  std::chrono::steady_clock::time_point last_visualization_time_{};
};

} // namespace hero_antibase
