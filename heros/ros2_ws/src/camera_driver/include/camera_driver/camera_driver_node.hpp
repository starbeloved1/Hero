#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace camera_driver
{

class CameraDriverNode : public rclcpp::Node
{
public:
  CameraDriverNode();
  ~CameraDriverNode() override;

private:
  struct Stream;
  struct Frame;

  void captureLoop(Stream & stream);
  bool readFrame(Stream & stream, bool convert_to_bgr, Frame & frame, std::string & error);
  bool readVideoFrame(Stream & stream, bool convert_to_bgr, Frame & frame, std::string & error);
  std::optional<rclcpp::Time> makeFrameStamp(
    Stream & stream, const Frame & frame, const rclcpp::Time & host_publish_stamp,
    rclcpp::Time & host_receive_stamp);
  std::optional<rclcpp::Time> makeBaseFrameStamp(
    Stream & stream, const Frame & frame, const rclcpp::Time & host_publish_stamp);

  std::atomic<bool> running_{false};
  std::atomic<std::uint8_t> gimbal_mode_{1U};
  rclcpp::SubscriptionBase::SharedPtr gimbal_state_sub_;
  std::vector<std::unique_ptr<Stream>> streams_;
};

}  // camera_driver
