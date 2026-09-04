#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "camera_router/camera_profile.hpp"
#include "hero_msgs/msg/gimbal_state.hpp"

namespace camera_router
{

class CameraRouterNode : public rclcpp::Node
{
public:
  CameraRouterNode();

private:
  void receiveGimbalState(const hero_msgs::msg::GimbalState & state);
  void forwardImage(CameraProfile source_profile, const sensor_msgs::msg::Image & image);
  void forwardCameraInfo(
    CameraProfile source_profile, const sensor_msgs::msg::CameraInfo & camera_info);

  bool base_camera_enabled_{true};
  CameraProfile active_profile_{CameraProfile::kAim8mm};
  std::string selected_frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr selected_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr selected_camera_info_pub_;
  rclcpp::Subscription<hero_msgs::msg::GimbalState>::SharedPtr gimbal_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr aim8mm_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr aim8mm_camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr base_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr base_camera_info_sub_;
};

}  // camera_router
