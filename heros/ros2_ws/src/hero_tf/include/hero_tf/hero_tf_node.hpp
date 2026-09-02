#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "hero_msgs/msg/gimbal_state.hpp"

namespace hero_tf
{

class HeroTfNode : public rclcpp::Node
{
public:
  HeroTfNode();

private:
  void publishDynamicTransform(const hero_msgs::msg::GimbalState & state);
  void publishStaticTransforms();

  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Subscription<hero_msgs::msg::GimbalState>::SharedPtr gimbal_state_sub_;
  std::string world_frame_id_;
  std::string gimbal_frame_id_;
  std::string camera_frame_id_;
  std::string camera_optical_frame_id_;
  double camera_trans_x_{0.0};
  double camera_trans_y_{0.0};
  double camera_trans_z_{0.0};
  double camera_yaw_deg_{0.0};
  double camera_pitch_deg_{0.0};
  double camera_roll_deg_{0.0};
};

}  // hero_tf
