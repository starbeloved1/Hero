#pragma once

#include <optional>
#include <string>

#include <hero_msgs/msg/armor_pose_array.hpp>
#include <hero_msgs/msg/control_command.hpp>
#include <hero_msgs/msg/gimbal_state.hpp>
#include <hero_msgs/msg/normal_aim_debug.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "aim_normal/normal_aimer.hpp"

namespace aim_normal
{

// nomalaim主流水线：选目标、解弹道、平滑角度并发布控制候选
class AimNormalNode : public rclcpp::Node
{
public:
  AimNormalNode();

private:
  void receiveGimbalState(const hero_msgs::msg::GimbalState::ConstSharedPtr & message);
  void receiveArmorPoses(const hero_msgs::msg::ArmorPoseArray::ConstSharedPtr & message);
  void publishDebug(
    const hero_msgs::msg::ArmorPoseArray & message,
    const std::optional<NormalAimResult> & result, bool shoot_status);
  void publishVisualization(
    const hero_msgs::msg::ArmorPoseArray & message,
    const std::optional<NormalAimResult> & result);

  rclcpp::Subscription<hero_msgs::msg::GimbalState>::SharedPtr gimbal_state_sub_;
  rclcpp::Subscription<hero_msgs::msg::ArmorPoseArray>::SharedPtr armor_pose_sub_;
  rclcpp::Publisher<hero_msgs::msg::ControlCommand>::SharedPtr control_candidate_pub_;
  rclcpp::Publisher<hero_msgs::msg::NormalAimDebug>::SharedPtr debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;

  std::optional<hero_msgs::msg::GimbalState> latest_gimbal_state_;
  NormalAimer aimer_;
  bool previous_right_clicked_{false};
  bool enabled_{false};
  bool enable_fire_{false};
  std::string target_frame_id_;
  int last_observed_marker_count_{0};
};

}  // aim_normal
