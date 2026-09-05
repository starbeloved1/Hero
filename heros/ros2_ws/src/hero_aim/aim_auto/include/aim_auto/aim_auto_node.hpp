#pragma once

#include <optional>
#include <string>

#include <hero_msgs/msg/auto_aim_debug.hpp>
#include <hero_msgs/msg/control_command.hpp>
#include <hero_msgs/msg/gimbal_state.hpp>
#include <hero_msgs/msg/target_state_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "aim_auto/auto_aimer.hpp"

namespace aim_auto
{

class AimAutoNode : public rclcpp::Node
{
public:
  AimAutoNode();

private:
  void receiveGimbalState(const hero_msgs::msg::GimbalState::ConstSharedPtr & message);
  void receiveTargetStates(const hero_msgs::msg::TargetStateArray::ConstSharedPtr & message);
  void publishDebug(
    const hero_msgs::msg::TargetStateArray & states, const std::optional<AutoAimResult> & result,
    bool shoot_status, const rclcpp::Time & control_time);

  rclcpp::Subscription<hero_msgs::msg::GimbalState>::SharedPtr gimbal_state_sub_;
  rclcpp::Subscription<hero_msgs::msg::TargetStateArray>::SharedPtr target_state_sub_;
  rclcpp::Publisher<hero_msgs::msg::ControlCommand>::SharedPtr control_candidate_pub_;
  rclcpp::Publisher<hero_msgs::msg::AutoAimDebug>::SharedPtr debug_pub_;

  std::optional<hero_msgs::msg::GimbalState> latest_gimbal_state_;
  AutoAimer aimer_;
  bool previous_right_clicked_{false};
  bool enabled_{false};
  bool enable_fire_{false};
  std::string target_frame_id_;
};

}  // aim_auto
