#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include <hero_msgs/msg/control_command.hpp>
#include <hero_msgs/msg/gimbal_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "command_mux/candidate_selector.hpp"

namespace command_mux {

class CommandMuxNode : public rclcpp::Node {
public:
  CommandMuxNode();

private:
  struct CachedCandidate {
    hero_msgs::msg::ControlCommand command;
  };

  static constexpr std::size_t kCandidateCount = 3U;

  static std::size_t sourceIndex(CommandSource source);
  void receiveGimbalState(
      const hero_msgs::msg::GimbalState::ConstSharedPtr &message);
  void receiveCandidate(
      CommandSource source,
      const hero_msgs::msg::ControlCommand::ConstSharedPtr &message);
  void publishSelectedCommand();
  hero_msgs::msg::ControlCommand
  makeSafeCommand(const hero_msgs::msg::GimbalState &state,
                  const rclcpp::Time &stamp) const;

  rclcpp::Publisher<hero_msgs::msg::ControlCommand>::SharedPtr control_pub_;
  rclcpp::Subscription<hero_msgs::msg::GimbalState>::SharedPtr
      gimbal_state_sub_;
  std::array<rclcpp::Subscription<hero_msgs::msg::ControlCommand>::SharedPtr,
             kCandidateCount>
      candidate_subs_;
  rclcpp::TimerBase::SharedPtr output_timer_;

  std::mutex mutex_;
  std::optional<hero_msgs::msg::GimbalState> latest_gimbal_state_;
  std::array<std::optional<CachedCandidate>, kCandidateCount> candidates_;
  double max_command_age_sec_{0.1};
};

} // namespace command_mux
