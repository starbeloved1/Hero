#include "command_mux/candidate_selector.hpp"

#include <cmath>

#include <hero_msgs/msg/gimbal_state.hpp>

namespace command_mux {

CommandSource sourceForMode(std::uint8_t mode) {
  switch (mode) {
  case hero_msgs::msg::GimbalState::MODE_NORMAL:
    return CommandSource::kNormal;
  case hero_msgs::msg::GimbalState::MODE_ANTI_TOP:
    return CommandSource::kAntiTop;
  case hero_msgs::msg::GimbalState::MODE_AUTO_AIM:
    return CommandSource::kAuto;
  case hero_msgs::msg::GimbalState::MODE_ANTI_BASE:
    // mode4 只发送反基地视频码流，不产生 yaw/pitch 控制候选。
    return CommandSource::kNone;
  default:
    return CommandSource::kNone;
  }
}

bool isFresh(double command_time_sec, double now_sec, double max_age_sec) {
  return std::isfinite(command_time_sec) && std::isfinite(now_sec) &&
         std::isfinite(max_age_sec) && max_age_sec >= 0.0 &&
         command_time_sec <= now_sec &&
         now_sec - command_time_sec <= max_age_sec;
}

bool sourceMatchesMode(CommandSource source, std::uint8_t mode) {
  return source == sourceForMode(mode) && source != CommandSource::kNone;
}

} // namespace command_mux
