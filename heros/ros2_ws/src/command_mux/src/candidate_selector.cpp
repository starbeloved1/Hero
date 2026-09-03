#include "command_mux/candidate_selector.hpp"

#include <cmath>

#include <hero_msgs/msg/gimbal_state.hpp>

namespace command_mux
{

CommandSource sourceForMode(std::uint8_t mode)
{
  switch (mode) {
    case hero_msgs::msg::GimbalState::MODE_NORMAL:
      return CommandSource::kNormalAim;
    case hero_msgs::msg::GimbalState::MODE_ANTI_TOP:
      return CommandSource::kAntiTop;
    case hero_msgs::msg::GimbalState::MODE_AUTO_AIM:
      return CommandSource::kAutoAim;
    case hero_msgs::msg::GimbalState::MODE_ANTI_BASE:
      return CommandSource::kAntiBase;
    default:
      return CommandSource::kNone;
  }
}

bool isFresh(double command_time_sec, double now_sec, double max_age_sec)
{
  return std::isfinite(command_time_sec) && std::isfinite(now_sec) &&
         std::isfinite(max_age_sec) && max_age_sec >= 0.0 &&
         command_time_sec <= now_sec && now_sec - command_time_sec <= max_age_sec;
}

}  // command_mux
