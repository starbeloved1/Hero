#pragma once

#include <cstdint>

namespace command_mux {

enum class CommandSource : std::uint8_t {
  kNone,
  kNormal,
  kAntiTop,
  kAuto,
};

// 根据云台模式确定唯一允许通过的策略候选命令。
CommandSource sourceForMode(std::uint8_t mode);

// 候选命令的时间戳必须不晚于当前时刻，且年龄不超过上限。
bool isFresh(double command_time_sec, double now_sec, double max_age_sec);

// 仅允许当前模式对应的独立入口通过。
bool sourceMatchesMode(CommandSource source, std::uint8_t mode);

} // namespace command_mux
