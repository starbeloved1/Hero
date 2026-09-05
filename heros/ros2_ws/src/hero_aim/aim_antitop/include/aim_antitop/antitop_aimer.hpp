#pragma once

#include <optional>

#include "aim_antitop/antitop_tracker.hpp"
#include "aim_core/ballistics.hpp"

namespace aim_antitop
{

struct AntitopAimerConfig
{
  aim_core::BallisticConfig ballistics;
};

// 成功瞄准后的输出
struct AntitopAimResult
{
  AntitopTrackerState tracker_state;
  double target_z_m{0.0};
  double raw_yaw_rad{0.0};
  double raw_pitch_rad{0.0};
  double command_yaw_rad{0.0};
  double command_pitch_rad{0.0};
  double flight_time_sec{0.0};
  bool pitch_locked{false};
};

// antitop 瞄准器
class AntitopAimer
{
public:
  explicit AntitopAimer(AntitopAimerConfig config);

  // 反前哨瞄准入口 aim：
  // 1. 检查 tracker_state.center_valid，旋转中心或当前云台角无效时直接返回空
  // 2. 选择目标高度：优先使用 controller 给出的 target_z_override_m；否则未标定时使用当前 tracking_armor.z，标定后使用最低层 z_layers_m[0]
  // 3. aim_core::solveBallistics()：以“旋转中心 XY + 目标高度 Z”解算弹道 yaw、pitch、飞行时间
  // 4. aim_core::unwrapNear()：将理论 yaw 调整到最接近当前云台 yaw 的等价角
  // 5. 未标定时 command_pitch 保持当前云台 pitch；标定后锁定为弹道 pitch
  // 6. 打包 AntitopAimResult 返回；本类不判射击区、周期或开火许可
  std::optional<AntitopAimResult> aim(
    const AntitopTrackerState & tracker_state, double gimbal_yaw_rad,
    double gimbal_pitch_rad, std::optional<double> target_z_override_m = std::nullopt) const;

private:
  AntitopAimerConfig config_;
};

}  // aim_antitop
