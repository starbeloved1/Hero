#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "aim_core/angle_utils.hpp"
#include "aim_core/ballistics.hpp"
#include "aim_normal/armor_selector.hpp"

namespace aim_normal
{

struct NormalAimerConfig
{
  double keep_target_distance_m{0.3};
  std::uint32_t max_lost_frames{0U};
  double large_armor_distance_factor{1.2};
  aim_core::BallisticConfig ballistic;
  aim_core::AngleSmootherConfig smoother;
};

struct NormalAimResult
{
  SelectedArmor selected;
  double raw_yaw_rad{0.0};
  double raw_pitch_rad{0.0};
  double command_yaw_rad{0.0};
  double command_pitch_rad{0.0};
  double flight_time_sec{0.0};
};

// normalaim的算法核心：连续目标选择、阻力弹道与角度平滑
class NormalAimer
{
public:
  explicit NormalAimer(NormalAimerConfig config);

  // 调用aim_core中的算法层，输出NormalAimResult
  std::optional<NormalAimResult> aim(
    const std::vector<ArmorObservation> & observations, double gimbal_yaw_rad,
    double gimbal_pitch_rad);
  void reset();

private:
  NormalAimerConfig config_;
  ArmorSelector selector_;
  aim_core::AngleSmoother smoother_;
};

}  // aim_normal
