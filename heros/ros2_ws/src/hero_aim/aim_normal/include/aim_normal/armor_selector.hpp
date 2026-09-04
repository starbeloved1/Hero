#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace aim_normal
{

struct ArmorObservation
{
  std::uint8_t id{0U};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  bool is_large{false};
};

struct SelectedArmor
{
  ArmorObservation observation;
  bool held_target{false};
  std::uint32_t lost_frames{0U};
};

// 普通模式策略的轻量目标选择器：优先保持同编号且空间连续的目标。
class ArmorSelector
{
public:
  ArmorSelector(double keep_target_distance_m, std::uint32_t max_lost_frames);

  std::optional<SelectedArmor> select(
    const std::vector<ArmorObservation> & observations, double current_yaw_rad,
    double current_pitch_rad);
  void reset();

private:
  double keep_target_distance_m_{0.3};
  std::uint32_t max_lost_frames_{0U};
  std::uint32_t lost_frames_{0U};
  std::optional<ArmorObservation> last_target_;
};

}  // aim_normal
