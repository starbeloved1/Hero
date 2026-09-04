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

// normalaim的轻量目标选择器：优先保持同编号且空间连续的目标
class ArmorSelector
{
public:
  ArmorSelector(double keep_target_distance_m, std::uint32_t max_lost_frames);

  //输入检测到的装甲板集合，输出选中结果
  /*
  1.优先找与上一帧同 ID、且三维距离连续的装甲板；
  2.没有连续目标且允许保持时，返回旧目标并标记 held_target=true；
  3.否则在当前所有目标中，选取最接近当前云台视线方向的目标；
  4.没有目标时返回空
  */
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
