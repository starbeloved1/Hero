#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "aim_antitop/antitop_tracker.hpp"

namespace aim_antitop
{

// 参数
struct AntitopControllerConfig
{
  std::size_t minimum_center_samples_for_zone{80U};
  double enter_zone_threshold_px{25.0};
  double exit_zone_threshold_px{35.0};
  std::size_t recent_z_window_size{20U};
  double z_runtime_match_threshold_m{0.05};
  double minimum_period_sec{0.5};
  double maximum_period_sec{1.0};
  std::size_t maximum_period_history_size{20U};
  double system_delay_sec{0.05};
  double clockwise_time_bias_sec{0.0};
  double counterclockwise_time_bias_sec{0.0};
};

struct AntitopControllerResult
{
  bool in_shoot_zone{false};
  std::int8_t matched_z_layer{-1};
  std::int8_t rotation_direction{-1};
  double target_z_m{0.0}; // 第二次 aim 使用的高度
  double average_period_sec{-1.0};
  bool countdown_active{false};
  double countdown_remaining_sec{0.0};
  bool shoot_ready{false};
};

// 反前哨控制器：管理射击区、周期、倒计时与一次性开火许可。
class AntitopController
{
public:
  explicit AntitopController(AntitopControllerConfig config);

  // 反前哨控制器入口 update：
  // 1. 保存当前跟踪板 Z，持续维护 recent_z_samples_ 的中位数窗口
  // 2. 比较“旋转中心投影 x”和“当前板像素中心 x”，以进入/退出阈值更新 in_shoot_zone_
  // 3. 区域上升沿记录一次旋转周期；至少得到两次有效周期后取最近三次均值
  // 4. 已标定且本次进入最低 Z 层时，用 3 * 周期 - 系统延迟 - 飞行时间 + 方向偏置启动倒计时
  // 5. 倒计时到达本帧时仅返回 shoot_ready=true 一次；节点再用 enable_fire 决定是否真正发射
  AntitopControllerResult update(
    const AntitopTrackerState & tracker_state, double center_image_x_px,
    double flight_time_sec, double control_stamp_sec);
  void reset();

private:
  [[nodiscard]] double recentZMedian() const;
  [[nodiscard]] std::int8_t matchZLayer(const AntitopTrackerState & tracker_state) const;
  [[nodiscard]] double averageRecentPeriods() const;
  void beginCountdown(
    const AntitopTrackerState & tracker_state, double flight_time_sec,
    double control_stamp_sec);
  [[nodiscard]] AntitopControllerResult makeResult(
    const AntitopTrackerState & tracker_state, double control_stamp_sec,
    bool shoot_ready) const;

  AntitopControllerConfig config_;
  std::vector<double> recent_z_samples_;
  std::vector<double> zone_periods_sec_;
  bool in_shoot_zone_{false};
  bool has_last_zone_entry_{false};
  double last_zone_entry_stamp_sec_{0.0};
  std::int8_t matched_z_layer_{-1};
  double average_period_sec_{-1.0};
  bool countdown_active_{false};
  double fire_stamp_sec_{0.0};
  std::optional<double> target_aim_z_m_;
};

}  // aim_antitop
