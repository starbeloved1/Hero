#include "aim_antitop/antitop_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace aim_antitop
{

AntitopController::AntitopController(AntitopControllerConfig config)
: config_(config)
{
  if (
    config_.minimum_center_samples_for_zone == 0U ||
    !std::isfinite(config_.enter_zone_threshold_px) ||
    !std::isfinite(config_.exit_zone_threshold_px) || config_.enter_zone_threshold_px <= 0.0 ||
    config_.exit_zone_threshold_px < config_.enter_zone_threshold_px ||
    config_.recent_z_window_size == 0U || !std::isfinite(config_.z_runtime_match_threshold_m) ||
    config_.z_runtime_match_threshold_m <= 0.0 || !std::isfinite(config_.minimum_period_sec) ||
    !std::isfinite(config_.maximum_period_sec) || config_.minimum_period_sec <= 0.0 ||
    config_.maximum_period_sec < config_.minimum_period_sec ||
    config_.maximum_period_history_size == 0U || !std::isfinite(config_.system_delay_sec) ||
    config_.system_delay_sec < 0.0 || !std::isfinite(config_.clockwise_time_bias_sec) ||
    !std::isfinite(config_.counterclockwise_time_bias_sec))
  {
    throw std::invalid_argument("反前哨开火状态机参数无效");
  }
}

AntitopControllerResult AntitopController::update(
  const AntitopTrackerState & tracker_state, double center_image_x_px,
  double flight_time_sec, double measurement_stamp_sec, double control_stamp_sec)
{
  if (
    !std::isfinite(center_image_x_px) || !std::isfinite(flight_time_sec) ||
    !std::isfinite(measurement_stamp_sec) || !std::isfinite(control_stamp_sec))
  {
    return makeResult(tracker_state, control_stamp_sec, false);
  }
  recent_z_samples_.push_back(tracker_state.tracked_armor.position_m.z());
  if (recent_z_samples_.size() > config_.recent_z_window_size) {
    recent_z_samples_.erase(recent_z_samples_.begin());
  }
  if (tracker_state.center_sample_count < config_.minimum_center_samples_for_zone) {
    return makeResult(tracker_state, control_stamp_sec, false);
  }
  if (tracker_state.calibrated && !target_aim_z_m_.has_value()) {
    target_aim_z_m_ = tracker_state.z_layers_m[0];
  }

  const double x_distance_px = std::abs(
    center_image_x_px - tracker_state.tracked_armor.image_center_x_px);
  const bool now_in_zone = x_distance_px < config_.enter_zone_threshold_px;
  const bool zone_rising = now_in_zone && !in_shoot_zone_;
  if (zone_rising) {
    if (has_last_zone_entry_) {
      // 周期属于目标物理运动，必须在图像采集时间 T0（这里记作 Tzone）上测量，
      // 不能被检测、PnP 或 ROS 调度延迟污染。
      const double period_sec = measurement_stamp_sec - last_zone_measurement_stamp_sec_;
      if (period_sec >= config_.minimum_period_sec && period_sec <= config_.maximum_period_sec) {
        zone_periods_sec_.push_back(period_sec);
        if (zone_periods_sec_.size() > config_.maximum_period_history_size) {
          zone_periods_sec_.erase(zone_periods_sec_.begin());
        }
      } else {
        zone_periods_sec_.clear();
        average_period_sec_ = -1.0;
        has_last_zone_entry_ = false;
      }
    }
    last_zone_measurement_stamp_sec_ = measurement_stamp_sec;
    has_last_zone_entry_ = true;
    average_period_sec_ = averageRecentPeriods();
  }
  if (now_in_zone) {
    in_shoot_zone_ = true;
  } else if (x_distance_px > config_.exit_zone_threshold_px) {
    in_shoot_zone_ = false;
  }

  if (zone_rising) {
    matched_z_layer_ = tracker_state.calibrated ? matchZLayer(tracker_state) : -1;
    if (matched_z_layer_ == 0) {
      const double observed_z = recentZMedian();
      if (std::isfinite(observed_z)) {
        target_aim_z_m_ = observed_z;
      }
    }
  }
  if (
    zone_rising && tracker_state.calibrated && !countdown_active_ && matched_z_layer_ == 0 &&
    average_period_sec_ > 0.0)
  {
    beginCountdown(tracker_state, flight_time_sec, measurement_stamp_sec);
  }
  // 目标事件先在 Tzone 时间轴上推出绝对 Tpermit；当前控制回调只负责比较 Tcontrol。
  const bool shoot_ready = countdown_active_ && control_stamp_sec >= permit_stamp_sec_;
  if (shoot_ready) {
    countdown_active_ = false;
  }
  return makeResult(tracker_state, control_stamp_sec, shoot_ready);
}

void AntitopController::reset()
{
  recent_z_samples_.clear();
  zone_periods_sec_.clear();
  in_shoot_zone_ = false;
  has_last_zone_entry_ = false;
  last_zone_measurement_stamp_sec_ = 0.0;
  matched_z_layer_ = -1;
  average_period_sec_ = -1.0;
  countdown_active_ = false;
  permit_stamp_sec_ = -1.0;
  target_aim_z_m_.reset();
}

double AntitopController::recentZMedian() const
{
  if (recent_z_samples_.size() < config_.recent_z_window_size) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto values = recent_z_samples_;
  std::sort(values.begin(), values.end());
  return values[values.size() / 2U];
}

std::int8_t AntitopController::matchZLayer(const AntitopTrackerState & tracker_state) const
{
  const double observed_z = recentZMedian();
  if (!std::isfinite(observed_z)) {
    return -2;
  }
  std::size_t best_index = 0U;
  double best_difference = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < tracker_state.z_layers_m.size(); ++index) {
    const double difference = std::abs(tracker_state.z_layers_m[index] - observed_z);
    if (difference < best_difference) {
      best_difference = difference;
      best_index = index;
    }
  }
  return best_difference <= config_.z_runtime_match_threshold_m ?
    static_cast<std::int8_t>(best_index) : -2;
}

double AntitopController::averageRecentPeriods() const
{
  if (zone_periods_sec_.size() < 2U) {
    return -1.0;
  }
  const std::size_t interval_count = std::min<std::size_t>(3U, zone_periods_sec_.size());
  double total = 0.0;
  for (std::size_t index = 0U; index < interval_count; ++index) {
    total += zone_periods_sec_[zone_periods_sec_.size() - index - 1U];
  }
  return total / static_cast<double>(interval_count);
}

void AntitopController::beginCountdown(
  const AntitopTrackerState & tracker_state, double flight_time_sec,
  double zone_stamp_sec)
{
  double bias_sec = 0.0;
  if (tracker_state.rotation_direction == 1) {
    bias_sec = config_.clockwise_time_bias_sec;
  } else if (tracker_state.rotation_direction == 0) {
    bias_sec = config_.counterclockwise_time_bias_sec;
  }
  permit_stamp_sec_ = zone_stamp_sec + 3.0 * average_period_sec_ -
    config_.system_delay_sec - flight_time_sec + bias_sec;
  countdown_active_ = true;
}

AntitopControllerResult AntitopController::makeResult(
  const AntitopTrackerState & tracker_state, double control_stamp_sec,
  bool shoot_ready) const
{
  AntitopControllerResult result;
  result.in_shoot_zone = in_shoot_zone_;
  result.matched_z_layer = matched_z_layer_;
  result.rotation_direction = tracker_state.rotation_direction;
  result.target_z_m = target_aim_z_m_.has_value() ? *target_aim_z_m_ :
    tracker_state.tracked_armor.position_m.z();
  result.average_period_sec = average_period_sec_;
  result.zone_stamp_sec = has_last_zone_entry_ ? last_zone_measurement_stamp_sec_ : -1.0;
  result.permit_stamp_sec = permit_stamp_sec_;
  result.countdown_active = countdown_active_;
  result.countdown_remaining_sec = countdown_active_ ?
    std::max(0.0, permit_stamp_sec_ - control_stamp_sec) : 0.0;
  result.shoot_ready = shoot_ready;
  return result;
}

}  // aim_antitop
