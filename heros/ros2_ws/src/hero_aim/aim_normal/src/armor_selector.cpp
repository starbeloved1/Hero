#include "aim_normal/armor_selector.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "aim_core/angle_utils.hpp"

namespace aim_normal
{

namespace
{

double squaredDistance(const ArmorObservation & first, const ArmorObservation & second)
{
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return dx * dx + dy * dy + dz * dz;
}

double viewDirectionDistance(
  const ArmorObservation & observation, double current_yaw_rad, double current_pitch_rad)
{
  const double horizontal_distance = std::hypot(observation.x, observation.y);
  if (horizontal_distance <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double yaw = std::atan2(observation.y, observation.x);
  const double pitch = std::atan2(observation.z, horizontal_distance);
  const double yaw_difference = aim_core::normalizeRadians(yaw - current_yaw_rad);
  const double pitch_difference = pitch - current_pitch_rad;
  return yaw_difference * yaw_difference + pitch_difference * pitch_difference;
}

}  // namespace

ArmorSelector::ArmorSelector(double keep_target_distance_m, std::uint32_t max_lost_frames)
: keep_target_distance_m_(keep_target_distance_m), max_lost_frames_(max_lost_frames)
{
  if (!std::isfinite(keep_target_distance_m_) || keep_target_distance_m_ <= 0.0) {
    throw std::invalid_argument("keep_target_distance_m 必须大于 0");
  }
}

std::optional<SelectedArmor> ArmorSelector::select(
  const std::vector<ArmorObservation> & observations, double current_yaw_rad,
  double current_pitch_rad)
{
  if (!std::isfinite(current_yaw_rad) || !std::isfinite(current_pitch_rad)) {
    throw std::invalid_argument("当前云台角度必须为有限值");
  }

  if (!observations.empty() && last_target_.has_value()) {
    std::optional<std::size_t> closest_same_id;
    double smallest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < observations.size(); ++index) {
      if (observations[index].id != last_target_->id) {
        continue;
      }
      const double distance = squaredDistance(observations[index], *last_target_);
      if (distance < smallest_distance) {
        smallest_distance = distance;
        closest_same_id = index;
      }
    }
    if (
      closest_same_id.has_value() &&
      smallest_distance <= keep_target_distance_m_ * keep_target_distance_m_)
    {
      last_target_ = observations[*closest_same_id];
      lost_frames_ = 0U;
      return SelectedArmor{*last_target_, false, lost_frames_};
    }
  }

  if (observations.empty() && last_target_.has_value() && lost_frames_ < max_lost_frames_) {
    ++lost_frames_;
    return SelectedArmor{*last_target_, true, lost_frames_};
  }

  if (observations.empty()) {
    last_target_.reset();
    lost_frames_ = 0U;
    return std::nullopt;
  }

  std::size_t selected_index = 0U;
  double smallest_view_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < observations.size(); ++index) {
    const double view_distance = viewDirectionDistance(
      observations[index], current_yaw_rad, current_pitch_rad);
    if (view_distance < smallest_view_distance) {
      smallest_view_distance = view_distance;
      selected_index = index;
    }
  }
  if (!std::isfinite(smallest_view_distance)) {
    return std::nullopt;
  }
  last_target_ = observations[selected_index];
  lost_frames_ = 0U;
  return SelectedArmor{*last_target_, false, lost_frames_};
}

void ArmorSelector::reset()
{
  last_target_.reset();
  lost_frames_ = 0U;
}

}  // aim_normal
