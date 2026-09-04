#include "aim_normal/normal_aimer.hpp"

#include <cmath>
#include <stdexcept>

#include "aim_core/angle_utils.hpp"

namespace aim_normal
{

NormalAimer::NormalAimer(NormalAimerConfig config)
: config_(config),
  selector_(config_.keep_target_distance_m, config_.max_lost_frames),
  smoother_(config_.smoother)
{
  if (
    !std::isfinite(config_.large_armor_distance_factor) ||
    config_.large_armor_distance_factor <= 0.0)
  {
    throw std::invalid_argument("large_armor_distance_factor 必须大于 0");
  }
}

std::optional<NormalAimResult> NormalAimer::aim(
  const std::vector<ArmorObservation> & observations, double gimbal_yaw_rad,
  double gimbal_pitch_rad)
{
  const auto selected = selector_.select(observations, gimbal_yaw_rad, gimbal_pitch_rad);
  if (!selected.has_value()) {
    return std::nullopt;
  }

  const double distance_scale = selected->observation.is_large ?
    1.0 / config_.large_armor_distance_factor : 1.0;
  const auto trajectory = aim_core::solveBallistics(
    config_.ballistic, selected->observation.x, selected->observation.y,
    selected->observation.z, distance_scale);
  if (!trajectory.has_value()) {
    return std::nullopt;
  }

  const double raw_yaw_rad = aim_core::unwrapNear(trajectory->yaw_rad, gimbal_yaw_rad);
  const auto command = smoother_.filter(raw_yaw_rad, trajectory->pitch_rad);
  return NormalAimResult{
    *selected,
    raw_yaw_rad,
    trajectory->pitch_rad,
    command.yaw_rad,
    command.pitch_rad,
    trajectory->flight_time_sec};
}

void NormalAimer::reset()
{
  selector_.reset();
  smoother_.reset();
}

}  // aim_normal
