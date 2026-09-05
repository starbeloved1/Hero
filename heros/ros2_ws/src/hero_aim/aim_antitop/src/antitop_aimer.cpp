#include "aim_antitop/antitop_aimer.hpp"

#include <cmath>

#include "aim_core/angle_utils.hpp"

namespace aim_antitop
{

AntitopAimer::AntitopAimer(AntitopAimerConfig config)
: config_(config)
{}

std::optional<AntitopAimResult> AntitopAimer::aim(
  const AntitopTrackerState & tracker_state, double gimbal_yaw_rad,
  double gimbal_pitch_rad, std::optional<double> target_z_override_m) const
{
  if (
    !tracker_state.center_valid || !tracker_state.rotation_center_m.allFinite() ||
    !std::isfinite(gimbal_yaw_rad) || !std::isfinite(gimbal_pitch_rad))
  {
    return std::nullopt;
  }
  const double target_z = target_z_override_m.has_value() ? *target_z_override_m :
    (tracker_state.calibrated ? tracker_state.z_layers_m[0] : tracker_state.tracked_armor.position_m.z());
  if (!std::isfinite(target_z)) {
    return std::nullopt;
  }
  const auto trajectory = aim_core::solveBallistics(
    config_.ballistics, tracker_state.rotation_center_m.x(), tracker_state.rotation_center_m.y(),
    target_z);
  if (!trajectory.has_value()) {
    return std::nullopt;
  }
  AntitopAimResult result;
  result.tracker_state = tracker_state;
  result.target_z_m = target_z;
  result.raw_yaw_rad = aim_core::unwrapNear(trajectory->yaw_rad, gimbal_yaw_rad);
  result.raw_pitch_rad = trajectory->pitch_rad;
  result.command_yaw_rad = result.raw_yaw_rad;
  result.command_pitch_rad = tracker_state.calibrated ? result.raw_pitch_rad : gimbal_pitch_rad;
  result.flight_time_sec = trajectory->flight_time_sec;
  result.pitch_locked = tracker_state.calibrated;
  return result;
}

}  // aim_antitop
