#include "aim_core/angle_utils.hpp"

#include <cmath>
#include <stdexcept>

namespace aim_core
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

void validateConfig(const AngleSmootherConfig & config)
{
  if (
    !std::isfinite(config.yaw_previous_weight) || !std::isfinite(config.pitch_previous_weight) ||
    config.yaw_previous_weight < 0.0 || config.yaw_previous_weight > 1.0 ||
    config.pitch_previous_weight < 0.0 || config.pitch_previous_weight > 1.0 ||
    !std::isfinite(config.yaw_jump_threshold_rad) ||
    !std::isfinite(config.pitch_jump_threshold_rad) ||
    config.yaw_jump_threshold_rad < 0.0 || config.pitch_jump_threshold_rad < 0.0)
  {
    throw std::invalid_argument("角度平滑参数无效");
  }
}

}  // namespace

double normalizeRadians(double angle_rad)
{
  if (!std::isfinite(angle_rad)) {
    return angle_rad;
  }
  return std::remainder(angle_rad, 2.0 * kPi);
}

double unwrapNear(double angle_rad, double reference_rad)
{
  if (!std::isfinite(angle_rad) || !std::isfinite(reference_rad)) {
    return angle_rad;
  }
  return reference_rad + normalizeRadians(angle_rad - reference_rad);
}

AngleSmoother::AngleSmoother(AngleSmootherConfig config)
: config_(config)
{
  validateConfig(config_);
}

AimAngles AngleSmoother::filter(double yaw_rad, double pitch_rad)
{
  if (!std::isfinite(yaw_rad) || !std::isfinite(pitch_rad)) {
    throw std::invalid_argument("待平滑角度必须为有限值");
  }

  AimAngles current{yaw_rad, pitch_rad};
  if (!has_previous_) {
    previous_ = current;
    has_previous_ = true;
    return current;
  }

  current.yaw_rad = unwrapNear(current.yaw_rad, previous_.yaw_rad);
  const double yaw_difference = std::abs(current.yaw_rad - previous_.yaw_rad);
  const double pitch_difference = std::abs(current.pitch_rad - previous_.pitch_rad);
  if (
    yaw_difference <= config_.yaw_jump_threshold_rad &&
    pitch_difference <= config_.pitch_jump_threshold_rad)
  {
    current.yaw_rad =
      previous_.yaw_rad * config_.yaw_previous_weight +
      current.yaw_rad * (1.0 - config_.yaw_previous_weight);
    current.pitch_rad =
      previous_.pitch_rad * config_.pitch_previous_weight +
      current.pitch_rad * (1.0 - config_.pitch_previous_weight);
  }
  previous_ = current;
  return current;
}

void AngleSmoother::reset()
{
  has_previous_ = false;
}

}  // aim_core
