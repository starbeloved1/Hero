#include "aim_core/ballistics.hpp"

#include <cmath>

namespace aim_core
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

bool isValidConfig(const BallisticConfig & config)
{
  return std::isfinite(config.bullet_speed_mps) && config.bullet_speed_mps > 0.0 &&
         std::isfinite(config.drag_coefficient) && config.drag_coefficient >= 0.0 &&
         std::isfinite(config.gravity_mps2) && config.gravity_mps2 > 0.0 &&
         std::isfinite(config.air_density_kgpm3) && config.air_density_kgpm3 > 0.0 &&
         std::isfinite(config.bullet_mass_kg) && config.bullet_mass_kg > 0.0 &&
         std::isfinite(config.bullet_radius_m) && config.bullet_radius_m > 0.0 &&
         std::isfinite(config.muzzle_offset_m) && config.iteration_count > 0;
}

}  // namespace

std::optional<Trajectory> solveBallistics(
  const BallisticConfig & config, double x_m, double y_m, double z_m,
  double horizontal_distance_scale)
{
  if (
    !isValidConfig(config) || !std::isfinite(x_m) || !std::isfinite(y_m) ||
    !std::isfinite(z_m) || !std::isfinite(horizontal_distance_scale) ||
    horizontal_distance_scale <= 0.0)
  {
    return std::nullopt;
  }

  const double horizontal_distance = std::hypot(x_m, y_m) * horizontal_distance_scale;
  if (horizontal_distance <= 1e-6) {
    return std::nullopt;
  }

  const double area = kPi * config.bullet_radius_m * config.bullet_radius_m;
  const double drag_factor =
    config.drag_coefficient * config.air_density_kgpm3 * area / (2.0 * config.bullet_mass_kg);
  double pitch_rad = std::atan2(z_m, horizontal_distance);
  double flight_time_sec = 0.0;

  for (int iteration = 0; iteration < config.iteration_count; ++iteration) {
    const double cosine = std::cos(pitch_rad);
    if (std::abs(cosine) <= 1e-6) {
      return std::nullopt;
    }
    const double muzzle_distance = horizontal_distance - config.muzzle_offset_m * cosine;
    const double muzzle_height = z_m - config.muzzle_offset_m * std::sin(pitch_rad);
    if (muzzle_distance <= 0.0) {
      return std::nullopt;
    }

    if (drag_factor <= 1e-9) {
      flight_time_sec = muzzle_distance / (config.bullet_speed_mps * cosine);
    } else {
      flight_time_sec =
        std::expm1(drag_factor * muzzle_distance) /
        (drag_factor * config.bullet_speed_mps * cosine);
    }
    if (!std::isfinite(flight_time_sec) || flight_time_sec <= 0.0) {
      return std::nullopt;
    }

    const double predicted_height =
      config.bullet_speed_mps * std::sin(pitch_rad) * flight_time_sec -
      0.5 * config.gravity_mps2 * flight_time_sec * flight_time_sec;
    const double error = muzzle_height - predicted_height;
    const double derivative =
      config.bullet_speed_mps * cosine * flight_time_sec +
      config.muzzle_offset_m * cosine;
    if (!std::isfinite(error) || !std::isfinite(derivative) || std::abs(derivative) <= 1e-9) {
      return std::nullopt;
    }
    pitch_rad += error / derivative;
    if (!std::isfinite(pitch_rad) || std::abs(pitch_rad) >= kPi / 2.0) {
      return std::nullopt;
    }
  }

  return Trajectory{std::atan2(y_m, x_m), pitch_rad, flight_time_sec};
}

}  // aim_core
