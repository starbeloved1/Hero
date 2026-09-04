#pragma once

#include <limits>
#include <optional>

namespace aim_core
{

struct BallisticConfig
{
  // 所有真实数值必须由 aim_core/config/ballistics.yaml 提供；NaN 表示尚未配置。
  double bullet_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double drag_coefficient{std::numeric_limits<double>::quiet_NaN()};
  double gravity_mps2{std::numeric_limits<double>::quiet_NaN()};
  double air_density_kgpm3{std::numeric_limits<double>::quiet_NaN()};
  double bullet_mass_kg{std::numeric_limits<double>::quiet_NaN()};
  double bullet_radius_m{std::numeric_limits<double>::quiet_NaN()};
  double muzzle_offset_m{std::numeric_limits<double>::quiet_NaN()};
  int iteration_count{0};
};

struct Trajectory
{
  double yaw_rad{0.0};
  double pitch_rad{0.0};
  double flight_time_sec{0.0};
};

// 阻力弹道迭代函数
// 输入world.x/y/z，输出yaw/pitch/flight_time
std::optional<Trajectory> solveBallistics(
  const BallisticConfig & config, double x_m, double y_m, double z_m,
  double horizontal_distance_scale = 1.0);

}  // aim_core
