#pragma once

#include <optional>

namespace aim_core
{

struct BallisticConfig
{
  double bullet_speed_mps{11.6};
  double drag_coefficient{0.29};
  double gravity_mps2{9.794};
  double air_density_kgpm3{1.169};
  double bullet_mass_kg{0.041};
  double bullet_radius_m{0.02125};
  double muzzle_offset_m{0.28};
  int iteration_count{10};
};

struct Trajectory
{
  double yaw_rad{0.0};
  double pitch_rad{0.0};
  double flight_time_sec{0.0};
};

// 迁移旧 NormalAim 的阻力弹道迭代。x/y/z 为目标在云台世界坐标系的位置。
// horizontal_distance_scale 只作用于俯仰距离，用于兼容旧大装甲板的 1/1.2 处理。
std::optional<Trajectory> solveBallistics(
  const BallisticConfig & config, double x_m, double y_m, double z_m,
  double horizontal_distance_scale = 1.0);

}  // aim_core
