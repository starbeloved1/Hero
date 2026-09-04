#pragma once

namespace aim_core
{

// 将角度规范到 [-pi, pi)。
double normalizeRadians(double angle_rad);

// 返回与 reference_rad 最连续的 equivalent_rad 的等价角。
double unwrapNear(double angle_rad, double reference_rad);

struct AimAngles
{
  double yaw_rad{0.0};
  double pitch_rad{0.0};
};

struct AngleSmootherConfig
{
  double yaw_previous_weight{0.3};
  double pitch_previous_weight{0.7};
  double yaw_jump_threshold_rad{0.05235987755982989};
  double pitch_jump_threshold_rad{0.017453292519943295};
};

// 迁移旧 BasicAimer 的角度连续化与小跳变低通滤波；输入输出均为弧度。
class AngleSmoother
{
public:
  explicit AngleSmoother(AngleSmootherConfig config = {});

  AimAngles filter(double yaw_rad, double pitch_rad);
  void reset();

private:
  AngleSmootherConfig config_;
  bool has_previous_{false};
  AimAngles previous_;
};

}  // aim_core
