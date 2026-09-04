#pragma once

namespace aim_core
{

//归一化角度
double normalizeRadians(double angle_rad);

//改写目标yaw为最接近当前yaw等价角度
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

class AngleSmoother
{
public:
  explicit AngleSmoother(AngleSmootherConfig config = {});

  //保存上一次控制角度，做平滑或者跟随策略
  AimAngles filter(double yaw_rad, double pitch_rad);
  void reset(); //右键上升沿或者离开mode时调用

private:
  AngleSmootherConfig config_;
  bool has_previous_{false};
  AimAngles previous_;
};

}  // aim_core
