#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "aim_core/angle_utils.hpp"
#include "aim_core/ballistics.hpp"

namespace aim_auto {

// 对应TargetState
struct AutoTarget {
  std::uint8_t id{0U};
  bool tracking{false};
  bool converged{false};
  Eigen::Vector3d center_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_mps{Eigen::Vector3d::Zero()};
  double yaw_rad{0.0};
  double angular_velocity_radps{0.0};
  double radius_m{0.0};
  double radius_offset_m{0.0};
  double height_offset_m{0.0};
};

// 参数
struct AutoAimConfig {
  aim_core::BallisticConfig ballistics;
  aim_core::AngleSmootherConfig smoother;
  double system_response_time_sec{0.05};
  double high_spin_threshold_radps{2.0};
  double coming_angle_rad{1.0471975512};
  double leaving_angle_rad{0.6981317008};
  double shoot_yaw_tolerance_rad{0.0349065850};
  double shoot_pitch_tolerance_rad{0.0349065850};
  int fire_confirm_frames{4};
  int fast_fire_confirm_frames{2};
  double high_acceleration_threshold_mps2{0.5};
  double stable_acceleration_threshold_mps2{0.05};
  int acceleration_stable_frames{5};
};

// 单块装甲板在本次控制预计命中时刻的预测结果。
struct AutoAimArmor {
  int index{-1};
  Eigen::Vector3d position_m{Eigen::Vector3d::Zero()};
  double inward_yaw_rad{0.0};
  double phase_error_rad{0.0};
};

// 输出结果，会作为 ControlCommand、AutoAimDebug 与控制可视化的输入。
struct AutoAimResult {
  bool valid{false};
  bool target_locked{false};
  std::uint8_t target_id{0U};
  int armor_index{-1};
  Eigen::Vector3d aim_point_m{Eigen::Vector3d::Zero()};
  std::array<AutoAimArmor, 4> predicted_armors{};
  double raw_yaw_rad{0.0};
  double raw_pitch_rad{0.0};
  double command_yaw_rad{0.0};
  double command_pitch_rad{0.0};
  double flight_time_sec{0.0};
  double aim_time_sec{0.0};
  double state_age_sec{0.0};
  double phase_error_rad{0.0};
  double yaw_error_rad{0.0};
  double pitch_error_rad{0.0};
  double planar_acceleration_mps2{0.0};
  bool target_converged{false};
  bool phase_ready{false};
  bool gimbal_ready{false};
  bool acceleration_safe{true};
  bool shoot_ready{false};
};

class AutoAimer {
public:
  explicit AutoAimer(AutoAimConfig config);

  // 主程序入口 aim:
  // 1. chooseTarget()：保持已锁定的有效目标；失效后选择最近的有效目标
  // 2. 计算 state_age_sec = Tcontrol - Tstate，初始化 flight_time_sec = 0
  // 3. 迭代最多 10 次:
  //    3.1 predictTarget()：从 Tstate 临时预测到
  //        Tcontrol + system_response_time_sec + 当前 flight_time_sec
  //    3.2 chooseArmor()：调用 buildArmors() 重建四块板，再按普通/高速转动策略选择本次瞄准板
  //    3.3 aim_core::solveBallistics()：解算弹道
  //    3.4 新旧飞行时间未收敛时，使用新的飞行时间继续预测；
  //        收敛后得到该次控制的 Taim 和 aim_point
  // 4. updateAcceleration() / updateAccelerationSafety()：
  //    根据相邻预测状态速度估算平面加速度，并更新高加速度禁火状态
  // 5. smoother_.filter()：对原始弹道 yaw、pitch 做连续化和平滑，得到控制命令角
  // 6. 计算 phase_ready、gimbal_ready，并结合加速度保护更新连续满足帧数
  // 7. 将所有数据打包为 AutoAimResult
  std::optional<AutoAimResult>
  aim(const std::vector<AutoTarget> &targets, double state_stamp_sec,
      double control_stamp_sec, double gimbal_yaw_rad, double gimbal_pitch_rad);
  void reset(); // reset

private:
  [[nodiscard]] static double normalizeRadians(double angle_rad);
  [[nodiscard]] static AutoTarget predictTarget(const AutoTarget &target,
                                                double delay_sec);
  [[nodiscard]] static std::array<AutoAimArmor, 4>
  buildArmors(const AutoTarget &target);
  [[nodiscard]] std::optional<AutoAimArmor>
  chooseArmor(const AutoTarget &target) const;
  [[nodiscard]] const AutoTarget *
  chooseTarget(const std::vector<AutoTarget> &targets);
  double updateAcceleration(const AutoTarget &target, double state_stamp_sec);
  bool updateAccelerationSafety(double acceleration_mps2);

  AutoAimConfig config_;
  aim_core::AngleSmoother smoother_;
  std::optional<std::uint8_t> locked_target_id_;
  int locked_armor_index_{-1};
  std::optional<Eigen::Vector2d> previous_velocity_mps_;
  double previous_velocity_stamp_sec_{0.0};
  bool high_acceleration_mode_{false};
  int acceleration_stable_count_{0};
  int fire_ready_count_{0};
};

} // namespace aim_auto
