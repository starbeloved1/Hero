#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Dense>

namespace aim_predictor
{

struct TrackerConfig
{
  double init_radius_m{0.2};
  int min_consecutive_detections{3};
  int max_lost_frames{10};
  double low_speed_process_noise_xy{100.0};
  double low_speed_process_noise_z{100.0};
  double low_speed_process_noise_yaw{400.0};
  double middle_speed_process_noise_xy{100.0};
  double middle_speed_process_noise_z{100.0};
  double middle_speed_process_noise_yaw{400.0};
  double high_speed_process_noise_xy{100.0};
  double high_speed_process_noise_z{100.0};
  double high_speed_process_noise_yaw{400.0};
  double middle_speed_angular_velocity_threshold{2.0};
  double high_speed_angular_velocity_threshold{4.0};
  double measurement_noise_yaw{0.004};
  double measurement_noise_pitch{0.004};
  double measurement_noise_distance_base{0.1};
  double measurement_noise_armor_yaw_base{0.09};
  double min_valid_radius_m{0.05};
  double max_valid_radius_m{0.5};
};

// armor在world坐标系下的位置与朝向
struct ArmorMeasurement
{
  Eigen::Vector3d position_m{Eigen::Vector3d::Zero()};
  double yaw_rad{0.0};
};

struct PredictedArmor
{
  Eigen::Vector3d position_m{Eigen::Vector3d::Zero()};
  double yaw_rad{0.0};
};

// 整车完整预测状态
struct TargetEstimate
{
  std::uint8_t id{0U};
  bool tracking{false};
  bool converged{false};
  bool jumped{false};
  Eigen::Vector3d center_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_mps{Eigen::Vector3d::Zero()};
  double yaw_rad{0.0};
  double angular_velocity_radps{0.0};
  double radius_m{0.0};
  double radius_offset_m{0.0};
  double height_offset_m{0.0};
  std::array<PredictedArmor, 4> armors{};
};

// 迁移旧 MotionModel：每个 Hero 编号对应一个四装甲板 EKF 状态
class TargetTracker
{
public:
  explicit TargetTracker(TrackerConfig config);

  // 初始化一台车的模型
  void initialize(std::uint8_t target_id, const ArmorMeasurement & measurement, double stamp_sec);
  // 输入时间戳，输出本时刻的预测状态，会修改tracker内部的时间戳，因此只用于update里面调用
  void predict(double stamp_sec);
  // 预测之后使用update输入实际观测修正预测
  void update(const ArmorMeasurement & measurement);
  void markLost();
  void reset();

  [[nodiscard]] bool active() const;
  [[nodiscard]] bool diverged() const;
  [[nodiscard]] int lostFrames() const;
  // 预测到Tstate时刻状态，不会修改tracker内部时间戳
  [[nodiscard]] TargetEstimate estimate(double stamp_sec) const;

private:
  static constexpr int kStateDimension = 11;
  using StateVector = Eigen::Matrix<double, kStateDimension, 1>;
  using StateMatrix = Eigen::Matrix<double, kStateDimension, kStateDimension>;

  [[nodiscard]] static double limitRadians(double angle_rad);
  [[nodiscard]] static Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & position_m);
  [[nodiscard]] static Eigen::Matrix3d xyz2ypdJacobian(const Eigen::Vector3d & position_m);
  [[nodiscard]] static bool finiteMeasurement(const ArmorMeasurement & measurement);
  [[nodiscard]] double sanitizeDt(double dt_sec, double max_dt_sec) const;
  [[nodiscard]] StateVector transition(const StateVector & state, double dt_sec) const;
  [[nodiscard]] StateMatrix transitionMatrix(double dt_sec) const;
  [[nodiscard]] StateMatrix processNoise(double dt_sec, const StateVector & state) const;
  [[nodiscard]] Eigen::Vector3d armorPosition(const StateVector & state, int armor_index) const;
  [[nodiscard]] Eigen::Vector4d observation(const StateVector & state, int armor_index) const;
  [[nodiscard]] Eigen::Matrix<double, 4, kStateDimension> observationJacobian(
    const StateVector & state, int armor_index) const;
  [[nodiscard]] int matchArmorIndex(const ArmorMeasurement & measurement) const;
  [[nodiscard]] std::array<PredictedArmor, 4> predictedArmors(const StateVector & state) const;
  void ekfUpdate(const ArmorMeasurement & measurement, int armor_index);

  TrackerConfig config_;
  bool initialized_{false};
  bool jumped_{false};
  int lost_frames_{0};
  int update_count_{0};
  int last_armor_index_{0};
  std::uint8_t target_id_{0U};
  double last_stamp_sec_{0.0};
  StateVector state_{StateVector::Zero()};
  StateMatrix covariance_{StateMatrix::Identity()};
};

}  // aim_predictor
