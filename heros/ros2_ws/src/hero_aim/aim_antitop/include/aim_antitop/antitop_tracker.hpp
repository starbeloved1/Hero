#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Dense>

namespace aim_antitop
{

// 来自 armor_solver 的前哨装甲板空间观测
struct AntitopObservation
{
  std::uint8_t id{0U};
  Eigen::Vector3d position_m{Eigen::Vector3d::Zero()};
  double image_center_x_px{0.0};
};

// 参数
struct AntitopTrackerConfig
{
  std::uint8_t outpost_armor_id{7U};
  std::size_t center_window_size{300U};
  std::size_t minimum_center_samples{10U};
  std::size_t calibration_start_center_samples{40U};
  std::size_t calibration_min_samples{240U};
  std::size_t calibration_max_samples{360U};
  std::size_t minimum_layer_samples{12U};
  double z_histogram_bin_size_m{0.01};
  double z_calibration_match_threshold_m{0.05};
  double z_layer_gap_min_m{0.08};
  double z_layer_gap_max_m{0.12};
  double max_pixel_jump_px{20.0};
  std::size_t direction_window_size{10U};
  double direction_threshold_px{0.0002};
};

// 输出给antitop_aimer与debug的状态快照
struct AntitopTrackerState
{
  AntitopObservation tracked_armor;
  Eigen::Vector3d rotation_center_m{Eigen::Vector3d::Zero()};
  std::array<double, 3> z_layers_m{};
  bool center_valid{false};
  bool calibrated{false};
  std::size_t center_sample_count{0U};
  std::size_t calibration_sample_count{0U};
  std::int8_t rotation_direction{-1};
};

// 反前哨的第一阶段：连续选择前哨板、拟合 XY 旋转中心、采样三层高度。
class AntitopTracker
{
public:
  explicit AntitopTracker(AntitopTrackerConfig config);

  // 反前哨第一阶段主入口 update：
  // 1. selectObservation()：仅保留前哨编号，优先选择与上一帧 last_observation_ 连续的板
  // 2. updateDirection()：利用同一块板的像素中心 x 位移，在稳定窗口后确定顺/逆时针方向
  // 3. updateRotationCenter()：将该板的 world XY 放入 center_samples_，滑动均值得到旋转中心
  // 4. updateCalibration()：中心样本达到门槛后收集 Z；样本充足时调用 tryBuildZLayers()
  //    从直方图中寻找满足层间距约束的低、中、高三层 Z
  // 5. 更新 last_observation_，调用 makeState() 输出本帧的中心、标定状态与三层高度快照
  // 若本帧没有有效前哨板，则返回空
  std::optional<AntitopTrackerState> update(const std::vector<AntitopObservation> & observations);
  void reset();

private:
  [[nodiscard]] static bool finiteObservation(const AntitopObservation & observation);
  [[nodiscard]] std::optional<AntitopObservation> selectObservation(
    const std::vector<AntitopObservation> & observations) const;
  void updateRotationCenter(const AntitopObservation & observation);
  void updateDirection(const AntitopObservation & observation);
  void updateCalibration(const AntitopObservation & observation);
  [[nodiscard]] bool tryBuildZLayers();
  [[nodiscard]] AntitopTrackerState makeState(const AntitopObservation & observation) const;

  AntitopTrackerConfig config_;
  std::optional<AntitopObservation> last_observation_;
  std::vector<Eigen::Vector2d> center_samples_;
  std::vector<double> calibration_z_samples_;
  std::vector<double> image_x_differences_;
  Eigen::Vector3d rotation_center_m_{Eigen::Vector3d::Zero()};
  std::array<double, 3> z_layers_m_{};
  bool calibrated_{false};
  std::int8_t rotation_direction_{-1};
};

}  // aim_antitop
