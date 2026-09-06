#include "aim_antitop/antitop_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

namespace aim_antitop
{

namespace
{

std::array<double, 3> emptyZLayers()
{
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  return {nan, nan, nan};
}

double squaredDistance(const Eigen::Vector3d & first, const Eigen::Vector3d & second)
{
  return (first - second).squaredNorm();
}

}  // namespace

AntitopTracker::AntitopTracker(AntitopTrackerConfig config)
: config_(config), z_layers_m_(emptyZLayers())
{
  if (
    config_.outpost_armor_id == 0U || config_.center_window_size == 0U ||
    config_.minimum_center_samples == 0U ||
    config_.minimum_center_samples > config_.center_window_size ||
    config_.calibration_start_center_samples < config_.minimum_center_samples ||
    config_.calibration_min_samples == 0U ||
    config_.calibration_max_samples < config_.calibration_min_samples ||
    config_.minimum_layer_samples == 0U || !std::isfinite(config_.z_histogram_bin_size_m) ||
    config_.z_histogram_bin_size_m <= 0.0 ||
    !std::isfinite(config_.z_calibration_match_threshold_m) ||
    config_.z_calibration_match_threshold_m <= 0.0 ||
    !std::isfinite(config_.z_layer_gap_min_m) || !std::isfinite(config_.z_layer_gap_max_m) ||
    config_.z_layer_gap_min_m <= 0.0 || config_.z_layer_gap_max_m < config_.z_layer_gap_min_m ||
    !std::isfinite(config_.max_pixel_jump_px) || config_.max_pixel_jump_px <= 0.0 ||
    config_.direction_window_size == 0U || !std::isfinite(config_.direction_threshold_px) ||
    config_.direction_threshold_px < 0.0)
  {
    throw std::invalid_argument("反前哨跟踪参数无效");
  }
}

std::optional<AntitopTrackerState> AntitopTracker::update(
  const std::vector<AntitopObservation> & observations)
{
  const auto selected = selectObservation(observations);
  if (!selected.has_value()) {
    return std::nullopt;
  }
  if (
    last_observation_.has_value() &&
    std::abs(selected->image_center_x_px - last_observation_->image_center_x_px) >=
    config_.max_pixel_jump_px)
  {
    // 保持旧 AntiTop 的处理：二维跳变帧不参与中心、方向和高度更新
    image_x_differences_.clear();
    last_observation_ = *selected;
    return std::nullopt;
  }
  updateDirection(*selected);
  updateRotationCenter(*selected);
  updateCalibration(*selected);
  last_observation_ = *selected;
  return makeState(*selected);
}

void AntitopTracker::reset()
{
  last_observation_.reset();
  center_samples_.clear();
  calibration_z_samples_.clear();
  image_x_differences_.clear();
  rotation_center_m_.setZero();
  z_layers_m_ = emptyZLayers();
  calibrated_ = false;
  rotation_direction_ = -1;
}

bool AntitopTracker::finiteObservation(const AntitopObservation & observation)
{
  return observation.id > 0U && observation.position_m.allFinite() &&
         std::isfinite(observation.image_center_x_px);
}

std::optional<AntitopObservation> AntitopTracker::selectObservation(
  const std::vector<AntitopObservation> & observations) const
{
  std::vector<const AntitopObservation *> candidates;
  for (const auto & observation : observations) {
    if (observation.id == config_.outpost_armor_id && finiteObservation(observation)) {
      candidates.push_back(&observation);
    }
  }
  if (candidates.empty()) {
    return std::nullopt;
  }
  if (!last_observation_.has_value()) {
    return *candidates.front();
  }
  const auto closest = std::min_element(
    candidates.begin(), candidates.end(), [this](const auto * first, const auto * second) {
      return squaredDistance(first->position_m, last_observation_->position_m) <
             squaredDistance(second->position_m, last_observation_->position_m);
    });
  return **closest;
}

void AntitopTracker::updateRotationCenter(const AntitopObservation & observation)
{
  center_samples_.emplace_back(observation.position_m.x(), observation.position_m.y());
  if (center_samples_.size() > config_.center_window_size) {
    center_samples_.erase(center_samples_.begin());
  }
  if (center_samples_.size() < config_.minimum_center_samples) {
    return;
  }
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  for (const auto & sample : center_samples_) {
    center += sample;
  }
  center /= static_cast<double>(center_samples_.size());
  rotation_center_m_.x() = center.x();
  rotation_center_m_.y() = center.y();
}

void AntitopTracker::updateDirection(const AntitopObservation & observation)
{
  if (rotation_direction_ != -1 || !last_observation_.has_value()) {
    return;
  }
  const double image_x_difference =
    observation.image_center_x_px - last_observation_->image_center_x_px;
  double dynamic_threshold = 0.0;
  if (!image_x_differences_.empty()) {
    double sum = 0.0;
    for (const double difference : image_x_differences_) {
      sum += std::abs(difference);
    }
    dynamic_threshold = 10.0 * sum / static_cast<double>(image_x_differences_.size());
  }
  if (image_x_differences_.empty() || std::abs(image_x_difference) <= dynamic_threshold) {
    image_x_differences_.push_back(image_x_difference);
    if (image_x_differences_.size() > config_.direction_window_size + 1U) {
      image_x_differences_.erase(image_x_differences_.begin());
    }
  } else {
    image_x_differences_.clear();
  }
  if (image_x_differences_.size() != config_.direction_window_size) {
    return;
  }
  double average = 0.0;
  for (const double difference : image_x_differences_) {
    average += difference;
  }
  average /= static_cast<double>(image_x_differences_.size());
  if (average < -config_.direction_threshold_px) {
    rotation_direction_ = 1;
  } else if (average > config_.direction_threshold_px) {
    rotation_direction_ = 0;
  } else {
    image_x_differences_.clear();
  }
}

void AntitopTracker::updateCalibration(const AntitopObservation & observation)
{
  if (calibrated_ || center_samples_.size() < config_.calibration_start_center_samples) {
    return;
  }
  calibration_z_samples_.push_back(observation.position_m.z());
  if (calibration_z_samples_.size() > config_.calibration_max_samples) {
    calibration_z_samples_.erase(calibration_z_samples_.begin());
  }
  if (calibration_z_samples_.size() >= config_.calibration_min_samples) {
    calibrated_ = tryBuildZLayers();
  }
}

bool AntitopTracker::tryBuildZLayers()
{
  struct Peak
  {
    int bin{0};
    std::size_t count{0U};
  };
  std::map<int, std::size_t> histogram;
  for (const double z : calibration_z_samples_) {
    if (std::isfinite(z)) {
      ++histogram[static_cast<int>(std::floor(z / config_.z_histogram_bin_size_m))];
    }
  }
  std::vector<Peak> peaks;
  peaks.reserve(histogram.size());
  for (const auto & [bin, count] : histogram) {
    peaks.push_back(Peak{bin, count});
  }
  std::sort(peaks.begin(), peaks.end(), [](const Peak & first, const Peak & second) {
    return first.count != second.count ? first.count > second.count : first.bin < second.bin;
  });
  if (peaks.size() > 40U) {
    peaks.resize(40U);
  }

  std::array<double, 3> selected = emptyZLayers();
  std::size_t best_score = 0U;
  for (std::size_t first = 0U; first < peaks.size(); ++first) {
    for (std::size_t second = first + 1U; second < peaks.size(); ++second) {
      for (std::size_t third = second + 1U; third < peaks.size(); ++third) {
        std::array<double, 3> candidate{
          (static_cast<double>(peaks[first].bin) + 0.5) * config_.z_histogram_bin_size_m,
          (static_cast<double>(peaks[second].bin) + 0.5) * config_.z_histogram_bin_size_m,
          (static_cast<double>(peaks[third].bin) + 0.5) * config_.z_histogram_bin_size_m};
        std::sort(candidate.begin(), candidate.end());
        const double first_gap = candidate[1] - candidate[0];
        const double second_gap = candidate[2] - candidate[1];
        if (
          first_gap < config_.z_layer_gap_min_m || first_gap > config_.z_layer_gap_max_m ||
          second_gap < config_.z_layer_gap_min_m || second_gap > config_.z_layer_gap_max_m)
        {
          continue;
        }
        const auto score = peaks[first].count + peaks[second].count + peaks[third].count;
        if (score > best_score) {
          selected = candidate;
          best_score = score;
        }
      }
    }
  }
  if (best_score == 0U) {
    return false;
  }

  std::array<std::vector<double>, 3> layers;
  for (const double z : calibration_z_samples_) {
    std::size_t best_index = 0U;
    double best_difference = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index < selected.size(); ++index) {
      const double difference = std::abs(z - selected[index]);
      if (difference < best_difference) {
        best_difference = difference;
        best_index = index;
      }
    }
    if (best_difference <= config_.z_calibration_match_threshold_m) {
      layers[best_index].push_back(z);
    }
  }
  for (std::size_t index = 0U; index < layers.size(); ++index) {
    if (layers[index].size() < config_.minimum_layer_samples) {
      return false;
    }
    std::sort(layers[index].begin(), layers[index].end());
    selected[index] = layers[index][layers[index].size() / 2U];
  }
  std::sort(selected.begin(), selected.end());
  z_layers_m_ = selected;
  return true;
}

AntitopTrackerState AntitopTracker::makeState(const AntitopObservation & observation) const
{
  AntitopTrackerState state;
  state.tracked_armor = observation;
  state.rotation_center_m = rotation_center_m_;
  state.z_layers_m = z_layers_m_;
  state.center_valid = center_samples_.size() >= config_.minimum_center_samples;
  state.calibrated = calibrated_;
  state.center_sample_count = center_samples_.size();
  state.calibration_sample_count = calibration_z_samples_.size();
  state.rotation_direction = rotation_direction_;
  return state;
}

}  // aim_antitop
