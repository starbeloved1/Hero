#include "aim_predictor/target_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace aim_predictor
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinDtSec = 0.001;
constexpr double kDefaultDtSec = 0.015;
constexpr double kMaxUpdateDtSec = 0.08;
constexpr double kMaxPredictionDtSec = 0.35;
constexpr double kMinDistance = 1e-6;

double square(double value)
{
  return value * value;
}

}  // namespace

TargetTracker::TargetTracker(TrackerConfig config)
: config_(std::move(config))
{
  if (
    config_.init_radius_m <= 0.0 || config_.min_valid_radius_m <= 0.0 ||
    config_.max_valid_radius_m <= config_.min_valid_radius_m ||
    config_.min_consecutive_detections <= 0 || config_.max_lost_frames < 0)
  {
    throw std::invalid_argument("预测器跟踪参数无效");
  }
}

void TargetTracker::initialize(
  std::uint8_t target_id, const ArmorMeasurement & measurement, double stamp_sec)
{
  if (!finiteMeasurement(measurement) || !std::isfinite(stamp_sec)) {
    return;
  }

  const double radius = std::max(config_.init_radius_m, config_.min_valid_radius_m);
  state_.setZero();
  state_[0] = measurement.position_m.x() + radius * std::cos(measurement.yaw_rad);
  state_[2] = measurement.position_m.y() + radius * std::sin(measurement.yaw_rad);
  state_[4] = measurement.position_m.z();
  state_[6] = limitRadians(measurement.yaw_rad);
  state_[8] = radius;

  StateVector initial_variance;
  initial_variance << 1.0, 64.0, 1.0, 64.0, 1.0, 64.0, 0.4, 100.0, 1.0, 1.0, 1.0;
  covariance_ = initial_variance.asDiagonal();
  target_id_ = target_id;
  last_stamp_sec_ = stamp_sec;
  initialized_ = true;
  jumped_ = false;
  lost_frames_ = 0;
  update_count_ = 1;
  last_armor_index_ = 0;
}

void TargetTracker::predict(double stamp_sec)
{
  if (!initialized_ || !std::isfinite(stamp_sec)) {
    return;
  }
  const double dt_sec = sanitizeDt(stamp_sec - last_stamp_sec_, kMaxUpdateDtSec);
  const StateMatrix transition_matrix = transitionMatrix(dt_sec);
  covariance_ = transition_matrix * covariance_ * transition_matrix.transpose() + processNoise(dt_sec, state_);
  state_ = transition(state_, dt_sec);
  last_stamp_sec_ = stamp_sec;
}

void TargetTracker::update(const ArmorMeasurement & measurement)
{
  if (!initialized_ || !finiteMeasurement(measurement)) {
    return;
  }
  const int armor_index = matchArmorIndex(measurement);
  jumped_ = update_count_ > 0 && armor_index != last_armor_index_;
  ekfUpdate(measurement, armor_index);
  last_armor_index_ = armor_index;
  lost_frames_ = 0;
  ++update_count_;
}

void TargetTracker::markLost()
{
  if (initialized_) {
    ++lost_frames_;
  }
}

void TargetTracker::reset()
{
  initialized_ = false;
  jumped_ = false;
  lost_frames_ = 0;
  update_count_ = 0;
  last_armor_index_ = 0;
  state_.setZero();
  covariance_.setIdentity();
}

bool TargetTracker::active() const
{
  return initialized_;
}

bool TargetTracker::diverged() const
{
  if (!initialized_) {
    return false;
  }
  const double first_radius = state_[8];
  const double second_radius = state_[8] + state_[9];
  return !(
    first_radius > config_.min_valid_radius_m && first_radius < config_.max_valid_radius_m &&
    second_radius > config_.min_valid_radius_m && second_radius < config_.max_valid_radius_m);
}

int TargetTracker::lostFrames() const
{
  return lost_frames_;
}

TargetEstimate TargetTracker::estimate(double stamp_sec) const
{
  TargetEstimate output;
  output.id = target_id_;
  output.tracking = initialized_ && !diverged() && lost_frames_ <= config_.max_lost_frames;
  output.converged = output.tracking && update_count_ >= config_.min_consecutive_detections;
  output.jumped = jumped_;
  if (!output.tracking) {
    return output;
  }

  const double dt_sec = sanitizeDt(stamp_sec - last_stamp_sec_, kMaxPredictionDtSec);
  const StateVector predicted_state = transition(state_, dt_sec);
  output.center_m = Eigen::Vector3d(predicted_state[0], predicted_state[2], predicted_state[4]);
  output.velocity_mps = Eigen::Vector3d(predicted_state[1], predicted_state[3], predicted_state[5]);
  output.yaw_rad = limitRadians(predicted_state[6]);
  output.angular_velocity_radps = predicted_state[7];
  output.radius_m = predicted_state[8];
  output.radius_offset_m = predicted_state[9];
  output.height_offset_m = predicted_state[10];
  output.armors = predictedArmors(predicted_state);
  return output;
}

double TargetTracker::limitRadians(double angle_rad)
{
  return std::remainder(angle_rad, 2.0 * kPi);
}

Eigen::Vector3d TargetTracker::xyz2ypd(const Eigen::Vector3d & position_m)
{
  const double horizontal_distance = std::hypot(position_m.x(), position_m.y());
  return Eigen::Vector3d(
    std::atan2(position_m.y(), position_m.x()),
    std::atan2(position_m.z(), horizontal_distance),
    std::max(position_m.norm(), kMinDistance));
}

Eigen::Matrix3d TargetTracker::xyz2ypdJacobian(const Eigen::Vector3d & position_m)
{
  const double x = position_m.x();
  const double y = position_m.y();
  const double z = position_m.z();
  const double horizontal_squared = std::max(x * x + y * y, kMinDistance);
  const double horizontal_distance = std::sqrt(horizontal_squared);
  const double distance_squared = std::max(horizontal_squared + z * z, kMinDistance);
  const double distance = std::sqrt(distance_squared);

  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
  jacobian(0, 0) = -y / horizontal_squared;
  jacobian(0, 1) = x / horizontal_squared;
  jacobian(1, 0) = -x * z / (horizontal_distance * distance_squared);
  jacobian(1, 1) = -y * z / (horizontal_distance * distance_squared);
  jacobian(1, 2) = horizontal_distance / distance_squared;
  jacobian(2, 0) = x / distance;
  jacobian(2, 1) = y / distance;
  jacobian(2, 2) = z / distance;
  return jacobian;
}

bool TargetTracker::finiteMeasurement(const ArmorMeasurement & measurement)
{
  return measurement.position_m.allFinite() && std::isfinite(measurement.yaw_rad) &&
    measurement.position_m.norm() > 0.1;
}

double TargetTracker::sanitizeDt(double dt_sec, double max_dt_sec) const
{
  if (!std::isfinite(dt_sec)) {
    return kDefaultDtSec;
  }
  return std::clamp(dt_sec, kMinDtSec, max_dt_sec);
}

TargetTracker::StateVector TargetTracker::transition(const StateVector & state, double dt_sec) const
{
  StateVector next = state;
  next[0] += state[1] * dt_sec;
  next[2] += state[3] * dt_sec;
  next[4] += state[5] * dt_sec;
  next[6] = limitRadians(state[6] + state[7] * dt_sec);
  return next;
}

TargetTracker::StateMatrix TargetTracker::transitionMatrix(double dt_sec) const
{
  StateMatrix matrix = StateMatrix::Identity();
  matrix(0, 1) = dt_sec;
  matrix(2, 3) = dt_sec;
  matrix(4, 5) = dt_sec;
  matrix(6, 7) = dt_sec;
  return matrix;
}

TargetTracker::StateMatrix TargetTracker::processNoise(double dt_sec, const StateVector & state) const
{
  const double middle_threshold = std::min(
    std::abs(config_.middle_speed_angular_velocity_threshold),
    std::abs(config_.high_speed_angular_velocity_threshold));
  const double high_threshold = std::max(
    std::abs(config_.middle_speed_angular_velocity_threshold),
    std::abs(config_.high_speed_angular_velocity_threshold));
  const double angular_velocity = std::abs(state[7]);

  double noise_xy = config_.high_speed_process_noise_xy;
  double noise_z = config_.high_speed_process_noise_z;
  double noise_yaw = config_.high_speed_process_noise_yaw;
  if (angular_velocity < middle_threshold) {
    noise_xy = config_.low_speed_process_noise_xy;
    noise_z = config_.low_speed_process_noise_z;
    noise_yaw = config_.low_speed_process_noise_yaw;
  } else if (angular_velocity < high_threshold) {
    noise_xy = config_.middle_speed_process_noise_xy;
    noise_z = config_.middle_speed_process_noise_z;
    noise_yaw = config_.middle_speed_process_noise_yaw;
  }

  const double position_noise = square(square(dt_sec)) / 4.0;
  const double cross_noise = dt_sec * dt_sec * dt_sec / 2.0;
  const double velocity_noise = square(dt_sec);
  StateMatrix output = StateMatrix::Zero();
  const auto addConstantVelocityNoise = [&output, position_noise, cross_noise, velocity_noise](
      int position_index, int velocity_index, double noise) {
      output(position_index, position_index) = position_noise * noise;
      output(position_index, velocity_index) = cross_noise * noise;
      output(velocity_index, position_index) = cross_noise * noise;
      output(velocity_index, velocity_index) = velocity_noise * noise;
    };
  addConstantVelocityNoise(0, 1, noise_xy);
  addConstantVelocityNoise(2, 3, noise_xy);
  addConstantVelocityNoise(4, 5, noise_z);
  addConstantVelocityNoise(6, 7, noise_yaw);
  output(8, 8) = 1e-4 * dt_sec;
  output(9, 9) = 1e-4 * dt_sec;
  output(10, 10) = 1e-4 * dt_sec;
  return output;
}

Eigen::Vector3d TargetTracker::armorPosition(const StateVector & state, int armor_index) const
{
  const double angle = limitRadians(state[6] + static_cast<double>(armor_index) * kPi / 2.0);
  const bool use_offset = armor_index == 1 || armor_index == 3;
  const double radius = use_offset ? state[8] + state[9] : state[8];
  return Eigen::Vector3d(
    state[0] - radius * std::cos(angle), state[2] - radius * std::sin(angle),
    use_offset ? state[4] + state[10] : state[4]);
}

Eigen::Vector4d TargetTracker::observation(const StateVector & state, int armor_index) const
{
  const Eigen::Vector3d position = armorPosition(state, armor_index);
  const Eigen::Vector3d ypd = xyz2ypd(position);
  return Eigen::Vector4d(
    ypd.x(), ypd.y(), ypd.z(),
    limitRadians(state[6] + static_cast<double>(armor_index) * kPi / 2.0));
}

Eigen::Matrix<double, 4, TargetTracker::kStateDimension> TargetTracker::observationJacobian(
  const StateVector & state, int armor_index) const
{
  const double angle = limitRadians(state[6] + static_cast<double>(armor_index) * kPi / 2.0);
  const bool use_offset = armor_index == 1 || armor_index == 3;
  const double radius = use_offset ? state[8] + state[9] : state[8];
  Eigen::Matrix<double, 4, kStateDimension> position_jacobian =
    Eigen::Matrix<double, 4, kStateDimension>::Zero();
  position_jacobian(0, 0) = 1.0;
  position_jacobian(0, 6) = radius * std::sin(angle);
  position_jacobian(0, 8) = -std::cos(angle);
  position_jacobian(0, 9) = use_offset ? -std::cos(angle) : 0.0;
  position_jacobian(1, 2) = 1.0;
  position_jacobian(1, 6) = -radius * std::cos(angle);
  position_jacobian(1, 8) = -std::sin(angle);
  position_jacobian(1, 9) = use_offset ? -std::sin(angle) : 0.0;
  position_jacobian(2, 4) = 1.0;
  position_jacobian(2, 10) = use_offset ? 1.0 : 0.0;
  position_jacobian(3, 6) = 1.0;

  Eigen::Matrix4d ypd_jacobian = Eigen::Matrix4d::Zero();
  ypd_jacobian.block<3, 3>(0, 0) = xyz2ypdJacobian(armorPosition(state, armor_index));
  ypd_jacobian(3, 3) = 1.0;
  return ypd_jacobian * position_jacobian;
}

int TargetTracker::matchArmorIndex(const ArmorMeasurement & measurement) const
{
  int best_index = 0;
  double best_error = std::numeric_limits<double>::infinity();
  const Eigen::Vector3d ypd = xyz2ypd(measurement.position_m);
  for (int index = 0; index < 4; ++index) {
    const Eigen::Vector4d predicted = observation(state_, index);
    const double error =
      std::abs(limitRadians(measurement.yaw_rad - predicted[3])) +
      std::abs(limitRadians(ypd.x() - predicted[0])) +
      0.25 * std::abs(ypd.z() - predicted[2]);
    if (error < best_error) {
      best_error = error;
      best_index = index;
    }
  }
  return best_index;
}

std::array<PredictedArmor, 4> TargetTracker::predictedArmors(const StateVector & state) const
{
  std::array<PredictedArmor, 4> output{};
  for (int index = 0; index < 4; ++index) {
    output[static_cast<std::size_t>(index)].position_m = armorPosition(state, index);
    output[static_cast<std::size_t>(index)].yaw_rad =
      limitRadians(state[6] + static_cast<double>(index) * kPi / 2.0);
  }
  return output;
}

void TargetTracker::ekfUpdate(const ArmorMeasurement & measurement, int armor_index)
{
  const Eigen::Vector3d ypd = xyz2ypd(measurement.position_m);
  const Eigen::Vector4d measured(ypd.x(), ypd.y(), ypd.z(), limitRadians(measurement.yaw_rad));
  const auto jacobian = observationJacobian(state_, armor_index);
  const double center_yaw = std::atan2(measurement.position_m.y(), measurement.position_m.x());
  const double delta_angle = std::abs(limitRadians(measurement.yaw_rad - center_yaw));
  Eigen::Vector4d noise;
  noise << config_.measurement_noise_yaw, config_.measurement_noise_pitch,
    std::log(std::abs(ypd.z()) + 1.0) / 200.0 + config_.measurement_noise_distance_base,
    std::log(delta_angle + 1.0) + config_.measurement_noise_armor_yaw_base;
  const Eigen::Matrix4d observation_noise = noise.asDiagonal();
  const Eigen::Matrix4d innovation_covariance = jacobian * covariance_ * jacobian.transpose() + observation_noise;
  const Eigen::Matrix<double, kStateDimension, 4> gain =
    covariance_ * jacobian.transpose() * innovation_covariance.inverse();
  Eigen::Vector4d innovation = measured - observation(state_, armor_index);
  innovation[0] = limitRadians(innovation[0]);
  innovation[1] = limitRadians(innovation[1]);
  innovation[3] = limitRadians(innovation[3]);
  state_ += gain * innovation;
  state_[6] = limitRadians(state_[6]);
  const StateMatrix identity = StateMatrix::Identity();
  covariance_ = (identity - gain * jacobian) * covariance_ * (identity - gain * jacobian).transpose() +
    gain * observation_noise * gain.transpose();
}

}  // aim_predictor
