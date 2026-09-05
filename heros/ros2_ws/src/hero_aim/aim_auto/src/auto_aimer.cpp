#include "aim_auto/auto_aimer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aim_auto {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool finiteTarget(const AutoTarget &target) {
  return target.id > 0U && target.tracking && target.converged &&
         target.center_m.allFinite() && target.velocity_mps.allFinite() &&
         std::isfinite(target.yaw_rad) &&
         std::isfinite(target.angular_velocity_radps);
}

} // namespace

AutoAimer::AutoAimer(AutoAimConfig config)
    : config_(std::move(config)), smoother_(config_.smoother) {
  if (config_.system_response_time_sec < 0.0 ||
      config_.fire_confirm_frames <= 0 ||
      config_.fast_fire_confirm_frames <= 0 ||
      config_.high_acceleration_threshold_mps2 < 0.0 ||
      config_.stable_acceleration_threshold_mps2 < 0.0 ||
      config_.acceleration_stable_frames <= 0) {
    throw std::invalid_argument("自瞄控制参数无效");
  }
}

std::optional<AutoAimResult>
AutoAimer::aim(const std::vector<AutoTarget> &targets, double state_stamp_sec,
               double control_stamp_sec, double gimbal_yaw_rad,
               double gimbal_pitch_rad) {
  if (!std::isfinite(state_stamp_sec) || !std::isfinite(control_stamp_sec) ||
      !std::isfinite(gimbal_yaw_rad) || !std::isfinite(gimbal_pitch_rad)) {
    return std::nullopt;
  }
  const AutoTarget *target = chooseTarget(targets);
  if (target == nullptr) {
    fire_ready_count_ = 0;
    return std::nullopt;
  }

  const double state_age_sec =
      std::max(0.0, control_stamp_sec - state_stamp_sec);
  double flight_time_sec = 0.0;
  std::optional<AutoAimArmor> selected;
  AutoTarget predicted = *target;
  aim_core::Trajectory trajectory;
  for (int iteration = 0; iteration < 10; ++iteration) {
    predicted = predictTarget(*target, state_age_sec +
                                           config_.system_response_time_sec +
                                           flight_time_sec);
    selected = chooseArmor(predicted);
    if (!selected.has_value()) {
      return std::nullopt;
    }
    const auto next = aim_core::solveBallistics(
        config_.ballistics, selected->position_m.x(), selected->position_m.y(),
        selected->position_m.z());
    if (!next.has_value()) {
      return std::nullopt;
    }
    trajectory = *next;
    if (std::abs(trajectory.flight_time_sec - flight_time_sec) < 0.001) {
      break;
    }
    flight_time_sec = trajectory.flight_time_sec;
  }
  if (!selected.has_value()) {
    return std::nullopt;
  }

  const double acceleration = updateAcceleration(*target, state_stamp_sec);
  const bool acceleration_safe = updateAccelerationSafety(acceleration);
  const auto command =
      smoother_.filter(trajectory.yaw_rad, trajectory.pitch_rad);
  const double yaw_error = normalizeRadians(command.yaw_rad - gimbal_yaw_rad);
  const double pitch_error = command.pitch_rad - gimbal_pitch_rad;
  const bool phase_ready =
      std::abs(selected->phase_error_rad) <= config_.coming_angle_rad;
  const bool gimbal_ready =
      std::abs(yaw_error) <= config_.shoot_yaw_tolerance_rad &&
      std::abs(pitch_error) <= config_.shoot_pitch_tolerance_rad;
  const bool fast = std::abs(predicted.angular_velocity_radps) >
                    config_.high_spin_threshold_radps;
  const int required_frames =
      fast ? config_.fast_fire_confirm_frames : config_.fire_confirm_frames;
  if (phase_ready && gimbal_ready && acceleration_safe) {
    ++fire_ready_count_;
  } else {
    fire_ready_count_ = 0;
  }

  locked_armor_index_ = selected->index;
  AutoAimResult result;
  result.valid = true;
  result.target_locked = locked_target_id_.has_value();
  result.target_id = target->id;
  result.armor_index = selected->index;
  result.aim_point_m = selected->position_m;
  result.predicted_armors = buildArmors(predicted);
  result.raw_yaw_rad = trajectory.yaw_rad;
  result.raw_pitch_rad = trajectory.pitch_rad;
  result.command_yaw_rad = command.yaw_rad;
  result.command_pitch_rad = command.pitch_rad;
  result.flight_time_sec = trajectory.flight_time_sec;
  result.aim_time_sec = control_stamp_sec + config_.system_response_time_sec +
                        trajectory.flight_time_sec;
  result.state_age_sec = state_age_sec;
  result.phase_error_rad = selected->phase_error_rad;
  result.yaw_error_rad = yaw_error;
  result.pitch_error_rad = pitch_error;
  result.planar_acceleration_mps2 = acceleration;
  result.target_converged = target->converged;
  result.phase_ready = phase_ready;
  result.gimbal_ready = gimbal_ready;
  result.acceleration_safe = acceleration_safe;
  result.shoot_ready = phase_ready && gimbal_ready && acceleration_safe &&
                       fire_ready_count_ >= required_frames;
  return result;
}

void AutoAimer::reset() {
  smoother_.reset();
  locked_target_id_.reset();
  locked_armor_index_ = -1;
  previous_velocity_mps_.reset();
  previous_velocity_stamp_sec_ = 0.0;
  high_acceleration_mode_ = false;
  acceleration_stable_count_ = 0;
  fire_ready_count_ = 0;
}

double AutoAimer::normalizeRadians(double angle_rad) {
  return std::remainder(angle_rad, 2.0 * kPi);
}

AutoTarget AutoAimer::predictTarget(const AutoTarget &target,
                                    double delay_sec) {
  AutoTarget output = target;
  output.center_m += output.velocity_mps * std::max(0.0, delay_sec);
  output.yaw_rad =
      normalizeRadians(output.yaw_rad + output.angular_velocity_radps *
                                            std::max(0.0, delay_sec));
  return output;
}

std::array<AutoAimArmor, 4> AutoAimer::buildArmors(const AutoTarget &target) {
  std::array<AutoAimArmor, 4> armors{};
  for (int index = 0; index < 4; ++index) {
    const bool offset = index == 1 || index == 3;
    const double inward_yaw = normalizeRadians(
        target.yaw_rad + static_cast<double>(index) * kPi / 2.0);
    const double radius =
        offset ? target.radius_m + target.radius_offset_m : target.radius_m;
    armors[static_cast<std::size_t>(index)].index = index;
    armors[static_cast<std::size_t>(index)].inward_yaw_rad = inward_yaw;
    armors[static_cast<std::size_t>(index)].position_m =
        Eigen::Vector3d(target.center_m.x() - radius * std::cos(inward_yaw),
                        target.center_m.y() - radius * std::sin(inward_yaw),
                        offset ? target.center_m.z() + target.height_offset_m
                               : target.center_m.z());
    const double surface_yaw = normalizeRadians(inward_yaw + kPi);
    const double to_shooter_yaw =
        std::atan2(-armors[static_cast<std::size_t>(index)].position_m.y(),
                   -armors[static_cast<std::size_t>(index)].position_m.x());
    armors[static_cast<std::size_t>(index)].phase_error_rad =
        normalizeRadians(surface_yaw - to_shooter_yaw);
  }
  return armors;
}

std::optional<AutoAimArmor>
AutoAimer::chooseArmor(const AutoTarget &target) const {
  const auto armors = buildArmors(target);
  const bool fast = std::abs(target.angular_velocity_radps) >
                    config_.high_spin_threshold_radps;
  if (!fast && locked_armor_index_ >= 0) {
    const auto &held = armors[static_cast<std::size_t>(locked_armor_index_)];
    if (std::abs(held.phase_error_rad) <= config_.coming_angle_rad) {
      return held;
    }
  }
  std::optional<AutoAimArmor> best;
  for (const auto &armor : armors) {
    if (fast && std::abs(armor.phase_error_rad) > config_.coming_angle_rad) {
      continue;
    }
    // 高速旋转时只选择正朝可射窗口靠近的一面，避免刚离开正面的面板。
    if (fast && target.angular_velocity_radps > 0.0 &&
        armor.phase_error_rad > config_.leaving_angle_rad) {
      continue;
    }
    if (fast && target.angular_velocity_radps < 0.0 &&
        armor.phase_error_rad < -config_.leaving_angle_rad) {
      continue;
    }
    if (!best.has_value() ||
        std::abs(armor.phase_error_rad) < std::abs(best->phase_error_rad)) {
      best = armor;
    }
  }
  return best;
}

const AutoTarget *
AutoAimer::chooseTarget(const std::vector<AutoTarget> &targets) {
  const auto valid = [](const AutoTarget &target) {
    return finiteTarget(target);
  };
  if (locked_target_id_.has_value()) {
    const auto it =
        std::find_if(targets.begin(), targets.end(),
                     [this, &valid](const AutoTarget &target) {
                       return target.id == *locked_target_id_ && valid(target);
                     });
    if (it != targets.end()) {
      return &*it;
    }
    locked_target_id_.reset();
    locked_armor_index_ = -1;
  }
  const auto it = std::min_element(
      targets.begin(), targets.end(),
      [&valid](const AutoTarget &lhs, const AutoTarget &rhs) {
        if (!valid(lhs)) {
          return false;
        }
        if (!valid(rhs)) {
          return true;
        }
        return lhs.center_m.squaredNorm() < rhs.center_m.squaredNorm();
      });
  if (it == targets.end() || !valid(*it)) {
    return nullptr;
  }
  locked_target_id_ = it->id;
  return &*it;
}

double AutoAimer::updateAcceleration(const AutoTarget &target,
                                     double state_stamp_sec) {
  const Eigen::Vector2d velocity(target.velocity_mps.x(),
                                 target.velocity_mps.y());
  double acceleration = 0.0;
  if (previous_velocity_mps_.has_value() &&
      state_stamp_sec > previous_velocity_stamp_sec_ + 1e-6) {
    acceleration = (velocity - *previous_velocity_mps_).norm() /
                   (state_stamp_sec - previous_velocity_stamp_sec_);
  }
  previous_velocity_mps_ = velocity;
  previous_velocity_stamp_sec_ = state_stamp_sec;
  return acceleration;
}

bool AutoAimer::updateAccelerationSafety(double acceleration_mps2) {
  if (acceleration_mps2 > config_.high_acceleration_threshold_mps2) {
    high_acceleration_mode_ = true;
    acceleration_stable_count_ = 0;
  } else if (high_acceleration_mode_) {
    if (acceleration_mps2 < config_.stable_acceleration_threshold_mps2) {
      ++acceleration_stable_count_;
      if (acceleration_stable_count_ >= config_.acceleration_stable_frames) {
        high_acceleration_mode_ = false;
        acceleration_stable_count_ = 0;
      }
    } else {
      acceleration_stable_count_ = 0;
    }
  }
  return !high_acceleration_mode_;
}

} // namespace aim_auto
