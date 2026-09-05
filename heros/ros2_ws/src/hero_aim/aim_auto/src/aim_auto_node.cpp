#include "aim_auto/aim_auto_node.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace aim_auto
{

namespace
{

constexpr double kDegreesToRadians = 0.017453292519943295;

rclcpp::QoS highRateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

double stampToSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time secondsToStamp(double seconds)
{
  builtin_interfaces::msg::Time stamp;
  const auto nanoseconds = static_cast<std::int64_t>(seconds * 1e9);
  stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

}  // namespace

AimAutoNode::AimAutoNode()
: Node("aim_auto_node"), aimer_(AutoAimConfig{})
{
  declare_parameter<bool>("enabled", false);
  declare_parameter<bool>("enable_fire", false);
  declare_parameter<std::string>("target_state_topic", "/hero/aim/autoaim/target_states");
  declare_parameter<std::string>("gimbal_state_topic", "/hero/gimbal/state");
  declare_parameter<std::string>("control_candidate_topic", "/hero/aim/autoaim/controller");
  declare_parameter<std::string>("debug_topic", "/hero/aim/autoaim/debug");
  declare_parameter<std::string>("target_frame_id", "world");
  declare_parameter("bullet_speed_mps", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("drag_coefficient", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("gravity_mps2", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("air_density_kgpm3", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("bullet_mass_kg", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("bullet_radius_m", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("muzzle_offset_m", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("ballistic_iteration_count", rclcpp::ParameterType::PARAMETER_INTEGER);
  // 公共平滑参数必须由 aim_core/config/angle_smoother.yaml 提供。
  declare_parameter<double>("system_response_time_sec", 0.05);
  declare_parameter<double>("high_spin_threshold_radps", 2.0);
  declare_parameter<double>("coming_angle_deg", 60.0);
  declare_parameter<double>("leaving_angle_deg", 40.0);
  declare_parameter<double>("shoot_yaw_tolerance_deg", 2.0);
  declare_parameter<double>("shoot_pitch_tolerance_deg", 2.0);
  declare_parameter<int>("fire_confirm_frames", 4);
  declare_parameter<int>("fast_fire_confirm_frames", 2);
  declare_parameter<double>("high_acceleration_threshold_mps2", 0.5);
  declare_parameter<double>("stable_acceleration_threshold_mps2", 0.05);
  declare_parameter<int>("acceleration_stable_frames", 5);
  declare_parameter("yaw_previous_weight", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("pitch_previous_weight", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("yaw_jump_threshold_deg", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("pitch_jump_threshold_deg", rclcpp::ParameterType::PARAMETER_DOUBLE);

  enabled_ = get_parameter("enabled").as_bool();
  enable_fire_ = get_parameter("enable_fire").as_bool();
  target_frame_id_ = get_parameter("target_frame_id").as_string();
  const auto target_state_topic = get_parameter("target_state_topic").as_string();
  const auto gimbal_state_topic = get_parameter("gimbal_state_topic").as_string();
  const auto control_candidate_topic = get_parameter("control_candidate_topic").as_string();
  const auto debug_topic = get_parameter("debug_topic").as_string();
  if (target_frame_id_.empty() || target_state_topic.empty() || gimbal_state_topic.empty() ||
    control_candidate_topic.empty() || debug_topic.empty())
  {
    throw std::invalid_argument("自瞄控制字符串参数不能为空");
  }

  AutoAimConfig config;
  config.ballistics.bullet_speed_mps = get_parameter("bullet_speed_mps").as_double();
  config.ballistics.drag_coefficient = get_parameter("drag_coefficient").as_double();
  config.ballistics.gravity_mps2 = get_parameter("gravity_mps2").as_double();
  config.ballistics.air_density_kgpm3 = get_parameter("air_density_kgpm3").as_double();
  config.ballistics.bullet_mass_kg = get_parameter("bullet_mass_kg").as_double();
  config.ballistics.bullet_radius_m = get_parameter("bullet_radius_m").as_double();
  config.ballistics.muzzle_offset_m = get_parameter("muzzle_offset_m").as_double();
  config.ballistics.iteration_count = get_parameter("ballistic_iteration_count").as_int();
  config.system_response_time_sec = get_parameter("system_response_time_sec").as_double();
  config.high_spin_threshold_radps = get_parameter("high_spin_threshold_radps").as_double();
  config.coming_angle_rad = get_parameter("coming_angle_deg").as_double() * kDegreesToRadians;
  config.leaving_angle_rad = get_parameter("leaving_angle_deg").as_double() * kDegreesToRadians;
  config.shoot_yaw_tolerance_rad = get_parameter("shoot_yaw_tolerance_deg").as_double() * kDegreesToRadians;
  config.shoot_pitch_tolerance_rad = get_parameter("shoot_pitch_tolerance_deg").as_double() * kDegreesToRadians;
  config.fire_confirm_frames = get_parameter("fire_confirm_frames").as_int();
  config.fast_fire_confirm_frames = get_parameter("fast_fire_confirm_frames").as_int();
  config.high_acceleration_threshold_mps2 = get_parameter("high_acceleration_threshold_mps2").as_double();
  config.stable_acceleration_threshold_mps2 = get_parameter("stable_acceleration_threshold_mps2").as_double();
  config.acceleration_stable_frames = get_parameter("acceleration_stable_frames").as_int();
  config.smoother.yaw_previous_weight = get_parameter("yaw_previous_weight").as_double();
  config.smoother.pitch_previous_weight = get_parameter("pitch_previous_weight").as_double();
  config.smoother.yaw_jump_threshold_rad = get_parameter("yaw_jump_threshold_deg").as_double() * kDegreesToRadians;
  config.smoother.pitch_jump_threshold_rad = get_parameter("pitch_jump_threshold_deg").as_double() * kDegreesToRadians;
  aimer_ = AutoAimer(config);

  const auto qos = highRateQos();
  control_candidate_pub_ = create_publisher<hero_msgs::msg::ControlCommand>(control_candidate_topic, qos);
  debug_pub_ = create_publisher<hero_msgs::msg::AutoAimDebug>(debug_topic, qos);
  gimbal_state_sub_ = create_subscription<hero_msgs::msg::GimbalState>(
    gimbal_state_topic, qos,
    [this](const hero_msgs::msg::GimbalState::ConstSharedPtr message) { receiveGimbalState(message); });
  target_state_sub_ = create_subscription<hero_msgs::msg::TargetStateArray>(
    target_state_topic, qos,
    [this](const hero_msgs::msg::TargetStateArray::ConstSharedPtr message) { receiveTargetStates(message); });

  RCLCPP_INFO(
    get_logger(), "自瞄控制节点已启动：输入 %s，候选输出 %s，当前%s、%s开火",
    target_state_topic.c_str(), control_candidate_topic.c_str(), enabled_ ? "启用" : "禁用",
    enable_fire_ ? "允许" : "禁止");
}

void AimAutoNode::receiveGimbalState(const hero_msgs::msg::GimbalState::ConstSharedPtr & message)
{
  if (message->right_clicked && !previous_right_clicked_) {
    aimer_.reset();
  }
  previous_right_clicked_ = message->right_clicked;
  if (message->mode != hero_msgs::msg::GimbalState::MODE_AUTO_AIM) {
    aimer_.reset();
  }
  latest_gimbal_state_ = *message;
}

void AimAutoNode::receiveTargetStates(const hero_msgs::msg::TargetStateArray::ConstSharedPtr & message)
{
  if (!latest_gimbal_state_.has_value() ||
    latest_gimbal_state_->mode != hero_msgs::msg::GimbalState::MODE_AUTO_AIM ||
    message->header.frame_id != target_frame_id_)
  {
    return;
  }
  std::vector<AutoTarget> targets;
  targets.reserve(message->targets.size());
  for (const auto & state : message->targets) {
    targets.push_back(AutoTarget{
      state.id, state.tracking, state.converged,
      Eigen::Vector3d(state.center.x, state.center.y, state.center.z),
      Eigen::Vector3d(state.velocity.x, state.velocity.y, state.velocity.z),
      state.yaw, state.angular_velocity, state.radius, state.radius_offset, state.height_offset});
  }
  const auto control_time = now();
  const auto result = aimer_.aim(
    targets, stampToSeconds(message->header.stamp), control_time.seconds(),
    latest_gimbal_state_->yaw, latest_gimbal_state_->pitch);
  const bool shoot_status = result.has_value() && enable_fire_ && result->shoot_ready;
  // 发送 debug 消息
  publishDebug(*message, result, shoot_status, control_time);
  if (!enabled_ || !result.has_value()) {
    return;
  }
  hero_msgs::msg::ControlCommand command;
  command.header.stamp = control_time;
  command.header.frame_id = target_frame_id_;
  command.yaw = static_cast<float>(result->command_yaw_rad);
  command.pitch = static_cast<float>(result->command_pitch_rad);
  command.shoot_status = shoot_status ? 1U : 0U;
  command.target_id = result->target_id;
  // 发送 command
  control_candidate_pub_->publish(command); 
}

void AimAutoNode::publishDebug(
  const hero_msgs::msg::TargetStateArray & states, const std::optional<AutoAimResult> & result,
  bool shoot_status, const rclcpp::Time & control_time)
{
  if (debug_pub_->get_subscription_count() == 0U) {
    return;
  }
  hero_msgs::msg::AutoAimDebug debug;
  debug.header.stamp = control_time;
  debug.header.frame_id = target_frame_id_;
  debug.measurement_stamp = states.measurement_stamp;
  debug.state_stamp = states.header.stamp;
  if (result.has_value()) {
    debug.valid = true;
    debug.target_locked = result->target_locked;
    debug.target_id = result->target_id;
    debug.armor_index = static_cast<std::int8_t>(result->armor_index);
    debug.aim_stamp = secondsToStamp(result->aim_time_sec);
    debug.aim_point.x = result->aim_point_m.x();
    debug.aim_point.y = result->aim_point_m.y();
    debug.aim_point.z = result->aim_point_m.z();
    debug.raw_yaw = static_cast<float>(result->raw_yaw_rad);
    debug.raw_pitch = static_cast<float>(result->raw_pitch_rad);
    debug.command_yaw = static_cast<float>(result->command_yaw_rad);
    debug.command_pitch = static_cast<float>(result->command_pitch_rad);
    debug.flight_time_sec = static_cast<float>(result->flight_time_sec);
    debug.state_age_sec = static_cast<float>(result->state_age_sec);
    debug.system_response_time_sec = static_cast<float>(result->aim_time_sec - control_time.seconds() - result->flight_time_sec);
    debug.phase_error_rad = static_cast<float>(result->phase_error_rad);
    debug.yaw_error_rad = static_cast<float>(result->yaw_error_rad);
    debug.pitch_error_rad = static_cast<float>(result->pitch_error_rad);
    debug.planar_acceleration_mps2 = static_cast<float>(result->planar_acceleration_mps2);
    debug.target_converged = result->target_converged;
    debug.phase_ready = result->phase_ready;
    debug.gimbal_ready = result->gimbal_ready;
    debug.acceleration_safe = result->acceleration_safe;
    debug.fire_enabled = enable_fire_;
    debug.shoot_status = shoot_status;
  }
  debug_pub_->publish(debug);
}

}  // aim_auto
