#include "aim_normal/aim_normal_node.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace aim_normal
{

namespace
{

rclcpp::QoS highRateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

}  // namespace

AimNormalNode::AimNormalNode()
: Node("aim_normal_node"), aimer_(NormalAimerConfig{})
{
  // 节点参数
  declare_parameter<bool>("enabled", false);
  declare_parameter<bool>("enable_fire", false);
  declare_parameter<std::string>("armor_pose_topic", "/hero/solver/armor_poses");
  declare_parameter<std::string>("gimbal_state_topic", "/hero/gimbal/state");
  declare_parameter<std::string>("control_candidate_topic", "/hero/aim/normalaim/controller");
  declare_parameter<std::string>("debug_topic", "/hero/aim/normalaim/debug");
  declare_parameter<std::string>("target_frame_id", "world");
  declare_parameter<double>("keep_target_distance_m", 0.3);
  declare_parameter<int>("max_lost_frames", 0);
  declare_parameter<double>("large_armor_distance_factor", 1.2);
  // 公共弹道参数没有 C++ 兜底值；必须由 aim_core/config/ballistics.yaml 提供。
  declare_parameter("bullet_speed_mps", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("drag_coefficient", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("gravity_mps2", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("air_density_kgpm3", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("bullet_mass_kg", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("bullet_radius_m", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("muzzle_offset_m", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("ballistic_iteration_count", rclcpp::ParameterType::PARAMETER_INTEGER);
  // 公共平滑参数必须由 aim_core/config/angle_smoother.yaml 提供。
  declare_parameter("yaw_previous_weight", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("pitch_previous_weight", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("yaw_jump_threshold_deg", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("pitch_jump_threshold_deg", rclcpp::ParameterType::PARAMETER_DOUBLE);

  enabled_ = get_parameter("enabled").as_bool();
  enable_fire_ = get_parameter("enable_fire").as_bool();
  target_frame_id_ = get_parameter("target_frame_id").as_string();
  const auto max_lost_frames = get_parameter("max_lost_frames").as_int();
  const auto ballistic_iteration_count = get_parameter("ballistic_iteration_count").as_int();
  if (max_lost_frames < 0 || ballistic_iteration_count <= 0) {
    throw std::invalid_argument("普通瞄准策略的整数参数无效");
  }
  constexpr double kDegreesToRadians = 0.017453292519943295;
  NormalAimerConfig config;
  config.keep_target_distance_m = get_parameter("keep_target_distance_m").as_double();
  config.max_lost_frames = static_cast<std::uint32_t>(max_lost_frames);
  config.large_armor_distance_factor = get_parameter("large_armor_distance_factor").as_double();
  config.ballistic.bullet_speed_mps = get_parameter("bullet_speed_mps").as_double();
  config.ballistic.drag_coefficient = get_parameter("drag_coefficient").as_double();
  config.ballistic.gravity_mps2 = get_parameter("gravity_mps2").as_double();
  config.ballistic.air_density_kgpm3 = get_parameter("air_density_kgpm3").as_double();
  config.ballistic.bullet_mass_kg = get_parameter("bullet_mass_kg").as_double();
  config.ballistic.bullet_radius_m = get_parameter("bullet_radius_m").as_double();
  config.ballistic.muzzle_offset_m = get_parameter("muzzle_offset_m").as_double();
  config.ballistic.iteration_count = ballistic_iteration_count;
  config.smoother.yaw_previous_weight = get_parameter("yaw_previous_weight").as_double();
  config.smoother.pitch_previous_weight = get_parameter("pitch_previous_weight").as_double();
  config.smoother.yaw_jump_threshold_rad =
    get_parameter("yaw_jump_threshold_deg").as_double() * kDegreesToRadians;
  config.smoother.pitch_jump_threshold_rad =
    get_parameter("pitch_jump_threshold_deg").as_double() * kDegreesToRadians;
  aimer_ = NormalAimer(config);
  const auto armor_pose_topic = get_parameter("armor_pose_topic").as_string();
  const auto gimbal_state_topic = get_parameter("gimbal_state_topic").as_string();
  const auto control_candidate_topic = get_parameter("control_candidate_topic").as_string();
  const auto debug_topic = get_parameter("debug_topic").as_string();
  if (
    target_frame_id_.empty() || armor_pose_topic.empty() || gimbal_state_topic.empty() ||
    control_candidate_topic.empty() || debug_topic.empty())
  {
    throw std::invalid_argument("普通瞄准策略字符串参数无效");
  }

  const auto qos = highRateQos();
  control_candidate_pub_ = create_publisher<hero_msgs::msg::ControlCommand>(control_candidate_topic, qos);
  debug_pub_ = create_publisher<hero_msgs::msg::NormalAimDebug>(debug_topic, qos);
  gimbal_state_sub_ = create_subscription<hero_msgs::msg::GimbalState>(
    gimbal_state_topic, qos,
    [this](const hero_msgs::msg::GimbalState::ConstSharedPtr message) { receiveGimbalState(message); });
  armor_pose_sub_ = create_subscription<hero_msgs::msg::ArmorPoseArray>(
    armor_pose_topic, qos,
    [this](const hero_msgs::msg::ArmorPoseArray::ConstSharedPtr message) { receiveArmorPoses(message); });

  RCLCPP_INFO(
    get_logger(), "普通瞄准策略已启动：输入 %s，候选输出 %s，当前%s、%s开火",
    armor_pose_topic.c_str(), control_candidate_topic.c_str(), enabled_ ? "启用" : "禁用",
    enable_fire_ ? "允许" : "禁止");
}

void AimNormalNode::receiveGimbalState(const hero_msgs::msg::GimbalState::ConstSharedPtr & message)
{
  if (message->right_clicked && !previous_right_clicked_) {
    aimer_.reset();
  }
  previous_right_clicked_ = message->right_clicked;
  if (message->mode != hero_msgs::msg::GimbalState::MODE_NORMAL) {
    aimer_.reset();
  }
  latest_gimbal_state_ = *message;
}

//每收到一次ArmorPoses就发布debug与command
void AimNormalNode::receiveArmorPoses(const hero_msgs::msg::ArmorPoseArray::ConstSharedPtr & message)
{
  if (
    !latest_gimbal_state_.has_value() ||
    latest_gimbal_state_->mode != hero_msgs::msg::GimbalState::MODE_NORMAL)
  {
    return;
  }
  if (message->header.frame_id != target_frame_id_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "普通瞄准策略仅接受 %s 坐标系位姿，当前为 %s",
      target_frame_id_.c_str(), message->header.frame_id.c_str());
    return;
  }

  std::vector<ArmorObservation> observations;
  observations.reserve(message->armors.size());
  for (const auto & armor : message->armors) {
    const auto & position = armor.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
      continue;
    }
    observations.push_back(ArmorObservation{armor.id, position.x, position.y, position.z, armor.is_large});
  }

  const auto result = aimer_.aim(
    observations, latest_gimbal_state_->yaw, latest_gimbal_state_->pitch);
  const bool shoot_status = result.has_value() && enable_fire_ && !result->selected.held_target;
  publishDebug(*message, result, shoot_status);
  if (!result.has_value() || !enabled_) {
    return;
  }

  hero_msgs::msg::ControlCommand command;
  command.header.stamp = now();
  command.header.frame_id = latest_gimbal_state_->header.frame_id;
  command.yaw = static_cast<float>(result->command_yaw_rad);
  command.pitch = static_cast<float>(result->command_pitch_rad);
  command.shoot_status = shoot_status ? 1U : 0U;
  command.target_id = result->selected.observation.id;
  control_candidate_pub_->publish(command);
}

void AimNormalNode::publishDebug(
  const hero_msgs::msg::ArmorPoseArray & message,
  const std::optional<NormalAimResult> & result, bool shoot_status)
{
  if (debug_pub_->get_subscription_count() == 0U) {
    return;
  }

  hero_msgs::msg::NormalAimDebug debug;
  debug.header = message.header;
  debug.valid = result.has_value();
  if (result.has_value()) {
    debug.held_target = result->selected.held_target;
    debug.lost_frames = result->selected.lost_frames;
    debug.target_id = result->selected.observation.id;
    debug.target_position.x = result->selected.observation.x;
    debug.target_position.y = result->selected.observation.y;
    debug.target_position.z = result->selected.observation.z;
    debug.raw_yaw = static_cast<float>(result->raw_yaw_rad);
    debug.raw_pitch = static_cast<float>(result->raw_pitch_rad);
    debug.command_yaw = static_cast<float>(result->command_yaw_rad);
    debug.command_pitch = static_cast<float>(result->command_pitch_rad);
    debug.flight_time_sec = static_cast<float>(result->flight_time_sec);
    debug.shoot_status = shoot_status;
  }
  debug_pub_->publish(debug);
}

}  // aim_normal
