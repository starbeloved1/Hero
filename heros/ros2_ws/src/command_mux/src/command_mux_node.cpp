#include "command_mux/command_mux_node.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace command_mux
{

namespace
{

rclcpp::QoS highRateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

std::chrono::nanoseconds periodFromRateHz(double rate_hz)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / rate_hz));
}

}  // namespace

CommandMuxNode::CommandMuxNode()
: Node("command_mux_node")
{
  declare_parameter<std::string>("gimbal_state_topic", "/hero/gimbal/state");
  declare_parameter<std::string>("control_topic", "/hero/gimbal/control");
  declare_parameter<std::string>("normal_control_topic", "/hero/normal_aim/control_candidate");
  declare_parameter<std::string>("anti_top_control_topic", "/hero/anti_top/control_candidate");
  declare_parameter<std::string>("auto_aim_control_topic", "/hero/auto_aim/control_candidate");
  declare_parameter<std::string>("anti_base_control_topic", "/hero/anti_base/control_candidate");
  declare_parameter<double>("output_rate_hz", 200.0);
  declare_parameter<double>("max_command_age_sec", 0.1);

  const auto output_rate_hz = get_parameter("output_rate_hz").as_double();
  max_command_age_sec_ = get_parameter("max_command_age_sec").as_double();
  if (output_rate_hz <= 0.0) {
    throw std::invalid_argument("output_rate_hz 必须大于 0");
  }
  if (max_command_age_sec_ < 0.0) {
    throw std::invalid_argument("max_command_age_sec 不能小于 0");
  }

  const auto qos = highRateQos();
  control_pub_ = create_publisher<hero_msgs::msg::ControlCommand>(
    get_parameter("control_topic").as_string(), qos);
  gimbal_state_sub_ = create_subscription<hero_msgs::msg::GimbalState>(
    get_parameter("gimbal_state_topic").as_string(), qos,
    [this](const hero_msgs::msg::GimbalState::ConstSharedPtr message) {
      receiveGimbalState(message);
    });

  const std::array<std::pair<CommandSource, const char *>, kCandidateCount> candidates{{
      {CommandSource::kNormalAim, "normal_control_topic"},
      {CommandSource::kAntiTop, "anti_top_control_topic"},
      {CommandSource::kAutoAim, "auto_aim_control_topic"},
      {CommandSource::kAntiBase, "anti_base_control_topic"},
    }};
  for (const auto & [source, parameter_name] : candidates) {
    candidate_subs_[sourceIndex(source)] = create_subscription<hero_msgs::msg::ControlCommand>(
      get_parameter(parameter_name).as_string(), qos,
      [this, source](const hero_msgs::msg::ControlCommand::ConstSharedPtr message) {
        receiveCandidate(source, message);
      });
  }

  output_timer_ = create_wall_timer(
    periodFromRateHz(output_rate_hz), [this]() { publishSelectedCommand(); });
  RCLCPP_INFO(
    get_logger(), "控制仲裁节点已启动：输出 %s，候选命令最大年龄 %.3f s",
    get_parameter("control_topic").as_string().c_str(), max_command_age_sec_);
}

std::size_t CommandMuxNode::sourceIndex(CommandSource source)
{
  switch (source) {
    case CommandSource::kNormalAim:
      return 0U;
    case CommandSource::kAntiTop:
      return 1U;
    case CommandSource::kAutoAim:
      return 2U;
    case CommandSource::kAntiBase:
      return 3U;
    case CommandSource::kNone:
      break;
  }
  throw std::invalid_argument("无效的候选控制来源");
}

void CommandMuxNode::receiveGimbalState(const hero_msgs::msg::GimbalState::ConstSharedPtr & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_gimbal_state_ = *message;
}

void CommandMuxNode::receiveCandidate(
  CommandSource source, const hero_msgs::msg::ControlCommand::ConstSharedPtr & message)
{
  if (!std::isfinite(message->yaw) || !std::isfinite(message->pitch)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "忽略包含非有限角度的候选控制命令");
    return;
  }
  if (rclcpp::Time(message->header.stamp).nanoseconds() <= 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "忽略没有有效时间戳的候选控制命令");
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  candidates_[sourceIndex(source)] = CachedCandidate{*message};
}

void CommandMuxNode::publishSelectedCommand()
{
  const auto publish_time = now();
  std::optional<hero_msgs::msg::GimbalState> state;
  std::optional<CachedCandidate> candidate;
  CommandSource source = CommandSource::kNone;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_gimbal_state_.has_value()) {
      return;
    }
    state = latest_gimbal_state_;
    source = sourceForMode(state->mode);
    if (source != CommandSource::kNone) {
      candidate = candidates_[sourceIndex(source)];
    }
  }

  auto output = makeSafeCommand(*state, publish_time);
  if (candidate.has_value()) {
    const auto candidate_time = rclcpp::Time(candidate->command.header.stamp);
    if (isFresh(candidate_time.seconds(), publish_time.seconds(), max_command_age_sec_)) {
      output = candidate->command;
      // 控制帧表示本次仲裁后立即送往下位机的命令，时间戳写为本次输出时刻。
      output.header.stamp = publish_time;
    }
  }
  control_pub_->publish(output);
}

hero_msgs::msg::ControlCommand CommandMuxNode::makeSafeCommand(
  const hero_msgs::msg::GimbalState & state, const rclcpp::Time & stamp) const
{
  hero_msgs::msg::ControlCommand command;
  command.header.stamp = stamp;
  command.header.frame_id = state.header.frame_id;
  command.yaw = state.yaw;
  command.pitch = state.pitch;
  command.shoot_status = 0U;
  command.target_id = 0U;
  return command;
}

}  // command_mux
