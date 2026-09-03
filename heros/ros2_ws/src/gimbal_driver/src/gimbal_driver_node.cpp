#include "gimbal_driver/gimbal_driver_node.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace gimbal_driver
{

namespace
{

constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

rclcpp::QoS highRateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

uint8_t toMode(uint8_t raw_mode_flag)
{
  switch (raw_mode_flag) {
    case hero_msgs::msg::GimbalState::MODE_NORMAL:
    case hero_msgs::msg::GimbalState::MODE_ANTI_TOP:
    case hero_msgs::msg::GimbalState::MODE_AUTO_AIM:
    case hero_msgs::msg::GimbalState::MODE_ANTI_BASE:
      return raw_mode_flag;
    default:
      return hero_msgs::msg::GimbalState::MODE_NORMAL;
  }
}

std::chrono::nanoseconds periodFromRateHz(double rate_hz)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / rate_hz));
}

}  // namespace

GimbalDriverNode::GimbalDriverNode()
: Node("gimbal_driver_node"), serial_port_(std::make_unique<SerialPort>())
{
  //节点参数
  declare_parameter<std::string>("port_name", "/dev/ttyACM0");
  declare_parameter<int>("baud_rate", 115200);
  declare_parameter<bool>("enable_crc_check", true);
  declare_parameter<bool>("enable_fire", false);
  declare_parameter<int>("command_flag", 0x05);
  declare_parameter<int>("virtual_mode", 1);
  declare_parameter<int>("virtual_robot_color", 0);
  declare_parameter<std::string>("state_topic", "/hero/gimbal/state");
  declare_parameter<std::string>("control_topic", "/hero/gimbal/control");
  declare_parameter<std::string>("gimbal_frame_id", "gimbal_link");
  declare_parameter<double>("state_publish_rate_hz", 200.0);
  declare_parameter<double>("command_send_rate_hz", 200.0);

  const auto state_rate = get_parameter("state_publish_rate_hz").as_double();
  const auto command_rate = get_parameter("command_send_rate_hz").as_double();
  if (state_rate <= 0.0 || command_rate <= 0.0) {
    throw std::invalid_argument("state_publish_rate_hz and command_send_rate_hz must be positive");
  }
  const auto command_flag = get_parameter("command_flag").as_int();
  if (command_flag < 0 || command_flag > 255) {
    throw std::invalid_argument("command_flag must fit in uint8");
  }
  enable_fire_ = get_parameter("enable_fire").as_bool();
  command_flag_ = static_cast<uint8_t>(command_flag);
  gimbal_frame_id_ = get_parameter("gimbal_frame_id").as_string();
  if (!virtual_state_.setMode(get_parameter("virtual_mode").as_int())) {
    throw std::invalid_argument("virtual_mode 必须是 1 到 4");
  }
  if (!virtual_state_.setRobotColor(get_parameter("virtual_robot_color").as_int())) {
    throw std::invalid_argument("virtual_robot_color 必须是 0 或 1");
  }

  //pub&sub
  state_pub_ = create_publisher<hero_msgs::msg::GimbalState>(
    get_parameter("state_topic").as_string(), highRateQos());
  control_sub_ = create_subscription<hero_msgs::msg::ControlCommand>(
    get_parameter("control_topic").as_string(), highRateQos(),
    [this](const hero_msgs::msg::ControlCommand::SharedPtr message) {receiveCommand(*message);});

  //注册串口回调
  serial_port_->setReadCallback([this](const LegacyReadFrame & frame) {receiveState(frame);});
  serial_port_->setErrorCallback([this](const std::string & message) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", message.c_str());
  });

  if (!serial_port_->start(
      get_parameter("port_name").as_string(), get_parameter("baud_rate").as_int(),
      get_parameter("enable_crc_check").as_bool()))
  {
    throw std::runtime_error("串口驱动启动失败");
  }
  virtual_serial_ = serial_port_->isVirtual();
  parameter_callback_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return handleParameters(parameters);
    });

  //创建state与command定时发布器
  state_timer_ = create_wall_timer(periodFromRateHz(state_rate), [this]() {publishState();});
  command_timer_ = create_wall_timer(periodFromRateHz(command_rate), [this]() {sendCommand();});
  RCLCPP_INFO(
    get_logger(), "云台串口驱动已启动：%s，开火输出%s",
    virtual_serial_ ? "虚拟串口" : "真实串口", enable_fire_ ? "已启用" : "未启用");
}

GimbalDriverNode::~GimbalDriverNode()
{
  serial_port_->stop();
}

void GimbalDriverNode::receiveState(const LegacyReadFrame & frame)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  int8_t exposure_step = 0;
  if (frame.up && !previous_up_ && !frame.down) {
    exposure_step = 1;
  } else if (frame.down && !previous_down_ && !frame.up) {
    exposure_step = -1;
  }
  previous_up_ = frame.up;
  previous_down_ = frame.down;
  latest_state_ = CachedState{frame, now(), exposure_step};
}

void GimbalDriverNode::publishState()
{
  if (virtual_serial_) {
    LegacyReadFrame frame;
    {
      std::lock_guard<std::mutex> lock(virtual_state_mutex_);
      frame = virtual_state_.frame();
    }
    publishState(frame, now(), 0);
    return;
  }

  std::optional<CachedState> state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = latest_state_;
    if (latest_state_.has_value()) {
      // 旧接口对一次曝光边沿只消费一次，不能因为状态发布频率高于串口接收频率而重复发布。
      latest_state_->exposure_step = 0;
    }
  }
  if (!state.has_value()) {
    return;
  }
  publishState(state->frame, state->stamp, state->exposure_step);
}

void GimbalDriverNode::publishState(
  const LegacyReadFrame & frame, const rclcpp::Time & stamp, int8_t exposure_step)
{
  hero_msgs::msg::GimbalState message;
  message.header.stamp = stamp;
  message.header.frame_id = gimbal_frame_id_;
  message.yaw = static_cast<float>(frame.yaw_deg * kDegreesToRadians);
  message.pitch = static_cast<float>(frame.pitch_deg * kDegreesToRadians);
  message.mode = toMode(frame.mode_flag);
  message.raw_mode_flag = frame.mode_flag;
  message.robot_color = frame.robot_color;
  message.right_clicked = frame.right_clicked;
  message.up = frame.up;
  message.down = frame.down;
  message.exposure_step = exposure_step;
  state_pub_->publish(message);
}

rcl_interfaces::msg::SetParametersResult GimbalDriverNode::handleParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  if (!virtual_serial_) {
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "virtual_mode" || parameter.get_name() == "virtual_robot_color") {
        result.successful = false;
        result.reason = "只有 port_name 为 virtual 时才能修改虚拟状态";
        return result;
      }
    }
    return result;
  }

  std::lock_guard<std::mutex> lock(virtual_state_mutex_);
  auto next_state = virtual_state_;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "virtual_mode" && !next_state.setMode(parameter.as_int())) {
      result.successful = false;
      result.reason = "virtual_mode 必须是 1 到 4";
      return result;
    }
    if (
      parameter.get_name() == "virtual_robot_color" &&
      !next_state.setRobotColor(parameter.as_int()))
    {
      result.successful = false;
      result.reason = "virtual_robot_color 必须是 0 或 1";
      return result;
    }
  }
  virtual_state_ = next_state;
  return result;
}

void GimbalDriverNode::receiveCommand(const hero_msgs::msg::ControlCommand & message)
{
  if (!std::isfinite(message.yaw) || !std::isfinite(message.pitch)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ignoring non-finite gimbal command");
    return;
  }
  std::lock_guard<std::mutex> lock(command_mutex_);
  latest_command_ = CachedCommand{message};
}

void GimbalDriverNode::sendCommand()
{
  std::optional<CachedCommand> command;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command = latest_command_;
  }
  if (!command.has_value()) {
    return;
  }
  LegacyWriteCommand wire_command;
  wire_command.yaw_deg = static_cast<float>(command->message.yaw * kRadiansToDegrees);
  wire_command.pitch_deg = static_cast<float>(command->message.pitch * kRadiansToDegrees);
  wire_command.shoot_status = enable_fire_ ? command->message.shoot_status : 0U;
  wire_command.target_id = command->message.target_id;
  wire_command.command_flag = command_flag_;
  serial_port_->write(wire_command);
}

}  // gimbal_driver
