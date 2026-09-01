#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "hero_gimbal_driver/serial_port.hpp"
#include "hero_msgs/msg/control_command.hpp"
#include "hero_msgs/msg/gimbal_state.hpp"

namespace hero_gimbal_driver
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

//ROS节点类:HeroGimbalDriverNode
class HeroGimbalDriverNode : public rclcpp::Node
{
public:
  HeroGimbalDriverNode()
  : Node("hero_gimbal_driver_node"), serial_port_(std::make_unique<SerialPort>())
  {
    //节点参数
    declare_parameter<std::string>("port_name", "/dev/ttyACM0");
    declare_parameter<int>("baud_rate", 115200);
    declare_parameter<bool>("enable_crc_check", true);
    declare_parameter<bool>("allow_virtual_serial", false);
    declare_parameter<bool>("enable_fire", false);
    declare_parameter<int>("command_flag", 0x05);
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

    //pub&sub
    //发布当前云台状态，订阅控制指令
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
    
    //启动串口
    if (!serial_port_->start(
        get_parameter("port_name").as_string(), get_parameter("baud_rate").as_int(),
        get_parameter("enable_crc_check").as_bool(),
        get_parameter("allow_virtual_serial").as_bool()))
    {
      throw std::runtime_error("串口驱动启动失败");
    }

    //创建pub_state与send_command定时器
    state_timer_ = create_wall_timer(
      periodFromRateHz(state_rate), [this]() {publishState();});
    command_timer_ = create_wall_timer(
      periodFromRateHz(command_rate), [this]() {sendCommand();});
    RCLCPP_INFO(
      get_logger(), "Hero serial driver ready; fire output is %s", enable_fire_ ? "enabled" : "disabled");
  }

  ~HeroGimbalDriverNode() override
  {
    serial_port_->stop();
  }

private:
  //State
  struct CachedState
  {
    LegacyReadFrame frame;
    rclcpp::Time stamp;
    int8_t exposure_step{0};
  };

  //Command
  struct CachedCommand
  {
    hero_msgs::msg::ControlCommand message;
  };

  //串口解析回调
  void receiveState(const LegacyReadFrame & frame)
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

  //定时发布state
  void publishState()
  {
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
    hero_msgs::msg::GimbalState message;
    message.header.stamp = state->stamp;
    message.header.frame_id = gimbal_frame_id_;
    message.yaw = static_cast<float>(state->frame.yaw_deg * kDegreesToRadians);
    message.pitch = static_cast<float>(state->frame.pitch_deg * kDegreesToRadians);
    message.mode = toMode(state->frame.mode_flag);
    message.raw_mode_flag = state->frame.mode_flag;
    message.enemy_color = state->frame.enemy_color;
    message.right_clicked = state->frame.right_clicked;
    message.up = state->frame.up;
    message.down = state->frame.down;
    message.exposure_step = state->exposure_step;
    state_pub_->publish(message);
  }

  //sub订阅回调
  void receiveCommand(const hero_msgs::msg::ControlCommand & message)
  {
    if (!std::isfinite(message.yaw) || !std::isfinite(message.pitch)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ignoring non-finite gimbal command");
      return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    latest_command_ = CachedCommand{message};
  }

  //定时发布command
  void sendCommand()
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

  std::unique_ptr<SerialPort> serial_port_;
  rclcpp::Publisher<hero_msgs::msg::GimbalState>::SharedPtr state_pub_;
  rclcpp::Subscription<hero_msgs::msg::ControlCommand>::SharedPtr control_sub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;

  std::mutex state_mutex_; //latest_state_锁
  std::mutex command_mutex_; //latest_command_锁
  std::optional<CachedState> latest_state_;
  std::optional<CachedCommand> latest_command_;
  
  std::string gimbal_frame_id_;
  uint8_t command_flag_{0x05U};
  bool enable_fire_{false};
  bool previous_up_{false};
  bool previous_down_{false};
};

}  // hero_gimbal_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<hero_gimbal_driver::HeroGimbalDriverNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("hero_gimbal_driver_node"), "节点初始化失败：%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
