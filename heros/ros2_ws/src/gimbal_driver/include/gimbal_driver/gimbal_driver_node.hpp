#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "gimbal_driver/serial_port.hpp"
#include "gimbal_driver/virtual_gimbal_state.hpp"
#include "hero_msgs/msg/control_command.hpp"
#include "hero_msgs/msg/gimbal_state.hpp"

namespace gimbal_driver
{

class GimbalDriverNode : public rclcpp::Node
{
public:
  GimbalDriverNode();
  ~GimbalDriverNode() override;

private:
  struct CachedState
  {
    LegacyReadFrame frame;
    rclcpp::Time stamp;
    int8_t exposure_step{0};
  };

  struct CachedCommand
  {
    hero_msgs::msg::ControlCommand message;
  };

  void receiveState(const LegacyReadFrame & frame);
  void publishState();
  void publishState(
    const LegacyReadFrame & frame, const rclcpp::Time & stamp, int8_t exposure_step);
  void receiveCommand(const hero_msgs::msg::ControlCommand & message);
  void sendCommand();
  rcl_interfaces::msg::SetParametersResult handleParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  std::unique_ptr<SerialPort> serial_port_;
  rclcpp::Publisher<hero_msgs::msg::GimbalState>::SharedPtr state_pub_;
  rclcpp::Subscription<hero_msgs::msg::ControlCommand>::SharedPtr control_sub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;

  std::mutex state_mutex_;
  std::mutex command_mutex_;
  std::mutex virtual_state_mutex_;
  std::optional<CachedState> latest_state_;
  std::optional<CachedCommand> latest_command_;
  VirtualGimbalState virtual_state_;

  std::string gimbal_frame_id_;
  uint8_t command_flag_{0x05U};
  bool enable_fire_{false};
  bool virtual_serial_{false};
  bool previous_up_{false};
  bool previous_down_{false};
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // gimbal_driver
