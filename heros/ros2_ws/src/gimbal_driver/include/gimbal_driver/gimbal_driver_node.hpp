#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "gimbal_driver/serial_port.hpp"
#include "gimbal_driver/virtual_gimbal_state.hpp"
#include "hero_msgs/msg/anti_base_packet.hpp"
#include "hero_msgs/msg/anti_base_tx_status.hpp"
#include "hero_msgs/msg/control_command.hpp"
#include "hero_msgs/msg/gimbal_state.hpp"

namespace gimbal_driver {

class GimbalDriverNode : public rclcpp::Node {
public:
  GimbalDriverNode();
  ~GimbalDriverNode() override;

private:
  struct CachedState {
    LegacyReadFrame frame;
    rclcpp::Time stamp;
    int8_t exposure_step{0};
  };

  struct CachedCommand {
    hero_msgs::msg::ControlCommand message;
  };

  struct PendingAntiBasePacket {
    hero_msgs::msg::AntiBasePacket message;
    std::chrono::steady_clock::time_point enqueued_at;
  };

  void receiveState(const LegacyReadFrame &frame);
  void publishState();
  void publishState(const LegacyReadFrame &frame, const rclcpp::Time &stamp,
                    int8_t exposure_step);
  void receiveCommand(const hero_msgs::msg::ControlCommand &message);
  void sendCommand();
  void receiveAntiBasePacket(const hero_msgs::msg::AntiBasePacket &message);
  void sendAntiBaseLoop();
  void initializeAntiBaseUdpMirror();
  void closeAntiBaseUdpMirror();
  void mirrorAntiBasePacket(const hero_msgs::msg::AntiBasePacket &packet);
  void publishAntiBaseTxStatus();
  void updateMode(uint8_t mode);
  rcl_interfaces::msg::SetParametersResult
  handleParameters(const std::vector<rclcpp::Parameter> &parameters);

  std::unique_ptr<SerialPort> serial_port_;
  rclcpp::Publisher<hero_msgs::msg::GimbalState>::SharedPtr state_pub_;
  rclcpp::Subscription<hero_msgs::msg::ControlCommand>::SharedPtr control_sub_;
  rclcpp::Subscription<hero_msgs::msg::AntiBasePacket>::SharedPtr
      antibase_packet_sub_;
  rclcpp::Publisher<hero_msgs::msg::AntiBaseTxStatus>::SharedPtr
      antibase_tx_status_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr antibase_status_timer_;

  std::mutex state_mutex_;
  std::mutex command_mutex_;
  std::mutex virtual_state_mutex_;
  std::mutex antibase_mutex_;
  std::condition_variable antibase_cv_;
  std::optional<CachedState> latest_state_;
  std::optional<CachedCommand> latest_command_;
  std::deque<PendingAntiBasePacket> antibase_packets_;
  std::thread antibase_send_thread_;
  std::atomic<bool> antibase_running_{false};
  std::atomic<uint8_t> current_mode_{hero_msgs::msg::GimbalState::MODE_NORMAL};
  std::atomic<uint64_t> antibase_sent_packets_{0U};
  std::atomic<uint64_t> antibase_dropped_packets_{0U};
  uint64_t antibase_gap_violation_count_{0U};
  float antibase_last_packet_gap_ms_{0.0F};
  uint64_t antibase_last_status_sent_packets_{0U};
  std::chrono::steady_clock::time_point antibase_last_status_time_{};
  std::optional<rclcpp::Time> antibase_last_send_time_;
  rclcpp::Time antibase_last_send_stamp_{0, 0, RCL_ROS_TIME};
  std::chrono::nanoseconds antibase_min_packet_gap_{0};
  std::chrono::nanoseconds antibase_chunk_gap_{0};
  std::size_t antibase_queue_depth_{64U};
  bool antibase_udp_mirror_enabled_{false};
  std::string antibase_udp_mirror_host_;
  uint16_t antibase_udp_mirror_port_{0U};
  int antibase_udp_mirror_fd_{-1};
  uint64_t antibase_udp_mirror_send_failures_{0U};
  VirtualGimbalState virtual_state_;

  std::string gimbal_frame_id_;
  uint8_t command_flag_{0x05U};
  bool enable_fire_{false};
  double state_timestamp_offset_{0.0};
  bool virtual_serial_{false};
  bool previous_up_{false};
  bool previous_down_{false};
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

} // namespace gimbal_driver
