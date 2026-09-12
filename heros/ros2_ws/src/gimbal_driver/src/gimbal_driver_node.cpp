#include "gimbal_driver/gimbal_driver_node.hpp"

#include <arpa/inet.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <netinet/in.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>

#include "gimbal_driver/antibase_protocol.hpp"

namespace gimbal_driver {

namespace {

constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

rclcpp::QoS highRateQos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

rclcpp::QoS antiBaseQos(std::size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();
}

uint8_t toMode(uint8_t raw_mode_flag) {
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

std::chrono::nanoseconds periodFromRateHz(double rate_hz) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz));
}

} // namespace

GimbalDriverNode::GimbalDriverNode()
    : Node("gimbal_driver_node"), serial_port_(std::make_unique<SerialPort>()) {
  //节点参数
  declare_parameter<std::string>("port_name", "/dev/ttyACM0");
  declare_parameter<int>("baud_rate", 1000000);
  declare_parameter<bool>("enable_crc_check", true);
  declare_parameter<bool>("enable_fire", false);
  declare_parameter<int>("command_flag", 0x05);
  declare_parameter<int>("virtual_mode", 1);
  declare_parameter<int>("virtual_robot_color", 0);
  declare_parameter<std::string>("state_topic", "/hero/gimbal/state");
  declare_parameter<std::string>("control_topic", "/hero/gimbal/control");
  declare_parameter<std::string>("gimbal_frame_id", "gimbal_link");
  declare_parameter<double>("state_publish_rate_hz", 200.0);
  declare_parameter<double>("state_timestamp_offset", 0.0);
  declare_parameter<double>("command_send_rate_hz", 200.0);
  declare_parameter<std::string>("antibase_packet_topic",
                                 "/hero/antibase/packets");
  declare_parameter<std::string>("antibase_tx_status_topic",
                                 "/hero/antibase/tx_status");
  declare_parameter<double>("antibase_min_packet_gap_ms", 21.0);
  declare_parameter<double>("antibase_chunk_gap_ms", 2.0);
  declare_parameter<int>("antibase_queue_depth", 64);
  // 仅用于本地裁判模拟：镜像最终已经串口分片成功的完整 300B 逻辑包
  // 正式车端保持 false，UDP 不属于比赛串口链路
  declare_parameter<bool>("antibase_udp_mirror_enabled", false);
  declare_parameter<std::string>("antibase_udp_mirror_host", "127.0.0.1");
  declare_parameter<int>("antibase_udp_mirror_port", 9999);

  const auto state_rate = get_parameter("state_publish_rate_hz").as_double();
  const auto command_rate = get_parameter("command_send_rate_hz").as_double();
  if (state_rate <= 0.0 || command_rate <= 0.0) {
    throw std::invalid_argument(
        "state_publish_rate_hz and command_send_rate_hz must be positive");
  }
  state_timestamp_offset_ = get_parameter("state_timestamp_offset").as_double();
  if (!std::isfinite(state_timestamp_offset_)) {
    throw std::invalid_argument("state_timestamp_offset 必须是有限数");
  }
  const auto command_flag = get_parameter("command_flag").as_int();
  if (command_flag < 0 || command_flag > 255) {
    throw std::invalid_argument("command_flag must fit in uint8");
  }
  const auto antibase_min_packet_gap_ms =
      get_parameter("antibase_min_packet_gap_ms").as_double();
  const auto antibase_chunk_gap_ms =
      get_parameter("antibase_chunk_gap_ms").as_double();
  const auto antibase_queue_depth =
      get_parameter("antibase_queue_depth").as_int();
  if (antibase_min_packet_gap_ms <= 20.0 || antibase_chunk_gap_ms < 0.0 ||
      antibase_queue_depth <= 0) {
    throw std::invalid_argument(
        "反基地包间隔必须大于 20 ms，分片间隔不能为负，队列深度必须为正");
  }
  antibase_min_packet_gap_ =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double, std::milli>(
              antibase_min_packet_gap_ms));
  antibase_chunk_gap_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double, std::milli>(antibase_chunk_gap_ms));
  antibase_queue_depth_ = static_cast<std::size_t>(antibase_queue_depth);
  antibase_udp_mirror_enabled_ =
      get_parameter("antibase_udp_mirror_enabled").as_bool();
  antibase_udp_mirror_host_ =
      get_parameter("antibase_udp_mirror_host").as_string();
  const auto antibase_udp_mirror_port =
      get_parameter("antibase_udp_mirror_port").as_int();
  if (antibase_udp_mirror_enabled_ &&
      (antibase_udp_mirror_host_.empty() || antibase_udp_mirror_port <= 0 ||
       antibase_udp_mirror_port > 65535)) {
    throw std::invalid_argument("反基地 UDP 镜像主机或端口无效");
  }
  antibase_udp_mirror_port_ =
      static_cast<uint16_t>(std::max<int64_t>(0, antibase_udp_mirror_port));
  initializeAntiBaseUdpMirror();
  enable_fire_ = get_parameter("enable_fire").as_bool();
  command_flag_ = static_cast<uint8_t>(command_flag);
  gimbal_frame_id_ = get_parameter("gimbal_frame_id").as_string();
  if (!virtual_state_.setMode(get_parameter("virtual_mode").as_int())) {
    throw std::invalid_argument("virtual_mode 必须是 1 到 4");
  }
  if (!virtual_state_.setRobotColor(
          get_parameter("virtual_robot_color").as_int())) {
    throw std::invalid_argument("virtual_robot_color 必须是 0 或 1");
  }

  // pub&sub
  state_pub_ = create_publisher<hero_msgs::msg::GimbalState>(
      get_parameter("state_topic").as_string(), highRateQos());
  control_sub_ = create_subscription<hero_msgs::msg::ControlCommand>(
      get_parameter("control_topic").as_string(), highRateQos(),
      [this](const hero_msgs::msg::ControlCommand::SharedPtr message) {
        receiveCommand(*message);
      });
  antibase_packet_sub_ = create_subscription<hero_msgs::msg::AntiBasePacket>(
      get_parameter("antibase_packet_topic").as_string(),
      antiBaseQos(antibase_queue_depth_),
      [this](const hero_msgs::msg::AntiBasePacket::SharedPtr message) {
        receiveAntiBasePacket(*message);
      });
  antibase_tx_status_pub_ = create_publisher<hero_msgs::msg::AntiBaseTxStatus>(
      get_parameter("antibase_tx_status_topic").as_string(), antiBaseQos(1U));

  //注册串口回调
  serial_port_->setReadCallback(
      [this](const LegacyReadFrame &frame) { receiveState(frame); });
  serial_port_->setErrorCallback([this](const std::string &message) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s",
                          message.c_str());
  });

  if (!serial_port_->start(get_parameter("port_name").as_string(),
                           get_parameter("baud_rate").as_int(),
                           get_parameter("enable_crc_check").as_bool())) {
    throw std::runtime_error("串口驱动启动失败");
  }
  virtual_serial_ = serial_port_->isVirtual();
  parameter_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        return handleParameters(parameters);
      });

  //创建state与command定时发布器
  state_timer_ = create_wall_timer(periodFromRateHz(state_rate),
                                   [this]() { publishState(); });
  command_timer_ = create_wall_timer(periodFromRateHz(command_rate),
                                     [this]() { sendCommand(); });
  antibase_status_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() { publishAntiBaseTxStatus(); });
  antibase_running_.store(true);
  antibase_send_thread_ = std::thread([this]() { sendAntiBaseLoop(); });
  RCLCPP_INFO(get_logger(), "云台串口驱动已启动：%s，开火输出%s",
              virtual_serial_ ? "虚拟串口" : "真实串口",
              enable_fire_ ? "已启用" : "未启用");
}

GimbalDriverNode::~GimbalDriverNode() {
  antibase_running_.store(false);
  antibase_cv_.notify_all();
  if (antibase_send_thread_.joinable()) {
    antibase_send_thread_.join();
  }
  closeAntiBaseUdpMirror();
  serial_port_->stop();
}

void GimbalDriverNode::receiveState(const LegacyReadFrame &frame) {
  updateMode(toMode(frame.mode_flag));
  std::lock_guard<std::mutex> lock(state_mutex_);
  int8_t exposure_step = 0;
  if (frame.up && !previous_up_ && !frame.down) {
    exposure_step = 1;
  } else if (frame.down && !previous_down_ && !frame.up) {
    exposure_step = -1;
  }
  previous_up_ = frame.up;
  previous_down_ = frame.down;
  // 真实串口没有下位机采样时刻，入口处立即记录主机接收时间；后续定时重发
  // 缓存状态时必须保留该时刻，不能刷新为发布时刻。
  latest_state_ = CachedState{
      frame,
      now() + rclcpp::Duration::from_seconds(state_timestamp_offset_),
      exposure_step};
}

void GimbalDriverNode::publishState() {
  if (virtual_serial_) {
    LegacyReadFrame frame;
    {
      std::lock_guard<std::mutex> lock(virtual_state_mutex_);
      frame = virtual_state_.frame();
    }
    updateMode(toMode(frame.mode_flag));
    publishState(frame, now(), 0);
    return;
  }

  std::optional<CachedState> state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = latest_state_;
    if (latest_state_.has_value()) {
      // 旧接口对一次曝光边沿只消费一次，不能因为状态发布频率高于串口接收频率而重复发布
      latest_state_->exposure_step = 0;
    }
  }
  if (!state.has_value()) {
    return;
  }
  publishState(state->frame, state->stamp, state->exposure_step);
}

void GimbalDriverNode::publishState(const LegacyReadFrame &frame,
                                    const rclcpp::Time &stamp,
                                    int8_t exposure_step) {
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
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  if (!virtual_serial_) {
    for (const auto &parameter : parameters) {
      if (parameter.get_name() == "virtual_mode" ||
          parameter.get_name() == "virtual_robot_color") {
        result.successful = false;
        result.reason = "只有 port_name 为 virtual 时才能修改虚拟状态";
        return result;
      }
    }
    return result;
  }

  std::lock_guard<std::mutex> lock(virtual_state_mutex_);
  auto next_state = virtual_state_;
  for (const auto &parameter : parameters) {
    if (parameter.get_name() == "virtual_mode" &&
        !next_state.setMode(parameter.as_int())) {
      result.successful = false;
      result.reason = "virtual_mode 必须是 1 到 4";
      return result;
    }
    if (parameter.get_name() == "virtual_robot_color" &&
        !next_state.setRobotColor(parameter.as_int())) {
      result.successful = false;
      result.reason = "virtual_robot_color 必须是 0 或 1";
      return result;
    }
  }
  virtual_state_ = next_state;
  return result;
}

void GimbalDriverNode::receiveCommand(
    const hero_msgs::msg::ControlCommand &message) {
  if (!std::isfinite(message.yaw) || !std::isfinite(message.pitch)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "ignoring non-finite gimbal command");
    return;
  }
  std::lock_guard<std::mutex> lock(command_mutex_);
  latest_command_ = CachedCommand{message};
}

void GimbalDriverNode::sendCommand() {
  if (current_mode_.load() == hero_msgs::msg::GimbalState::MODE_ANTI_BASE) {
    return;
  }
  std::optional<CachedCommand> command;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command = latest_command_;
  }
  if (!command.has_value()) {
    return;
  }
  LegacyWriteCommand wire_command;
  wire_command.yaw_deg =
      static_cast<float>(command->message.yaw * kRadiansToDegrees);
  wire_command.pitch_deg =
      static_cast<float>(command->message.pitch * kRadiansToDegrees);
  wire_command.shoot_status = enable_fire_ ? command->message.shoot_status : 0U;
  wire_command.target_id = command->message.target_id;
  wire_command.command_flag = command_flag_;
  serial_port_->write(wire_command);
}

void GimbalDriverNode::receiveAntiBasePacket(
    const hero_msgs::msg::AntiBasePacket &message) {
  if (current_mode_.load() != hero_msgs::msg::GimbalState::MODE_ANTI_BASE) {
    antibase_dropped_packets_.fetch_add(1U);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(antibase_mutex_);
    if (antibase_packets_.size() >= antibase_queue_depth_) {
      antibase_packets_.pop_front();
      antibase_dropped_packets_.fetch_add(1U);
    }
    antibase_packets_.push_back(
        PendingAntiBasePacket{message, std::chrono::steady_clock::now()});
  }
  antibase_cv_.notify_one();
}

void GimbalDriverNode::sendAntiBaseLoop() {
  std::optional<std::chrono::steady_clock::time_point> last_packet_start;
  while (antibase_running_.load()) {
    hero_msgs::msg::AntiBasePacket packet;
    {
      std::unique_lock<std::mutex> lock(antibase_mutex_);
      antibase_cv_.wait(lock, [this]() {
        return !antibase_running_.load() || !antibase_packets_.empty();
      });
      if (!antibase_running_.load()) {
        return;
      }
      if (current_mode_.load() != hero_msgs::msg::GimbalState::MODE_ANTI_BASE) {
        antibase_packets_.clear();
        continue;
      }
      if (last_packet_start.has_value()) {
        const auto due = *last_packet_start + antibase_min_packet_gap_;
        if (std::chrono::steady_clock::now() < due) {
          antibase_cv_.wait_until(lock, due, [this]() {
            return !antibase_running_.load() ||
                   current_mode_.load() !=
                       hero_msgs::msg::GimbalState::MODE_ANTI_BASE;
          });
          continue;
        }
      }
      packet = std::move(antibase_packets_.front().message);
      antibase_packets_.pop_front();
    }

    if (current_mode_.load() != hero_msgs::msg::GimbalState::MODE_ANTI_BASE) {
      antibase_dropped_packets_.fetch_add(1U);
      continue;
    }
    const auto packet_start = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(antibase_mutex_);
      if (last_packet_start.has_value()) {
        const auto gap_ms = std::chrono::duration<float, std::milli>(
                                packet_start - *last_packet_start)
                                .count();
        antibase_last_packet_gap_ms_ = gap_ms;
        if (gap_ms <= 20.0F) {
          ++antibase_gap_violation_count_;
          RCLCPP_ERROR(get_logger(),
                       "反基地完整包间隔 %.3f ms，不满足严格大于 20 ms 的要求",
                       gap_ms);
        }
      }
      last_packet_start = packet_start;
      antibase_last_send_stamp_ = now();
    }
    const auto chunks = encodeAntiBasePacket(packet);
    bool sent = true;
    for (std::size_t index = 0; index < chunks.size(); ++index) {
      sent =
          serial_port_->writeRaw(chunks[index].data(), chunks[index].size()) &&
          sent;
      if (index + 1U < chunks.size()) {
        std::this_thread::sleep_for(antibase_chunk_gap_);
      }
    }
    if (sent) {
      // 镜像只在五个串口分片均已写成功后触发。它既不参与限速，也不影响
      // 正式串口路径；本地裁判模拟器据此收到一个完整 300B 逻辑包
      mirrorAntiBasePacket(packet);
      antibase_sent_packets_.fetch_add(1U);
    } else {
      antibase_dropped_packets_.fetch_add(1U);
    }
  }
}

void GimbalDriverNode::initializeAntiBaseUdpMirror() {
  if (!antibase_udp_mirror_enabled_) {
    return;
  }

  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  if (inet_pton(AF_INET, antibase_udp_mirror_host_.c_str(),
                &destination.sin_addr) != 1) {
    throw std::invalid_argument("反基地 UDP 镜像仅支持有效 IPv4 地址");
  }
  antibase_udp_mirror_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (antibase_udp_mirror_fd_ < 0) {
    throw std::runtime_error("无法创建反基地 UDP 镜像 socket");
  }
  RCLCPP_WARN(get_logger(),
              "反基地本地 UDP 镜像已启用：最终 300B 包将发送至 %s:%u；不要在实车启用",
              antibase_udp_mirror_host_.c_str(), antibase_udp_mirror_port_);
}

void GimbalDriverNode::closeAntiBaseUdpMirror() {
  if (antibase_udp_mirror_fd_ >= 0) {
    ::close(antibase_udp_mirror_fd_);
    antibase_udp_mirror_fd_ = -1;
  }
}

void GimbalDriverNode::mirrorAntiBasePacket(
    const hero_msgs::msg::AntiBasePacket &packet) {
  if (!antibase_udp_mirror_enabled_ || antibase_udp_mirror_fd_ < 0) {
    return;
  }

  std::array<uint8_t, kAntiBasePacketBytes> payload{};
  static_assert(sizeof(packet.sequence_id) == 8U,
                "反基地序号必须是 8 字节");
  std::memcpy(payload.data(), &packet.sequence_id, sizeof(packet.sequence_id));
  std::copy(packet.data.begin(), packet.data.end(), payload.begin() + 8);

  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(antibase_udp_mirror_port_);
  const auto parsed = inet_pton(AF_INET, antibase_udp_mirror_host_.c_str(),
                                &destination.sin_addr);
  if (parsed != 1) {
    ++antibase_udp_mirror_send_failures_;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "反基地 UDP 镜像地址失效：%s",
                          antibase_udp_mirror_host_.c_str());
    return;
  }
  const auto sent = ::sendto(
      antibase_udp_mirror_fd_, payload.data(), payload.size(), 0,
      reinterpret_cast<const sockaddr *>(&destination), sizeof(destination));
  if (sent != static_cast<ssize_t>(payload.size())) {
    ++antibase_udp_mirror_send_failures_;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "反基地 UDP 镜像发送失败：%zd/%zu B（累计失败 %lu）", sent,
        payload.size(), antibase_udp_mirror_send_failures_);
  }
}

void GimbalDriverNode::publishAntiBaseTxStatus() {
  hero_msgs::msg::AntiBaseTxStatus status;
  status.header.stamp = now();
  status.header.frame_id = gimbal_frame_id_;
  status.sent_packets = antibase_sent_packets_.load();
  status.dropped_packets = antibase_dropped_packets_.load();
  {
    std::lock_guard<std::mutex> lock(antibase_mutex_);
    const auto status_now = std::chrono::steady_clock::now();
    if (antibase_last_status_time_.time_since_epoch().count() != 0) {
      const auto elapsed_s =
          std::chrono::duration<float>(status_now - antibase_last_status_time_)
              .count();
      const auto sent_delta = status.sent_packets >= antibase_last_status_sent_packets_
                                  ? status.sent_packets - antibase_last_status_sent_packets_
                                  : 0U;
      status.tx_rate_hz = elapsed_s > 0.0F
                              ? static_cast<float>(sent_delta) / elapsed_s
                              : 0.0F;
    }
    antibase_last_status_time_ = status_now;
    antibase_last_status_sent_packets_ = status.sent_packets;
    status.last_send_stamp = antibase_last_send_stamp_;
    status.last_packet_gap_ms = antibase_last_packet_gap_ms_;
    status.packet_gap_violation_count = antibase_gap_violation_count_;
    status.pending_packets = static_cast<uint32_t>(antibase_packets_.size());
    if (!antibase_packets_.empty()) {
      status.oldest_pending_ms = static_cast<float>(
          std::max(0.0, std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() -
                            antibase_packets_.front().enqueued_at)
                            .count()));
    }
  }
  antibase_tx_status_pub_->publish(status);
}

void GimbalDriverNode::updateMode(uint8_t mode) {
  const auto previous_mode = current_mode_.exchange(mode);
  if (previous_mode == mode) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(antibase_mutex_);
    antibase_dropped_packets_.fetch_add(antibase_packets_.size());
    antibase_packets_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    latest_command_.reset();
  }
  antibase_cv_.notify_all();
}

} // namespace gimbal_driver
