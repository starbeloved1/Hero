#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace gimbal_driver
{

constexpr std::size_t kReadFrameSize = 16U;
constexpr std::size_t kWriteFrameSize = 14U;
constexpr uint8_t kFrameStart = static_cast<uint8_t>('!');

// 这些结构保留旧串口单位：yaw/pitch 使用角度。
struct LegacyReadFrame
{
  uint8_t mode_flag{0U};
  float pitch_deg{0.0F};
  float yaw_deg{0.0F};
  bool up{false};
  bool down{false};
  uint8_t robot_color{0U};
  bool right_clicked{false};
};

struct LegacyWriteCommand
{
  float pitch_deg{0.0F};
  float yaw_deg{0.0F};
  uint8_t shoot_status{0U};
  uint8_t target_id{0U};
  uint8_t command_flag{0x05U};
};

uint16_t crc16(const uint8_t * data, std::size_t length);
bool verifyReadFrame(const std::array<uint8_t, kReadFrameSize> & bytes, bool verify_crc);
std::optional<LegacyReadFrame> decodeReadFrame(
  const std::array<uint8_t, kReadFrameSize> & bytes, bool verify_crc);
std::array<uint8_t, kWriteFrameSize> encodeWriteCommand(const LegacyWriteCommand & command);

}  // gimbal_driver
