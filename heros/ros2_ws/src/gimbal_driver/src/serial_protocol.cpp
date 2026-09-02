#include "gimbal_driver/serial_protocol.hpp"

#include <cmath>
#include <cstring>

namespace gimbal_driver
{

namespace
{

template<typename T>
T readScalar(const uint8_t * bytes)
{
  T value{};
  std::memcpy(&value, bytes, sizeof(T));
  return value;
}

template<typename T>
void writeScalar(uint8_t * bytes, T value)
{
  std::memcpy(bytes, &value, sizeof(T));
}

float normalizeYawDegrees(float yaw)
{
  while (yaw > 180.0F) {
    yaw -= 360.0F;
  }
  while (yaw < -180.0F) {
    yaw += 360.0F;
  }
  return yaw;
}

}  // namespace

uint16_t crc16(const uint8_t * data, std::size_t length)
{
  uint16_t crc = 0xFFFFU;
  for (std::size_t index = 0; index < length; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U ? static_cast<uint16_t>((crc >> 1U) ^ 0x8408U) :
        static_cast<uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

bool verifyReadFrame(const std::array<uint8_t, kReadFrameSize> & bytes, bool verify_crc)
{
  if (bytes[0] != kFrameStart) {
    return false;
  }
  if (!verify_crc) {
    return true;
  }
  const uint16_t expected = crc16(bytes.data(), kReadFrameSize - 2U);
  const uint16_t received = static_cast<uint16_t>(bytes[14]) |
    static_cast<uint16_t>(static_cast<uint16_t>(bytes[15]) << 8U);
  return expected == received;
}

std::optional<LegacyReadFrame> decodeReadFrame(
  const std::array<uint8_t, kReadFrameSize> & bytes, bool verify_crc)
{
  if (!verifyReadFrame(bytes, verify_crc)) {
    return std::nullopt;
  }

  LegacyReadFrame frame;
  frame.mode_flag = bytes[1];
  frame.pitch_deg = readScalar<float>(bytes.data() + 2U);
  frame.yaw_deg = normalizeYawDegrees(readScalar<float>(bytes.data() + 6U));
  frame.up = bytes[10] != 0U;
  frame.down = bytes[11] != 0U;
  frame.enemy_color = bytes[12];
  frame.right_clicked = bytes[13] != 0U;
  if (!std::isfinite(frame.pitch_deg) || !std::isfinite(frame.yaw_deg)) {
    return std::nullopt;
  }
  return frame;
}

std::array<uint8_t, kWriteFrameSize> encodeWriteCommand(const LegacyWriteCommand & command)
{
  std::array<uint8_t, kWriteFrameSize> bytes{};
  bytes[0] = kFrameStart;
  bytes[1] = command.command_flag;
  writeScalar<float>(bytes.data() + 2U, command.pitch_deg);
  writeScalar<float>(bytes.data() + 6U, command.yaw_deg);
  bytes[10] = command.shoot_status;
  bytes[11] = command.target_id;
  const uint16_t checksum = crc16(bytes.data(), kWriteFrameSize - 2U);
  bytes[12] = static_cast<uint8_t>(checksum & 0x00FFU);
  bytes[13] = static_cast<uint8_t>((checksum >> 8U) & 0x00FFU);
  return bytes;
}

}  // gimbal_driver
