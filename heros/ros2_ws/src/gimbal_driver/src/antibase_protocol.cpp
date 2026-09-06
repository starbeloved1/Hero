#include "gimbal_driver/antibase_protocol.hpp"

#include <algorithm>
#include <cstring>

#include "gimbal_driver/serial_protocol.hpp"

namespace gimbal_driver {

AntiBaseChunkArray
encodeAntiBasePacket(const hero_msgs::msg::AntiBasePacket &packet) {
  std::array<uint8_t, kAntiBasePacketBytes> logical_packet{};
  static_assert(sizeof(packet.sequence_id) == 8U, "反基地序号必须是 8 字节");
  std::memcpy(logical_packet.data(), &packet.sequence_id,
              sizeof(packet.sequence_id));
  std::copy(packet.data.begin(), packet.data.end(), logical_packet.begin() + 8);

  AntiBaseChunkArray chunks{};
  for (std::size_t index = 0; index < kAntiBaseChunkCount; ++index) {
    auto &chunk = chunks[index];
    chunk[0] = static_cast<uint8_t>('#');
    chunk[1] = static_cast<uint8_t>(index);
    const auto offset = index * kAntiBaseChunkPayloadBytes;
    std::copy_n(logical_packet.begin() + static_cast<std::ptrdiff_t>(offset),
                kAntiBaseChunkPayloadBytes, chunk.begin() + 2);
    const auto crc = crc16(chunk.data(), 62U);
    chunk[62] = static_cast<uint8_t>(crc & 0xFFU);
    chunk[63] = static_cast<uint8_t>((crc >> 8U) & 0xFFU);
  }
  return chunks;
}

} // namespace gimbal_driver
