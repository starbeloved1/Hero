#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "hero_msgs/msg/anti_base_packet.hpp"

namespace gimbal_driver {

constexpr std::size_t kAntiBasePacketBytes = 300U;
constexpr std::size_t kAntiBaseChunkCount = 5U;
constexpr std::size_t kAntiBaseChunkBytes = 64U;
constexpr std::size_t kAntiBaseChunkPayloadBytes = 60U;

using AntiBaseChunk = std::array<uint8_t, kAntiBaseChunkBytes>;
using AntiBaseChunkArray = std::array<AntiBaseChunk, kAntiBaseChunkCount>;

// 将 ROS 逻辑包编码为旧工程兼容的 5 个串口帧：'#'、分片序号、60B 数据和 CRC16
AntiBaseChunkArray
encodeAntiBasePacket(const hero_msgs::msg::AntiBasePacket &packet);

} // namespace gimbal_driver
