#include <gtest/gtest.h>

#include "gimbal_driver/antibase_protocol.hpp"
#include "gimbal_driver/serial_protocol.hpp"

TEST(AntiBaseProtocol, EncodesFiveLegacyChunks) {
  hero_msgs::msg::AntiBasePacket packet;
  packet.sequence_id = 0x0102030405060708ULL;
  for (std::size_t index = 0; index < packet.data.size(); ++index) {
    packet.data[index] = static_cast<uint8_t>(index);
  }

  const auto chunks = gimbal_driver::encodeAntiBasePacket(packet);
  for (std::size_t index = 0; index < chunks.size(); ++index) {
    EXPECT_EQ(chunks[index][0], static_cast<uint8_t>('#'));
    EXPECT_EQ(chunks[index][1], index);
    const auto crc = gimbal_driver::crc16(chunks[index].data(), 62U);
    EXPECT_EQ(chunks[index][62], static_cast<uint8_t>(crc & 0xFFU));
    EXPECT_EQ(chunks[index][63], static_cast<uint8_t>((crc >> 8U) & 0xFFU));
  }
  EXPECT_EQ(chunks[0][2], 0x08U);
  EXPECT_EQ(chunks[0][9], 0x01U);
  EXPECT_EQ(chunks[0][10], 0U);
  EXPECT_EQ(chunks[4][61], static_cast<uint8_t>(291U));
}
