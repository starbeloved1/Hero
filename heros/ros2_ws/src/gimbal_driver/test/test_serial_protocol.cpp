#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <gtest/gtest.h>

#include "gimbal_driver/serial_protocol.hpp"

namespace gimbal_driver
{

namespace
{

template<typename T>
void writeScalar(uint8_t * bytes, T value)
{
  std::memcpy(bytes, &value, sizeof(T));
}

TEST(SerialProtocol, DecodesLegacyReadFrameAndNormalizesYaw)
{
  std::array<uint8_t, kReadFrameSize> bytes{};
  bytes[0] = kFrameStart;
  bytes[1] = 0x03U;
  writeScalar<float>(bytes.data() + 2U, -8.5F);
  writeScalar<float>(bytes.data() + 6U, 190.0F);
  bytes[10] = 1U;
  bytes[12] = 2U;
  bytes[13] = 1U;
  const auto checksum = crc16(bytes.data(), kReadFrameSize - 2U);
  bytes[14] = static_cast<uint8_t>(checksum & 0xFFU);
  bytes[15] = static_cast<uint8_t>(checksum >> 8U);

  const auto decoded = decodeReadFrame(bytes, true);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->mode_flag, 0x03U);
  EXPECT_FLOAT_EQ(decoded->pitch_deg, -8.5F);
  EXPECT_FLOAT_EQ(decoded->yaw_deg, -170.0F);
  EXPECT_TRUE(decoded->up);
  EXPECT_FALSE(decoded->down);
  EXPECT_EQ(decoded->enemy_color, 2U);
  EXPECT_TRUE(decoded->right_clicked);
}

TEST(SerialProtocol, MatchesLegacyCrc16Variant)
{
  const std::array<uint8_t, 2U> bytes{kFrameStart, 0x05U};
  // 旧 SerialPort::getCRC16 以 0xffff 初始化，并使用反射形式的 0x8408 多项式。
  EXPECT_EQ(crc16(bytes.data(), bytes.size()), 0x9DFEU);
}

TEST(SerialProtocol, RejectsBadCrc)
{
  std::array<uint8_t, kReadFrameSize> bytes{};
  bytes[0] = kFrameStart;
  EXPECT_FALSE(decodeReadFrame(bytes, true).has_value());
}

TEST(SerialProtocol, EncodesLegacyWriteFrame)
{
  const auto bytes = encodeWriteCommand(LegacyWriteCommand{1.25F, -2.5F, 1U, 7U, 0x05U});
  EXPECT_EQ(bytes[0], kFrameStart);
  EXPECT_EQ(bytes[1], 0x05U);
  float pitch = 0.0F;
  float yaw = 0.0F;
  std::memcpy(&pitch, bytes.data() + 2U, sizeof(float));
  std::memcpy(&yaw, bytes.data() + 6U, sizeof(float));
  EXPECT_FLOAT_EQ(pitch, 1.25F);
  EXPECT_FLOAT_EQ(yaw, -2.5F);
  EXPECT_EQ(bytes[10], 1U);
  EXPECT_EQ(bytes[11], 7U);
  const auto checksum = crc16(bytes.data(), kWriteFrameSize - 2U);
  EXPECT_EQ(bytes[12], static_cast<uint8_t>(checksum & 0xFFU));
  EXPECT_EQ(bytes[13], static_cast<uint8_t>(checksum >> 8U));
}

}  // namespace

}  // gimbal_driver
