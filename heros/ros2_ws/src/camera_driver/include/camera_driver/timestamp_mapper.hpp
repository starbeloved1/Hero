#pragma once

#include <cstdint>
#include <string>

namespace camera_driver
{

enum class TimestampUpdate
{
  kInitialized,
  kUpdated,
  kReset,
};

enum class TimestampMode
{
  kHost,
  kDevice,
};

TimestampMode parseTimestampMode(const std::string & mode_name);
bool hasRosSteadyClockJump(std::int64_t previous_offset_ns,
                           std::int64_t current_offset_ns,
                           std::int64_t threshold_ns);

struct TimestampMapping
{
  TimestampUpdate update{TimestampUpdate::kReset};
  std::int64_t mapped_host_steady_ns{0};
  double residual_offset_sec{0.0};
};

// 将相机设备计数映射到主机单调时钟。设备时间只用于帧间间隔；首帧作为
// 相对原点，因此不要求设备与主机拥有共同纪元。
class DeviceTimestampMapper
{
public:
  explicit DeviceTimestampMapper(double offset_alpha = 0.02);

  TimestampMapping update(std::uint64_t device_ticks,
                          std::uint64_t tick_frequency_hz,
                          std::int64_t host_steady_ns);
  void reset();
  bool initialized() const;
  std::uint32_t resetCount() const;

private:
  double offset_alpha_{0.02};
  bool initialized_{false};
  std::uint64_t first_device_ticks_{0U};
  std::uint64_t previous_device_ticks_{0U};
  std::uint64_t tick_frequency_hz_{0U};
  std::int64_t first_host_steady_ns_{0};
  double residual_offset_sec_{0.0};
  std::uint32_t reset_count_{0U};
};

}  // namespace camera_driver
