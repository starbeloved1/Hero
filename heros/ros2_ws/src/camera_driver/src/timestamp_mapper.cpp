#include "camera_driver/timestamp_mapper.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace camera_driver
{

TimestampMode parseTimestampMode(const std::string & mode_name)
{
  if (mode_name == "host") {
    return TimestampMode::kHost;
  }
  if (mode_name == "device") {
    return TimestampMode::kDevice;
  }
  throw std::invalid_argument("timestamp_mode 必须是 host 或 device，当前值为：" +
                              mode_name);
}

const char * timestampModeName(TimestampMode mode)
{
  switch (mode) {
  case TimestampMode::kHost:
    return "host";
  case TimestampMode::kDevice:
    return "device";
  }
  return "unknown";
}

bool hasRosSteadyClockJump(std::int64_t previous_offset_ns,
                           std::int64_t current_offset_ns,
                           std::int64_t threshold_ns)
{
  if (threshold_ns < 0) {
    throw std::invalid_argument("ROS 时钟跳变阈值不能为负数");
  }
  const long double difference = static_cast<long double>(current_offset_ns) -
                                 static_cast<long double>(previous_offset_ns);
  return std::abs(difference) > static_cast<long double>(threshold_ns);
}

DeviceTimestampMapper::DeviceTimestampMapper(double offset_alpha)
    : offset_alpha_(offset_alpha)
{
  if (!std::isfinite(offset_alpha_) || offset_alpha_ <= 0.0 ||
      offset_alpha_ > 1.0) {
    throw std::invalid_argument("设备时间戳低通系数必须在 (0, 1] 内");
  }
}

TimestampMapping DeviceTimestampMapper::update(
    std::uint64_t device_ticks, std::uint64_t tick_frequency_hz,
    std::int64_t host_steady_ns)
{
  if (tick_frequency_hz == 0U) {
    throw std::invalid_argument("设备时间戳频率必须大于 0");
  }
  if (!initialized_) {
    initialized_ = true;
    first_device_ticks_ = device_ticks;
    previous_device_ticks_ = device_ticks;
    tick_frequency_hz_ = tick_frequency_hz;
    first_host_steady_ns_ = host_steady_ns;
    residual_offset_sec_ = 0.0;
    return {TimestampUpdate::kInitialized, host_steady_ns, 0.0};
  }
  if (tick_frequency_hz != tick_frequency_hz_ ||
      device_ticks <= previous_device_ticks_) {
    reset();
    return {TimestampUpdate::kReset, 0, 0.0};
  }

  previous_device_ticks_ = device_ticks;
  const long double device_elapsed_sec =
      static_cast<long double>(device_ticks - first_device_ticks_) /
      static_cast<long double>(tick_frequency_hz_);
  const long double expected_host_ns =
      static_cast<long double>(first_host_steady_ns_) + device_elapsed_sec * 1.0e9L;
  const double observed_residual_sec =
      (static_cast<long double>(host_steady_ns) - expected_host_ns) / 1.0e9L;
  residual_offset_sec_ = (1.0 - offset_alpha_) * residual_offset_sec_ +
                         offset_alpha_ * observed_residual_sec;
  const long double mapped_host_ns =
      expected_host_ns + static_cast<long double>(residual_offset_sec_) * 1.0e9L;
  if (mapped_host_ns < static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
      mapped_host_ns > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    reset();
    return {TimestampUpdate::kReset, 0, 0.0};
  }
  return {TimestampUpdate::kUpdated,
          static_cast<std::int64_t>(std::llround(mapped_host_ns)),
          residual_offset_sec_};
}

void DeviceTimestampMapper::reset()
{
  initialized_ = false;
  ++reset_count_;
}

bool DeviceTimestampMapper::initialized() const { return initialized_; }

std::uint32_t DeviceTimestampMapper::resetCount() const { return reset_count_; }

}  // namespace camera_driver
