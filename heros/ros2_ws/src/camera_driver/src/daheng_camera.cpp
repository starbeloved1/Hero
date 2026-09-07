#include "camera_driver/daheng_camera.hpp"

#include <mutex>
#include <sstream>

namespace camera_driver
{

namespace
{

std::mutex library_mutex;
std::size_t library_users{0};

}  // namespace

DahengCamera::~DahengCamera()
{
  close();
}

bool DahengCamera::check(GX_STATUS status, std::string & error)
{
  if (status == GX_STATUS_SUCCESS) {
    return true;
  }
  std::ostringstream stream;
  stream << "Galaxy SDK 错误码 " << status;
  error = stream.str();
  return false;
}

bool DahengCamera::acquireLibrary(std::string & error)
{
  std::lock_guard<std::mutex> lock(library_mutex);
  if (library_users == 0U && !check(GXInitLib(), error)) {
    return false;
  }
  ++library_users;
  library_acquired_ = true;
  return true;
}

bool DahengCamera::open(const DahengCameraConfig & config, std::string & error)
{
  if (config.serial_number.empty()) {
    error = "相机序列号不能为空";
    return false;
  }
  if (!acquireLibrary(error)) {
    return false;
  }

  std::uint32_t count = 0;
  if (!check(GXUpdateDeviceList(&count, 1000), error) || count == 0U) {
    if (count == 0U) {
      error = "未发现大恒相机";
    }
    close();
    return false;
  }
  GX_OPEN_PARAM parameter{};
  parameter.openMode = GX_OPEN_SN;
  parameter.pszContent = const_cast<char *>(config.serial_number.c_str());
  parameter.accessMode = GX_ACCESS_EXCLUSIVE;
  if (!check(GXOpenDevice(&parameter, &device_), error)) {
    close();
    return false;
  }
  bool color_supported = false;
  if (!check(GXIsImplemented(device_, GX_ENUM_PIXEL_COLOR_FILTER, &color_supported), error) ||
    !color_supported || !check(GXGetEnum(device_, GX_ENUM_PIXEL_COLOR_FILTER, &color_filter_), error) ||
    !configure(config, error))
  {
    if (error.empty()) {
      error = "相机不支持彩色 Bayer 输出";
    }
    close();
    return false;
  }
  return true;
}

bool DahengCamera::configure(const DahengCameraConfig & config, std::string & error)
{
  return check(GXSetInt(device_, GX_INT_WIDTH, 64), error) &&
         check(GXSetInt(device_, GX_INT_HEIGHT, 64), error) &&
         check(GXSetInt(device_, GX_INT_OFFSET_X, config.offset_x), error) &&
         check(GXSetInt(device_, GX_INT_OFFSET_Y, config.offset_y), error) &&
         check(GXSetInt(device_, GX_INT_WIDTH, config.width), error) &&
         check(GXSetInt(device_, GX_INT_HEIGHT, config.height), error) &&
         check(GXSetEnum(device_, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED), error) &&
         check(GXSetEnum(device_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF), error) &&
         check(GXSetFloat(device_, GX_FLOAT_EXPOSURE_TIME, config.exposure_time_us), error) &&
         check(GXSetEnum(device_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF), error) &&
         check(GXSetFloat(device_, GX_FLOAT_GAIN, config.gain), error) &&
         check(GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS), error) &&
         check(GXSetEnum(device_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS), error) &&
         check(GXSetEnum(device_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF), error) &&
         check(GXSetAcqusitionBufferNumber(device_, 2), error) &&
         check(GXStreamOn(device_), error) && (stream_on_ = true);
}

bool DahengCamera::read(DahengFrame & frame, std::string & error)
{
  PGX_FRAME_BUFFER buffer = nullptr;
  if (!check(GXDQBuf(device_, &buffer, 1500), error)) {
    return false;
  }
  if (buffer == nullptr || buffer->nStatus != GX_FRAME_STATUS_SUCCESS) {
    if (buffer != nullptr) {
      GXQBuf(device_, buffer);
    }
    error = "相机返回无效图像帧";
    return false;
  }
  frame.width = buffer->nWidth;
  frame.height = buffer->nHeight;
  frame.device_timestamp = buffer->nTimestamp;
  frame.host_receive_steady = std::chrono::steady_clock::now();
  frame.bgr_data.resize(static_cast<std::size_t>(frame.width) * frame.height * 3U);
  const auto convert_status = DxRaw8toRGB24Ex(
    static_cast<unsigned char *>(buffer->pImgBuf), frame.bgr_data.data(), frame.width, frame.height,
    RAW2RGB_NEIGHBOUR, static_cast<DX_PIXEL_COLOR_FILTER>(color_filter_), false, DX_ORDER_BGR);
  const auto queue_status = GXQBuf(device_, buffer);
  return check(convert_status, error) && check(queue_status, error);
}

bool DahengCamera::timestampTickFrequencyHz(std::uint64_t & frequency_hz,
                                            std::string & error) const
{
  if (device_ == nullptr) {
    error = "相机尚未打开，无法读取时间戳频率";
    return false;
  }
  bool implemented = false;
  if (!check(GXIsImplemented(device_, GX_INT_TIMESTAMP_TICK_FREQUENCY,
                             &implemented), error) ||
      !implemented) {
    if (error.empty()) {
      error = "相机不支持时间戳频率查询";
    }
    return false;
  }
  std::int64_t value = 0;
  if (!check(GXGetInt(device_, GX_INT_TIMESTAMP_TICK_FREQUENCY, &value), error) ||
      value <= 0) {
    if (error.empty()) {
      error = "相机返回无效时间戳频率";
    }
    return false;
  }
  frequency_hz = static_cast<std::uint64_t>(value);
  return true;
}

void DahengCamera::close()
{
  if (device_ != nullptr && stream_on_) {
    GXStreamOff(device_);
    stream_on_ = false;
  }
  if (device_ != nullptr) {
    GXCloseDevice(device_);
    device_ = nullptr;
  }
  if (library_acquired_) {
    std::lock_guard<std::mutex> lock(library_mutex);
    if (--library_users == 0U) {
      GXCloseLib();
    }
    library_acquired_ = false;
  }
}

}  // camera_driver
