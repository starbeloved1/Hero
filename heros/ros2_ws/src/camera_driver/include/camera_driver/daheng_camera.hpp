#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "DxImageProc.h"
#include "GxIAPI.h"

namespace camera_driver
{

struct DahengCameraConfig
{
  std::string serial_number;
  std::int64_t width{1280};
  std::int64_t height{1024};
  std::int64_t offset_x{0};
  std::int64_t offset_y{0};
  double exposure_time_us{4000.0};
  double gain{16.0};
  double acquisition_rate_hz{0.0};
};

struct DahengFrame
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint64_t device_timestamp{0};
  std::vector<std::uint8_t> bgr_data;
};

class DahengCamera
{
public:
  DahengCamera() = default;
  ~DahengCamera();
  DahengCamera(const DahengCamera &) = delete;
  DahengCamera & operator=(const DahengCamera &) = delete;

  bool open(const DahengCameraConfig & config, std::string & error);
  bool read(DahengFrame & frame, std::string & error);
  void close();

private:
  bool acquireLibrary(std::string & error);
  bool configure(const DahengCameraConfig & config, std::string & error);
  static bool check(GX_STATUS status, std::string & error);

  GX_DEV_HANDLE device_{nullptr};
  bool library_acquired_{false};
  bool stream_on_{false};
  std::int64_t color_filter_{BAYERBG};
};

}  // camera_driver
