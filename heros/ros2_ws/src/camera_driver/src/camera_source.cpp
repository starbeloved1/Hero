#include "camera_driver/camera_source.hpp"

#include <stdexcept>

namespace camera_driver
{

CameraSource parseCameraSource(const std::string & source_name)
{
  if (source_name == "daheng") {
    return CameraSource::kDaheng;
  }
  if (source_name == "video") {
    return CameraSource::kVideo;
  }
  throw std::invalid_argument("相机输入源必须是 daheng 或 video，当前值为：" + source_name);
}

const char * cameraSourceName(CameraSource source)
{
  switch (source) {
    case CameraSource::kDaheng:
      return "daheng";
    case CameraSource::kVideo:
      return "video";
    default:
      return "unknown";
  }
}

}  // camera_driver
