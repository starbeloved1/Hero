#pragma once

#include <string>

namespace camera_driver
{

enum class CameraSource
{
  kDaheng,
  kVideo,
};

CameraSource parseCameraSource(const std::string & source_name);
const char * cameraSourceName(CameraSource source);

}  // camera_driver
