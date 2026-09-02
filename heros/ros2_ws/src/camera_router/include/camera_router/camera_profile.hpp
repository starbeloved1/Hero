#pragma once

#include <cstdint>

namespace camera_router
{

enum class CameraProfile : std::uint8_t
{
  kAim8mm,
  kBase,
};

CameraProfile selectProfile(std::uint8_t mode, bool base_camera_enabled);
const char * profileName(CameraProfile profile);

}  // camera_router
