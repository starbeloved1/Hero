#include <gtest/gtest.h>

#include <stdexcept>

#include "camera_driver/camera_source.hpp"

TEST(CameraSource, ParsesSupportedSources)
{
  EXPECT_EQ(camera_driver::parseCameraSource("daheng"), camera_driver::CameraSource::kDaheng);
  EXPECT_EQ(camera_driver::parseCameraSource("video"), camera_driver::CameraSource::kVideo);
}

TEST(CameraSource, RejectsUnsupportedSource)
{
  EXPECT_THROW(camera_driver::parseCameraSource("usb"), std::invalid_argument);
}
