#include <cmath>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include "hero_tf/transform_utils.hpp"

namespace hero_tf
{

namespace
{

TEST(TransformUtils, LegacyCameraRotationUsesInverseDirection)
{
  // 旧标定 yaw=-90 度时，camera_link 的前向轴应在 gimbal_link 中指向 +Y。
  const tf2::Matrix3x3 rotation(makeCamera2Gimbal(-90.0, 0.0, 0.0));
  const tf2::Vector3 transformed = rotation * tf2::Vector3(1.0, 0.0, 0.0);
  EXPECT_NEAR(transformed.x(), 0.0, 1e-9);
  EXPECT_NEAR(transformed.y(), 1.0, 1e-9);
  EXPECT_NEAR(transformed.z(), 0.0, 1e-9);
}

TEST(TransformUtils, OpticalForwardAxisMatchesCameraLinkForwardAxis)
{
  const tf2::Matrix3x3 rotation(makeFLU2RDF());
  const tf2::Vector3 transformed = rotation * tf2::Vector3(0.0, 0.0, 1.0);
  EXPECT_NEAR(transformed.x(), 1.0, 1e-9);
  EXPECT_NEAR(transformed.y(), 0.0, 1e-9);
  EXPECT_NEAR(transformed.z(), 0.0, 1e-9);
}

}  // namespace

}  // hero_tf
