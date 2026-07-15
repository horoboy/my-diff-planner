#include <fuel_realflight_adapter/pose_math.h>

#include <gtest/gtest.h>

#include <cmath>

namespace fuel_realflight_adapter {
namespace {

TEST(PoseMath, InterpolatesTranslationAndRotation) {
  Pose3d first;
  Pose3d second;
  second.position = Eigen::Vector3d(2.0, 4.0, 6.0);
  second.orientation =
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) * Eigen::Quaterniond::Identity();

  const Pose3d midpoint = interpolate(first, second, 0.5);
  EXPECT_NEAR(midpoint.position.x(), 1.0, 1e-12);
  EXPECT_NEAR(midpoint.position.y(), 2.0, 1e-12);
  EXPECT_NEAR(midpoint.position.z(), 3.0, 1e-12);

  const Eigen::Vector3d rotated = midpoint.orientation * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(rotated.x(), std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(rotated.y(), std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(rotated.z(), 0.0, 1e-12);
}

TEST(PoseMath, ComposesBodyToSensorExtrinsic) {
  Pose3d world_body;
  world_body.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  world_body.orientation =
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) * Eigen::Quaterniond::Identity();

  Pose3d body_sensor;
  body_sensor.position = Eigen::Vector3d(1.0, 0.0, 0.0);
  body_sensor.orientation =
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()) * Eigen::Quaterniond::Identity();

  const Pose3d world_sensor = compose(world_body, body_sensor);
  EXPECT_NEAR(world_sensor.position.x(), 1.0, 1e-12);
  EXPECT_NEAR(world_sensor.position.y(), 3.0, 1e-12);
  EXPECT_NEAR(world_sensor.position.z(), 3.0, 1e-12);

  const Eigen::Quaterniond expected =
      (world_body.orientation * body_sensor.orientation).normalized();
  EXPECT_NEAR(std::abs(world_sensor.orientation.dot(expected)), 1.0, 1e-12);
}

}  // namespace
}  // namespace fuel_realflight_adapter

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
