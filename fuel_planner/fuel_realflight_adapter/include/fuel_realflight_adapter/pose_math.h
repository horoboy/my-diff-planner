#ifndef FUEL_REALFLIGHT_ADAPTER_POSE_MATH_H_
#define FUEL_REALFLIGHT_ADAPTER_POSE_MATH_H_

#include <Eigen/Geometry>

#include <cmath>

namespace fuel_realflight_adapter {

struct Pose3d {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

inline bool isFinite(const Pose3d& pose) {
  return pose.position.allFinite() && pose.orientation.coeffs().allFinite() &&
         std::isfinite(pose.orientation.norm());
}

inline Pose3d interpolate(const Pose3d& first, const Pose3d& second, const double ratio) {
  Pose3d result;
  result.position = first.position + ratio * (second.position - first.position);

  Eigen::Quaterniond second_orientation = second.orientation;
  if (first.orientation.dot(second_orientation) < 0.0) {
    second_orientation.coeffs() *= -1.0;
  }
  result.orientation = first.orientation.slerp(ratio, second_orientation).normalized();
  return result;
}

inline Pose3d compose(const Pose3d& world_body, const Pose3d& body_sensor) {
  Pose3d world_sensor;
  world_sensor.position =
      world_body.position + world_body.orientation * body_sensor.position;
  world_sensor.orientation =
      (world_body.orientation * body_sensor.orientation).normalized();
  return world_sensor;
}

}  // namespace fuel_realflight_adapter

#endif  // FUEL_REALFLIGHT_ADAPTER_POSE_MATH_H_
