#include <fuel_command_safety/command_safety.h>

#include <algorithm>
#include <cmath>

namespace fuel_command_safety {
namespace {

bool finite(double value) { return std::isfinite(value); }

bool finiteVector(const geometry_msgs::Vector3& vector) {
  return finite(vector.x) && finite(vector.y) && finite(vector.z);
}

void limitVector(geometry_msgs::Vector3* vector, double maximum, bool* limited) {
  const double norm = vectorNorm(*vector);
  if (norm <= maximum || norm <= 1e-12) return;

  const double scale = maximum / norm;
  vector->x *= scale;
  vector->y *= scale;
  vector->z *= scale;
  *limited = true;
}

}  // namespace

double vectorNorm(const geometry_msgs::Vector3& vector) {
  return std::sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

double positionDistance(const geometry_msgs::Point& first, const geometry_msgs::Point& second) {
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double normalizeAngle(double angle) {
  constexpr double kTwoPi = 6.28318530717958647692;
  return std::remainder(angle, kTwoPi);
}

bool isFiniteCommand(const quadrotor_msgs::PositionCommand& command) {
  if (!finite(command.position.x) || !finite(command.position.y) ||
      !finite(command.position.z) || !finiteVector(command.velocity) ||
      !finiteVector(command.acceleration) || !finiteVector(command.jerk) ||
      !finite(command.yaw) || !finite(command.yaw_dot)) {
    return false;
  }

  for (size_t i = 0; i < command.kx.size(); ++i) {
    if (!finite(command.kx[i]) || !finite(command.kv[i])) return false;
  }
  return true;
}

bool isTimestampValid(const ros::Time& now, const ros::Time& stamp, double max_age,
                      double max_future_offset, std::string* reason) {
  if (stamp.isZero()) {
    if (reason != nullptr) *reason = "zero_stamp";
    return false;
  }

  const double age = (now - stamp).toSec();
  if (age > max_age) {
    if (reason != nullptr) *reason = "stale_stamp";
    return false;
  }
  if (age < -max_future_offset) {
    if (reason != nullptr) *reason = "future_stamp";
    return false;
  }
  return true;
}

bool isPositionWithinBounds(const quadrotor_msgs::PositionCommand& command,
                            const Limits& limits) {
  return command.position.x >= limits.min_x && command.position.x <= limits.max_x &&
         command.position.y >= limits.min_y && command.position.y <= limits.max_y &&
         command.position.z >= limits.min_z && command.position.z <= limits.max_z;
}

bool isPositionStepValid(const quadrotor_msgs::PositionCommand& previous,
                         const quadrotor_msgs::PositionCommand& current, double dt,
                         double max_dt, const Limits& limits, double* distance,
                         double* allowed_distance) {
  const double safe_dt = std::min(std::max(0.0, dt), std::max(0.0, max_dt));
  const double allowed = limits.max_position_step +
                         limits.position_step_velocity_scale * limits.max_velocity * safe_dt;
  const double actual = positionDistance(previous.position, current.position);
  if (distance != nullptr) *distance = actual;
  if (allowed_distance != nullptr) *allowed_distance = allowed;
  return actual <= allowed;
}

LimitResult limitCommand(const quadrotor_msgs::PositionCommand& command, const Limits& limits) {
  LimitResult result;
  result.command = command;

  limitVector(&result.command.velocity, limits.max_velocity, &result.limited);
  limitVector(&result.command.acceleration, limits.max_acceleration, &result.limited);
  limitVector(&result.command.jerk, limits.max_jerk, &result.limited);

  const double normalized_yaw = normalizeAngle(result.command.yaw);
  if (std::abs(normalized_yaw - result.command.yaw) > 1e-12) result.limited = true;
  result.command.yaw = normalized_yaw;

  const double limited_yaw_rate =
      std::max(-limits.max_yaw_rate, std::min(limits.max_yaw_rate, result.command.yaw_dot));
  if (std::abs(limited_yaw_rate - result.command.yaw_dot) > 1e-12) result.limited = true;
  result.command.yaw_dot = limited_yaw_rate;
  return result;
}

}  // namespace fuel_command_safety
