#pragma once

#include <quadrotor_msgs/PositionCommand.h>
#include <ros/time.h>

#include <string>

namespace fuel_command_safety {

struct Limits {
  double min_x{-3.0};
  double min_y{-3.0};
  double min_z{0.3};
  double max_x{3.0};
  double max_y{3.0};
  double max_z{1.5};
  double max_velocity{0.35};
  double max_acceleration{0.6};
  double max_jerk{1.0};
  double max_yaw_rate{0.5};
  double max_position_step{0.20};
  double position_step_velocity_scale{1.5};
  double max_tracking_error{0.75};
};

struct LimitResult {
  quadrotor_msgs::PositionCommand command;
  bool limited{false};
};

bool isFiniteCommand(const quadrotor_msgs::PositionCommand& command);

bool isTimestampValid(const ros::Time& now, const ros::Time& stamp, double max_age,
                      double max_future_offset, std::string* reason);

bool isPositionWithinBounds(const quadrotor_msgs::PositionCommand& command,
                            const Limits& limits);

bool isPositionStepValid(const quadrotor_msgs::PositionCommand& previous,
                         const quadrotor_msgs::PositionCommand& current, double dt,
                         double max_dt, const Limits& limits, double* distance,
                         double* allowed_distance);

LimitResult limitCommand(const quadrotor_msgs::PositionCommand& command, const Limits& limits);

double positionDistance(const geometry_msgs::Point& first, const geometry_msgs::Point& second);
double vectorNorm(const geometry_msgs::Vector3& vector);
double normalizeAngle(double angle);

}  // namespace fuel_command_safety
