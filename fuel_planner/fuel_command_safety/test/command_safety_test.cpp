#include <fuel_command_safety/command_safety.h>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace fuel_command_safety {
namespace {

quadrotor_msgs::PositionCommand validCommand() {
  quadrotor_msgs::PositionCommand command;
  command.header.stamp = ros::Time(100.0);
  command.position.x = 0.0;
  command.position.y = 0.0;
  command.position.z = 1.0;
  command.yaw = 0.0;
  command.kx = {{5.7, 5.7, 6.2}};
  command.kv = {{3.4, 3.4, 4.0}};
  command.trajectory_id = 1;
  command.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  return command;
}

TEST(CommandSafety, RejectsNonFiniteFields) {
  auto command = validCommand();
  EXPECT_TRUE(isFiniteCommand(command));
  command.acceleration.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(isFiniteCommand(command));
}

TEST(CommandSafety, ChecksTimestampAgeAndFutureOffset) {
  std::string reason;
  EXPECT_TRUE(isTimestampValid(ros::Time(100.10), ros::Time(100.0), 0.15, 0.05, &reason));
  EXPECT_FALSE(isTimestampValid(ros::Time(100.20), ros::Time(100.0), 0.15, 0.05, &reason));
  EXPECT_EQ("stale_stamp", reason);
  EXPECT_FALSE(isTimestampValid(ros::Time(100.0), ros::Time(100.10), 0.15, 0.05, &reason));
  EXPECT_EQ("future_stamp", reason);
  EXPECT_FALSE(isTimestampValid(ros::Time(100.0), ros::Time(), 0.15, 0.05, &reason));
  EXPECT_EQ("zero_stamp", reason);
}

TEST(CommandSafety, LimitsDerivativeNormsAndYaw) {
  Limits limits;
  limits.max_velocity = 1.0;
  limits.max_acceleration = 2.0;
  limits.max_jerk = 3.0;
  limits.max_yaw_rate = 0.5;
  auto command = validCommand();
  command.velocity.x = 3.0;
  command.velocity.y = 4.0;
  command.acceleration.z = -4.0;
  command.jerk.x = 6.0;
  command.yaw = 4.0 * 3.14159265358979323846;
  command.yaw_dot = -2.0;

  const auto result = limitCommand(command, limits);
  EXPECT_TRUE(result.limited);
  EXPECT_NEAR(1.0, vectorNorm(result.command.velocity), 1e-9);
  EXPECT_NEAR(2.0, vectorNorm(result.command.acceleration), 1e-9);
  EXPECT_NEAR(3.0, vectorNorm(result.command.jerk), 1e-9);
  EXPECT_NEAR(0.0, result.command.yaw, 1e-9);
  EXPECT_DOUBLE_EQ(-0.5, result.command.yaw_dot);
}

TEST(CommandSafety, RejectsPositionOutsideBounds) {
  Limits limits;
  auto command = validCommand();
  EXPECT_TRUE(isPositionWithinBounds(command, limits));
  command.position.z = limits.max_z + 0.01;
  EXPECT_FALSE(isPositionWithinBounds(command, limits));
}

TEST(CommandSafety, DetectsPositionJump) {
  Limits limits;
  limits.max_position_step = 0.1;
  limits.max_velocity = 1.0;
  limits.position_step_velocity_scale = 1.0;
  auto previous = validCommand();
  auto current = previous;
  current.position.x = 0.15;
  EXPECT_TRUE(isPositionStepValid(previous, current, 0.05, 0.2, limits, nullptr, nullptr));
  current.position.x = 0.16;
  EXPECT_FALSE(isPositionStepValid(previous, current, 0.05, 0.2, limits, nullptr, nullptr));
}

TEST(CommandSafety, CapsJumpAllowanceAfterInputGap) {
  Limits limits;
  limits.max_position_step = 0.25;
  limits.max_velocity = 0.8;
  limits.position_step_velocity_scale = 1.5;
  auto previous = validCommand();
  auto current = previous;
  current.position.x = 0.75;

  double allowed_distance = 0.0;
  EXPECT_FALSE(
      isPositionStepValid(previous, current, 1.2, 0.25, limits, nullptr, &allowed_distance));
  EXPECT_NEAR(0.55, allowed_distance, 1e-9);
}

}  // namespace
}  // namespace fuel_command_safety

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
