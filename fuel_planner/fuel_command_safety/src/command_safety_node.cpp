#include <fuel_command_safety/command_safety.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>

namespace fuel_command_safety {
namespace {

enum class Mode {
  WAITING_INPUT,
  FORWARDING,
  HOLD_TIMEOUT,
  HOLD_INVALID,
  HOLD_JUMP,
  HOLD_PLANNER_FINISHED,
  FAILSAFE_ODOM_LOST,
};

const char* modeName(Mode mode) {
  switch (mode) {
    case Mode::WAITING_INPUT:
      return "WAITING_INPUT";
    case Mode::FORWARDING:
      return "FORWARDING";
    case Mode::HOLD_TIMEOUT:
      return "HOLD_TIMEOUT";
    case Mode::HOLD_INVALID:
      return "HOLD_INVALID";
    case Mode::HOLD_JUMP:
      return "HOLD_JUMP";
    case Mode::HOLD_PLANNER_FINISHED:
      return "HOLD_PLANNER_FINISHED";
    case Mode::FAILSAFE_ODOM_LOST:
      return "FAILSAFE_ODOM_LOST";
  }
  return "UNKNOWN";
}

bool finite(double value) { return std::isfinite(value); }

std::string toString(uint64_t value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

std::string toString(double value) {
  std::ostringstream stream;
  stream.precision(6);
  stream << std::fixed << value;
  return stream.str();
}

}  // namespace

class CommandSafetyNode {
 public:
  CommandSafetyNode() : nh_(), pnh_("~") {
    loadParameters();

    command_sub_ = nh_.subscribe("raw_command", 100, &CommandSafetyNode::commandCallback, this,
                                 ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh_.subscribe("odom", 100, &CommandSafetyNode::odomCallback, this,
                              ros::TransportHints().tcpNoDelay());
    if (allow_external_disable_) {
      enable_sub_ = pnh_.subscribe("enable", 1, &CommandSafetyNode::enableCallback, this);
    }
    command_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("safe_command", 50);
    state_pub_ = pnh_.advertise<std_msgs::String>("state", 10, true);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);

    output_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                    &CommandSafetyNode::outputTimer, this);
    diagnostics_timer_ = nh_.createTimer(ros::Duration(0.2),
                                         &CommandSafetyNode::diagnosticsTimer, this);
    setMode(Mode::WAITING_INPUT, "startup");
    ROS_INFO("[command_safety] ready: %.1f Hz, input timeout %.3f s, odom timeout %.3f s",
             publish_rate_, input_timeout_, odom_timeout_);
  }

 private:
  void loadParameters() {
    pnh_.param("publish_rate", publish_rate_, 100.0);
    pnh_.param("input_timeout", input_timeout_, 0.20);
    pnh_.param("max_input_age", max_input_age_, 0.15);
    pnh_.param("max_future_offset", max_future_offset_, 0.05);
    pnh_.param("odom_timeout", odom_timeout_, 0.20);
    pnh_.param("fault_latch_duration", fault_latch_duration_, 0.50);
    pnh_.param("allow_external_disable", allow_external_disable_, false);
    pnh_.param("planner_finish_timeout", planner_finish_timeout_, 1.0);
    pnh_.param("settled_position_tolerance", settled_position_tolerance_, 0.02);
    pnh_.param("settled_velocity", settled_velocity_, 0.03);
    pnh_.param("settled_acceleration", settled_acceleration_, 0.05);

    pnh_.param("bounds/min_x", limits_.min_x, limits_.min_x);
    pnh_.param("bounds/min_y", limits_.min_y, limits_.min_y);
    pnh_.param("bounds/min_z", limits_.min_z, limits_.min_z);
    pnh_.param("bounds/max_x", limits_.max_x, limits_.max_x);
    pnh_.param("bounds/max_y", limits_.max_y, limits_.max_y);
    pnh_.param("bounds/max_z", limits_.max_z, limits_.max_z);
    pnh_.param("limits/max_velocity", limits_.max_velocity, limits_.max_velocity);
    pnh_.param("limits/max_acceleration", limits_.max_acceleration, limits_.max_acceleration);
    pnh_.param("limits/max_jerk", limits_.max_jerk, limits_.max_jerk);
    pnh_.param("limits/max_yaw_rate", limits_.max_yaw_rate, limits_.max_yaw_rate);
    pnh_.param("limits/max_position_step", limits_.max_position_step,
                limits_.max_position_step);
    pnh_.param("limits/position_step_velocity_scale", limits_.position_step_velocity_scale,
                limits_.position_step_velocity_scale);
    pnh_.param("limits/max_tracking_error", limits_.max_tracking_error,
                limits_.max_tracking_error);

    if (publish_rate_ <= 0.0 || input_timeout_ <= 0.0 || max_input_age_ <= 0.0 ||
        odom_timeout_ <= 0.0 || limits_.max_velocity <= 0.0 ||
        limits_.max_acceleration <= 0.0 || limits_.max_jerk <= 0.0 ||
        limits_.max_yaw_rate <= 0.0 || limits_.max_position_step <= 0.0 ||
        limits_.min_x >= limits_.max_x || limits_.min_y >= limits_.max_y ||
        limits_.min_z >= limits_.max_z) {
      ROS_FATAL("[command_safety] invalid safety parameters");
      throw std::runtime_error("invalid command safety parameters");
    }
  }

  bool odomFresh(const ros::Time& now) const {
    return have_odom_ && (now - last_odom_receive_).toSec() <= odom_timeout_;
  }

  void enableCallback(const std_msgs::BoolConstPtr& message) {
    enabled_ = message->data;
    if (!enabled_) {
      setMode(Mode::WAITING_INPUT, "externally_disabled");
      ROS_WARN("[command_safety] output disabled by the SITL-only enable topic");
    } else {
      hold_captured_ = false;
      ROS_INFO("[command_safety] output re-enabled");
    }
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& message) {
    const auto& position = message->pose.pose.position;
    const auto& orientation = message->pose.pose.orientation;
    const auto& velocity = message->twist.twist.linear;
    const double quaternion_norm =
        std::sqrt(orientation.x * orientation.x + orientation.y * orientation.y +
                  orientation.z * orientation.z + orientation.w * orientation.w);
    if (!finite(position.x) || !finite(position.y) || !finite(position.z) ||
        !finite(orientation.x) || !finite(orientation.y) || !finite(orientation.z) ||
        !finite(orientation.w) || !finite(velocity.x) || !finite(velocity.y) ||
        !finite(velocity.z) || quaternion_norm < 1e-6) {
      ROS_ERROR_THROTTLE(1.0, "[command_safety] invalid odometry ignored");
      return;
    }

    odom_ = *message;
    have_odom_ = true;
    last_odom_receive_ = ros::Time::now();
  }

  void rejectCommand(const std::string& reason, bool jump) {
    ++rejected_count_;
    last_fault_reason_ = reason;
    fault_is_jump_ = jump;
    fault_latched_until_ = ros::Time::now() + ros::Duration(fault_latch_duration_);
    captureHold(ros::Time::now());
    ROS_ERROR_THROTTLE(0.5, "[command_safety] rejected raw command: %s", reason.c_str());
  }

  void commandCallback(const quadrotor_msgs::PositionCommandConstPtr& message) {
    const ros::Time now = ros::Time::now();
    std::string timestamp_reason;
    if (!isTimestampValid(now, message->header.stamp, max_input_age_, max_future_offset_,
                          &timestamp_reason)) {
      rejectCommand(timestamp_reason, false);
      return;
    }
    if (!isFiniteCommand(*message)) {
      rejectCommand("non_finite", false);
      return;
    }

    if (message->trajectory_flag ==
        quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED) {
      if (active_) {
        planner_finished_ = true;
        completed_trajectory_id_ = message->trajectory_id;
        captureHold(now);
      }
      return;
    }
    if (message->trajectory_flag != quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY) {
      rejectCommand("invalid_trajectory_flag", false);
      return;
    }
    if (message->trajectory_id == 0) {
      rejectCommand("zero_trajectory_id", false);
      return;
    }
    if (have_previous_command_ && message->trajectory_id < last_trajectory_id_) {
      rejectCommand("trajectory_id_regression", false);
      return;
    }

    if (planner_finished_) {
      if (message->trajectory_id <= completed_trajectory_id_) return;
      planner_finished_ = false;
      hold_captured_ = false;
    }

    if (!isPositionWithinBounds(*message, limits_)) {
      rejectCommand("position_out_of_bounds", false);
      return;
    }
    if (!odomFresh(now)) {
      ++rejected_count_;
      last_fault_reason_ = "odom_unavailable";
      return;
    }

    if (have_previous_command_) {
      const double dt =
          std::max(0.0, (message->header.stamp - previous_command_.header.stamp).toSec());
      double distance = 0.0;
      double allowed_distance = 0.0;
      if (!isPositionStepValid(previous_command_, *message, dt, input_timeout_, limits_, &distance,
                               &allowed_distance)) {
        std::ostringstream reason;
        reason << "position_jump:" << distance << ">" << allowed_distance;
        rejectCommand(reason.str(), true);
        return;
      }
    } else if (positionDistance(message->position, odom_.pose.pose.position) >
               limits_.max_tracking_error) {
      rejectCommand("command_too_far_from_odometry", true);
      return;
    }

    LimitResult result = limitCommand(*message, limits_);
    latest_command_ = result.command;
    previous_command_ = *message;
    have_previous_command_ = true;
    have_valid_command_ = true;
    active_ = true;
    last_valid_receive_ = now;
    last_trajectory_id_ = message->trajectory_id;
    ++accepted_count_;
    if (result.limited) ++limited_count_;
    last_command_was_limited_ = result.limited;

    updatePlannerFinishedDetection(now, result.command);
  }

  void updatePlannerFinishedDetection(const ros::Time& now,
                                      const quadrotor_msgs::PositionCommand& command) {
    const bool settled = vectorNorm(command.velocity) <= settled_velocity_ &&
                         vectorNorm(command.acceleration) <= settled_acceleration_;
    const bool same_trajectory = settle_tracking_ && command.trajectory_id == settle_trajectory_id_;
    const bool same_position =
        same_trajectory &&
        positionDistance(command.position, settle_position_) <= settled_position_tolerance_;

    if (!settled || !same_position) {
      settle_tracking_ = settled;
      settle_start_ = now;
      settle_position_ = command.position;
      settle_trajectory_id_ = command.trajectory_id;
      return;
    }

    if ((now - settle_start_).toSec() >= planner_finish_timeout_) {
      planner_finished_ = true;
      completed_trajectory_id_ = command.trajectory_id;
      captureHold(now);
    }
  }

  void captureHold(const ros::Time& now) {
    if (hold_captured_ || !odomFresh(now)) return;

    hold_command_ = quadrotor_msgs::PositionCommand();
    hold_command_.header.frame_id = odom_.header.frame_id.empty() ? "world" : odom_.header.frame_id;
    hold_command_.trajectory_id = std::max<uint32_t>(1, last_trajectory_id_);
    hold_command_.trajectory_flag =
        quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    hold_command_.position.x =
        std::max(limits_.min_x, std::min(limits_.max_x, odom_.pose.pose.position.x));
    hold_command_.position.y =
        std::max(limits_.min_y, std::min(limits_.max_y, odom_.pose.pose.position.y));
    hold_command_.position.z =
        std::max(limits_.min_z, std::min(limits_.max_z, odom_.pose.pose.position.z));

    const auto& q = odom_.pose.pose.orientation;
    const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
    const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    hold_command_.yaw = std::atan2(sin_yaw, cos_yaw);
    hold_command_.yaw_dot = 0.0;
    hold_command_.kx = {{5.7, 5.7, 6.2}};
    hold_command_.kv = {{3.4, 3.4, 4.0}};
    hold_captured_ = true;
  }

  void publishHold(const ros::Time& now, Mode mode, const std::string& reason) {
    captureHold(now);
    if (!hold_captured_) {
      setMode(Mode::FAILSAFE_ODOM_LOST, "cannot_capture_hold_without_odom");
      return;
    }
    hold_command_.header.stamp = now;
    command_pub_.publish(hold_command_);
    setMode(mode, reason);
  }

  void outputTimer(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (!enabled_) {
      setMode(Mode::WAITING_INPUT, "externally_disabled");
      return;
    }
    if (!odomFresh(now)) {
      setMode(Mode::FAILSAFE_ODOM_LOST, "odom_timeout");
      return;
    }
    if (!active_) {
      setMode(Mode::WAITING_INPUT, "no_valid_command_yet");
      return;
    }
    if (planner_finished_) {
      publishHold(now, Mode::HOLD_PLANNER_FINISHED, "planner_finished");
      return;
    }
    if (now < fault_latched_until_) {
      publishHold(now, fault_is_jump_ ? Mode::HOLD_JUMP : Mode::HOLD_INVALID,
                  last_fault_reason_);
      return;
    }
    if (!have_valid_command_ || (now - last_valid_receive_).toSec() > input_timeout_) {
      publishHold(now, Mode::HOLD_TIMEOUT, "input_timeout");
      return;
    }

    quadrotor_msgs::PositionCommand output = latest_command_;
    output.header.stamp = now;
    command_pub_.publish(output);
    hold_captured_ = false;
    setMode(Mode::FORWARDING, last_command_was_limited_ ? "limited" : "ok");
  }

  void setMode(Mode mode, const std::string& reason) {
    if (mode == mode_ && reason == mode_reason_) return;
    mode_ = mode;
    mode_reason_ = reason;
    std_msgs::String state;
    state.data = std::string(modeName(mode_)) + ":" + mode_reason_;
    state_pub_.publish(state);

    if (mode_ == Mode::FORWARDING || mode_ == Mode::WAITING_INPUT) {
      ROS_INFO("[command_safety] %s (%s)", modeName(mode_), mode_reason_.c_str());
    } else {
      ROS_WARN("[command_safety] %s (%s)", modeName(mode_), mode_reason_.c_str());
    }
  }

  void addDiagnostic(diagnostic_msgs::DiagnosticStatus* status, const std::string& key,
                     const std::string& value) const {
    diagnostic_msgs::KeyValue item;
    item.key = key;
    item.value = value;
    status->values.push_back(item);
  }

  void diagnosticsTimer(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = now;
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "fuel_command_safety";
    status.hardware_id = "command_bridge";
    status.message = std::string(modeName(mode_)) + ":" + mode_reason_;
    status.level = mode_ == Mode::FORWARDING || mode_ == Mode::WAITING_INPUT
                       ? diagnostic_msgs::DiagnosticStatus::OK
                       : (mode_ == Mode::FAILSAFE_ODOM_LOST
                              ? diagnostic_msgs::DiagnosticStatus::ERROR
                              : diagnostic_msgs::DiagnosticStatus::WARN);

    addDiagnostic(&status, "mode", modeName(mode_));
    addDiagnostic(&status, "reason", mode_reason_);
    addDiagnostic(&status, "accepted", toString(accepted_count_));
    addDiagnostic(&status, "rejected", toString(rejected_count_));
    addDiagnostic(&status, "limited", toString(limited_count_));
    addDiagnostic(&status, "input_age",
                  have_valid_command_ ? toString((now - last_valid_receive_).toSec()) : "inf");
    addDiagnostic(&status, "odom_age",
                  have_odom_ ? toString((now - last_odom_receive_).toSec()) : "inf");
    addDiagnostic(&status, "trajectory_id",
                  toString(static_cast<uint64_t>(last_trajectory_id_)));
    array.status.push_back(status);
    diagnostics_pub_.publish(array);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber command_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber enable_sub_;
  ros::Publisher command_pub_;
  ros::Publisher state_pub_;
  ros::Publisher diagnostics_pub_;
  ros::Timer output_timer_;
  ros::Timer diagnostics_timer_;

  Limits limits_;
  double publish_rate_{100.0};
  double input_timeout_{0.20};
  double max_input_age_{0.15};
  double max_future_offset_{0.05};
  double odom_timeout_{0.20};
  double fault_latch_duration_{0.50};
  double planner_finish_timeout_{1.0};
  double settled_position_tolerance_{0.02};
  double settled_velocity_{0.03};
  double settled_acceleration_{0.05};

  nav_msgs::Odometry odom_;
  quadrotor_msgs::PositionCommand latest_command_;
  quadrotor_msgs::PositionCommand previous_command_;
  quadrotor_msgs::PositionCommand hold_command_;
  geometry_msgs::Point settle_position_;

  ros::Time last_odom_receive_;
  ros::Time last_valid_receive_;
  ros::Time fault_latched_until_;
  ros::Time settle_start_;

  bool have_odom_{false};
  bool have_valid_command_{false};
  bool have_previous_command_{false};
  bool active_{false};
  bool hold_captured_{false};
  bool fault_is_jump_{false};
  bool planner_finished_{false};
  bool settle_tracking_{false};
  bool last_command_was_limited_{false};
  bool allow_external_disable_{false};
  bool enabled_{true};

  uint32_t last_trajectory_id_{0};
  uint32_t completed_trajectory_id_{0};
  uint32_t settle_trajectory_id_{0};
  uint64_t accepted_count_{0};
  uint64_t rejected_count_{0};
  uint64_t limited_count_{0};

  Mode mode_{Mode::WAITING_INPUT};
  std::string mode_reason_;
  std::string last_fault_reason_;
};

}  // namespace fuel_command_safety

int main(int argc, char** argv) {
  ros::init(argc, argv, "command_safety");
  try {
    fuel_command_safety::CommandSafetyNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("[command_safety] startup failed: %s", exception.what());
    return 1;
  }
  return 0;
}
