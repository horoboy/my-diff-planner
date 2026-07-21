#!/usr/bin/env python3

import json
import math
import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import PositionCommand, TakeoffLand
from std_msgs.msg import String


class ScenarioRunner:
    def __init__(self):
        self.output_file = rospy.get_param("~output_file", "/tmp/command_safety_scenarios.json")
        self.skip_takeoff = rospy.get_param("~skip_takeoff", False)
        self.recovery_only = rospy.get_param("~recovery_only", False)
        self.max_recovery_start_error = rospy.get_param("~max_recovery_start_error", 0.75)
        self.state = ""
        self.fault_mode = ""
        self.mavros_state = None
        self.odom = None
        self.raw_messages = []
        self.safe_messages = []
        self.results = []

        rospy.Subscriber("/fuel_command_safety/state", String, self.state_callback, queue_size=50)
        rospy.Subscriber("/command_fault_injector/mode", String, self.mode_status_callback, queue_size=10)
        rospy.Subscriber("/sitl/mavros/state", State, self.mavros_callback, queue_size=20)
        rospy.Subscriber("/sitl/ground_truth/odom", Odometry, self.odom_callback, queue_size=100)
        rospy.Subscriber(
            "/fuel/position_cmd_raw", PositionCommand, self.raw_callback, queue_size=200
        )
        rospy.Subscriber(
            "/sitl/setpoints_cmd", PositionCommand, self.safe_callback, queue_size=200
        )

        self.mode_publisher = rospy.Publisher(
            "/sitl_test/fault_mode", String, queue_size=1, latch=True
        )
        self.takeoff_publisher = rospy.Publisher(
            "/sitl/takeoff_land", TakeoffLand, queue_size=1
        )
        self.goal_publisher = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1
        )
        self.trigger_probe_publisher = rospy.Publisher(
            "/waypoint_generator/waypoints", Path, queue_size=1
        )

    def state_callback(self, message):
        self.state = message.data

    def mode_status_callback(self, message):
        self.fault_mode = message.data

    def mavros_callback(self, message):
        self.mavros_state = message

    def odom_callback(self, message):
        self.odom = message

    def raw_callback(self, message):
        self.raw_messages.append((rospy.Time.now().to_sec(), message))
        if len(self.raw_messages) > 10000:
            del self.raw_messages[:1000]

    def safe_callback(self, message):
        self.safe_messages.append((rospy.Time.now().to_sec(), message))
        if len(self.safe_messages) > 10000:
            del self.safe_messages[:1000]

    def wait_for(self, description, predicate, timeout):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            rospy.sleep(0.05)
        raise RuntimeError("timeout waiting for " + description)

    def set_mode(self, mode):
        self.mode_publisher.publish(String(data=mode))
        self.wait_for("fault mode " + mode, lambda: self.fault_mode == mode, 3.0)

    @staticmethod
    def vector_norm(vector):
        return math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)

    @staticmethod
    def finite_command(command):
        values = [
            command.position.x,
            command.position.y,
            command.position.z,
            command.velocity.x,
            command.velocity.y,
            command.velocity.z,
            command.acceleration.x,
            command.acceleration.y,
            command.acceleration.z,
            command.jerk.x,
            command.jerk.y,
            command.jerk.z,
            command.yaw,
            command.yaw_dot,
        ] + list(command.kx) + list(command.kv)
        return all(math.isfinite(value) for value in values)

    def publish_takeoff(self):
        message = TakeoffLand(takeoff_land_cmd=TakeoffLand.TAKEOFF)
        for _ in range(3):
            self.takeoff_publisher.publish(message)
            rospy.sleep(0.1)

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"
        goal.pose.position.x = 1.0
        goal.pose.position.z = 1.0
        goal.pose.orientation.w = 1.0
        for _ in range(3):
            self.goal_publisher.publish(goal)
            rospy.sleep(0.1)

    def latest_odom_position(self):
        position = self.odom.pose.pose.position
        return [position.x, position.y, position.z]

    @staticmethod
    def distance(first, second):
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))

    def collect_safe(self, start_time, duration):
        rospy.sleep(duration)
        return [message for stamp, message in self.safe_messages if stamp >= start_time]

    def assert_output_limits(self, messages):
        if not messages:
            raise RuntimeError("no safe commands observed")
        if not all(self.finite_command(message) for message in messages):
            raise RuntimeError("non-finite value reached safe output")

        maxima = {
            "velocity": max(self.vector_norm(message.velocity) for message in messages),
            "acceleration": max(self.vector_norm(message.acceleration) for message in messages),
            "jerk": max(self.vector_norm(message.jerk) for message in messages),
            "yaw_rate": max(abs(message.yaw_dot) for message in messages),
        }
        if maxima["velocity"] > 0.800001:
            raise RuntimeError("velocity limit exceeded")
        if maxima["acceleration"] > 1.200001:
            raise RuntimeError("acceleration limit exceeded")
        if maxima["jerk"] > 2.500001:
            raise RuntimeError("jerk limit exceeded")
        if maxima["yaw_rate"] > 0.600001:
            raise RuntimeError("yaw-rate limit exceeded")
        return maxima

    def run_normal(self):
        self.set_mode("pass")
        self.wait_for(
            "FUEL trigger chain",
            lambda: self.goal_publisher.get_num_connections() > 0
            and self.trigger_probe_publisher.get_num_connections() > 0,
            30.0,
        )
        self.publish_goal()
        self.wait_for("raw FUEL command", lambda: len(self.raw_messages) >= 10, 20.0)
        self.wait_for("bridge forwarding", lambda: self.state.startswith("FORWARDING"), 10.0)
        start = rospy.Time.now().to_sec()
        start_position = self.latest_odom_position()
        messages = self.collect_safe(start, 6.0)
        maxima = self.assert_output_limits(messages)
        trajectory_ids = sorted(set(message.trajectory_id for message in messages))
        end_position = self.latest_odom_position()
        if self.distance(start_position, end_position) < 0.20:
            raise RuntimeError("vehicle did not follow the normal FUEL trajectory")
        self.results.append(
            {
                "scenario": "normal_fuel",
                "passed": True,
                "safe_messages": len(messages),
                "trajectory_ids": trajectory_ids,
                "vehicle_displacement": self.distance(start_position, end_position),
                "maxima": maxima,
            }
        )

    def run_recovery(self):
        self.set_mode("pass")
        self.wait_for(
            "FUEL recovery trigger chain",
            lambda: self.goal_publisher.get_num_connections() > 0
            and self.trigger_probe_publisher.get_num_connections() > 0,
            30.0,
        )

        recovery_origin = self.latest_odom_position()
        raw_start_index = len(self.raw_messages)
        self.publish_goal()
        self.wait_for(
            "new raw FUEL recovery command",
            lambda: len(self.raw_messages) >= raw_start_index + 10,
            30.0,
        )

        first_raw = self.raw_messages[raw_start_index][1]
        first_raw_position = [
            first_raw.position.x,
            first_raw.position.y,
            first_raw.position.z,
        ]
        start_error = self.distance(recovery_origin, first_raw_position)
        if start_error > self.max_recovery_start_error:
            raise RuntimeError(
                "recovery trajectory did not start near the current hover position"
            )

        self.wait_for(
            "bridge recovery forwarding",
            lambda: self.state.startswith("FORWARDING"),
            15.0,
        )
        start = rospy.Time.now().to_sec()
        movement_start = self.latest_odom_position()
        messages = self.collect_safe(start, 5.0)
        maxima = self.assert_output_limits(messages)
        movement_end = self.latest_odom_position()
        displacement = self.distance(movement_start, movement_end)
        if displacement < 0.20:
            raise RuntimeError("vehicle did not follow the recovered FUEL trajectory")

        self.results.append(
            {
                "scenario": "recovery_from_current_hover",
                "passed": True,
                "safe_messages": len(messages),
                "trajectory_ids": sorted(
                    set(message.trajectory_id for message in messages)
                ),
                "recovery_origin": recovery_origin,
                "first_raw_position": first_raw_position,
                "first_raw_start_error": start_error,
                "vehicle_displacement": displacement,
                "maxima": maxima,
            }
        )

    def run_fault(self, name, mode, expected_state, recover=True):
        start_position = self.latest_odom_position()
        self.set_mode(mode)
        self.wait_for(expected_state, lambda: self.state.startswith(expected_state), 5.0)
        start = rospy.Time.now().to_sec()
        messages = self.collect_safe(start, 1.2)
        maxima = self.assert_output_limits(messages)
        if max(self.vector_norm(message.velocity) for message in messages) > 1e-9:
            raise RuntimeError(name + " did not publish zero-velocity hold")
        if max(self.vector_norm(message.acceleration) for message in messages) > 1e-9:
            raise RuntimeError(name + " did not publish zero-acceleration hold")
        hold_positions = [
            [message.position.x, message.position.y, message.position.z] for message in messages
        ]
        command_spread = max(
            self.distance(hold_positions[0], position) for position in hold_positions
        )
        vehicle_drift = self.distance(start_position, self.latest_odom_position())
        if command_spread > 1e-6:
            raise RuntimeError(name + " hold setpoint changed")

        self.results.append(
            {
                "scenario": name,
                "passed": True,
                "bridge_state": self.state,
                "safe_messages": len(messages),
                "hold_command_spread": command_spread,
                "vehicle_drift": vehicle_drift,
                "maxima": maxima,
            }
        )

        if recover:
            self.set_mode("pass")
            self.wait_for(name + " recovery", lambda: self.state.startswith("FORWARDING"), 8.0)
            rospy.sleep(1.0)

    def run(self):
        self.wait_for("MAVROS connection", lambda: self.mavros_state and self.mavros_state.connected, 40.0)
        self.wait_for("ground-truth odometry", lambda: self.odom is not None, 20.0)
        if self.skip_takeoff:
            self.wait_for(
                "existing PX4 hover",
                lambda: self.mavros_state
                and self.mavros_state.armed
                and self.mavros_state.mode == "OFFBOARD"
                and self.odom.pose.pose.position.z > 0.60,
                10.0,
            )
        else:
            initial_altitude = self.odom.pose.pose.position.z
            self.publish_takeoff()
            self.wait_for(
                "PX4 takeoff",
                lambda: self.mavros_state
                and self.mavros_state.armed
                and self.mavros_state.mode == "OFFBOARD"
                and self.odom.pose.pose.position.z > initial_altitude + 0.55,
                35.0,
            )
            rospy.sleep(2.0)

        if self.recovery_only:
            self.run_recovery()
            return

        self.run_normal()
        self.run_fault("stale_timestamp", "stale", "HOLD_INVALID", recover=False)
        self.run_fault("input_stream_loss", "drop", "HOLD_TIMEOUT", recover=False)
        self.run_fault("non_finite", "nan", "HOLD_INVALID", recover=False)
        self.run_fault("position_jump", "jump", "HOLD_JUMP", recover=False)
        self.run_fault("planner_finished", "complete", "HOLD_PLANNER_FINISHED", recover=False)

    def write_report(self, passed, error=""):
        report = {
            "passed": passed,
            "error": error,
            "bridge_state": self.state,
            "fault_mode": self.fault_mode,
            "scenarios": self.results,
        }
        directory = os.path.dirname(self.output_file)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(self.output_file, "w", encoding="utf-8") as output:
            json.dump(report, output, indent=2, sort_keys=True)


def main():
    rospy.init_node("command_safety_scenario_runner")
    runner = ScenarioRunner()
    try:
        runner.run()
        runner.write_report(True)
        rospy.loginfo("all command-safety SITL scenarios passed")
        return 0
    except Exception as exception:  # pylint: disable=broad-except
        runner.write_report(False, str(exception))
        rospy.logerr("command-safety SITL scenario failed: %s", exception)
        return 1


if __name__ == "__main__":
    sys.exit(main())
