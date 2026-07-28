#!/usr/bin/env python3

import math
import threading

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from competition_msgs.msg import MissionStatus
from quadrotor_msgs.msg import PositionCommand


class CommandMux:
    DIFF = "DIFF"
    FUEL = "FUEL"
    HOLD = "HOLD"

    def __init__(self):
        self.lock = threading.RLock()
        self.odom = None
        self.state = MissionStatus.WAIT_START
        self.source = self.HOLD
        self.source_changed = rospy.Time.now()
        self.commands = {self.DIFF: None, self.FUEL: None}
        self.received_at = {self.DIFF: rospy.Time(0), self.FUEL: rospy.Time(0)}
        self.max_command_age = float(rospy.get_param("~max_command_age", 0.25))
        self.frame_id = rospy.get_param("/competition/frame_id", "world")

        odom_topic = rospy.get_param(
            "~odom_topic", "/drone_0_visual_slam/odom"
        )
        diff_topic = rospy.get_param(
            "~diff_command_topic", "/competition/diff_position_cmd"
        )
        fuel_topic = rospy.get_param(
            "~fuel_command_topic", "/competition/fuel_position_cmd"
        )
        output_topic = rospy.get_param(
            "~output_topic", "/competition/active_position_cmd"
        )

        self.output_pub = rospy.Publisher(
            output_topic, PositionCommand, queue_size=10
        )
        self.source_pub = rospy.Publisher(
            "/competition/command_source", String, queue_size=1, latch=True
        )
        self.odom_sub = rospy.Subscriber(
            odom_topic, Odometry, self._odom_callback, queue_size=1
        )
        self.state_sub = rospy.Subscriber(
            "/competition/state",
            MissionStatus,
            self._state_callback,
            queue_size=1,
        )
        self.diff_sub = rospy.Subscriber(
            diff_topic,
            PositionCommand,
            self._command_callback,
            callback_args=self.DIFF,
            queue_size=1,
        )
        self.fuel_sub = rospy.Subscriber(
            fuel_topic,
            PositionCommand,
            self._command_callback,
            callback_args=self.FUEL,
            queue_size=1,
        )
        self.timer = rospy.Timer(rospy.Duration(0.01), self._publish)
        self._set_source(self.HOLD)

    def _odom_callback(self, msg):
        with self.lock:
            self.odom = msg

    def _command_callback(self, msg, source):
        with self.lock:
            if not self._finite_command(msg):
                rospy.logerr_throttle(
                    1.0, "[competition] rejected non-finite %s command", source
                )
                return
            self.commands[source] = msg
            self.received_at[source] = rospy.Time.now()

    def _state_callback(self, msg):
        with self.lock:
            self.state = msg.state
            desired = self.FUEL if msg.state == MissionStatus.SEARCH else self.DIFF
            if msg.state in (
                MissionStatus.WAIT_START,
                MissionStatus.COMPLETE,
                MissionStatus.ABORT,
            ):
                desired = self.HOLD
            self._set_source(desired)

    def _set_source(self, source):
        if source == self.source:
            return
        self.source = source
        self.source_changed = rospy.Time.now()
        self.source_pub.publish(String(data=source))
        rospy.logwarn("[competition] command source -> %s", source)

    @staticmethod
    def _finite_command(msg):
        values = (
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.velocity.x,
            msg.velocity.y,
            msg.velocity.z,
            msg.acceleration.x,
            msg.acceleration.y,
            msg.acceleration.z,
            msg.jerk.x,
            msg.jerk.y,
            msg.jerk.z,
            msg.yaw,
            msg.yaw_dot,
        )
        return all(math.isfinite(value) for value in values)

    def _hold_command(self, now):
        if self.odom is None:
            return None
        position = self.odom.pose.pose.position
        orientation = self.odom.pose.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )

        msg = PositionCommand()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.position.x = position.x
        msg.position.y = position.y
        msg.position.z = position.z
        msg.yaw = math.atan2(sin_yaw, cos_yaw)
        msg.kx = [5.7, 5.7, 6.2]
        msg.kv = [3.4, 3.4, 4.0]
        msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        return msg

    def _publish(self, _event):
        with self.lock:
            now = rospy.Time.now()
            command = None
            if self.source in (self.DIFF, self.FUEL):
                received_at = self.received_at[self.source]
                fresh = (
                    received_at >= self.source_changed
                    and (now - received_at).to_sec() <= self.max_command_age
                )
                if fresh:
                    command = self.commands[self.source]
            if command is None:
                command = self._hold_command(now)
            if command is not None:
                self.output_pub.publish(command)


if __name__ == "__main__":
    rospy.init_node("competition_command_mux")
    CommandMux()
    rospy.spin()
