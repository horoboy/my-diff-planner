#!/usr/bin/env python3

import copy
import math

import rospy
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String


class CommandFaultInjector:
    MODES = {"pass", "stale", "drop", "nan", "jump", "complete"}

    def __init__(self):
        self.mode = "pass"
        self.stale_seconds = rospy.get_param("~stale_seconds", 1.0)
        self.jump_distance = rospy.get_param("~jump_distance", 0.75)
        self.publisher = rospy.Publisher("output", PositionCommand, queue_size=100)
        self.status_publisher = rospy.Publisher("~mode", String, queue_size=1, latch=True)
        self.command_subscriber = rospy.Subscriber(
            "input", PositionCommand, self.command_callback, queue_size=100, tcp_nodelay=True
        )
        self.mode_subscriber = rospy.Subscriber(
            "fault_mode", String, self.mode_callback, queue_size=10
        )
        self.publish_mode()

    def publish_mode(self):
        self.status_publisher.publish(String(data=self.mode))

    def mode_callback(self, message):
        requested = message.data.strip().lower()
        if requested not in self.MODES:
            rospy.logerr("[fault_injector] unsupported mode: %s", requested)
            return
        if requested != self.mode:
            self.mode = requested
            rospy.logwarn("[fault_injector] mode -> %s", self.mode)
            self.publish_mode()

    def command_callback(self, message):
        if self.mode == "drop":
            return

        output = copy.deepcopy(message)
        if self.mode == "stale":
            output.header.stamp = rospy.Time.now() - rospy.Duration(self.stale_seconds)
        elif self.mode == "nan":
            output.acceleration.y = math.nan
        elif self.mode == "jump":
            output.position.x += self.jump_distance
        elif self.mode == "complete":
            output.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_COMPLETED
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("command_fault_injector")
    CommandFaultInjector()
    rospy.spin()
