#!/usr/bin/env python3

import sys
import time

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import TakeoffLand


MAV_STATE_STANDBY = 3


class TakeoffPreparation:
    def __init__(self):
        self.state = None
        self.odom = None
        self.publisher = rospy.Publisher("/sitl/takeoff_land", TakeoffLand, queue_size=1)
        rospy.Subscriber("/sitl/mavros/state", State, self.state_callback, queue_size=20)
        rospy.Subscriber("/sitl/ground_truth/odom", Odometry, self.odom_callback, queue_size=100)

    def state_callback(self, message):
        self.state = message

    def odom_callback(self, message):
        self.odom = message

    def wait_for(self, description, predicate, timeout):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            rospy.sleep(0.05)
        raise RuntimeError("timeout waiting for " + description)

    def run(self):
        self.wait_for("MAVROS connection", lambda: self.state and self.state.connected, 40.0)
        self.wait_for("ground-truth odometry", lambda: self.odom is not None, 20.0)
        self.wait_for(
            "PX4 preflight readiness",
            lambda: self.state
            and not self.state.armed
            and self.state.system_status == MAV_STATE_STANDBY
            and self.odom
            and abs(self.odom.twist.twist.linear.x) < 0.05
            and abs(self.odom.twist.twist.linear.y) < 0.05
            and abs(self.odom.twist.twist.linear.z) < 0.05,
            45.0,
        )
        rospy.sleep(1.0)
        initial_altitude = self.odom.pose.pose.position.z
        command = TakeoffLand(takeoff_land_cmd=TakeoffLand.TAKEOFF)
        for _ in range(3):
            self.publisher.publish(command)
            rospy.sleep(0.1)
        self.wait_for(
            "armed OFFBOARD takeoff",
            lambda: self.state
            and self.state.armed
            and self.state.mode == "OFFBOARD"
            and self.odom.pose.pose.position.z > initial_altitude + 0.55,
            35.0,
        )
        rospy.sleep(2.0)


def main():
    rospy.init_node("prepare_sitl_takeoff")
    try:
        TakeoffPreparation().run()
        rospy.loginfo("PX4 SITL is airborne and hovering")
        return 0
    except Exception as exception:  # pylint: disable=broad-except
        rospy.logerr("PX4 SITL takeoff preparation failed: %s", exception)
        return 1


if __name__ == "__main__":
    sys.exit(main())
