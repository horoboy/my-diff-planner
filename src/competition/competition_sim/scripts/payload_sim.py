#!/usr/bin/env python3

import threading

import rospy
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker

from competition_msgs.msg import DropState


class PayloadSim:
    def __init__(self):
        self.frame_id = rospy.get_param("/competition/frame_id", "world")
        self.lock = threading.Lock()
        self.odom = None
        self.released = False
        self.state = DropState()
        self.state.header.frame_id = self.frame_id
        self.state.detail = "payload_attached"

        odom_topic = rospy.get_param(
            "~odom_topic", "/drone_0_visual_slam/odom"
        )
        self.state_pub = rospy.Publisher(
            "/competition/drop_state", DropState, queue_size=1, latch=True
        )
        self.marker_pub = rospy.Publisher(
            "/competition/payload_marker", Marker, queue_size=1, latch=True
        )
        self.odom_sub = rospy.Subscriber(
            odom_topic, Odometry, self._odom_callback, queue_size=1
        )
        self.release_service = rospy.Service(
            "/competition/release_payload", Trigger, self._release
        )
        self.timer = rospy.Timer(rospy.Duration(0.2), self._publish)

    def _odom_callback(self, msg):
        with self.lock:
            self.odom = msg

    def _release(self, _request):
        with self.lock:
            if self.released:
                return TriggerResponse(success=False, message="payload_already_released")
            if self.odom is None:
                return TriggerResponse(success=False, message="odometry_not_available")

            position = self.odom.pose.pose.position
            self.state.header.stamp = rospy.Time.now()
            self.state.released = True
            self.state.release_position.x = position.x
            self.state.release_position.y = position.y
            self.state.release_position.z = position.z
            self.state.detail = "payload_released"
            self.released = True
            rospy.logwarn(
                "[competition] Payload released at (%.3f, %.3f, %.3f)",
                position.x,
                position.y,
                position.z,
            )
        self._publish(None)
        return TriggerResponse(success=True, message="payload_released")

    def _publish(self, _event):
        with self.lock:
            odom = self.odom
            state = self.state
            released = self.released
        state.header.stamp = rospy.Time.now()
        self.state_pub.publish(state)

        if odom is None:
            return
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = "competition_payload"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        if released:
            marker.pose.position = state.release_position
            marker.pose.position.z = 0.08
        else:
            marker.pose.position.x = odom.pose.pose.position.x
            marker.pose.position.y = odom.pose.pose.position.y
            marker.pose.position.z = max(0.08, odom.pose.pose.position.z - 0.15)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.16
        marker.scale.y = 0.16
        marker.scale.z = 0.16
        marker.color.r = 0.95
        marker.color.g = 0.75
        marker.color.b = 0.10
        marker.color.a = 1.0
        self.marker_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("payload_sim")
    PayloadSim()
    rospy.spin()
