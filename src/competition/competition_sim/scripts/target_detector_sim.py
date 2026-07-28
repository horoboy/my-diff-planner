#!/usr/bin/env python3

import math
import random
import threading

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from competition_msgs.msg import Forest, TargetDetection


class TargetDetectorSim:
    def __init__(self):
        target = rospy.get_param(
            "/competition/target_position", [7.1, 0.8, 0.0]
        )
        self.target = tuple(float(value) for value in target)
        self.target_type = rospy.get_param("/competition/target_type", "red")
        self.frame_id = rospy.get_param("/competition/frame_id", "world")
        self.detection_range = float(
            rospy.get_param("/competition/detection_range", 1.15)
        )
        self.noise_std = float(
            rospy.get_param("/competition/detection_noise_std", 0.025)
        )
        self.false_negative_probability = float(
            rospy.get_param(
                "/competition/detection_false_negative_probability", 0.03
            )
        )
        self.seed = int(rospy.get_param("/competition/map/seed", 17)) + 101
        self.rng = random.Random(self.seed)

        self.lock = threading.Lock()
        self.odom = None
        self.forest = None

        odom_topic = rospy.get_param(
            "~odom_topic", "/drone_0_visual_slam/odom"
        )
        self.detection_pub = rospy.Publisher(
            "/competition/target_detection",
            TargetDetection,
            queue_size=10,
        )
        self.marker_pub = rospy.Publisher(
            "/competition/target_marker", Marker, queue_size=1, latch=True
        )
        self.odom_sub = rospy.Subscriber(
            odom_topic, Odometry, self._odom_callback, queue_size=1
        )
        self.forest_sub = rospy.Subscriber(
            "/competition/forest", Forest, self._forest_callback, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(0.1), self._update)
        self._publish_marker()

    def _odom_callback(self, msg):
        with self.lock:
            self.odom = msg

    def _forest_callback(self, msg):
        with self.lock:
            self.forest = msg

    @staticmethod
    def _distance_to_segment(px, py, ax, ay, bx, by):
        dx = bx - ax
        dy = by - ay
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-9:
            return math.hypot(px - ax, py - ay)
        ratio = ((px - ax) * dx + (py - ay) * dy) / length_squared
        ratio = max(0.0, min(1.0, ratio))
        closest_x = ax + ratio * dx
        closest_y = ay + ratio * dy
        return math.hypot(px - closest_x, py - closest_y)

    def _has_line_of_sight(self, x, y, forest):
        if forest is None:
            return False
        tx, ty, _tz = self.target
        for center, radius in zip(forest.centers, forest.radii):
            if self._distance_to_segment(
                center.x, center.y, x, y, tx, ty
            ) < radius + 0.05:
                if math.hypot(center.x - x, center.y - y) > radius + 0.10:
                    return False
        return True

    def _update(self, _event):
        with self.lock:
            odom = self.odom
            forest = self.forest
        if odom is None:
            return

        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        distance = math.hypot(x - self.target[0], y - self.target[1])
        visible = (
            distance <= self.detection_range
            and self._has_line_of_sight(x, y, forest)
            and self.rng.random() >= self.false_negative_probability
        )

        msg = TargetDetection()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.detected = visible
        msg.target_type = self.target_type
        if visible:
            msg.pose.pose.position.x = self.target[0] + self.rng.gauss(
                0.0, self.noise_std
            )
            msg.pose.pose.position.y = self.target[1] + self.rng.gauss(
                0.0, self.noise_std
            )
            msg.pose.pose.position.z = self.target[2]
            msg.pose.pose.orientation.w = 1.0
            variance = self.noise_std * self.noise_std
            msg.pose.covariance[0] = variance
            msg.pose.covariance[7] = variance
            msg.pose.covariance[14] = variance
            msg.confidence = max(0.5, 0.98 - 0.15 * distance / self.detection_range)
        try:
            self.detection_pub.publish(msg)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise

    def _publish_marker(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = "competition_target"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.target[0]
        marker.pose.position.y = self.target[1]
        marker.pose.position.z = 0.025
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.55
        marker.scale.y = 0.55
        marker.scale.z = 0.05
        marker.color.r = 0.9
        marker.color.g = 0.05
        marker.color.b = 0.05
        marker.color.a = 1.0
        self.marker_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("target_detector_sim")
    TargetDetectorSim()
    rospy.spin()
