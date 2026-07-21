#!/usr/bin/env python3

import math

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


class GroundTruthOdom:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "iris")
        self.output_topic = rospy.get_param(
            "~output_topic", "/sitl/ground_truth/odom"
        )
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.publish_rate = float(rospy.get_param("~publish_rate", 200.0))
        self.pose = None
        self.twist = None

        self.publisher = rospy.Publisher(
            self.output_topic, Odometry, queue_size=20
        )
        self.subscriber = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.model_states_callback,
            queue_size=1,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.publish
        )

    def model_states_callback(self, message):
        try:
            index = message.name.index(self.model_name)
        except ValueError:
            rospy.logwarn_throttle(
                2.0, "Waiting for Gazebo model '%s'", self.model_name
            )
            return

        pose = message.pose[index]
        twist = message.twist[index]
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            rospy.logerr_throttle(1.0, "Gazebo ground-truth odometry is non-finite")
            return

        self.pose = pose
        self.twist = twist

    def publish(self, _event):
        if rospy.is_shutdown() or self.pose is None or self.twist is None:
            return

        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.frame_id
        message.child_frame_id = self.child_frame_id
        message.pose.pose = self.pose
        message.twist.twist = self.twist
        try:
            self.publisher.publish(message)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise


def main():
    rospy.init_node("gazebo_ground_truth_odom")
    GroundTruthOdom()
    rospy.spin()


if __name__ == "__main__":
    main()
