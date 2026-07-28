#!/usr/bin/env python3

import copy

import rospy
from sensor_msgs.msg import PointCloud2


class CloudRestamper:
    def __init__(self):
        input_topic = rospy.get_param(
            "~input_topic", "/drone_0_pcl_render_node/cloud"
        )
        output_topic = rospy.get_param(
            "~output_topic", "/competition/fuel_cloud"
        )
        self.output_frame = rospy.get_param("~output_frame", "world")
        self.publisher = rospy.Publisher(
            output_topic, PointCloud2, queue_size=2
        )
        self.subscriber = rospy.Subscriber(
            input_topic, PointCloud2, self._callback, queue_size=1
        )

    def _callback(self, msg):
        output = copy.copy(msg)
        output.header.stamp = rospy.Time.now()
        output.header.frame_id = self.output_frame
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("competition_cloud_restamper")
    CloudRestamper()
    rospy.spin()
