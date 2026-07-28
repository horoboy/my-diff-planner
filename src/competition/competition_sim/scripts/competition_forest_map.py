#!/usr/bin/env python3

import math
import random

import rospy
from geometry_msgs.msg import Point
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from competition_msgs.msg import Forest


class CompetitionForestMap:
    def __init__(self):
        self.frame_id = rospy.get_param("/competition/frame_id", "world")
        self.x_size = float(rospy.get_param("/competition/map/x_size", 28.0))
        self.y_size = float(rospy.get_param("/competition/map/y_size", 18.0))
        self.resolution = float(rospy.get_param("/competition/map/resolution", 0.1))
        self.tree_count = int(rospy.get_param("/competition/map/tree_count", 52))
        self.seed = int(rospy.get_param("/competition/map/seed", 17))
        self.radius_min = float(rospy.get_param("/competition/map/radius_min", 0.20))
        self.radius_max = float(rospy.get_param("/competition/map/radius_max", 0.34))
        self.min_center_distance = float(
            rospy.get_param("/competition/map/min_center_distance", 1.10)
        )
        self.height = float(rospy.get_param("/competition/map/trunk_height", 3.0))

        home = rospy.get_param("/competition/home", [-10.0, 0.0, 0.2])
        roi = rospy.get_param("/competition/roi_center", [5.0, 0.0])
        target = rospy.get_param("/competition/target_position", [7.1, 0.8, 0.0])
        target_clearance_position = rospy.get_param(
            "/competition/map/target_clearance_position",
            [target[0], target[1]],
        )
        self.exclusions = [
            (
                float(home[0]),
                float(home[1]),
                float(rospy.get_param("/competition/map/start_clearance", 1.8)),
            ),
            (float(roi[0]), float(roi[1]), 0.75),
            (
                float(target_clearance_position[0]),
                float(target_clearance_position[1]),
                float(rospy.get_param("/competition/map/target_clearance", 0.75)),
            ),
        ]

        self.trees = self._generate_trees()
        self.cloud_points = self._make_cloud_points()

        self.cloud_pub = rospy.Publisher(
            "/map_generator/global_cloud", PointCloud2, queue_size=1, latch=True
        )
        self.forest_pub = rospy.Publisher(
            "/competition/forest", Forest, queue_size=1, latch=True
        )
        self.marker_pub = rospy.Publisher(
            "/competition/forest_markers", MarkerArray, queue_size=1, latch=True
        )

        self.timer = rospy.Timer(rospy.Duration(1.0), self._publish)
        rospy.sleep(0.2)
        self._publish(None)
        rospy.loginfo(
            "[competition] Generated deterministic forest: %d trees, seed=%d",
            len(self.trees),
            self.seed,
        )

    def _generate_trees(self):
        rng = random.Random(self.seed)
        trees = []
        x_margin = max(0.5, self.radius_max + 0.2)
        y_margin = max(0.5, self.radius_max + 0.2)
        attempts = 0
        max_attempts = self.tree_count * 1000

        while len(trees) < self.tree_count and attempts < max_attempts:
            attempts += 1
            x = rng.uniform(-self.x_size / 2.0 + x_margin, self.x_size / 2.0 - x_margin)
            y = rng.uniform(-self.y_size / 2.0 + y_margin, self.y_size / 2.0 - y_margin)
            radius = rng.uniform(self.radius_min, self.radius_max)

            if any(
                math.hypot(x - ex, y - ey) < clear + radius
                for ex, ey, clear in self.exclusions
            ):
                continue

            if any(
                math.hypot(x - tx, y - ty)
                < self.min_center_distance + radius + tree_radius
                for tx, ty, tree_radius in trees
            ):
                continue

            trees.append((x, y, radius))

        if len(trees) != self.tree_count:
            rospy.logwarn(
                "[competition] Requested %d trees but generated %d after %d attempts",
                self.tree_count,
                len(trees),
                attempts,
            )
        return trees

    def _make_cloud_points(self):
        points = []
        z_step = max(self.resolution * 1.5, 0.12)
        for x, y, radius in self.trees:
            circumference_samples = max(
                12, int(math.ceil(2.0 * math.pi * radius / self.resolution))
            )
            z = 0.0
            while z <= self.height:
                for index in range(circumference_samples):
                    angle = 2.0 * math.pi * index / circumference_samples
                    points.append(
                        (
                            x + radius * math.cos(angle),
                            y + radius * math.sin(angle),
                            z,
                        )
                    )
                z += z_step
        return points

    def _publish(self, _event):
        now = rospy.Time.now()
        header = Header(stamp=now, frame_id=self.frame_id)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, self.cloud_points))

        forest = Forest()
        forest.header = header
        forest.height = self.height
        for x, y, radius in self.trees:
            center = Point(x=x, y=y, z=0.0)
            forest.centers.append(center)
            forest.radii.append(radius)
        self.forest_pub.publish(forest)

        markers = MarkerArray()
        for index, (x, y, radius) in enumerate(self.trees):
            marker = Marker()
            marker.header = header
            marker.ns = "competition_trees"
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = self.height / 2.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * radius
            marker.scale.y = 2.0 * radius
            marker.scale.z = self.height
            marker.color.r = 0.30
            marker.color.g = 0.20
            marker.color.b = 0.08
            marker.color.a = 0.85
            markers.markers.append(marker)
        self.marker_pub.publish(markers)


if __name__ == "__main__":
    rospy.init_node("competition_forest_map")
    CompetitionForestMap()
    rospy.spin()
