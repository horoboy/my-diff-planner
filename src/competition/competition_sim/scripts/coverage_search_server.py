#!/usr/bin/env python3

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from competition_msgs.msg import Forest
from competition_msgs.srv import (
    NextSearchViewpoint,
    NextSearchViewpointResponse,
)


class CoverageSearchServer:
    def __init__(self):
        roi = rospy.get_param("/competition/roi_center", [5.0, 0.0])
        self.cx = float(roi[0])
        self.cy = float(roi[1])
        self.radius = float(rospy.get_param("/competition/roi_radius", 3.0))
        self.altitude = float(rospy.get_param("/competition/search_height", 1.2))
        self.spacing = float(rospy.get_param("/competition/search_spacing", 0.8))
        self.tree_clearance = float(
            rospy.get_param("/competition/search_tree_clearance", 0.45)
        )
        self.frame_id = rospy.get_param("/competition/frame_id", "world")

        self.lock = threading.Lock()
        self.forest = None
        self.viewpoints = []
        self.index = 0
        self.path_pub = rospy.Publisher(
            "/competition/search_path", Path, queue_size=1, latch=True
        )
        self.forest_sub = rospy.Subscriber(
            "/competition/forest", Forest, self._forest_callback, queue_size=1
        )
        self.service = rospy.Service(
            "/competition/next_search_viewpoint",
            NextSearchViewpoint,
            self._handle_request,
        )

    def _forest_callback(self, msg):
        with self.lock:
            self.forest = msg
            if not self.viewpoints:
                self.viewpoints = self._generate_viewpoints(msg)
                self._publish_path()
                rospy.loginfo(
                    "[competition] Coverage search prepared %d viewpoints",
                    len(self.viewpoints),
                )

    def _is_free(self, x, y, forest):
        for center, radius in zip(forest.centers, forest.radii):
            if math.hypot(x - center.x, y - center.y) <= radius + self.tree_clearance:
                return False
        return True

    def _generate_viewpoints(self, forest):
        rows = []
        y = self.cy - self.radius
        row_index = 0
        while y <= self.cy + self.radius + 1e-6:
            dy = y - self.cy
            half_width = math.sqrt(max(0.0, self.radius * self.radius - dy * dy))
            x_min = self.cx - half_width
            x_max = self.cx + half_width
            row = []
            x = x_min
            while x <= x_max + 1e-6:
                if self._is_free(x, y, forest):
                    row.append((x, y, self.altitude))
                x += self.spacing
            if row_index % 2 == 1:
                row.reverse()
            rows.extend(row)
            row_index += 1
            y += self.spacing
        return rows

    def _publish_path(self):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame_id
        for x, y, z in self.viewpoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    def _handle_request(self, request):
        response = NextSearchViewpointResponse()
        with self.lock:
            if self.forest is None:
                response.status = NextSearchViewpointResponse.PENDING
                response.detail = "forest_not_received"
                return response

            if request.reset:
                self.index = 0
                if len(self.viewpoints) >= 2:
                    first = self.viewpoints[0]
                    last = self.viewpoints[-1]
                    first_distance = math.hypot(
                        request.current_position.x - first[0],
                        request.current_position.y - first[1],
                    )
                    last_distance = math.hypot(
                        request.current_position.x - last[0],
                        request.current_position.y - last[1],
                    )
                    if last_distance < first_distance:
                        self.viewpoints.reverse()
                        self._publish_path()

            if self.index >= len(self.viewpoints):
                response.status = NextSearchViewpointResponse.EXHAUSTED
                response.coverage = 1.0
                response.detail = "coverage_complete"
                return response

            x, y, z = self.viewpoints[self.index]
            self.index += 1
            response.status = NextSearchViewpointResponse.AVAILABLE
            response.goal.header.stamp = rospy.Time.now()
            response.goal.header.frame_id = self.frame_id
            response.goal.pose.position.x = x
            response.goal.pose.position.y = y
            response.goal.pose.position.z = z
            response.goal.pose.orientation.w = 1.0
            response.coverage = float(self.index) / max(1, len(self.viewpoints))
            response.detail = "viewpoint_%d_of_%d" % (
                self.index,
                len(self.viewpoints),
            )
            return response


if __name__ == "__main__":
    rospy.init_node("coverage_search_server")
    CoverageSearchServer()
    rospy.spin()
