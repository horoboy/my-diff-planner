#!/usr/bin/env python3

import json
import math
import os
import threading

import rospy
from nav_msgs.msg import Odometry

from competition_msgs.msg import (
    DropState,
    Forest,
    MissionResult,
    MissionStatus,
)


class MissionEvaluator:
    def __init__(self):
        target = rospy.get_param(
            "/competition/target_position", [7.1, 0.8, 0.0]
        )
        self.target = tuple(float(value) for value in target)
        self.uav_radius = float(rospy.get_param("/competition/uav_radius", 0.28))
        self.drop_max_error = float(
            rospy.get_param("/competition/drop_max_error", 0.45)
        )
        self.output_file = rospy.get_param(
            "~output_file", "/tmp/competition_single_result.json"
        )
        self.frame_id = rospy.get_param("/competition/frame_id", "world")

        self.lock = threading.Lock()
        self.forest = None
        self.drop_state = None
        self.min_clearance = float("inf")
        self.active_collisions = set()
        self.collision_count = 0
        self.start_time = None
        self.final_status = None
        self.finished = False

        odom_topic = rospy.get_param(
            "~odom_topic", "/drone_0_visual_slam/odom"
        )
        self.result_pub = rospy.Publisher(
            "/competition/result", MissionResult, queue_size=1, latch=True
        )
        self.odom_sub = rospy.Subscriber(
            odom_topic, Odometry, self._odom_callback, queue_size=10
        )
        self.forest_sub = rospy.Subscriber(
            "/competition/forest", Forest, self._forest_callback, queue_size=1
        )
        self.drop_sub = rospy.Subscriber(
            "/competition/drop_state", DropState, self._drop_callback, queue_size=1
        )
        self.status_sub = rospy.Subscriber(
            "/competition/state",
            MissionStatus,
            self._status_callback,
            queue_size=10,
        )

    def _forest_callback(self, msg):
        with self.lock:
            self.forest = msg

    def _drop_callback(self, msg):
        with self.lock:
            self.drop_state = msg

    def _odom_callback(self, msg):
        with self.lock:
            if self.forest is None or self.finished:
                return
            position = msg.pose.pose.position
            current_collisions = set()
            for index, (center, radius) in enumerate(
                zip(self.forest.centers, self.forest.radii)
            ):
                horizontal = math.hypot(
                    position.x - center.x, position.y - center.y
                )
                clearance = horizontal - radius - self.uav_radius
                self.min_clearance = min(self.min_clearance, clearance)
                if clearance < 0.0 and 0.0 <= position.z <= self.forest.height:
                    current_collisions.add(index)

            new_collisions = current_collisions - self.active_collisions
            if new_collisions:
                self.collision_count += len(new_collisions)
                rospy.logerr(
                    "[competition] Collision detected with tree ids: %s",
                    sorted(new_collisions),
                )
            self.active_collisions = current_collisions

    def _status_callback(self, msg):
        with self.lock:
            if self.finished:
                return
            if msg.state != MissionStatus.WAIT_START and self.start_time is None:
                self.start_time = msg.header.stamp
            if msg.state in (MissionStatus.COMPLETE, MissionStatus.ABORT):
                self.final_status = msg
                self._finish()

    def _finish(self):
        drop_error = float("inf")
        released = self.drop_state is not None and self.drop_state.released
        if released:
            drop_error = math.hypot(
                self.drop_state.release_position.x - self.target[0],
                self.drop_state.release_position.y - self.target[1],
            )

        completed = self.final_status.state == MissionStatus.COMPLETE
        success = (
            completed
            and released
            and drop_error <= self.drop_max_error
            and self.collision_count == 0
        )
        reasons = []
        if not completed:
            reasons.append(self.final_status.detail)
        if not released:
            reasons.append("payload_not_released")
        if released and drop_error > self.drop_max_error:
            reasons.append("drop_error_exceeded")
        if self.collision_count:
            reasons.append("collision_detected")
        reason = "success" if success else ",".join(reasons)

        duration = self.final_status.elapsed
        min_clearance = (
            self.min_clearance if math.isfinite(self.min_clearance) else 999.0
        )
        result = MissionResult()
        result.header.stamp = rospy.Time.now()
        result.header.frame_id = self.frame_id
        result.finished = True
        result.success = success
        result.reason = reason
        result.duration = duration
        result.drop_error = drop_error
        result.min_obstacle_clearance = min_clearance
        result.collision_count = self.collision_count
        self.result_pub.publish(result)

        output = {
            "finished": True,
            "success": success,
            "reason": reason,
            "duration": duration,
            "drop_error": drop_error,
            "drop_max_error": self.drop_max_error,
            "min_obstacle_clearance": min_clearance,
            "collision_count": self.collision_count,
            "final_state": self.final_status.state_name,
            "goal_sequence": self.final_status.goal_sequence,
        }
        output_dir = os.path.dirname(os.path.abspath(self.output_file))
        if output_dir and not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        with open(self.output_file, "w", encoding="utf-8") as handle:
            json.dump(output, handle, indent=2, sort_keys=True)
            handle.write("\n")

        self.finished = True
        rospy.logwarn(
            "[competition] MISSION_RESULT success=%s reason=%s duration=%.2f "
            "drop_error=%.3f min_clearance=%.3f collisions=%d output=%s",
            success,
            reason,
            duration,
            drop_error,
            min_clearance,
            self.collision_count,
            self.output_file,
        )


if __name__ == "__main__":
    rospy.init_node("mission_evaluator")
    MissionEvaluator()
    rospy.spin()
