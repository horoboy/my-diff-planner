#!/usr/bin/env python3

import json
import math
import os
import threading

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32

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
        self.min_flight_altitude = float(
            rospy.get_param("/competition/min_flight_altitude", 0.0)
        )
        self.max_flight_altitude = float(
            rospy.get_param("/competition/max_flight_altitude", 2.6)
        )
        self.altitude_violation_tolerance = float(
            rospy.get_param(
                "/competition/altitude_violation_tolerance", 0.02
            )
        )
        self.output_file = rospy.get_param(
            "~output_file", "/tmp/competition_single_result.json"
        )
        self.frame_id = rospy.get_param("/competition/frame_id", "world")

        self.lock = threading.Lock()
        self.forest = None
        self.drop_state = None
        self.min_clearance = float("inf")
        self.min_clearance_position = None
        self.min_clearance_tree_id = -1
        self.min_clearance_tree_center = None
        self.min_clearance_tree_radius = None
        self.min_clearance_state = "WAIT_START"
        self.current_state = "WAIT_START"
        self.active_collisions = set()
        self.collision_count = 0
        self.min_altitude = float("inf")
        self.max_altitude = -float("inf")
        self.altitude_violation_active = False
        self.altitude_violation_count = 0
        self.planner_recovery_count = 0
        self.last_position = None
        self.last_speed = None
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
        self.recovery_sub = rospy.Subscriber(
            "/competition/planner_watchdog/recovery_count",
            UInt32,
            self._recovery_callback,
            queue_size=1,
        )

    def _forest_callback(self, msg):
        with self.lock:
            self.forest = msg

    def _drop_callback(self, msg):
        with self.lock:
            self.drop_state = msg

    def _recovery_callback(self, msg):
        with self.lock:
            self.planner_recovery_count = msg.data

    def _odom_callback(self, msg):
        with self.lock:
            if self.finished:
                return
            position = msg.pose.pose.position
            velocity = msg.twist.twist.linear
            self.last_position = [position.x, position.y, position.z]
            self.last_speed = math.sqrt(
                velocity.x * velocity.x
                + velocity.y * velocity.y
                + velocity.z * velocity.z
            )
            self.min_altitude = min(self.min_altitude, position.z)
            self.max_altitude = max(self.max_altitude, position.z)
            altitude_violation = (
                position.z
                < self.min_flight_altitude - self.altitude_violation_tolerance
                or position.z
                > self.max_flight_altitude
                + self.altitude_violation_tolerance
            )
            if altitude_violation and not self.altitude_violation_active:
                self.altitude_violation_count += 1
                rospy.logerr(
                    "[competition] Altitude boundary violation at z=%.3f; "
                    "allowed=[%.3f, %.3f] tolerance=%.3f",
                    position.z,
                    self.min_flight_altitude,
                    self.max_flight_altitude,
                    self.altitude_violation_tolerance,
                )
            self.altitude_violation_active = altitude_violation

            if self.forest is None:
                return
            current_collisions = set()
            for index, (center, radius) in enumerate(
                zip(self.forest.centers, self.forest.radii)
            ):
                horizontal = math.hypot(
                    position.x - center.x, position.y - center.y
                )
                clearance = horizontal - radius - self.uav_radius
                if clearance < self.min_clearance:
                    self.min_clearance = clearance
                    self.min_clearance_position = [
                        position.x,
                        position.y,
                        position.z,
                    ]
                    self.min_clearance_tree_id = index
                    self.min_clearance_tree_center = [
                        center.x,
                        center.y,
                        center.z,
                    ]
                    self.min_clearance_tree_radius = radius
                    self.min_clearance_state = self.current_state
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
            self.current_state = msg.state_name
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
            and self.altitude_violation_count == 0
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
        if self.altitude_violation_count:
            reasons.append("altitude_boundary_violation")
        reason = "success" if success else ",".join(reasons)

        duration = self.final_status.elapsed
        min_clearance = (
            self.min_clearance if math.isfinite(self.min_clearance) else 999.0
        )
        min_altitude = (
            self.min_altitude if math.isfinite(self.min_altitude) else 999.0
        )
        max_altitude = (
            self.max_altitude if math.isfinite(self.max_altitude) else -999.0
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
        result.min_altitude = min_altitude
        result.max_altitude = max_altitude
        result.altitude_violation_count = self.altitude_violation_count
        result.planner_recovery_count = self.planner_recovery_count
        self.result_pub.publish(result)

        output = {
            "finished": True,
            "success": success,
            "reason": reason,
            "duration": duration,
            "drop_error": drop_error,
            "drop_max_error": self.drop_max_error,
            "min_obstacle_clearance": min_clearance,
            "min_clearance_position": self.min_clearance_position,
            "min_clearance_tree_id": self.min_clearance_tree_id,
            "min_clearance_tree_center": self.min_clearance_tree_center,
            "min_clearance_tree_radius": self.min_clearance_tree_radius,
            "min_clearance_state": self.min_clearance_state,
            "collision_count": self.collision_count,
            "min_altitude": min_altitude,
            "max_altitude": max_altitude,
            "min_flight_altitude": self.min_flight_altitude,
            "max_flight_altitude": self.max_flight_altitude,
            "altitude_violation_tolerance": self.altitude_violation_tolerance,
            "altitude_violation_count": self.altitude_violation_count,
            "planner_recovery_count": self.planner_recovery_count,
            "final_position": self.last_position,
            "final_speed": self.last_speed,
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
            "drop_error=%.3f min_clearance=%.3f collisions=%d "
            "altitude=[%.3f, %.3f] altitude_violations=%d output=%s",
            success,
            reason,
            duration,
            drop_error,
            min_clearance,
            self.collision_count,
            min_altitude,
            max_altitude,
            self.altitude_violation_count,
            self.output_file,
        )
        rospy.logwarn(
            "[competition] MIN_CLEARANCE position=%s tree_id=%d "
            "tree_center=%s tree_radius=%s state=%s",
            self.min_clearance_position,
            self.min_clearance_tree_id,
            self.min_clearance_tree_center,
            self.min_clearance_tree_radius,
            self.min_clearance_state,
        )


if __name__ == "__main__":
    rospy.init_node("mission_evaluator")
    MissionEvaluator()
    rospy.spin()
