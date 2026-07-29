#!/usr/bin/env python3

import math
import threading

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from competition_msgs.msg import DropState, MissionStatus, TargetDetection
from competition_msgs.srv import (
    NextSearchViewpoint,
    NextSearchViewpointRequest,
    PlanSafeRoute,
    PlanSafeRouteRequest,
)


STATE_NAMES = {
    MissionStatus.WAIT_START: "WAIT_START",
    MissionStatus.TAKEOFF: "TAKEOFF",
    MissionStatus.TRANSIT_TO_ROI: "TRANSIT_TO_ROI",
    MissionStatus.SEARCH: "SEARCH",
    MissionStatus.TARGET_CONFIRM: "TARGET_CONFIRM",
    MissionStatus.APPROACH: "APPROACH",
    MissionStatus.STABILIZE: "STABILIZE",
    MissionStatus.DROP: "DROP",
    MissionStatus.VERIFY_RELEASE: "VERIFY_RELEASE",
    MissionStatus.RETURN: "RETURN",
    MissionStatus.LAND: "LAND",
    MissionStatus.COMPLETE: "COMPLETE",
    MissionStatus.ABORT: "ABORT",
}


class CompetitionMissionManager:
    def __init__(self):
        self.frame_id = rospy.get_param("/competition/frame_id", "world")
        home = rospy.get_param("/competition/home", [-10.0, 0.0, 0.2])
        roi = rospy.get_param("/competition/roi_center", [5.0, 0.0])
        self.home = tuple(float(value) for value in home)
        self.roi = tuple(float(value) for value in roi)
        self.takeoff_height = float(
            rospy.get_param("/competition/takeoff_height", 1.0)
        )
        self.land_height = float(rospy.get_param("/competition/land_height", 0.2))
        self.search_height = float(
            rospy.get_param("/competition/search_height", 1.2)
        )
        self.drop_height = float(rospy.get_param("/competition/drop_height", 1.0))
        self.arrival_tolerance = float(
            rospy.get_param("/competition/arrival_tolerance", 0.35)
        )
        self.landing_tolerance = float(
            rospy.get_param("/competition/landing_tolerance", 0.25)
        )
        self.arrival_speed = float(
            rospy.get_param("/competition/arrival_speed", 0.20)
        )
        self.stabilize_time = float(
            rospy.get_param("/competition/stabilize_time", 1.5)
        )
        self.goal_timeout = float(rospy.get_param("/competition/goal_timeout", 45.0))
        self.mission_timeout = float(
            rospy.get_param("/competition/mission_timeout", 240.0)
        )
        self.detection_confirmations = int(
            rospy.get_param("/competition/detection_confirmations", 4)
        )
        self.detection_timeout = float(
            rospy.get_param("/competition/detection_timeout", 1.0)
        )
        self.target_type = rospy.get_param("/competition/target_type", "red")
        self.search_backend = rospy.get_param("~search_backend", "coverage")
        if self.search_backend not in ("coverage", "fuel"):
            raise ValueError(
                "search_backend must be 'coverage' or 'fuel', got %r"
                % self.search_backend
            )
        self.search_timeout = float(
            rospy.get_param("/competition/search_timeout", 90.0)
        )
        self.search_goal_timeout = float(
            rospy.get_param("/competition/search_goal_timeout", 15.0)
        )
        self.search_max_consecutive_skips = int(
            rospy.get_param(
                "/competition/search_max_consecutive_skips", 8
            )
        )
        self.search_pass_radius = float(
            rospy.get_param("/competition/search_pass_radius", 0.65)
        )
        self.search_pass_cross_track = float(
            rospy.get_param("/competition/search_pass_cross_track", 0.90)
        )
        self.search_progress_timeout = float(
            rospy.get_param("/competition/search_progress_timeout", 4.0)
        )
        self.search_progress_epsilon = float(
            rospy.get_param("/competition/search_progress_epsilon", 0.05)
        )
        self.motion_progress_timeout = float(
            rospy.get_param(
                "/competition/motion_progress_timeout",
                rospy.get_param(
                    "/competition/transit_progress_timeout", 4.0
                ),
            )
        )
        self.motion_progress_epsilon = float(
            rospy.get_param(
                "/competition/motion_progress_epsilon",
                rospy.get_param(
                    "/competition/transit_progress_epsilon", 0.05
                ),
            )
        )
        self.motion_retry_delay = float(
            rospy.get_param(
                "/competition/motion_retry_delay",
                rospy.get_param("/competition/transit_retry_delay", 0.5),
            )
        )
        self.motion_max_retries = int(
            rospy.get_param(
                "/competition/motion_max_retries",
                rospy.get_param("/competition/transit_max_retries", 2),
            )
        )
        self.motion_waypoint_delay = float(
            rospy.get_param("/competition/motion_waypoint_delay", 0.75)
        )
        self.motion_waypoint_tolerance = float(
            rospy.get_param("/competition/motion_waypoint_tolerance", 0.15)
        )
        self.motion_waypoint_settle_time = float(
            rospy.get_param(
                "/competition/motion_waypoint_settle_time", 0.15
            )
        )
        self.auto_start = bool(rospy.get_param("~auto_start", False))
        self.auto_start_delay = float(rospy.get_param("~auto_start_delay", 3.0))

        self.lock = threading.RLock()
        self.odom = None
        self.drop_state = None
        self.state = MissionStatus.WAIT_START
        self.detail = "waiting_for_odometry"
        self.active_goal = None
        self.goal_sequence = 0
        self.goal_started = None
        self.arrival_since = None
        self.first_odom_time = None
        self.mission_started = None
        self.state_started = rospy.Time.now()
        self.start_requested = False
        self.search_reset = True
        self.last_search_request = rospy.Time(0)
        self.detections = []
        self.last_detection_time = rospy.Time(0)
        self.confirmed_target = None
        self.stable_since = None
        self.drop_requested = False
        self.release_verified_time = None
        self.search_started = None
        self.search_consecutive_skips = 0
        self.search_goal_origin = None
        self.search_best_distance = float("inf")
        self.search_last_progress = None
        self.motion_route = []
        self.motion_route_index = 0
        self.motion_route_detail = ""
        self.motion_retry_count = 0
        self.motion_best_distance = float("inf")
        self.motion_last_progress = None
        self.motion_retry_pending = None
        self.motion_next_pending = None
        self.last_route_request = rospy.Time(0)

        odom_topic = rospy.get_param(
            "~odom_topic", "/drone_0_visual_slam/odom"
        )
        self.goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher(
            "/competition/state", MissionStatus, queue_size=10, latch=True
        )
        self.stop_pub = rospy.Publisher(
            "/mandatory_stop_to_planner", Empty, queue_size=1
        )
        self.fuel_trigger_pub = rospy.Publisher(
            "/waypoint_generator/waypoints", Path, queue_size=1, latch=True
        )
        self.odom_sub = rospy.Subscriber(
            odom_topic, Odometry, self._odom_callback, queue_size=1
        )
        self.detection_sub = rospy.Subscriber(
            "/competition/target_detection",
            TargetDetection,
            self._detection_callback,
            queue_size=10,
        )
        self.drop_sub = rospy.Subscriber(
            "/competition/drop_state",
            DropState,
            self._drop_callback,
            queue_size=1,
        )
        self.start_sub = rospy.Subscriber(
            "/competition/start", Empty, self._start_callback, queue_size=1
        )
        self.search_client = rospy.ServiceProxy(
            "/competition/next_search_viewpoint", NextSearchViewpoint
        )
        self.route_client = rospy.ServiceProxy(
            "/competition/plan_safe_route", PlanSafeRoute
        )
        self.drop_client = rospy.ServiceProxy(
            "/competition/release_payload", Trigger
        )
        self.timer = rospy.Timer(rospy.Duration(0.05), self._update)
        self.status_timer = rospy.Timer(rospy.Duration(0.2), self._publish_status)
        self._publish_status(None)

    def _odom_callback(self, msg):
        with self.lock:
            self.odom = msg
            if self.first_odom_time is None:
                self.first_odom_time = rospy.Time.now()
                self.detail = "odometry_ready"

    def _drop_callback(self, msg):
        with self.lock:
            self.drop_state = msg

    def _start_callback(self, _msg):
        with self.lock:
            self.start_requested = True
            self.detail = "start_requested"

    def _detection_callback(self, msg):
        with self.lock:
            if self.state != MissionStatus.SEARCH:
                return
            if not msg.detected or msg.target_type != self.target_type:
                self.detections = []
                return
            self.last_detection_time = rospy.Time.now()
            self.detections.append(
                (
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                )
            )
            if len(self.detections) > self.detection_confirmations:
                self.detections.pop(0)

    @staticmethod
    def _speed(odom):
        velocity = odom.twist.twist.linear
        return math.sqrt(
            velocity.x * velocity.x
            + velocity.y * velocity.y
            + velocity.z * velocity.z
        )

    @staticmethod
    def _distance(odom, goal):
        position = odom.pose.pose.position
        target = goal.pose.position
        return math.sqrt(
            (position.x - target.x) ** 2
            + (position.y - target.y) ** 2
            + (position.z - target.z) ** 2
        )

    def _make_goal(self, x, y, z):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = float(z)
        goal.pose.orientation.w = 1.0
        return goal

    def _publish_goal(self, goal, detail):
        self.active_goal = goal
        self.goal_sequence += 1
        self.goal_started = rospy.Time.now()
        self.arrival_since = None
        self.detail = detail
        self.goal_pub.publish(goal)
        rospy.loginfo(
            "[competition] Goal %d (%s): %.3f %.3f %.3f",
            self.goal_sequence,
            detail,
            goal.pose.position.x,
            goal.pose.position.y,
            goal.pose.position.z,
        )

    def _set_state(self, state, detail):
        previous = self.state
        self.state = state
        self.state_started = rospy.Time.now()
        self.detail = detail
        self.arrival_since = None
        rospy.logwarn(
            "[competition] %s -> %s: %s",
            STATE_NAMES[previous],
            STATE_NAMES[state],
            detail,
        )
        self._publish_status(None)

    def _goal_arrived(
        self,
        now,
        landing=False,
        tolerance=None,
        settle_time=0.35,
        position_only_timeout=None,
    ):
        if self.odom is None or self.active_goal is None:
            return False
        if tolerance is None:
            tolerance = (
                self.landing_tolerance
                if landing
                else self.arrival_tolerance
            )
        position_arrived = (
            self._distance(self.odom, self.active_goal) <= tolerance
        )
        if not position_arrived:
            self.arrival_since = None
            return False
        if self.arrival_since is None:
            self.arrival_since = now
            return False
        dwell = (now - self.arrival_since).to_sec()
        if (
            self._speed(self.odom) <= self.arrival_speed
            and dwell >= settle_time
        ):
            return True
        return (
            position_only_timeout is not None
            and dwell >= position_only_timeout
        )

    def _motion_goal_arrived(self, now):
        intermediate = (
            self.motion_route
            and self.motion_route_index + 1 < len(self.motion_route)
        )
        tolerance = (
            self.motion_waypoint_tolerance
            if intermediate
            else self.arrival_tolerance
        )
        settle_time = (
            self.motion_waypoint_settle_time
            if intermediate
            else 0.35
        )
        return self._goal_arrived(
            now, tolerance=tolerance, settle_time=settle_time
        )

    def _check_goal_timeout(self, now):
        if self.goal_started is None:
            return False
        if (now - self.goal_started).to_sec() > self.goal_timeout:
            self._abort("goal_timeout_in_%s" % STATE_NAMES[self.state])
            return True
        return False

    def _search_goal_timed_out(self, now):
        return (
            self.goal_started is not None
            and (now - self.goal_started).to_sec() > self.search_goal_timeout
        )

    def _reset_motion_tracking(self, now):
        self.motion_best_distance = self._distance(
            self.odom, self.active_goal
        )
        self.motion_last_progress = now
        self.motion_retry_pending = None

    def _motion_stalled(self, now):
        distance = self._distance(self.odom, self.active_goal)
        if distance < (
            self.motion_best_distance - self.motion_progress_epsilon
        ):
            self.motion_best_distance = distance
            self.motion_last_progress = now
            return False
        return (
            self.motion_last_progress is not None
            and (now - self.motion_last_progress).to_sec()
            > self.motion_progress_timeout
        )

    def _schedule_motion_retry(self, now):
        if self.motion_retry_count >= self.motion_max_retries:
            self._abort(
                "%s_no_progress_after_%d_retries"
                % (STATE_NAMES[self.state].lower(), self.motion_retry_count)
            )
            return

        self.motion_retry_count += 1
        self.motion_retry_pending = now
        self.active_goal = None
        self.goal_started = None
        self.arrival_since = None
        self.detail = "motion_recovery_wait_%d" % self.motion_retry_count
        rospy.logwarn(
            "[competition] Motion recovery in %s %d/%d scheduled after "
            "%.1f s without progress",
            STATE_NAMES[self.state],
            self.motion_retry_count,
            self.motion_max_retries,
            self.motion_progress_timeout,
        )

    def _continue_motion_retry(self, now):
        if self.motion_retry_pending is None:
            return False
        if (
            now - self.motion_retry_pending
        ).to_sec() < self.motion_retry_delay:
            return True

        self._publish_motion_route_goal(
            now, "retry_%d" % self.motion_retry_count
        )
        rospy.logwarn(
            "[competition] Motion recovery in %s %d/%d goal republished",
            STATE_NAMES[self.state],
            self.motion_retry_count,
            self.motion_max_retries,
        )
        return True

    def _start_motion_route(self, goal, detail, now):
        if (now - self.last_route_request).to_sec() < 0.5:
            return False
        self.last_route_request = now

        route = [goal]
        if self.search_backend == "coverage":
            try:
                request = PlanSafeRouteRequest()
                position = self.odom.pose.pose.position
                request.start = Point(
                    x=position.x, y=position.y, z=position.z
                )
                request.goal = Point(
                    x=goal.pose.position.x,
                    y=goal.pose.position.y,
                    z=goal.pose.position.z,
                )
                response = self.route_client(request)
                if not response.success or not response.waypoints:
                    self.detail = "safe_route_waiting:%s" % response.detail
                    return False
                route = list(response.waypoints)
            except (rospy.ServiceException, rospy.ROSException) as error:
                self.detail = "safe_route_waiting:%s" % error
                return False

        self.motion_route = route
        self.motion_route_index = 0
        self.motion_route_detail = detail
        self.motion_retry_count = 0
        self.motion_next_pending = None
        self._publish_motion_route_goal(now)
        return True

    def _publish_motion_route_goal(self, now, suffix=None):
        goal = self.motion_route[self.motion_route_index]
        count = len(self.motion_route)
        if count == 1:
            detail = self.motion_route_detail
        else:
            detail = "%s_route_%d_of_%d" % (
                self.motion_route_detail,
                self.motion_route_index + 1,
                count,
            )
        if suffix:
            detail = "%s_%s" % (detail, suffix)
        goal.header.stamp = now
        self._publish_goal(goal, detail)
        self._reset_motion_tracking(now)

    def _advance_motion_route(self, now):
        if self.motion_route_index + 1 >= len(self.motion_route):
            self.motion_route = []
            self.motion_route_index = 0
            self.motion_route_detail = ""
            self.motion_retry_pending = None
            self.motion_next_pending = None
            return True
        self.motion_route_index += 1
        self.motion_retry_count = 0
        self.motion_next_pending = now
        self.active_goal = None
        self.goal_started = None
        self.arrival_since = None
        self.detail = "motion_waypoint_transition_wait"
        return False

    def _continue_motion_route(self, now):
        if self.motion_next_pending is None:
            return False
        if (
            now - self.motion_next_pending
        ).to_sec() < self.motion_waypoint_delay:
            return True
        self.motion_next_pending = None
        self._publish_motion_route_goal(now)
        return True

    def _reset_search_goal_tracking(self, now):
        position = self.odom.pose.pose.position
        self.search_goal_origin = (position.x, position.y, position.z)
        self.search_best_distance = self._distance(
            self.odom, self.active_goal
        )
        self.search_last_progress = now

    def _clear_search_goal_tracking(self):
        self.search_goal_origin = None
        self.search_best_distance = float("inf")
        self.search_last_progress = None

    def _search_point_passed(self):
        if self.odom is None or self.active_goal is None:
            return False

        if self._distance(self.odom, self.active_goal) <= self.search_pass_radius:
            return True
        if self.search_goal_origin is None:
            return False

        start_x, start_y, _start_z = self.search_goal_origin
        target = self.active_goal.pose.position
        position = self.odom.pose.pose.position
        dx = target.x - start_x
        dy = target.y - start_y
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-9:
            return True

        progress = (
            (position.x - start_x) * dx + (position.y - start_y) * dy
        ) / length_squared
        if progress < 1.0:
            return False

        cross_track = abs(
            (position.x - start_x) * dy
            - (position.y - start_y) * dx
        ) / math.sqrt(length_squared)
        return cross_track <= self.search_pass_cross_track

    def _search_goal_stalled(self, now):
        if self.odom is None or self.active_goal is None:
            return False

        distance = self._distance(self.odom, self.active_goal)
        if distance < self.search_best_distance - self.search_progress_epsilon:
            self.search_best_distance = distance
            self.search_last_progress = now
            return False
        return (
            self.search_last_progress is not None
            and (now - self.search_last_progress).to_sec()
            > self.search_progress_timeout
        )

    def _skip_search_goal(self, reason):
        self.search_consecutive_skips += 1
        rospy.logwarn(
            "[competition] Skipping search goal %d: %s (%d consecutive)",
            self.goal_sequence,
            reason,
            self.search_consecutive_skips,
        )
        self.active_goal = None
        self.goal_started = None
        self.arrival_since = None
        self._clear_search_goal_tracking()
        self.detail = "search_goal_skipped:%s" % reason
        if (
            self.search_consecutive_skips
            > self.search_max_consecutive_skips
        ):
            self._abort("too_many_unreachable_search_goals")
            return False
        return True

    def _request_search_goal(self, now):
        if (now - self.last_search_request).to_sec() < 0.5:
            return
        self.last_search_request = now
        try:
            request = NextSearchViewpointRequest()
            position = self.odom.pose.pose.position
            request.current_position = Point(
                x=position.x, y=position.y, z=position.z
            )
            request.reset = self.search_reset
            response = self.search_client(request)
            if response.status == response.AVAILABLE:
                self.search_reset = False
                self._publish_goal(response.goal, response.detail)
                self._reset_search_goal_tracking(now)
            elif response.status == response.EXHAUSTED:
                self._abort("target_not_found_after_search")
            else:
                self.detail = response.detail
        except (rospy.ServiceException, rospy.ROSException) as error:
            self.detail = "search_service_waiting:%s" % error

    def _trigger_fuel_search(self, now):
        path = Path()
        path.header.stamp = now
        path.header.frame_id = self.frame_id
        trigger = PoseStamped()
        trigger.header = path.header
        trigger.pose.position.x = self.odom.pose.pose.position.x
        trigger.pose.position.y = self.odom.pose.pose.position.y
        trigger.pose.position.z = self.search_height
        trigger.pose.orientation.w = 1.0
        path.poses.append(trigger)
        self.fuel_trigger_pub.publish(path)
        self.detail = "fuel_search_triggered"
        rospy.logwarn("[competition] FUEL exploration trigger published")

    def _abort(self, reason):
        if self.state in (MissionStatus.COMPLETE, MissionStatus.ABORT):
            return
        self.motion_route = []
        self.motion_retry_pending = None
        self.motion_next_pending = None
        self._set_state(MissionStatus.ABORT, reason)
        self.stop_pub.publish(Empty())

    def _update(self, _event):
        with self.lock:
            now = rospy.Time.now()
            if self.odom is None:
                return

            if self.state not in (
                MissionStatus.WAIT_START,
                MissionStatus.COMPLETE,
                MissionStatus.ABORT,
            ):
                if (
                    self.mission_started is not None
                    and (now - self.mission_started).to_sec() > self.mission_timeout
                ):
                    self._abort("mission_timeout")
                    return

            if self.state == MissionStatus.WAIT_START:
                auto_ready = (
                    self.auto_start
                    and self.first_odom_time is not None
                    and (now - self.first_odom_time).to_sec() >= self.auto_start_delay
                )
                if self.start_requested or auto_ready:
                    self.mission_started = now
                    self._set_state(MissionStatus.TAKEOFF, "mission_started")
                    self._publish_goal(
                        self._make_goal(
                            self.home[0], self.home[1], self.takeoff_height
                        ),
                        "takeoff",
                    )

            elif self.state == MissionStatus.TAKEOFF:
                if self._check_goal_timeout(now):
                    return
                if self._goal_arrived(
                    now, position_only_timeout=1.0
                ):
                    if not self._start_motion_route(
                        self._make_goal(
                            self.roi[0], self.roi[1], self.search_height
                        ),
                        "transit_to_roi",
                        now,
                    ):
                        return
                    self._set_state(
                        MissionStatus.TRANSIT_TO_ROI, "takeoff_complete"
                    )

            elif self.state == MissionStatus.TRANSIT_TO_ROI:
                if self._continue_motion_route(now):
                    return
                if self._continue_motion_retry(now):
                    return
                if self._check_goal_timeout(now):
                    return
                if self._motion_goal_arrived(now):
                    if not self._advance_motion_route(now):
                        return
                    self._set_state(MissionStatus.SEARCH, "roi_reached")
                    self.active_goal = None
                    self.goal_started = None
                    self.search_reset = True
                    self.search_started = now
                    self.search_consecutive_skips = 0
                    self._clear_search_goal_tracking()
                    if self.search_backend == "fuel":
                        self._trigger_fuel_search(now)
                    else:
                        self._request_search_goal(now)
                elif self._motion_stalled(now):
                    self._schedule_motion_retry(now)

            elif self.state == MissionStatus.SEARCH:
                if (
                    self.search_started is not None
                    and (now - self.search_started).to_sec() > self.search_timeout
                ):
                    self._abort("target_not_found_before_search_timeout")
                    return

                if (
                    self.last_detection_time != rospy.Time(0)
                    and (now - self.last_detection_time).to_sec()
                    > self.detection_timeout
                ):
                    self.detections = []

                if len(self.detections) >= self.detection_confirmations:
                    count = float(len(self.detections))
                    self.confirmed_target = (
                        sum(item[0] for item in self.detections) / count,
                        sum(item[1] for item in self.detections) / count,
                        sum(item[2] for item in self.detections) / count,
                    )
                    self._set_state(
                        MissionStatus.TARGET_CONFIRM, "target_confirmed"
                    )
                    return

                if self.search_backend == "coverage":
                    if self.active_goal is None:
                        self._request_search_goal(now)
                    elif self._search_point_passed():
                        self.search_consecutive_skips = 0
                        self.active_goal = None
                        self.goal_started = None
                        self._clear_search_goal_tracking()
                        self._request_search_goal(now)
                    elif self._search_goal_stalled(now):
                        if self._skip_search_goal("no_progress"):
                            self._request_search_goal(now)
                    elif self._search_goal_timed_out(now):
                        if self._skip_search_goal("timeout"):
                            self._request_search_goal(now)

            elif self.state == MissionStatus.TARGET_CONFIRM:
                if (now - self.state_started).to_sec() >= 0.25:
                    if not self._start_motion_route(
                        self._make_goal(
                            self.confirmed_target[0],
                            self.confirmed_target[1],
                            self.drop_height,
                        ),
                        "target_overhead",
                        now,
                    ):
                        return
                    self._set_state(MissionStatus.APPROACH, "approaching_target")

            elif self.state == MissionStatus.APPROACH:
                if self._continue_motion_route(now):
                    return
                if self._continue_motion_retry(now):
                    return
                if self._check_goal_timeout(now):
                    return
                if self._motion_goal_arrived(now):
                    if not self._advance_motion_route(now):
                        return
                    self._set_state(MissionStatus.STABILIZE, "over_target")
                    self.stable_since = now
                elif self._motion_stalled(now):
                    self._schedule_motion_retry(now)

            elif self.state == MissionStatus.STABILIZE:
                if not self._goal_arrived(now):
                    self.stable_since = None
                elif self.stable_since is None:
                    self.stable_since = now
                elif (now - self.stable_since).to_sec() >= self.stabilize_time:
                    self._set_state(MissionStatus.DROP, "stabilized")

            elif self.state == MissionStatus.DROP:
                if not self.drop_requested:
                    self.drop_requested = True
                    try:
                        response = self.drop_client()
                        if not response.success:
                            self._abort("drop_rejected:%s" % response.message)
                            return
                        self._set_state(
                            MissionStatus.VERIFY_RELEASE, response.message
                        )
                    except (rospy.ServiceException, rospy.ROSException) as error:
                        self._abort("drop_service_failed:%s" % error)

            elif self.state == MissionStatus.VERIFY_RELEASE:
                if self.drop_state is not None and self.drop_state.released:
                    if self.release_verified_time is None:
                        self.release_verified_time = now
                    elif (now - self.release_verified_time).to_sec() >= 0.5:
                        if not self._start_motion_route(
                            self._make_goal(
                                self.home[0], self.home[1], self.takeoff_height
                            ),
                            "return_home",
                            now,
                        ):
                            return
                        self._set_state(
                            MissionStatus.RETURN, "release_verified"
                        )
                elif (now - self.state_started).to_sec() > 3.0:
                    self._abort("release_not_verified")

            elif self.state == MissionStatus.RETURN:
                if self._continue_motion_route(now):
                    return
                if self._continue_motion_retry(now):
                    return
                if self._check_goal_timeout(now):
                    return
                if self._motion_goal_arrived(now):
                    if not self._advance_motion_route(now):
                        return
                    self._set_state(MissionStatus.LAND, "home_reached")
                    self._publish_goal(
                        self._make_goal(
                            self.home[0], self.home[1], self.land_height
                        ),
                        "land",
                    )
                elif self._motion_stalled(now):
                    self._schedule_motion_retry(now)

            elif self.state == MissionStatus.LAND:
                if self._check_goal_timeout(now):
                    return
                if self._goal_arrived(now, landing=True):
                    self._set_state(MissionStatus.COMPLETE, "mission_complete")
                    self.active_goal = None
                    self.goal_started = None

    def _publish_status(self, _event):
        with self.lock:
            msg = MissionStatus()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.state = self.state
            msg.state_name = STATE_NAMES[self.state]
            msg.detail = self.detail
            if self.active_goal is not None:
                msg.active_goal = self.active_goal
            msg.goal_sequence = self.goal_sequence
            if self.mission_started is not None:
                msg.elapsed = (rospy.Time.now() - self.mission_started).to_sec()
            self.status_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("competition_mission_manager")
    CompetitionMissionManager()
    rospy.spin()
