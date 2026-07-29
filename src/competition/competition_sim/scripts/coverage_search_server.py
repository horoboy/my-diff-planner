#!/usr/bin/env python3

import heapq
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from competition_msgs.msg import Forest
from competition_msgs.srv import (
    NextSearchViewpoint,
    NextSearchViewpointResponse,
    PlanSafeRoute,
    PlanSafeRouteResponse,
)


class CoverageSearchServer:
    def __init__(self):
        roi = rospy.get_param("/competition/roi_center", [5.0, 0.0])
        self.cx = float(roi[0])
        self.cy = float(roi[1])
        self.radius = float(rospy.get_param("/competition/roi_radius", 3.0))
        self.altitude = float(rospy.get_param("/competition/search_height", 1.2))
        legacy_spacing = float(
            rospy.get_param("/competition/search_spacing", 0.8)
        )
        self.lane_spacing = float(
            rospy.get_param(
                "/competition/search_lane_spacing", legacy_spacing
            )
        )
        self.planner_spacing = float(
            rospy.get_param("/competition/search_planner_spacing", 2.0)
        )
        self.path_direction = rospy.get_param(
            "/competition/search_path_direction", "nearest"
        )
        if self.lane_spacing <= 0.0 or self.planner_spacing <= 0.0:
            raise ValueError("search spacing values must be positive")
        if self.path_direction not in (
            "nearest",
            "bottom_to_top",
            "top_to_bottom",
        ):
            raise ValueError(
                "search_path_direction must be nearest, bottom_to_top, "
                "or top_to_bottom"
            )
        self.tree_clearance = float(
            rospy.get_param("/competition/search_tree_clearance", 0.45)
        )
        self.segment_clearance = float(
            rospy.get_param(
                "/competition/search_segment_clearance",
                self.tree_clearance,
            )
        )
        self.mission_route_clearance = float(
            rospy.get_param(
                "/competition/mission_route_clearance",
                self.segment_clearance,
            )
        )
        self.mission_route_fallback_clearance = float(
            rospy.get_param(
                "/competition/mission_route_fallback_clearance",
                self.mission_route_clearance,
            )
        )
        self.route_resolution = float(
            rospy.get_param("/competition/search_route_resolution", 0.20)
        )
        self.route_corridor_margin = float(
            rospy.get_param(
                "/competition/search_route_corridor_margin", 2.0
            )
        )
        self.map_x_size = float(
            rospy.get_param("/competition/map/x_size", 28.0)
        )
        self.map_y_size = float(
            rospy.get_param("/competition/map/y_size", 18.0)
        )
        self.uav_radius = float(
            rospy.get_param("/competition/uav_radius", 0.28)
        )
        self.required_obstacle_clearance = float(
            rospy.get_param(
                "/competition/required_obstacle_clearance", 0.20
            )
        )
        minimum_tree_clearance = (
            self.uav_radius + self.required_obstacle_clearance
        )
        if self.tree_clearance < minimum_tree_clearance:
            raise ValueError(
                "search_tree_clearance %.3f is below uav_radius + "
                "required_obstacle_clearance %.3f"
                % (self.tree_clearance, minimum_tree_clearance)
            )
        if self.segment_clearance < minimum_tree_clearance:
            raise ValueError(
                "search_segment_clearance %.3f is below uav_radius + "
                "required_obstacle_clearance %.3f"
                % (self.segment_clearance, minimum_tree_clearance)
            )
        if self.mission_route_clearance < minimum_tree_clearance:
            raise ValueError(
                "mission_route_clearance %.3f is below uav_radius + "
                "required_obstacle_clearance %.3f"
                % (self.mission_route_clearance, minimum_tree_clearance)
            )
        if self.mission_route_fallback_clearance < minimum_tree_clearance:
            raise ValueError(
                "mission_route_fallback_clearance %.3f is below "
                "uav_radius + required_obstacle_clearance %.3f"
                % (
                    self.mission_route_fallback_clearance,
                    minimum_tree_clearance,
                )
            )
        if (
            self.mission_route_fallback_clearance
            > self.mission_route_clearance
        ):
            raise ValueError(
                "mission_route_fallback_clearance must not exceed "
                "mission_route_clearance"
            )
        if self.route_resolution <= 0.0:
            raise ValueError("search_route_resolution must be positive")
        if self.route_corridor_margin <= 0.0:
            raise ValueError("search_route_corridor_margin must be positive")
        self.frame_id = rospy.get_param("/competition/frame_id", "world")

        self.lock = threading.Lock()
        self.forest = None
        self.coverage_path = []
        self.base_viewpoints = []
        self.viewpoints = []
        self.generation_error = None
        self.generation_attempted = False
        self.generation_in_progress = False
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
        self.route_service = rospy.Service(
            "/competition/plan_safe_route",
            PlanSafeRoute,
            self._handle_route_request,
        )

    def _forest_callback(self, msg):
        with self.lock:
            self.forest = msg
            if (
                self.base_viewpoints
                or self.generation_attempted
                or self.generation_in_progress
            ):
                return
            self.generation_attempted = True
            self.generation_in_progress = True

        try:
            coverage_path, base_viewpoints = self._generate_viewpoints(msg)
            if not base_viewpoints:
                raise RuntimeError("no_safe_search_viewpoints")
        except RuntimeError as error:
            with self.lock:
                self.generation_error = str(error)
                self.generation_in_progress = False
            rospy.logerr(
                "[competition] Coverage route unavailable: %s",
                self.generation_error,
            )
            return

        with self.lock:
            self.coverage_path = coverage_path
            self.base_viewpoints = base_viewpoints
            self.viewpoints = list(self.base_viewpoints)
            self.generation_error = None
            self.generation_in_progress = False
            self._publish_path()
        rospy.loginfo(
            "[competition] Coverage search prepared %d samples and "
            "%d safe planner goals, direction=%s",
            len(coverage_path),
            len(base_viewpoints),
            self.path_direction,
        )

    def _is_free(self, x, y, forest):
        return self._is_free_with_clearance(
            x, y, forest, self.tree_clearance
        )

    @staticmethod
    def _is_free_with_clearance(x, y, forest, clearance):
        for center, radius in zip(forest.centers, forest.radii):
            if math.hypot(x - center.x, y - center.y) <= radius + clearance:
                return False
        return True

    @staticmethod
    def _point_segment_distance(point, start, end):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-12:
            return math.hypot(point[0] - start[0], point[1] - start[1])
        projection = (
            (point[0] - start[0]) * dx
            + (point[1] - start[1]) * dy
        ) / length_squared
        projection = max(0.0, min(1.0, projection))
        closest_x = start[0] + projection * dx
        closest_y = start[1] + projection * dy
        return math.hypot(point[0] - closest_x, point[1] - closest_y)

    def _segment_is_free(self, start, end, forest, clearance=None):
        required_clearance = (
            self.segment_clearance if clearance is None else clearance
        )
        for center, radius in zip(forest.centers, forest.radii):
            distance = self._point_segment_distance(
                (center.x, center.y), start, end
            )
            if distance <= radius + required_clearance:
                return False
        return True

    def _planner_goals_for_row(self, row):
        if not row:
            return []

        goals = [row[0]]
        for point in row[1:]:
            if (
                math.hypot(
                    point[0] - goals[-1][0],
                    point[1] - goals[-1][1],
                )
                >= self.planner_spacing
            ):
                goals.append(point)

        if goals[-1] != row[-1]:
            goals.append(row[-1])
        return goals

    def _route_grid(
        self,
        forest,
        start=None,
        goal=None,
        altitude=None,
        clearance=None,
    ):
        if start is None or goal is None:
            x_min = self.cx - self.radius - self.route_corridor_margin
            x_max = self.cx + self.radius + self.route_corridor_margin
            y_min = self.cy - self.radius - self.route_corridor_margin
            y_max = self.cy + self.radius + self.route_corridor_margin
            x_min = max(-self.map_x_size / 2.0, x_min)
            x_max = min(self.map_x_size / 2.0, x_max)
            y_min = max(-self.map_y_size / 2.0, y_min)
            y_max = min(self.map_y_size / 2.0, y_max)
        else:
            x_min = min(start[0], goal[0]) - self.route_corridor_margin
            x_max = max(start[0], goal[0]) + self.route_corridor_margin
            y_min = min(start[1], goal[1]) - self.route_corridor_margin
            y_max = max(start[1], goal[1]) + self.route_corridor_margin
            x_min = max(-self.map_x_size / 2.0, x_min)
            x_max = min(self.map_x_size / 2.0, x_max)
            y_min = max(-self.map_y_size / 2.0, y_min)
            y_max = min(self.map_y_size / 2.0, y_max)

        route_altitude = self.altitude if altitude is None else altitude
        required_clearance = (
            self.segment_clearance if clearance is None else clearance
        )
        x_count = int(math.ceil((x_max - x_min) / self.route_resolution))
        y_count = int(math.ceil((y_max - y_min) / self.route_resolution))
        nodes = {}
        for ix in range(x_count + 1):
            x = min(x_max, x_min + ix * self.route_resolution)
            for iy in range(y_count + 1):
                y = min(y_max, y_min + iy * self.route_resolution)
                if self._is_free_with_clearance(
                    x, y, forest, required_clearance
                ):
                    nodes[(ix, iy)] = (x, y, route_altitude)
        return nodes

    def _nearest_visible_node(
        self, point, nodes, forest, clearance=None
    ):
        candidates = sorted(
            nodes.items(),
            key=lambda item: (
                item[1][0] - point[0]
            ) ** 2 + (item[1][1] - point[1]) ** 2,
        )
        for key, candidate in candidates:
            if self._segment_is_free(
                point, candidate, forest, clearance
            ):
                return key
        return None

    def _astar_route(
        self, start, goal, forest, nodes, clearance=None
    ):
        start_key = self._nearest_visible_node(
            start, nodes, forest, clearance
        )
        goal_key = self._nearest_visible_node(
            goal, nodes, forest, clearance
        )
        if start_key is None or goal_key is None:
            return None

        frontier = [(0.0, start_key)]
        parent = {start_key: None}
        cost = {start_key: 0.0}
        neighbor_offsets = (
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        )
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal_key:
                break
            current_point = nodes[current]
            for dx, dy in neighbor_offsets:
                neighbor = (current[0] + dx, current[1] + dy)
                if neighbor not in nodes:
                    continue
                neighbor_point = nodes[neighbor]
                if not self._segment_is_free(
                    current_point, neighbor_point, forest, clearance
                ):
                    continue
                step_cost = math.hypot(
                    neighbor_point[0] - current_point[0],
                    neighbor_point[1] - current_point[1],
                )
                new_cost = cost[current] + step_cost
                if neighbor in cost and new_cost >= cost[neighbor]:
                    continue
                cost[neighbor] = new_cost
                parent[neighbor] = current
                heuristic = math.hypot(
                    neighbor_point[0] - nodes[goal_key][0],
                    neighbor_point[1] - nodes[goal_key][1],
                )
                heapq.heappush(
                    frontier, (new_cost + heuristic, neighbor)
                )

        if goal_key not in parent:
            return None

        grid_path = []
        current = goal_key
        while current is not None:
            grid_path.append(nodes[current])
            current = parent[current]
        grid_path.reverse()
        route = [start] + grid_path + [goal]
        return self._shortcut_route(route, forest, clearance)

    @staticmethod
    def _append_unique(points, point):
        if not points:
            points.append(point)
            return
        previous = points[-1]
        distance = math.sqrt(
            (previous[0] - point[0]) ** 2
            + (previous[1] - point[1]) ** 2
            + (previous[2] - point[2]) ** 2
        )
        if distance > 1e-6:
            points.append(point)

    def _plan_safe_route(self, start, goal, forest, clearance):
        route_altitude = max(start[2], goal[2])
        horizontal_start = (start[0], start[1], route_altitude)
        horizontal_goal = (goal[0], goal[1], route_altitude)

        if self._segment_is_free(
            horizontal_start,
            horizontal_goal,
            forest,
            clearance,
        ):
            horizontal_route = [horizontal_start, horizontal_goal]
        else:
            nodes = self._route_grid(
                forest,
                horizontal_start,
                horizontal_goal,
                route_altitude,
                clearance,
            )
            horizontal_route = self._astar_route(
                horizontal_start,
                horizontal_goal,
                forest,
                nodes,
                clearance,
            )
            if horizontal_route is None:
                return None

        route = []
        if abs(start[2] - route_altitude) > 1e-6:
            self._append_unique(route, horizontal_start)
        for point in horizontal_route[1:]:
            self._append_unique(route, point)
        self._append_unique(route, goal)
        return route

    def _plan_mission_route(self, start, goal, forest):
        route = self._plan_safe_route(
            start, goal, forest, self.mission_route_clearance
        )
        used_clearance = self.mission_route_clearance
        if (
            route is None
            and self.mission_route_fallback_clearance
            < self.mission_route_clearance
        ):
            used_clearance = self.mission_route_fallback_clearance
            route = self._plan_safe_route(
                start, goal, forest, used_clearance
            )
            if route is not None:
                rospy.logwarn(
                    "[competition] Mission route required fallback "
                    "clearance %.2fm instead of %.2fm",
                    used_clearance,
                    self.mission_route_clearance,
                )
        return route, used_clearance

    def _shortcut_route(self, route, forest, clearance=None):
        if len(route) < 3:
            return route
        shortened = [route[0]]
        index = 0
        while index < len(route) - 1:
            next_index = len(route) - 1
            while next_index > index + 1:
                if self._segment_is_free(
                    route[index],
                    route[next_index],
                    forest,
                    clearance,
                ):
                    break
                next_index -= 1
            shortened.append(route[next_index])
            index = next_index
        return shortened

    def _route_planner_goals(self, goals, forest):
        if len(goals) < 2:
            return goals

        nodes = self._route_grid(forest)
        routed = [goals[0]]
        detour_count = 0
        inserted_count = 0
        for goal in goals[1:]:
            if self._segment_is_free(routed[-1], goal, forest):
                routed.append(goal)
                continue
            route = self._astar_route(routed[-1], goal, forest, nodes)
            if route is None:
                raise RuntimeError(
                    "no clearance-safe route between search goals "
                    "(%.3f, %.3f) and (%.3f, %.3f)"
                    % (
                        routed[-1][0],
                        routed[-1][1],
                        goal[0],
                        goal[1],
                    )
                )
            detour_count += 1
            inserted_count += max(0, len(route) - 2)
            routed.extend(route[1:])

        rospy.loginfo(
            "[competition] Search route repaired %d unsafe segments with "
            "%d detour goals at %.2fm clearance",
            detour_count,
            inserted_count,
            self.segment_clearance,
        )
        return routed

    def _generate_viewpoints(self, forest):
        coverage_path = []
        planner_goals = []
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
                x += self.lane_spacing
            if row_index % 2 == 1:
                row.reverse()
            coverage_path.extend(row)
            planner_goals.extend(self._planner_goals_for_row(row))
            row_index += 1
            y += self.lane_spacing
        return (
            coverage_path,
            self._route_planner_goals(planner_goals, forest),
        )

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

    def _prepend_safe_entry_route(self, current_position):
        if not self.viewpoints:
            return True
        start = (
            current_position.x,
            current_position.y,
            current_position.z,
        )
        goal = self.viewpoints[0]
        route, _used_clearance = self._plan_mission_route(
            start, goal, self.forest
        )
        if route is None:
            return False
        inserted = max(0, len(route) - 1)
        self.viewpoints = route + self.viewpoints[1:]
        if inserted:
            rospy.loginfo(
                "[competition] Search entry route inserted %d safe goals",
                inserted,
            )
        self._publish_path()
        return True

    def _reverse_paths(self):
        self.viewpoints.reverse()
        self.coverage_path.reverse()
        self._publish_path()

    def _orient_paths(self, current_position):
        if len(self.viewpoints) < 2:
            return

        first = self.viewpoints[0]
        last = self.viewpoints[-1]
        if self.path_direction == "bottom_to_top":
            if first[1] > last[1]:
                self._reverse_paths()
            return
        if self.path_direction == "top_to_bottom":
            if first[1] < last[1]:
                self._reverse_paths()
            return

        first_distance = math.hypot(
            current_position.x - first[0],
            current_position.y - first[1],
        )
        last_distance = math.hypot(
            current_position.x - last[0],
            current_position.y - last[1],
        )
        if last_distance < first_distance:
            self._reverse_paths()

    def _handle_request(self, request):
        response = NextSearchViewpointResponse()
        with self.lock:
            if self.forest is None:
                response.status = NextSearchViewpointResponse.PENDING
                response.detail = "forest_not_received"
                return response
            if not self.base_viewpoints:
                response.status = NextSearchViewpointResponse.PENDING
                response.detail = self.generation_error or "route_not_prepared"
                return response

            if request.reset:
                self.index = 0
                self.viewpoints = list(self.base_viewpoints)
                self._orient_paths(request.current_position)
                if not self._prepend_safe_entry_route(
                    request.current_position
                ):
                    response.status = NextSearchViewpointResponse.PENDING
                    response.detail = "safe_search_entry_route_unavailable"
                    return response

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
            response.detail = "planner_goal_%d_of_%d" % (
                self.index,
                len(self.viewpoints),
            )
            return response

    def _handle_route_request(self, request):
        response = PlanSafeRouteResponse()
        with self.lock:
            if self.forest is None:
                response.success = False
                response.detail = "forest_not_received"
                return response
            forest = self.forest

        start = (
            request.start.x,
            request.start.y,
            request.start.z,
        )
        goal = (
            request.goal.x,
            request.goal.y,
            request.goal.z,
        )
        route, used_clearance = self._plan_mission_route(
            start, goal, forest
        )
        if route is None:
            response.success = False
            response.detail = (
                "no_clearance_safe_route_from_"
                "%.2f_%.2f_to_%.2f_%.2f"
                % (start[0], start[1], goal[0], goal[1])
            )
            return response

        stamp = rospy.Time.now()
        for x, y, z in route:
            waypoint = PoseStamped()
            waypoint.header.stamp = stamp
            waypoint.header.frame_id = self.frame_id
            waypoint.pose.position.x = x
            waypoint.pose.position.y = y
            waypoint.pose.position.z = z
            waypoint.pose.orientation.w = 1.0
            response.waypoints.append(waypoint)
        response.success = True
        response.detail = "safe_route_with_%d_goals_at_%.2fm" % (
            len(route),
            used_clearance,
        )
        rospy.loginfo(
            "[competition] Safe route planned from "
            "(%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f): "
            "%d goals at %.2fm clearance",
            start[0],
            start[1],
            start[2],
            goal[0],
            goal[1],
            goal[2],
            len(route),
            used_clearance,
        )
        return response


if __name__ == "__main__":
    rospy.init_node("coverage_search_server")
    CoverageSearchServer()
    rospy.spin()
