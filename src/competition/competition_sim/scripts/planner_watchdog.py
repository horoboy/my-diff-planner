#!/usr/bin/env python3

import threading

import rosnode
import rospy
from std_msgs.msg import Empty, String, UInt32


class PlannerWatchdog:
    def __init__(self):
        self.node_name = rospy.get_param(
            "~planner_node", "/drone_0_diff_planner_node"
        )
        self.heartbeat_topic = rospy.get_param(
            "~heartbeat_topic", "/drone_0_traj_server/heartbeat"
        )
        self.heartbeat_timeout = float(
            rospy.get_param("~heartbeat_timeout", 0.8)
        )
        self.recovery_cooldown = float(
            rospy.get_param("~recovery_cooldown", 2.0)
        )
        self.max_recoveries = int(rospy.get_param("~max_recoveries", 3))

        self.lock = threading.Lock()
        self.last_heartbeat = None
        self.last_recovery = rospy.Time(0)
        self.recovery_count = 0
        self.waiting_for_restart = False

        self.count_pub = rospy.Publisher(
            "/competition/planner_watchdog/recovery_count",
            UInt32,
            queue_size=1,
            latch=True,
        )
        self.state_pub = rospy.Publisher(
            "/competition/planner_watchdog/state",
            String,
            queue_size=1,
            latch=True,
        )
        self.heartbeat_sub = rospy.Subscriber(
            self.heartbeat_topic,
            Empty,
            self._heartbeat_callback,
            queue_size=10,
        )
        self.timer = rospy.Timer(rospy.Duration(0.1), self._check)
        self._publish("WAITING_FOR_FIRST_HEARTBEAT")

    def _publish(self, state):
        self.count_pub.publish(UInt32(data=self.recovery_count))
        self.state_pub.publish(String(data=state))

    def _heartbeat_callback(self, _msg):
        with self.lock:
            restarted = self.waiting_for_restart
            self.last_heartbeat = rospy.Time.now()
            self.waiting_for_restart = False
            if restarted:
                rospy.logwarn(
                    "[competition] Planner heartbeat restored after recovery %d",
                    self.recovery_count,
                )
            self._publish("HEALTHY")

    def _check(self, _event):
        with self.lock:
            now = rospy.Time.now()
            if self.last_heartbeat is None or self.waiting_for_restart:
                return
            if (
                now - self.last_heartbeat
            ).to_sec() <= self.heartbeat_timeout:
                return
            if (
                now - self.last_recovery
            ).to_sec() < self.recovery_cooldown:
                return
            if self.recovery_count >= self.max_recoveries:
                self._publish("RECOVERY_LIMIT_REACHED")
                rospy.logerr_throttle(
                    5.0,
                    "[competition] Planner heartbeat lost and recovery "
                    "limit %d reached",
                    self.max_recoveries,
                )
                return

            self.last_recovery = now
            age = (now - self.last_heartbeat).to_sec()
            successful, failed = rosnode.kill_nodes([self.node_name])
            if self.node_name not in successful:
                rospy.logerr(
                    "[competition] Failed to stop unresponsive planner %s: %s",
                    self.node_name,
                    failed,
                )
                self._publish("RECOVERY_REQUEST_FAILED")
                return

            self.recovery_count += 1
            self.waiting_for_restart = True
            self._publish("WAITING_FOR_RESPAWN")
            rospy.logerr(
                "[competition] Planner heartbeat stale for %.3f s; "
                "requested restart %d/%d",
                age,
                self.recovery_count,
                self.max_recoveries,
            )


if __name__ == "__main__":
    rospy.init_node("competition_planner_watchdog")
    PlannerWatchdog()
    rospy.spin()
