#!/usr/bin/env python3

import sys
import time

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval


MESSAGES = (
    (105, "HIGHRES_IMU"),
    (31, "ATTITUDE_QUATERNION"),
    (32, "LOCAL_POSITION_NED"),
)


def main():
    rospy.init_node("configure_sitl_streams")
    service_name = rospy.get_param(
        "~service", "/sitl/mavros/set_message_interval"
    )
    state_topic = rospy.get_param("~state_topic", "/sitl/mavros/state")
    message_rate = float(rospy.get_param("~message_rate", 250.0))
    wait_timeout = float(rospy.get_param("~wait_timeout", 30.0))

    try:
        deadline = time.monotonic() + wait_timeout
        while not rospy.is_shutdown():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise rospy.ROSException("timed out waiting for FCU connection")
            try:
                state = rospy.wait_for_message(
                    state_topic, State, timeout=min(1.0, remaining)
                )
            except rospy.ROSException:
                continue
            if state.connected:
                break

        rospy.wait_for_service(service_name, timeout=wait_timeout)
        set_interval = rospy.ServiceProxy(service_name, MessageInterval)
        for message_id, message_name in MESSAGES:
            response = set_interval(message_id, message_rate)
            if not response.success:
                raise RuntimeError(
                    "PX4 rejected {} ({})".format(message_name, message_id)
                )
            rospy.loginfo(
                "Configured %s (%d) at %.1f Hz",
                message_name,
                message_id,
                message_rate,
            )
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as exc:
        rospy.logerr("Unable to configure SITL MAVLink streams: %s", exc)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
