#!/usr/bin/env bash
set -euo pipefail

export ROS_MASTER_URI="http://127.0.0.1:${PX4CTRL_SITL_ROS_PORT:-16666}"
export ROS_IP="127.0.0.1"
unset ROS_HOSTNAME

exec roslaunch fuel_px4ctrl_sitl px4ctrl_isolated.launch "$@"
