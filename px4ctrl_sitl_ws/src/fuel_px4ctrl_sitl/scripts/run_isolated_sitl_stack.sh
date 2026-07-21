#!/usr/bin/env bash
set -euo pipefail

PX4_AUTOPILOT_DIR="${PX4_AUTOPILOT_DIR:-/root/catkin_ws/PX4-Autopilot-v1.15.4}"
PX4CTRL_SITL_WS="${PX4CTRL_SITL_WS:-/root/catkin_ws/px4ctrl_sitl_ws}"

source /opt/ros/noetic/setup.bash
source "${PX4CTRL_SITL_WS}/devel/setup.bash"

export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
source "${PX4_AUTOPILOT_DIR}/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
  "${PX4_AUTOPILOT_DIR}" "${PX4_AUTOPILOT_DIR}/build/px4_sitl_default"

export ROS_PACKAGE_PATH="${PX4_AUTOPILOT_DIR}:${PX4_AUTOPILOT_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic:${ROS_PACKAGE_PATH}"
export ROS_MASTER_URI="http://127.0.0.1:${PX4CTRL_SITL_ROS_PORT:-16666}"
export ROS_IP="127.0.0.1"
unset ROS_HOSTNAME

export ROS_HOME="${ROS_HOME:-/tmp/px4ctrl_sitl_ros_home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/px4ctrl_sitl_ros_log}"

exec roslaunch fuel_px4ctrl_sitl full_stack_isolated.launch "$@"
