#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="${ROOT_DIR:-/root/catkin_ws}"
PX4_AUTOPILOT_DIR="${PX4_AUTOPILOT_DIR:-${ROOT_DIR}/PX4-Autopilot-v1.15.4}"
PX4CTRL_SITL_WS="${PX4CTRL_SITL_WS:-${ROOT_DIR}/px4ctrl_sitl_ws}"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="${OUT_DIR:-${ROOT_DIR}/baseline/command_safety_sitl_${RUN_ID}}"
ROS_PORT="${COMMAND_SAFETY_ROS_PORT:-16668}"

mkdir -p "${OUT_DIR}"

source /opt/ros/noetic/setup.bash
source "${ROOT_DIR}/devel/setup.bash"
source "${PX4CTRL_SITL_WS}/devel/setup.bash"
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
source "${PX4_AUTOPILOT_DIR}/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
  "${PX4_AUTOPILOT_DIR}" "${PX4_AUTOPILOT_DIR}/build/px4_sitl_default"

export ROS_PACKAGE_PATH="${PX4_AUTOPILOT_DIR}:${PX4_AUTOPILOT_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic:${ROS_PACKAGE_PATH}"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP="127.0.0.1"
unset ROS_HOSTNAME
export ROS_HOME="${OUT_DIR}/ros_home"
export ROS_LOG_DIR="${OUT_DIR}/ros_logs"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"

BASE_LAUNCH_PID=""
BRIDGE_LAUNCH_PID=""
FUEL_LAUNCH_PID=""
BAG_PID=""

cleanup() {
  if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" >/dev/null 2>&1; then
    kill -INT "${BAG_PID}" >/dev/null 2>&1 || true
    wait "${BAG_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${FUEL_LAUNCH_PID}" ]] && kill -0 "${FUEL_LAUNCH_PID}" >/dev/null 2>&1; then
    kill -INT "${FUEL_LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${FUEL_LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${BRIDGE_LAUNCH_PID}" ]] && kill -0 "${BRIDGE_LAUNCH_PID}" >/dev/null 2>&1; then
    kill -INT "${BRIDGE_LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${BRIDGE_LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${BASE_LAUNCH_PID}" ]] && kill -0 "${BASE_LAUNCH_PID}" >/dev/null 2>&1; then
    kill -INT "${BASE_LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${BASE_LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

roslaunch fuel_px4ctrl_sitl command_safety_base_sitl.launch gui:=false \
  >"${OUT_DIR}/base_roslaunch.log" 2>&1 &
BASE_LAUNCH_PID=$!

for _ in $(seq 1 90); do
  if rostopic list >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

roslaunch fuel_px4ctrl_sitl command_safety_bridge_sitl.launch \
  >"${OUT_DIR}/bridge_roslaunch.log" 2>&1 &
BRIDGE_LAUNCH_PID=$!
sleep 1

rosbag record -O "${OUT_DIR}/command_safety.bag" \
  /fuel/position_cmd_raw \
  /fuel/position_cmd_test \
  /fuel_command_safety/state \
  /diagnostics \
  /sitl/setpoints_cmd \
  /sitl_test/fault_mode \
  /command_fault_injector/mode \
  /planning/bspline \
  /sitl/ground_truth/odom \
  /sitl/mavros/state \
  /sitl/mavros/extended_state \
  /sitl/mavros/setpoint_raw/attitude \
  >"${OUT_DIR}/rosbag_record.log" 2>&1 &
BAG_PID=$!

rosrun fuel_px4ctrl_sitl prepare_sitl_takeoff.py \
  >"${OUT_DIR}/takeoff_prepare.log" 2>&1

roslaunch fuel_px4ctrl_sitl fuel_planner_px4_sitl.launch \
  >"${OUT_DIR}/fuel_roslaunch.log" 2>&1 &
FUEL_LAUNCH_PID=$!
sleep 2

set +e
rosrun fuel_px4ctrl_sitl run_command_safety_scenarios.py \
  _output_file:="${OUT_DIR}/scenario_results.json" \
  _skip_takeoff:=true \
  >"${OUT_DIR}/scenario_runner.log" 2>&1
SCENARIO_STATUS=$?
set -e

RECOVERY_STATUS=125
if [[ "${SCENARIO_STATUS}" -eq 0 ]]; then
  # Return px4ctrl to AUTO_HOVER before discarding the old planner state.
  rostopic pub -1 /fuel_command_safety/enable std_msgs/Bool \
    '{data: false}' >"${OUT_DIR}/disable_bridge_before_recovery.log" 2>&1 || true
  sleep 1

  if [[ -n "${FUEL_LAUNCH_PID}" ]] && kill -0 "${FUEL_LAUNCH_PID}" >/dev/null 2>&1; then
    kill -INT "${FUEL_LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${FUEL_LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
  FUEL_LAUNCH_PID=""
  if [[ -n "${BRIDGE_LAUNCH_PID}" ]] && kill -0 "${BRIDGE_LAUNCH_PID}" >/dev/null 2>&1; then
    kill -INT "${BRIDGE_LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${BRIDGE_LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
  BRIDGE_LAUNCH_PID=""

  rostopic pub -1 /sitl_test/fault_mode std_msgs/String \
    '{data: pass}' >"${OUT_DIR}/reset_fault_mode.log" 2>&1 || true

  roslaunch fuel_px4ctrl_sitl command_safety_bridge_sitl.launch \
    >"${OUT_DIR}/recovery_bridge_roslaunch.log" 2>&1 &
  BRIDGE_LAUNCH_PID=$!
  sleep 1
  roslaunch fuel_px4ctrl_sitl fuel_planner_px4_sitl.launch \
    >"${OUT_DIR}/recovery_fuel_roslaunch.log" 2>&1 &
  FUEL_LAUNCH_PID=$!
  sleep 2

  set +e
  rosrun fuel_px4ctrl_sitl run_command_safety_scenarios.py \
    _output_file:="${OUT_DIR}/recovery_results.json" \
    _skip_takeoff:=true \
    _recovery_only:=true \
    >"${OUT_DIR}/recovery_runner.log" 2>&1
  RECOVERY_STATUS=$?
  set -e
fi

# Stop bridge output so px4ctrl returns to AUTO_HOVER before accepting LAND.
rostopic pub -1 /fuel_command_safety/enable std_msgs/Bool \
  '{data: false}' >"${OUT_DIR}/disable_bridge_before_land.log" 2>&1 || true
sleep 1
rostopic pub -1 /sitl/takeoff_land quadrotor_msgs/TakeoffLand \
  '{takeoff_land_cmd: 2}' >"${OUT_DIR}/land.log" 2>&1 || true
sleep 15

if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" >/dev/null 2>&1; then
  kill -INT "${BAG_PID}" >/dev/null 2>&1 || true
  wait "${BAG_PID}" >/dev/null 2>&1 || true
fi
BAG_PID=""

rosbag info "${OUT_DIR}/command_safety.bag" >"${OUT_DIR}/rosbag_info.txt" 2>&1 || true
FINAL_STATUS="${SCENARIO_STATUS}"
if [[ "${FINAL_STATUS}" -eq 0 && "${RECOVERY_STATUS}" -ne 0 ]]; then
  FINAL_STATUS="${RECOVERY_STATUS}"
fi
printf 'run_id=%s\nscenario_status=%s\nrecovery_status=%s\nfinal_status=%s\noutput=%s\n' \
  "${RUN_ID}" "${SCENARIO_STATUS}" "${RECOVERY_STATUS}" "${FINAL_STATUS}" \
  "${OUT_DIR}" >"${OUT_DIR}/summary.txt"

exit "${FINAL_STATUS}"
