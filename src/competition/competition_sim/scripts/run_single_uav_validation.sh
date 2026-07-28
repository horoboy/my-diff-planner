#!/usr/bin/env bash

set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
PACKAGE_DIR="$(cd "$(dirname "${SCRIPT_PATH}")/.." && pwd)"
WORKSPACE_DIR="$(cd "${PACKAGE_DIR}/../../.." && pwd)"
FUEL_WORKSPACE_DIR="${FUEL_WORKSPACE_DIR:-$(cd "${WORKSPACE_DIR}/.." && pwd)}"

source /opt/ros/noetic/setup.bash
source "${FUEL_WORKSPACE_DIR}/devel/setup.bash"
source "${WORKSPACE_DIR}/devel/setup.bash" --extend

ROS_PORT="${ROS_PORT:-11321}"
RESULT_FILE="${RESULT_FILE:-/tmp/competition_single_result_$(date +%Y%m%dT%H%M%S).json}"
LOG_FILE="${LOG_FILE:-${RESULT_FILE%.json}.log}"
MISSION_TIMEOUT="${MISSION_TIMEOUT:-180}"
SEARCH_BACKEND="${SEARCH_BACKEND:-coverage}"

export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
export ROS_HOME="${ROS_HOME:-/tmp/competition_ros_home_${ROS_PORT}}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/competition_ros_logs_${ROS_PORT}}"
export FUEL_LD_LIBRARY_PATH="${FUEL_WORKSPACE_DIR}/devel/lib:/opt/ros/noetic/lib"

if timeout 1 rosnode list >/dev/null 2>&1; then
  echo "ERROR: a ROS master is already running at ${ROS_MASTER_URI}" >&2
  exit 2
fi

mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"

LAUNCH_PID=""
cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Starting single-UAV competition simulation"
echo "  ROS master: ${ROS_MASTER_URI}"
echo "  result:     ${RESULT_FILE}"
echo "  log:        ${LOG_FILE}"

roslaunch competition_sim single_uav_competition.launch \
  auto_start:=true \
  start_rviz:=false \
  search_backend:="${SEARCH_BACKEND}" \
  result_file:="${RESULT_FILE}" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

deadline=$((SECONDS + MISSION_TIMEOUT))
while [[ ! -s "${RESULT_FILE}" ]]; do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "ERROR: simulation exited before producing a result" >&2
    tail -n 80 "${LOG_FILE}" >&2
    exit 3
  fi
  if (( SECONDS >= deadline )); then
    echo "ERROR: mission timed out after ${MISSION_TIMEOUT}s" >&2
    tail -n 80 "${LOG_FILE}" >&2
    exit 4
  fi
  sleep 1
done

sleep 0.2
python3 - "${RESULT_FILE}" <<'PY'
import json
import sys

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as stream:
    result = json.load(stream)

print("Mission result")
for key in (
    "success",
    "reason",
    "final_state",
    "duration",
    "goal_sequence",
    "collision_count",
    "min_obstacle_clearance",
    "drop_error",
):
    print("  %s: %s" % (key, result.get(key)))

sys.exit(0 if result.get("success") else 1)
PY
