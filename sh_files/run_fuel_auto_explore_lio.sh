#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIFF_WS="${DIFF_WS:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
FUEL_WS="${FUEL_WS:-/home/nv/fuel_flight_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
DIFF_SETUP="${DIFF_SETUP:-${DIFF_WS}/devel/setup.bash}"
FUEL_SETUP="${FUEL_SETUP:-${FUEL_WS}/devel/setup.bash}"

EXPECTED_FUEL_COMMIT="${EXPECTED_FUEL_COMMIT:-38542058f17ba890d254f4a115890857de14df25}"
EXPECTED_POSITION_COMMAND_MD5="${EXPECTED_POSITION_COMMAND_MD5:-2809eb0c779bbce5b8d66b95a05bd27b}"
MIN_EPOCH="${MIN_EPOCH:-1700000000}"
MIN_BATTERY_VOLTAGE="${MIN_BATTERY_VOLTAGE:-22.0}"
ORIGIN_XY_LIMIT="${ORIGIN_XY_LIMIT:-1.0}"
TAKEOFF_TIMEOUT="${TAKEOFF_TIMEOUT:-45.0}"
PLANNER_TIMEOUT="${PLANNER_TIMEOUT:-25.0}"
LAND_TIMEOUT="${LAND_TIMEOUT:-60.0}"
FUEL_CH7_ARM_POSITION="${FUEL_CH7_ARM_POSITION:-}"
START_RVIZ="${START_RVIZ:-true}"

ROS_MASTER_URI="http://localhost:11311"
export ROS_MASTER_URI
unset ROS_IP
unset ROS_HOSTNAME

STATE_FILE="${STATE_FILE:-/tmp/fuel_auto_explore_${UID:-$(id -u)}.pids}"
RUN_DIR=""
MODE=""
cleanup_needed=false
flight_command_sent=false

declare -a STARTED_NAMES=()
declare -a STARTED_PIDS=()

log() {
  printf '[fuel-auto] %s\n' "$*"
}

warn() {
  printf '[fuel-auto] WARNING: %s\n' "$*" >&2
}

die() {
  printf '[fuel-auto] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage:
  run_fuel_auto_explore_lio.sh --execute (--ch7-high | --ch7-low)
  run_fuel_auto_explore_lio.sh --land (--ch7-high | --ch7-low)
  run_fuel_auto_explore_lio.sh --stop

--execute  Start the complete real-flight stack, auto-take off, then trigger FUEL.
--land     Stop FUEL commands, return PX4Ctrl to hover, then command auto-land.
--ch7-high Declare that CH7 > 1750 is this aircraft's arm-enabled position.
--ch7-low  Declare that CH7 < 1250 is this aircraft's arm-enabled position.
--stop     Stop processes started by this script. Refuses while the FCU is armed.

Optional environment:
  START_RVIZ=true   Start the read-only FUEL real-flight visualization (default).
  START_RVIZ=false  Do not start RViz, for example during a headless SSH run.

Required RC positions before --execute:
  CH1-CH4 centered, CH5=1999, CH6=1999, CH7=arm-enabled, CH8=999.

The CH7 direction must be stated explicitly for every real-flight run. It can
also be supplied through FUEL_CH7_ARM_POSITION=high or low.

For --land, first set CH5=1999 and CH6=999. The script stops FUEL's command
publisher and then asks for CH6=1999 before it sends the landing command.
EOF
}

while (($#)); do
  case "$1" in
    --execute)
      [[ -z "${MODE}" ]] || die "choose only one mode"
      MODE="execute"
      ;;
    --stop)
      [[ -z "${MODE}" ]] || die "choose only one mode"
      MODE="stop"
      ;;
    --land)
      [[ -z "${MODE}" ]] || die "choose only one mode"
      MODE="land"
      ;;
    --ch7-high)
      [[ -z "${FUEL_CH7_ARM_POSITION}" || "${FUEL_CH7_ARM_POSITION}" == "high" ]] ||
        die "conflicting CH7 arm positions"
      FUEL_CH7_ARM_POSITION="high"
      ;;
    --ch7-low)
      [[ -z "${FUEL_CH7_ARM_POSITION}" || "${FUEL_CH7_ARM_POSITION}" == "low" ]] ||
        die "conflicting CH7 arm positions"
      FUEL_CH7_ARM_POSITION="low"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      die "unknown argument: $1"
      ;;
  esac
  shift
done

[[ -n "${MODE}" ]] || {
  usage >&2
  exit 2
}

if [[ "${MODE}" == "execute" || "${MODE}" == "land" ]]; then
  [[ "${FUEL_CH7_ARM_POSITION}" == "high" || "${FUEL_CH7_ARM_POSITION}" == "low" ]] ||
    die "--${MODE} requires --ch7-high or --ch7-low"
fi

if [[ "${MODE}" == "execute" ]]; then
  [[ "${START_RVIZ}" == "true" || "${START_RVIZ}" == "false" ]] ||
    die "START_RVIZ must be true or false"
  if [[ "${START_RVIZ}" == "true" ]]; then
    [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]] ||
      die "START_RVIZ=true but no graphical display is available; use START_RVIZ=false for headless runs"
  fi
fi

require_file() {
  [[ -f "$1" ]] || die "missing file: $1"
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "missing command: $1"
}

require_file "${ROS_SETUP}"
require_file "${DIFF_SETUP}"

# Keep the parent process on Diff-Planner's message definitions. The FUEL launch
# is started in a dedicated child environment because both workspaces provide a
# package named quadrotor_msgs.
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${DIFF_SETUP}"

export ROS_MASTER_URI
unset ROS_IP
unset ROS_HOSTNAME

vehicle_armed_state() {
  timeout 3 python3 - <<'PY'
import rospy
from mavros_msgs.msg import State

rospy.init_node("fuel_auto_armed_check", anonymous=True, disable_signals=True)
try:
    state = rospy.wait_for_message("/mavros/state", State, timeout=2.0)
except Exception:
    raise SystemExit(2)
print("true" if state.armed else "false")
PY
}

stop_saved_processes() {
  [[ -f "${STATE_FILE}" ]] || die "no saved run state: ${STATE_FILE}"

  local live=false
  local name pid
  while read -r name pid; do
    [[ -n "${pid:-}" ]] || continue
    if kill -0 "${pid}" 2>/dev/null; then
      live=true
      break
    fi
  done < "${STATE_FILE}"

  if [[ "${live}" == false ]]; then
    rm -f "${STATE_FILE}"
    log "removed stale state file; no managed process is running"
    return
  fi

  local armed
  armed="$(vehicle_armed_state 2>/dev/null || true)"
  [[ "${armed}" == "false" ]] || die "cannot verify armed=false; refusing to stop the flight stack"

  mapfile -t saved_lines < "${STATE_FILE}"
  for ((i=${#saved_lines[@]} - 1; i >= 0; --i)); do
    read -r name pid <<<"${saved_lines[i]}"
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    if kill -0 "${pid}" 2>/dev/null; then
      log "stopping ${name} (process group ${pid})"
      kill -INT -- "-${pid}" 2>/dev/null || true
    fi
  done

  local deadline=$((SECONDS + 8))
  while ((SECONDS < deadline)); do
    live=false
    for line in "${saved_lines[@]}"; do
      read -r name pid <<<"${line}"
      [[ "${pid}" =~ ^[0-9]+$ ]] || continue
      if kill -0 "${pid}" 2>/dev/null; then
        live=true
        break
      fi
    done
    [[ "${live}" == false ]] && break
    sleep 0.5
  done

  local remaining=false
  for line in "${saved_lines[@]}"; do
    read -r name pid <<<"${line}"
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    if kill -0 "${pid}" 2>/dev/null; then
      remaining=true
      warn "${name} is still running as PID ${pid}; inspect it manually"
    fi
  done
  [[ "${remaining}" == false ]] || die "some managed processes did not stop; state file retained"
  rm -f "${STATE_FILE}"
  log "managed stack stopped"
}

managed_component_pid() {
  local wanted_name="$1"
  local name pid

  [[ -f "${STATE_FILE}" ]] || return 1
  while read -r name pid; do
    if [[ "${name}" == "${wanted_name}" && "${pid:-}" =~ ^[0-9]+$ ]]; then
      printf '%s\n' "${pid}"
      return 0
    fi
  done < "${STATE_FILE}"
  return 1
}

stop_managed_component() {
  local name="$1"
  local pid

  pid="$(managed_component_pid "${name}" || true)"
  [[ -n "${pid}" ]] || die "managed component is not recorded: ${name}"
  if ! kill -0 "${pid}" 2>/dev/null; then
    log "${name} is already stopped"
    return
  fi

  log "stopping ${name} command publisher (process group ${pid})"
  kill -INT -- "-${pid}" 2>/dev/null || die "failed to signal ${name}"
}

land_saved_stack() {
  [[ -f "${STATE_FILE}" ]] || die "no saved run state: ${STATE_FILE}"
  for command in rosnode rosmsg timeout python3; do
    require_command "${command}"
  done
  rosmsg show quadrotor_msgs/TakeoffLand >/dev/null ||
    die "TakeoffLand message is unavailable in the active workspace"

  for node in /mavros /ekf /px4ctrl; do
    rosnode list 2>/dev/null | grep -Fxq "${node}" ||
      die "required landing node is missing: ${node}"
  done

  cat <<'EOF'

Landing phase 1:
  Keep CH5=1999 and move CH6 to 999 (forced hover).
  Keep CH7 in its arm-enabled position and center all sticks.
  The script will wait for these positions before stopping FUEL commands.
EOF

  timeout 35 python3 - "${FUEL_CH7_ARM_POSITION}" <<'PY'
import math
import sys
import time

import rospy
from mavros_msgs.msg import RCIn, State
from nav_msgs.msg import Odometry

ch7_mode = sys.argv[1]
rospy.init_node("fuel_auto_land_hover_gate", anonymous=True, disable_signals=True)
deadline = time.monotonic() + 30.0
stable_since = None
last_report = 0.0

while time.monotonic() < deadline:
    try:
        state = rospy.wait_for_message("/mavros/state", State, timeout=0.5)
        rc = rospy.wait_for_message("/mavros/rc/in", RCIn, timeout=0.5)
        odom = rospy.wait_for_message("/ekf/ekf_odom", Odometry, timeout=0.5)
    except rospy.ROSException:
        continue

    channels = list(rc.channels)
    values = (
        odom.pose.pose.position.x,
        odom.pose.pose.position.y,
        odom.pose.pose.position.z,
        odom.twist.twist.linear.x,
        odom.twist.twist.linear.y,
        odom.twist.twist.linear.z,
    )
    odom_age = (rospy.Time.now() - odom.header.stamp).to_sec()
    ch7_ok = (
        len(channels) >= 8
        and ((ch7_mode == "high" and channels[6] > 1750)
             or (ch7_mode == "low" and channels[6] < 1250))
    )
    switches_ok = (
        len(channels) >= 8
        and all(1375 <= channels[index] <= 1625 for index in range(4))
        and channels[4] > 1750
        and channels[5] < 1250
        and ch7_ok
    )
    speed = math.sqrt(sum(value * value for value in values[3:]))
    ready = (
        state.connected
        and state.armed
        and state.mode == "OFFBOARD"
        and switches_ok
        and -0.05 <= odom_age <= 0.25
        and all(math.isfinite(value) for value in values)
        and speed <= 0.20
    )
    now = time.monotonic()
    if ready:
        if stable_since is None:
            stable_since = now
        elif now - stable_since >= 1.0:
            print("Forced-hover gate passed: z=%.3fm speed=%.3fm/s" %
                  (values[2], speed), flush=True)
            raise SystemExit(0)
    else:
        stable_since = None

    if now - last_report >= 2.0:
        print("Waiting for armed OFFBOARD, CH5=1999, CH6=999, centered sticks; "
              "mode=%s armed=%s channels=%s odom_age=%.3fs" %
              (state.mode, state.armed, channels[:8], odom_age), flush=True)
        last_report = now
    time.sleep(0.05)

raise SystemExit("forced-hover landing gate was not satisfied within 30 seconds")
PY

  stop_managed_component fuel

  timeout 15 python3 - <<'PY'
import time

import rosgraph
import rospy

rospy.init_node("fuel_auto_land_command_gap", anonymous=True, disable_signals=True)
master = rosgraph.Master(rospy.get_name())
deadline = time.monotonic() + 12.0
clear_since = None

while time.monotonic() < deadline:
    try:
        publishers_raw, _, _ = master.getSystemState()
    except Exception:
        time.sleep(0.1)
        continue
    publishers = {topic: nodes for topic, nodes in publishers_raw}
    command_publishers = publishers.get("/setpoints_cmd", [])
    fuel_nodes = {
        "/cloud_pose_adapter",
        "/exploration_node",
        "/fuel_command_safety",
        "/traj_server",
        "/waypoint_generator",
    }
    running_nodes = {node for nodes in publishers.values() for node in nodes}
    clear = not command_publishers and not (fuel_nodes & running_nodes)
    now = time.monotonic()
    if clear:
        if clear_since is None:
            clear_since = now
        elif now - clear_since >= 1.0:
            print("/setpoints_cmd has been silent for more than 1 second", flush=True)
            raise SystemExit(0)
    else:
        clear_since = None
    time.sleep(0.1)

raise SystemExit("FUEL command publisher did not stop; PX4Ctrl remains in hover")
PY

  cat <<'EOF'

Landing phase 2:
  FUEL commands are stopped and PX4Ctrl is holding.
  Now move CH6 back to 1999. Keep CH5=1999, CH7 arm-enabled, and sticks centered.
  The landing command will be sent automatically after the switches are stable.
  Do not move CH6 back to 999 during AUTO_LAND; that cancels auto-land.
EOF

  timeout 120 python3 - "${FUEL_CH7_ARM_POSITION}" "${LAND_TIMEOUT}" <<'PY'
import math
import sys
import threading
import time

import rospy
from mavros_msgs.msg import RCIn, State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import TakeoffLand

ch7_mode = sys.argv[1]
land_timeout = float(sys.argv[2])
rospy.init_node("fuel_auto_land_command", anonymous=True, disable_signals=True)
lock = threading.Lock()
latest = {}

def save(name):
    def callback(message):
        with lock:
            latest[name] = message
    return callback

subs = [
    rospy.Subscriber("/mavros/state", State, save("state"), queue_size=20),
    rospy.Subscriber("/mavros/rc/in", RCIn, save("rc"), queue_size=20),
    rospy.Subscriber("/ekf/ekf_odom", Odometry, save("odom"), queue_size=100),
]
publisher = rospy.Publisher("/px4ctrl/takeoff_land", TakeoffLand, queue_size=1)

def controls_ready(state, rc, odom):
    channels = list(rc.channels)
    ch7_ok = (
        len(channels) >= 8
        and ((ch7_mode == "high" and channels[6] > 1750)
             or (ch7_mode == "low" and channels[6] < 1250))
    )
    odom_age = (rospy.Time.now() - odom.header.stamp).to_sec()
    values = (
        odom.pose.pose.position.x,
        odom.pose.pose.position.y,
        odom.pose.pose.position.z,
        odom.twist.twist.linear.x,
        odom.twist.twist.linear.y,
        odom.twist.twist.linear.z,
    )
    speed = math.sqrt(sum(value * value for value in values[3:]))
    return (
        state.connected
        and state.armed
        and state.mode == "OFFBOARD"
        and len(channels) >= 8
        and all(1375 <= channels[index] <= 1625 for index in range(4))
        and channels[4] > 1750
        and channels[5] > 1750
        and ch7_ok
        and -0.05 <= odom_age <= 0.25
        and all(math.isfinite(value) for value in values)
        and speed <= 0.20
    )

deadline = time.monotonic() + 40.0
stable_since = None
last_report = 0.0
while time.monotonic() < deadline:
    with lock:
        state = latest.get("state")
        rc = latest.get("rc")
        odom = latest.get("odom")
    if state is not None and not state.armed:
        print("FCU is already disarmed", flush=True)
        raise SystemExit(0)
    ready = (
        state is not None
        and rc is not None
        and odom is not None
        and controls_ready(state, rc, odom)
        and publisher.get_num_connections() > 0
    )
    now = time.monotonic()
    if ready:
        if stable_since is None:
            stable_since = now
        elif now - stable_since >= 1.0:
            break
    else:
        stable_since = None
    if now - last_report >= 2.0:
        channels = list(rc.channels)[:8] if rc is not None else []
        mode = state.mode if state is not None else "missing"
        armed = state.armed if state is not None else "missing"
        print("Waiting for CH6=1999 landing gate: mode=%s armed=%s channels=%s" %
              (mode, armed, channels), flush=True)
        last_report = now
    time.sleep(0.05)
else:
    raise SystemExit("landing command gate was not satisfied within 40 seconds")

command = TakeoffLand()
command.takeoff_land_cmd = TakeoffLand.LAND
for _ in range(5):
    publisher.publish(command)
    time.sleep(0.10)
print("LAND command sent; keep CH5=1999 and CH6=1999 until armed=false", flush=True)

deadline = time.monotonic() + land_timeout
while time.monotonic() < deadline:
    with lock:
        state = latest.get("state")
        rc = latest.get("rc")
        odom = latest.get("odom")
    if state is None:
        time.sleep(0.05)
        continue
    if not state.armed:
        print("Landing complete: armed=false", flush=True)
        raise SystemExit(0)
    if not state.connected:
        raise SystemExit("MAVROS disconnected during auto-land")
    if rc is None or odom is None:
        raise SystemExit("RC or EKF odometry disappeared during auto-land")
    channels = list(rc.channels)
    if len(channels) < 8 or channels[4] <= 1750:
        raise SystemExit("CH5 left hover mode during auto-land; control may be manual")
    if channels[5] <= 1750:
        raise SystemExit("CH6 left command mode; PX4Ctrl cancels auto-land in this position")
    odom_age = (rospy.Time.now() - odom.header.stamp).to_sec()
    if odom_age < -0.05 or odom_age > 0.25:
        raise SystemExit("EKF odometry became stale during auto-land")
    time.sleep(0.10)

raise SystemExit("auto-land did not reach armed=false within %.1f seconds" % land_timeout)
PY

  log "landing and disarm confirmed; stopping the managed stack"
  stop_saved_processes
}

if [[ "${MODE}" == "stop" ]]; then
  stop_saved_processes
  exit 0
fi

if [[ "${MODE}" == "land" ]]; then
  land_saved_stack
  exit 0
fi

require_file "${FUEL_SETUP}"
for command in roscore roslaunch rosnode rostopic rosrun rosmsg rospack rosparam timeout setsid python3 git; do
  require_command "${command}"
done

if [[ -f "${STATE_FILE}" ]]; then
  while read -r _ pid; do
    if [[ "${pid:-}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
      die "a managed stack is already running; use --stop only after landing and disarming"
    fi
  done < "${STATE_FILE}"
  rm -f "${STATE_FILE}"
fi

stop_started_processes() {
  for ((i=${#STARTED_PIDS[@]} - 1; i >= 0; --i)); do
    local pid="${STARTED_PIDS[i]}"
    local name="${STARTED_NAMES[i]}"
    if kill -0 "${pid}" 2>/dev/null; then
      log "stopping ${name} (process group ${pid})"
      kill -INT -- "-${pid}" 2>/dev/null || true
    fi
  done
  sleep 3

  local remaining=false
  for ((i=${#STARTED_PIDS[@]} - 1; i >= 0; --i)); do
    if kill -0 "${STARTED_PIDS[i]}" 2>/dev/null; then
      remaining=true
      warn "${STARTED_NAMES[i]} is still running as PID ${STARTED_PIDS[i]}"
    fi
  done
  if [[ "${remaining}" == false ]]; then
    rm -f "${STATE_FILE}"
  else
    warn "state file retained for manual inspection: ${STATE_FILE}"
  fi
}

on_exit() {
  local rc=$?
  trap - EXIT
  if [[ "${cleanup_needed}" == true ]]; then
    if [[ "${flight_command_sent}" == true ]]; then
      warn "a takeoff command was sent; leaving all control processes running"
      warn "use CH6=999 for forced hover, CH5=1499 for manual takeover"
      warn "for controlled landing, run from another terminal: $0 --land --ch7-${FUEL_CH7_ARM_POSITION}"
      warn "after any manual landing and armed=false, run: $0 --stop"
    else
      stop_started_processes
    fi
  fi
  exit "${rc}"
}

trap on_exit EXIT
trap 'exit 130' INT TERM HUP

start_component() {
  local name="$1"
  shift
  local logfile="${RUN_DIR}/${name}.log"

  log "starting ${name}; log: ${logfile}"
  setsid "$@" >"${logfile}" 2>&1 < /dev/null &
  local pid=$!
  STARTED_NAMES+=("${name}")
  STARTED_PIDS+=("${pid}")
  printf '%s %s\n' "${name}" "${pid}" >> "${STATE_FILE}"

  sleep 0.5
  if ! kill -0 "${pid}" 2>/dev/null; then
    tail -n 40 "${logfile}" >&2 || true
    die "${name} exited during startup"
  fi
}

wait_for_master() {
  local timeout_sec="$1"
  local deadline=$((SECONDS + timeout_sec))
  until rosnode list >/dev/null 2>&1; do
    ((SECONDS < deadline)) || die "ROS master did not become available"
    sleep 0.5
  done
}

wait_for_node() {
  local node="$1"
  local timeout_sec="$2"
  local deadline=$((SECONDS + timeout_sec))
  until rosnode list 2>/dev/null | grep -Fxq "${node}"; do
    ((SECONDS < deadline)) || die "node did not appear: ${node}"
    sleep 0.5
  done
}

wait_for_topic_message() {
  local topic="$1"
  local timeout_sec="$2"
  timeout "${timeout_sec}" rostopic echo -n 1 "${topic}" >/dev/null 2>&1 ||
    die "no message received from ${topic} within ${timeout_sec}s"
}

epoch="$(date +%s)"
[[ "${epoch}" =~ ^[0-9]+$ ]] || die "cannot read system epoch"
((epoch >= MIN_EPOCH)) || die "system time is not valid: epoch=${epoch}"

[[ -e /dev/ttyTHS1 ]] || die "/dev/ttyTHS1 does not exist"
[[ -r /dev/ttyTHS1 && -w /dev/ttyTHS1 ]] || die "current user cannot read/write /dev/ttyTHS1"
id -nG | tr ' ' '\n' | grep -Fxq dialout || die "current user is not in group dialout"

fuel_repo="${FUEL_WS}/src/FUEL"
git -C "${fuel_repo}" rev-parse --is-inside-work-tree >/dev/null 2>&1 ||
  die "FUEL source is not a Git worktree: ${fuel_repo}"
fuel_head="$(git -C "${fuel_repo}" rev-parse HEAD)"
[[ "${fuel_head}" == "${EXPECTED_FUEL_COMMIT}" ]] ||
  die "unexpected FUEL commit: ${fuel_head}"
[[ -z "$(git -C "${fuel_repo}" status --short)" ]] || die "FUEL worktree is not clean"

quadrotor_path="$(rospack find quadrotor_msgs)"
expected_quadrotor_path="${DIFF_WS}/src/Utils/quadrotor_msgs"
[[ "$(readlink -f "${quadrotor_path}")" == "$(readlink -f "${expected_quadrotor_path}")" ]] ||
  die "Diff-Planner quadrotor_msgs is not active: ${quadrotor_path}"

position_command_md5="$(rosmsg md5 quadrotor_msgs/PositionCommand)"
[[ "${position_command_md5}" == "${EXPECTED_POSITION_COMMAND_MD5}" ]] ||
  die "unexpected PositionCommand MD5: ${position_command_md5}"
rosmsg show quadrotor_msgs/TakeoffLand >/dev/null || die "TakeoffLand message is unavailable"

RUN_DIR="${HOME}/.ros/fuel_auto_explore/$(date +%Y%m%dT%H%M%S)"
mkdir -p "${RUN_DIR}"
: > "${STATE_FILE}"
cleanup_needed=true

if rosnode list >/dev/null 2>&1; then
  existing_nodes="$(rosnode list 2>/dev/null || true)"
  printf '[fuel-auto] ERROR: an existing ROS master was found. Stop the old graph first.\n' >&2
  printf '[fuel-auto] Existing nodes:\n%s\n' "${existing_nodes:-<none reported>}" >&2
  exit 1
fi

start_component roscore roscore
wait_for_master 10

if [[ "$(rosparam get /use_sim_time 2>/dev/null || printf false)" == "true" ]]; then
  die "/use_sim_time is true; real flight requires wall-clock time"
fi

start_component mavros roslaunch mavros px4.launch
wait_for_node /mavros 20

timeout 15 python3 - <<'PY'
import time

import rospy
from mavros_msgs.msg import State

rospy.init_node("fuel_auto_wait_mavros", anonymous=True, disable_signals=True)
deadline = time.monotonic() + 12.0
while not rospy.is_shutdown() and time.monotonic() < deadline:
    try:
        state = rospy.wait_for_message("/mavros/state", State, timeout=1.0)
    except Exception:
        continue
    if state.connected:
        if state.armed:
            raise SystemExit("FCU is already armed")
        print("MAVROS connected, mode=%s, armed=false" % state.mode)
        raise SystemExit(0)
raise SystemExit("MAVROS did not connect to the FCU")
PY

for message_id in 31 105 83 147 106; do
  timeout 8 rosrun mavros mavcmd long 511 "${message_id}" 5000 0 0 0 0 0 >/dev/null ||
    die "failed to configure MAVLink message ${message_id}"
done

start_component faster_lio roslaunch faster_lio mapping_mid360.launch
wait_for_node /laserMapping 30
wait_for_topic_message /laserMapping/odometry 30
wait_for_topic_message /laserMapping/cloud_registered 30

start_component ekf roslaunch ekf ekf_lidar.launch
wait_for_node /ekf 20
wait_for_topic_message /ekf/ekf_odom 20

log "waiting for base sensors and required RC switch positions"
python3 - "${MIN_BATTERY_VOLTAGE}" "${ORIGIN_XY_LIMIT}" "${FUEL_CH7_ARM_POSITION}" <<'PY'
import math
import sys
import threading
import time

import rospy
from mavros_msgs.msg import RCIn, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, PointCloud2

min_voltage = float(sys.argv[1])
origin_limit = float(sys.argv[2])
ch7_mode = sys.argv[3]
if ch7_mode not in ("high", "low"):
    raise SystemExit("FUEL_CH7_ARM_POSITION must be high or low")

rospy.init_node("fuel_auto_base_preflight", anonymous=True, disable_signals=True)
lock = threading.Lock()
latest = {}
counts = {}

def callback(name):
    def inner(message):
        with lock:
            latest[name] = message
            counts[name] = counts.get(name, 0) + 1
    return inner

subs = [
    rospy.Subscriber("/mavros/state", State, callback("state"), queue_size=10),
    rospy.Subscriber("/mavros/battery", BatteryState, callback("battery"), queue_size=10),
    rospy.Subscriber("/mavros/imu/data", Imu, callback("imu"), queue_size=100),
    rospy.Subscriber("/mavros/rc/in", RCIn, callback("rc"), queue_size=20),
    rospy.Subscriber("/ekf/ekf_odom", Odometry, callback("odom"), queue_size=100),
    rospy.Subscriber("/laserMapping/cloud_registered", PointCloud2, callback("cloud"),
                     queue_size=1, buff_size=32 * 1024 * 1024),
]

required = {"state", "battery", "imu", "rc", "odom", "cloud"}
deadline = time.monotonic() + 20.0
while time.monotonic() < deadline:
    with lock:
        missing = required.difference(latest)
    if not missing:
        break
    time.sleep(0.1)
else:
    raise SystemExit("missing preflight topics: %s" % sorted(missing))

last_report = 0.0
deadline = time.monotonic() + 120.0
while time.monotonic() < deadline:
    with lock:
        state = latest["state"]
        rc = latest["rc"]
    channels = list(rc.channels)
    if len(channels) < 8:
        raise SystemExit("/mavros/rc/in has fewer than 8 channels")
    switches_ok = (
        all(1375 <= channels[index] <= 1625 for index in range(4))
        and channels[4] > 1750
        and channels[5] > 1750
        and channels[7] < 1250
        and ((ch7_mode == "high" and channels[6] > 1750)
             or (ch7_mode == "low" and channels[6] < 1250))
    )
    if state.connected and not state.armed and switches_ok:
        break
    if time.monotonic() - last_report >= 2.0:
        print("Waiting: connected=%s armed=%s channels=%s" %
              (state.connected, state.armed, channels[:8]), flush=True)
        last_report = time.monotonic()
    time.sleep(0.1)
else:
    raise SystemExit("RC/pre-arm configuration was not satisfied within 120 seconds")

with lock:
    start_counts = dict(counts)
    initial_position = latest["odom"].pose.pose.position
    initial_xyz = (initial_position.x, initial_position.y, initial_position.z)
start_time = time.monotonic()
wall_start = time.time()
time.sleep(2.0)
elapsed = time.monotonic() - start_time
wall_elapsed = time.time() - wall_start
with lock:
    state = latest["state"]
    battery = latest["battery"]
    imu = latest["imu"]
    rc = latest["rc"]
    odom = latest["odom"]
    cloud = latest["cloud"]
    rates = {name: (counts.get(name, 0) - start_counts.get(name, 0)) / elapsed
             for name in ("imu", "rc", "odom", "cloud")}

errors = []
if abs(wall_elapsed - elapsed) > 0.20:
    errors.append("system clock jumped by %.3fs during preflight" % (wall_elapsed - elapsed))
if not state.connected or state.armed:
    errors.append("FCU must be connected and disarmed")
if not math.isfinite(battery.voltage) or battery.voltage < min_voltage:
    errors.append("battery voltage %.3fV is below %.3fV" % (battery.voltage, min_voltage))
if odom.header.frame_id != "world":
    errors.append("EKF frame is %r, expected 'world'" % odom.header.frame_id)
if cloud.header.frame_id != "world":
    errors.append("cloud frame is %r, expected 'world'" % cloud.header.frame_id)
if cloud.width * cloud.height < 10:
    errors.append("registered cloud is empty or too small")

now = rospy.Time.now()
for name, message, max_age in (("imu", imu, 0.20), ("odom", odom, 0.20), ("cloud", cloud, 0.30)):
    age = (now - message.header.stamp).to_sec()
    if age < -0.05 or age > max_age:
        errors.append("%s timestamp age %.3fs is invalid" % (name, age))

velocity = odom.twist.twist.linear
speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
position = odom.pose.pose.position
orientation = odom.pose.pose.orientation
numeric_values = (
    position.x, position.y, position.z,
    velocity.x, velocity.y, velocity.z,
    orientation.x, orientation.y, orientation.z, orientation.w,
)
if not all(math.isfinite(value) for value in numeric_values):
    errors.append("EKF odometry contains NaN or Inf")
quaternion_norm = math.sqrt(
    orientation.x ** 2 + orientation.y ** 2 + orientation.z ** 2 + orientation.w ** 2)
if not math.isfinite(quaternion_norm) or not 0.95 <= quaternion_norm <= 1.05:
    errors.append("EKF quaternion norm %.6f is invalid" % quaternion_norm)
if speed > 0.08:
    errors.append("pre-takeoff EKF speed %.3fm/s exceeds 0.08m/s" % speed)
if abs(position.x) > origin_limit or abs(position.y) > origin_limit:
    errors.append("start position (%.3f, %.3f) is not near the origin" % (position.x, position.y))
if position.z < -0.20 or position.z > 0.20:
    errors.append("ground start z %.3f is outside [-0.20, 0.20]m" % position.z)
static_drift = math.sqrt(sum((current - initial) ** 2
                             for current, initial in zip(
                                 (position.x, position.y, position.z), initial_xyz)))
if not all(math.isfinite(value) for value in initial_xyz):
    errors.append("initial EKF position contains NaN or Inf")
if not math.isfinite(static_drift) or static_drift > 0.05:
    errors.append("EKF drifted %.3fm during the 2s static check" % static_drift)

minimum_rates = {"imu": 100.0, "rc": 2.0, "odom": 80.0, "cloud": 5.0}
for name, minimum in minimum_rates.items():
    if rates[name] < minimum:
        errors.append("%s rate %.1fHz is below %.1fHz" % (name, rates[name], minimum))

print("Preflight rates: imu=%.1fHz rc=%.1fHz ekf=%.1fHz cloud=%.1fHz" %
      (rates["imu"], rates["rc"], rates["odom"], rates["cloud"]))
print("Preflight pose: (%.3f, %.3f, %.3f), speed=%.3fm/s" %
      (position.x, position.y, position.z, speed))
print("Preflight 2s static drift: %.3fm; cloud points: %d" %
      (static_drift, cloud.width * cloud.height))
print("Preflight battery: %.3fV; RC channels: %s" % (battery.voltage, list(rc.channels)[:8]))

if errors:
    raise SystemExit("preflight failed:\n- " + "\n- ".join(errors))
PY

start_component fuel bash -c 'source "$1"; exec roslaunch fuel_command_safety fuel_safe_real_lio.launch' fuel "${FUEL_SETUP}"
for node in /cloud_pose_adapter /exploration_node /traj_server /waypoint_generator /fuel_command_safety; do
  wait_for_node "${node}" 30
done
wait_for_topic_message /fuel/sensor_pose 20

start_component px4ctrl roslaunch px4ctrl run_ctrl_lio.launch
wait_for_node /px4ctrl 20

if [[ "${START_RVIZ}" == "true" ]]; then
  start_component rviz roslaunch odom_visualization fuel_real_lio_rviz.launch
  wait_for_node /fuel_real_lio_rviz 20
fi

python3 - <<'PY'
import time

import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String

rospy.init_node("fuel_auto_graph_preflight", anonymous=True, disable_signals=True)
rate_counts = {"sensor_pose": 0, "ekf": 0}

def count_message(name):
    def callback(_message):
        rate_counts[name] += 1
    return callback

rate_subscribers = [
    rospy.Subscriber("/fuel/sensor_pose", PoseStamped, count_message("sensor_pose"), queue_size=20),
    rospy.Subscriber("/ekf/ekf_odom", Odometry, count_message("ekf"), queue_size=200),
]
rate_start = time.monotonic()
time.sleep(2.0)
rate_elapsed = time.monotonic() - rate_start
sensor_pose_rate = rate_counts["sensor_pose"] / rate_elapsed
ekf_rate = rate_counts["ekf"] / rate_elapsed

master = rosgraph.Master(rospy.get_name())
publishers_raw, subscribers_raw, _ = master.getSystemState()
publishers = {topic: set(nodes) for topic, nodes in publishers_raw}
subscribers = {topic: set(nodes) for topic, nodes in subscribers_raw}

required_edges = [
    (publishers, "/fuel/position_cmd_raw", "/traj_server"),
    (subscribers, "/fuel/position_cmd_raw", "/fuel_command_safety"),
    (publishers, "/setpoints_cmd", "/fuel_command_safety"),
    (subscribers, "/setpoints_cmd", "/px4ctrl"),
    (publishers, "/traj_start_trigger", "/px4ctrl"),
    (subscribers, "/traj_start_trigger", "/waypoint_generator"),
    (subscribers, "/fuel/exploration_goal", "/waypoint_generator"),
    (publishers, "/waypoint_generator/waypoints", "/waypoint_generator"),
    (subscribers, "/waypoint_generator/waypoints", "/exploration_node"),
]
errors = []
if sensor_pose_rate < 5.0:
    errors.append("full-stack sensor pose rate %.1fHz is below 5Hz" % sensor_pose_rate)
if ekf_rate < 80.0:
    errors.append("full-stack EKF rate %.1fHz is below 80Hz" % ekf_rate)
for graph_side, topic, node in required_edges:
    if node not in graph_side.get(topic, set()):
        errors.append("missing graph edge %s on %s" % (node, topic))

takeoff_publishers = publishers.get("/px4ctrl/takeoff_land", set())
if takeoff_publishers:
    errors.append("unexpected takeoff publisher(s): %s" % sorted(takeoff_publishers))
if "/waypoint_generator" in subscribers.get("/move_base_simple/goal", set()):
    errors.append("waypoint generator is still connected to MAVROS /move_base_simple/goal")

state = rospy.wait_for_message("/mavros/state", State, timeout=3.0)
if not state.connected or state.armed:
    errors.append("FCU must be connected and disarmed")

sensor_pose = rospy.wait_for_message("/fuel/sensor_pose", PoseStamped, timeout=3.0)
if sensor_pose.header.frame_id != "world":
    errors.append("/fuel/sensor_pose frame must be world")
sensor_age = (rospy.Time.now() - sensor_pose.header.stamp).to_sec()
if sensor_age < -0.05 or sensor_age > 0.20:
    errors.append("/fuel/sensor_pose age %.3fs is invalid" % sensor_age)

safety = rospy.wait_for_message("/fuel_command_safety/state", String, timeout=3.0)
if not safety.data.startswith("WAITING_INPUT"):
    errors.append("safety bridge is not waiting: %s" % safety.data)

for topic in ("/fuel/position_cmd_raw", "/setpoints_cmd"):
    try:
        rospy.wait_for_message(topic, PositionCommand, timeout=1.0)
        errors.append("unexpected pre-trigger command on %s" % topic)
    except rospy.ROSException:
        pass

if rospy.get_param("/use_sim_time", False):
    errors.append("/use_sim_time must be false")
if not rospy.get_param("/px4ctrl/auto_takeoff_land/enable", False):
    errors.append("px4ctrl auto takeoff is disabled")
if not rospy.get_param("/px4ctrl/auto_takeoff_land/enable_auto_arm", False):
    errors.append("px4ctrl auto arm is disabled")

print("FUEL safety state: %s" % safety.data)
print("FUEL sensor pose age: %.3fs" % sensor_age)
print("Full-stack rates: sensor_pose=%.1fHz ekf=%.1fHz" % (sensor_pose_rate, ekf_rate))
if errors:
    raise SystemExit("graph preflight failed:\n- " + "\n- ".join(errors))
PY

cat <<'EOF'

All software gates passed. Before continuing, verify physically:
  1. Propellers are correct; the configured 6m x 6m, 1.5m-high area is clear.
  2. A pilot is holding the transmitter and can use CH6=999 for forced hover.
  3. CH5=1499 provides manual takeover; CH7 provides the proven emergency motor stop.
  4. CH5=1999, CH6=1999, CH7=arm-enabled, CH8=999, all sticks centered now.

Important: this PX4Ctrl does not process CH5/CH6 state changes while it is in
AUTO_TAKEOFF. During motor spin-up and ascent, use the independently proven CH7
emergency motor stop if immediate interruption is required. CH6 forced hover and
CH5 manual takeover apply after PX4Ctrl reaches AUTO_HOVER/CMD_CTRL.

Exploration completion commands a hold; it does not auto-land. For controlled
landing, run this script's --land mode from another terminal. That mode first
forces a command gap, lands, confirms armed=false, and then stops the stack.
EOF

[[ -t 0 ]] || die "interactive physical confirmation is required for real flight"
read -r -p 'Type AUTO-FUEL to arm, take off, and start exploration: ' confirmation
[[ "${confirmation}" == "AUTO-FUEL" ]] || die "confirmation did not match"

flight_command_sent=true
log "sending auto-takeoff command; do not close control processes while airborne"

python3 - "${TAKEOFF_TIMEOUT}" "${PLANNER_TIMEOUT}" "${ORIGIN_XY_LIMIT}" "${FUEL_CH7_ARM_POSITION}" <<'PY'
import math
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import RCIn, State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand, TakeoffLand
from rosgraph_msgs.msg import Log
from std_msgs.msg import String

takeoff_timeout = float(sys.argv[1])
planner_timeout = float(sys.argv[2])
origin_limit = float(sys.argv[3])
ch7_mode = sys.argv[4]

def rc_ready(channels):
    return (
        len(channels) >= 8
        and all(1375 <= channels[index] <= 1625 for index in range(4))
        and channels[4] > 1750
        and channels[5] > 1750
        and channels[7] < 1250
        and ((ch7_mode == "high" and channels[6] > 1750)
             or (ch7_mode == "low" and channels[6] < 1250))
    )

rospy.init_node("fuel_auto_takeoff_trigger", anonymous=True, disable_signals=True)
lock = threading.Lock()
latest = {}
raw_count = 0
safe_count = 0
trigger_pose = None
hover_seen_at = None
cmd_ctrl_seen = False

def save(name):
    def callback(message):
        with lock:
            latest[name] = message
    return callback

def raw_callback(_message):
    global raw_count
    with lock:
        raw_count += 1

def safe_callback(_message):
    global safe_count
    with lock:
        safe_count += 1

def trigger_callback(message):
    global trigger_pose
    with lock:
        trigger_pose = message

def log_callback(message):
    global hover_seen_at, cmd_ctrl_seen
    text = message.msg
    with lock:
        if "AUTO_TAKEOFF --> AUTO_HOVER" in text:
            hover_seen_at = time.monotonic()
        if "AUTO_HOVER(L2) --> CMD_CTRL(L3)" in text:
            cmd_ctrl_seen = True

subs = [
    rospy.Subscriber("/mavros/state", State, save("state"), queue_size=20),
    rospy.Subscriber("/mavros/rc/in", RCIn, save("rc"), queue_size=20),
    rospy.Subscriber("/ekf/ekf_odom", Odometry, save("odom"), queue_size=100),
    rospy.Subscriber("/fuel_command_safety/state", String, save("safety"), queue_size=20),
    rospy.Subscriber("/fuel/position_cmd_raw", PositionCommand, raw_callback, queue_size=100),
    rospy.Subscriber("/setpoints_cmd", PositionCommand, safe_callback, queue_size=100),
    rospy.Subscriber("/traj_start_trigger", PoseStamped, trigger_callback, queue_size=20),
    rospy.Subscriber("/rosout", Log, log_callback, queue_size=100),
]

takeoff_pub = rospy.Publisher("/px4ctrl/takeoff_land", TakeoffLand, queue_size=1)
fallback_pub = rospy.Publisher("/fuel/exploration_goal", PoseStamped, queue_size=1)

deadline = time.monotonic() + 8.0
while time.monotonic() < deadline:
    with lock:
        ready = all(name in latest for name in ("state", "rc", "odom", "safety"))
    if ready and takeoff_pub.get_num_connections() > 0 and fallback_pub.get_num_connections() > 0:
        break
    time.sleep(0.05)
else:
    raise SystemExit("takeoff orchestrator did not connect to all required topics")

with lock:
    state = latest["state"]
    rc = latest["rc"]
    odom = latest["odom"]
channels = list(rc.channels)
if state.armed or not state.connected:
    raise SystemExit("FCU is not connected-and-disarmed immediately before takeoff")
if not rc_ready(channels):
    raise SystemExit("sticks or CH5/CH6/CH7/CH8 changed immediately before takeoff")

start_position = odom.pose.pose.position
start_z = start_position.z
if not all(math.isfinite(value) for value in
           (start_position.x, start_position.y, start_position.z)):
    raise SystemExit("takeoff origin contains NaN or Inf")
print("Takeoff origin: (%.3f, %.3f, %.3f)" %
      (start_position.x, start_position.y, start_position.z), flush=True)

takeoff = TakeoffLand()
takeoff.takeoff_land_cmd = 1
for _ in range(3):
    takeoff_pub.publish(takeoff)
    time.sleep(0.10)

takeoff_started = time.monotonic()
armed_seen = False
physical_hover_stable = False
stable_since = None
fallback_sent = False

while time.monotonic() - takeoff_started < takeoff_timeout:
    with lock:
        state = latest.get("state")
        rc = latest.get("rc")
        odom = latest.get("odom")
        local_hover_seen_at = hover_seen_at
        local_trigger = trigger_pose
    if state is None or rc is None or odom is None:
        time.sleep(0.05)
        continue
    if not state.connected:
        raise SystemExit("MAVROS disconnected after takeoff command")
    odom_age = (rospy.Time.now() - odom.header.stamp).to_sec()
    if odom_age < -0.05 or odom_age > 0.25:
        raise SystemExit("EKF odometry became stale after takeoff command")

    channels = list(rc.channels)
    if not rc_ready(channels):
        raise SystemExit("sticks or CH5/CH6/CH7/CH8 changed during auto takeoff")

    armed_seen = armed_seen or state.armed
    if armed_seen and not state.armed:
        raise SystemExit("FCU disarmed during auto takeoff")
    velocity = odom.twist.twist.linear
    position = odom.pose.pose.position
    values = (position.x, position.y, position.z, velocity.x, velocity.y, velocity.z)
    if not all(math.isfinite(value) for value in values):
        raise SystemExit("EKF odometry contains NaN or Inf during auto takeoff")
    dz = position.z - start_z
    speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
    if dz >= 0.65 and speed <= 0.15:
        if stable_since is None:
            stable_since = time.monotonic()
        elif time.monotonic() - stable_since >= 1.0:
            physical_hover_stable = True
    else:
        stable_since = None
        physical_hover_stable = False

    px4ctrl_hover_confirmed = local_hover_seen_at is not None or local_trigger is not None
    if physical_hover_stable and px4ctrl_hover_confirmed and state.armed and state.mode == "OFFBOARD":
        break
    time.sleep(0.05)
else:
    raise SystemExit("auto takeoff did not reach a stable hover before timeout")

print("Stable auto hover reached; waiting for the PX4Ctrl trigger", flush=True)
trigger_deadline = time.monotonic() + 4.0
while time.monotonic() < trigger_deadline:
    with lock:
        current_trigger = trigger_pose
        odom = latest["odom"]
        state = latest["state"]
        rc = latest["rc"]
    if not state.connected or not state.armed or state.mode != "OFFBOARD":
        raise SystemExit("FCU left armed OFFBOARD while waiting for the exploration trigger")
    if not rc_ready(list(rc.channels)):
        raise SystemExit("sticks or CH5/CH6/CH7/CH8 changed while waiting for the trigger")
    odom_age = (rospy.Time.now() - odom.header.stamp).to_sec()
    if odom_age < -0.05 or odom_age > 0.25:
        raise SystemExit("EKF odometry became stale while waiting for the trigger")
    if current_trigger is not None:
        break
    time.sleep(0.05)
else:
    point = odom.pose.pose.position
    velocity = odom.twist.twist.linear
    speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
    if (hover_seen_at is None
            or not state.armed
            or state.mode != "OFFBOARD"
            or speed > 0.15
            or not 0.65 <= point.z - start_z <= 1.2
            or not all(math.isfinite(value) for value in (point.x, point.y, point.z))
            or abs(point.x) > origin_limit + 0.5
            or abs(point.y) > origin_limit + 0.5
            or math.hypot(point.x - start_position.x, point.y - start_position.y) > 0.5):
        raise SystemExit("PX4Ctrl hover was not confirmed; refusing fallback trigger")
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "world"
    goal.pose = odom.pose.pose
    fallback_pub.publish(goal)
    fallback_sent = True
    print("PX4Ctrl trigger timeout; published current hover pose on /fuel/exploration_goal", flush=True)

with lock:
    current_trigger = trigger_pose
if current_trigger is not None:
    point = current_trigger.pose.position
    print("PX4Ctrl trigger pose: (%.3f, %.3f, %.3f)" % (point.x, point.y, point.z), flush=True)
    if (not all(math.isfinite(value) for value in (point.x, point.y, point.z))
            or abs(point.x) > origin_limit + 0.5
            or abs(point.y) > origin_limit + 0.5
            or not 0.5 <= point.z - start_z <= 1.2
            or math.hypot(point.x - start_position.x, point.y - start_position.y) > 0.5):
        raise SystemExit("trigger pose is unexpectedly far from the origin")
elif not fallback_sent:
    raise SystemExit("no exploration trigger was produced")

planner_deadline = time.monotonic() + planner_timeout
while time.monotonic() < planner_deadline:
    with lock:
        state = latest.get("state")
        rc = latest.get("rc")
        odom = latest.get("odom")
        safety = latest.get("safety")
        local_raw_count = raw_count
        local_safe_count = safe_count
        local_cmd_ctrl_seen = cmd_ctrl_seen
    if state is None or not state.connected or not state.armed:
        raise SystemExit("FCU left the armed/connected state while waiting for FUEL")
    if state.mode != "OFFBOARD":
        raise SystemExit("FCU left OFFBOARD while waiting for FUEL")
    if rc is None or not rc_ready(list(rc.channels)):
        raise SystemExit("sticks or CH5/CH6/CH7/CH8 changed while waiting for FUEL")
    if odom is None:
        raise SystemExit("EKF odometry disappeared while waiting for FUEL")
    odom_age = (rospy.Time.now() - odom.header.stamp).to_sec()
    if odom_age < -0.05 or odom_age > 0.25:
        raise SystemExit("EKF odometry became stale while waiting for FUEL")
    forwarding = safety is not None and safety.data.startswith("FORWARDING")
    if forwarding and local_raw_count > 0 and local_safe_count > 0:
        print("FUEL exploration active: safety=%s raw=%d safe=%d CMD_CTRL_log=%s" %
              (safety.data, local_raw_count, local_safe_count, local_cmd_ctrl_seen), flush=True)
        raise SystemExit(0)
    time.sleep(0.05)

with lock:
    safety_text = latest.get("safety").data if latest.get("safety") is not None else "missing"
    summary = (safety_text, raw_count, safe_count, cmd_ctrl_seen)
raise SystemExit("FUEL did not enter command control before timeout: safety=%s raw=%d safe=%d CMD_CTRL=%s" % summary)
PY

log "FUEL autonomous exploration is active"
log "logs: ${RUN_DIR}"
log "forced hover: keep CH5=1999 and move CH6 to 999"
log "manual takeover: move CH5 to 1499"
log "controlled landing: $0 --land --ch7-${FUEL_CH7_ARM_POSITION}"
log "after any manual landing and armed=false, stop this stack with: $0 --stop"

while true; do
  sleep 5
  if [[ ! -f "${STATE_FILE}" ]]; then
    cleanup_needed=false
    log "managed stack was stopped from another terminal"
    exit 0
  fi
  if ! rosnode list 2>/dev/null | grep -Fxq /px4ctrl; then
    warn "/px4ctrl disappeared; use the proven RC emergency procedure"
  fi
  if ! rosnode list 2>/dev/null | grep -Fxq /exploration_node; then
    warn "/exploration_node disappeared; use CH6=999 for forced hover"
  fi
done
