#!/usr/bin/env bash

set -euo pipefail

source_setup_file() {
  local setup_file="$1"

  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

wait_for_ros_master() {
  local timeout_s="${ROS_MASTER_TIMEOUT:-0}"
  local start_s
  local now_s

  start_s="$(date +%s)"
  echo "waiting for ROS master at ${ROS_MASTER_URI}"
  while ! rosnode list >/dev/null 2>&1; do
    if [[ "${timeout_s}" != "0" ]]; then
      now_s="$(date +%s)"
      if (( now_s - start_s >= timeout_s )); then
        echo "timed out waiting for ROS master after ${timeout_s}s" >&2
        return 1
      fi
    fi
    sleep 1
  done
}

WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/devel/setup.bash"
DEFAULT_EXTRA_SETUP="/home/jetson/catkin_pkg/devel/setup.bash"
DLIO_START_DELAY_SECONDS="${DLIO_START_DELAY_SECONDS:-10}"

export ROS_HOME="${ROS_HOME:-/tmp/ros_home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${ROS_HOME}/log}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"

mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "missing ROS setup: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "missing workspace setup: ${WORKSPACE_SETUP}" >&2
  exit 1
fi

source_setup_file "${ROS_SETUP}"

if [[ -z "${EXTRA_SETUP_FILES:-}" && -f "${DEFAULT_EXTRA_SETUP}" ]]; then
  EXTRA_SETUP_FILES="${DEFAULT_EXTRA_SETUP}"
fi

if [[ -n "${EXTRA_SETUP_FILES:-}" ]]; then
  IFS=':' read -r -a extra_setup_files <<< "${EXTRA_SETUP_FILES}"
  for setup_file in "${extra_setup_files[@]}"; do
    if [[ -n "${setup_file}" && -f "${setup_file}" ]]; then
      source_setup_file "${setup_file}"
    fi
  done
fi

source_setup_file "${WORKSPACE_SETUP}"

wait_for_ros_master

echo "delaying DLIO startup for ${DLIO_START_DELAY_SECONDS}s"
sleep "${DLIO_START_DELAY_SECONDS}"

python3 "${WORKSPACE_DIR}/scripts/wait_for_livox_lidar_stable.py" \
  --topic "${DLIO_LIDAR_TOPIC:-/livox/lidar}" \
  --stable-seconds "${DLIO_LIDAR_STABLE_SECONDS:-5.0}" \
  --min-valid-points "${DLIO_LIDAR_MIN_VALID_POINTS:-1000}" \
  --min-good-frames "${DLIO_LIDAR_MIN_GOOD_FRAMES:-5}" \
  --min-range "${DLIO_LIDAR_MIN_RANGE:-1.0}" \
  --max-age "${DLIO_LIDAR_MAX_AGE:-1.0}" \
  --timeout "${DLIO_LIDAR_CHECK_TIMEOUT:-0}"

echo "starting DLIO"
exec roslaunch direct_lidar_inertial_odometry dlio.launch "$@"
