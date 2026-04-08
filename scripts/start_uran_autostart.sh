#!/usr/bin/env bash

set -euo pipefail

source_setup_file() {
  local setup_file="$1"

  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/devel/setup.bash"
DEFAULT_EXTRA_SETUP="/home/jetson/catkin_pkg/devel/setup.bash"

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
      # Source overlay workspaces before the main workspace.
      source_setup_file "${setup_file}"
    fi
  done
fi

source_setup_file "${WORKSPACE_SETUP}"

exec roslaunch uran_core uran_autostart.launch "$@"
