#!/bin/bash

# Shared ROS 2 environment loader for non-interactive startup paths.
set -euo pipefail

SCRIPT_DIR="$(builtin cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_WORKSPACE_DIR="$(builtin cd -- "${SCRIPT_DIR}/.." && pwd)"
TARGET_WORKSPACE_DIR="${URAN_WORKSPACE_DIR:-}"

if [ -z "${TARGET_WORKSPACE_DIR}" ]; then
  if [ -d /SDCARD/uran_ws ]; then
    TARGET_WORKSPACE_DIR=/SDCARD/uran_ws
  else
    TARGET_WORKSPACE_DIR="${DEFAULT_WORKSPACE_DIR}"
  fi
fi

source_if_exists() {
  local candidate="$1"
  if [ -f "$candidate" ]; then
    local had_nounset=0
    case $- in
      *u*)
        had_nounset=1
        set +u
        ;;
    esac
    # shellcheck disable=SC1090
    . "$candidate"
    local rc=$?
    if [ "${had_nounset}" -eq 1 ]; then
      set -u
    fi
    return "${rc}"
  fi
  return 1
}

export URAN_WORKSPACE_DIR="${TARGET_WORKSPACE_DIR}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

if ! source_if_exists "${URAN_WORKSPACE_DIR}/install/setup.bash"; then
  source_if_exists /opt/ros/humble/setup.bash \
    || source_if_exists /opt/ros2/galactic/setup.bash \
    || {
      echo "Failed to load ROS environment from workspace or known ROS install." >&2
      exit 1
    }
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "Failed to load ROS environment: ros2 command not found." >&2
  exit 1
fi
