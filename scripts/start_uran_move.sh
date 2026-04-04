#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(builtin cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/load_ros_env.sh"

cd "${URAN_WORKSPACE_DIR}"
exec ros2 run uran_move uran_move_node
