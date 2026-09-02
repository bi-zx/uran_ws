#!/bin/bash
set -euo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
source /opt/ros2/galactic/setup.bash

ros2 launch realsense2_camera on_dog.py \
  enable_color:=false \
  enable_depth:=true \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_fisheye1:=false \
  enable_fisheye2:=false \
  enable_confidence:=false \
  enable_gyro:=false \
  enable_accel:=false \
  enable_pose:=false \
  pointcloud.enable:=true \
  pointcloud.stream_filter:=0 \
  depth_module.profile:="640,480,30" \
  depth_module.emitter_enabled:=1 \
  depth_module.emitter_on_off:=false \
  initial_reset:=false &
launch_pid=$!

cleanup() {
  if kill -0 "$launch_pid" 2>/dev/null; then
    kill -TERM "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

active=false
for _ in $(seq 1 60); do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    wait "$launch_pid"
    exit $?
  fi

  state="$(ros2 lifecycle get /camera/camera 2>/dev/null || true)"
  case "$state" in
    *unconfigured*)
      ros2 lifecycle set /camera/camera configure >/dev/null 2>&1 || true
      ;;
    *inactive*)
      ros2 lifecycle set /camera/camera activate >/dev/null 2>&1 || true
      ;;
    *active*)
      active=true
      break
      ;;
  esac
  sleep 0.5
done

if [[ "$active" != true ]]; then
  echo "RealSense lifecycle node did not become active within 30 seconds" >&2
  exit 1
fi

wait "$launch_pid"
