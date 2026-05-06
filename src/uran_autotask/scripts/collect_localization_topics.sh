#!/usr/bin/env bash
set -u

NS="${1:-}"
DURATION_S="${2:-10}"

if [ -z "$NS" ]; then
  echo "usage: collect_localization_topics.sh /robot_namespace [duration_s]"
  echo "example: collect_localization_topics.sh /mi_desktop_48_b0_2d_5f_b6_d0 10"
  exit 2
fi

NS="/${NS#/}"
GPS_TOPIC="${NS}/gps_payload"
MAP_TOPIC="${NS}/dog_pose"

echo "namespace: ${NS}"
echo "duration_s: ${DURATION_S}"
echo

echo "== ROS environment =="
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
echo

echo "== GPS topic =="
ros2 topic info -v "${GPS_TOPIC}" || true
timeout "${DURATION_S}" ros2 topic hz "${GPS_TOPIC}" || true
echo

echo "== Map pose topic =="
ros2 topic info -v "${MAP_TOPIC}" || true
timeout "${DURATION_S}" ros2 topic hz "${MAP_TOPIC}" || true
echo

echo "== Candidate local odometry topics =="
ros2 topic list | grep -Ei 'dog_pose|odom|mivins|vio|vins|visual' || true
echo

for topic in \
  "${NS}/odometry" \
  "${NS}/pose_filtered" \
  "${NS}/odom_slam" \
  "${NS}/mivins/odometry" \
  "${NS}/mivins/imuodom_slam" \
  "${NS}/mivins/reloc_odom" \
  "${NS}/odom_out"
do
  if ros2 topic list | grep -Fxq "${topic}"; then
    echo "== ${topic} =="
    ros2 topic info -v "${topic}" || true
    timeout "${DURATION_S}" ros2 topic hz "${topic}" || true
    echo
  fi
done

echo "== Suggested rosbag command =="
echo "ros2 bag record ${GPS_TOPIC} ${MAP_TOPIC} ${NS}/odometry ${NS}/pose_filtered ${NS}/odom_slam ${NS}/mivins/odometry ${NS}/mivins/imuodom_slam ${NS}/mivins/reloc_odom ${NS}/odom_out /tf /tf_static"
