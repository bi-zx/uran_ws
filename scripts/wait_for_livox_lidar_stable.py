#!/usr/bin/env python3
"""Wait until Livox PointCloud2 frames are stable enough for DLIO startup."""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


class LidarStabilityGate:
    def __init__(self, args: argparse.Namespace):
        self._args = args
        self._lock = threading.Lock()
        self._stable_since = None
        self._last_frame_time = None
        self._good_frames = 0
        self._last_result = "waiting for pointcloud"

    def feed(self, msg: PointCloud2) -> None:
        now = time.monotonic()
        raw_points = msg.width * msg.height
        valid_points = self._count_valid_points(msg)
        ok = valid_points >= self._args.min_valid_points

        with self._lock:
            self._last_frame_time = now
            if ok:
                if self._stable_since is None:
                    self._stable_since = now
                    self._good_frames = 0
                self._good_frames += 1
                stable_for = now - self._stable_since
                self._last_result = (
                    f"ok: valid={valid_points}, raw={raw_points}, "
                    f"stable={stable_for:.2f}s, frames={self._good_frames}"
                )
            else:
                self._stable_since = None
                self._good_frames = 0
                self._last_result = (
                    f"blocked: valid={valid_points} < {self._args.min_valid_points}, "
                    f"raw={raw_points}"
                )

    def status(self) -> tuple[bool, str]:
        now = time.monotonic()
        with self._lock:
            if self._last_frame_time is None:
                return False, f"waiting for pointcloud on {self._args.topic}"

            age = now - self._last_frame_time
            if age > self._args.max_age:
                self._stable_since = None
                self._good_frames = 0
                return False, f"pointcloud stale ({age:.2f}s > {self._args.max_age:.2f}s)"

            if self._stable_since is None:
                return False, self._last_result

            stable_for = now - self._stable_since
            if stable_for < self._args.stable_seconds:
                return False, self._last_result

            if self._good_frames < self._args.min_good_frames:
                return False, (
                    f"waiting for enough good frames: {self._good_frames} "
                    f"< {self._args.min_good_frames}"
                )

            return True, self._last_result

    def _count_valid_points(self, msg: PointCloud2) -> int:
        field_names = {field.name for field in msg.fields}
        if not {"x", "y", "z"}.issubset(field_names):
            return 0

        min_range_sq = self._args.min_range * self._args.min_range
        valid = 0
        for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False):
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            range_sq = x * x + y * y + z * z
            if range_sq <= min_range_sq:
                continue
            valid += 1
        return valid


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Block DLIO startup until /livox/lidar publishes consecutive "
            "PointCloud2 frames with enough valid points."
        )
    )
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument("--stable-seconds", type=float, default=5.0)
    parser.add_argument("--min-valid-points", type=int, default=1000)
    parser.add_argument("--min-good-frames", type=int, default=5)
    parser.add_argument(
        "--min-range",
        type=float,
        default=1.0,
        help="Ignore points within this range from the lidar origin.",
    )
    parser.add_argument("--max-age", type=float, default=1.0)
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.0,
        help="Maximum seconds to wait. 0 means wait forever.",
    )
    parser.add_argument("--log-period", type=float, default=1.0)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def validate_args(args: argparse.Namespace) -> int:
    if args.stable_seconds < 0:
        print("--stable-seconds must be >= 0", file=sys.stderr)
        return 2
    if args.min_valid_points < 1:
        print("--min-valid-points must be >= 1", file=sys.stderr)
        return 2
    if args.min_good_frames < 1:
        print("--min-good-frames must be >= 1", file=sys.stderr)
        return 2
    if args.min_range < 0:
        print("--min-range must be >= 0", file=sys.stderr)
        return 2
    if args.max_age <= 0:
        print("--max-age must be > 0", file=sys.stderr)
        return 2
    if args.timeout < 0:
        print("--timeout must be >= 0", file=sys.stderr)
        return 2
    return 0


def main() -> int:
    args = parse_args()
    validation_error = validate_args(args)
    if validation_error:
        return validation_error

    rospy.init_node("livox_lidar_stability_gate", anonymous=True)
    gate = LidarStabilityGate(args)
    rospy.Subscriber(args.topic, PointCloud2, gate.feed, queue_size=20)

    rospy.loginfo(
        "[dlio_lidar_gate] waiting for %s: valid_points >= %d, stable %.2fs, "
        "min_good_frames=%d, min_range=%.2fm",
        args.topic,
        args.min_valid_points,
        args.stable_seconds,
        args.min_good_frames,
        args.min_range,
    )

    start_time = time.monotonic()
    last_log_time = 0.0
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        now = time.monotonic()
        ok, reason = gate.status()
        if ok:
            rospy.loginfo("[dlio_lidar_gate] pointcloud gate passed: %s", reason)
            return 0

        if now - last_log_time >= args.log_period:
            rospy.logwarn("[dlio_lidar_gate] startup blocked: %s", reason)
            last_log_time = now

        if args.timeout > 0 and now - start_time >= args.timeout:
            rospy.logerr("[dlio_lidar_gate] timed out waiting for stable pointcloud")
            return 1

        rate.sleep()

    return 130


if __name__ == "__main__":
    sys.exit(main())
