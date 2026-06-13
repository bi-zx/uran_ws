#!/usr/bin/env python3
"""Wait until RC channels are in the safe px4ctrl startup positions."""

from __future__ import annotations

import argparse
import sys
import time

import rospy
from mavros_msgs.msg import RCIn


def _channel_number(value: str) -> int:
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError("channel numbers are 1-based and must be >= 1")
    return number


class RcSafetyGate:
    def __init__(self, args: argparse.Namespace):
        self._args = args
        self._last_msg = None
        self._last_msg_time = None

    def feed(self, msg: RCIn) -> None:
        self._last_msg = msg
        self._last_msg_time = time.monotonic()

    def check(self, now: float) -> tuple[bool, str]:
        if self._last_msg is None or self._last_msg_time is None:
            return False, f"waiting for RC data on {self._args.topic}"

        age = now - self._last_msg_time
        if age > self._args.max_age:
            return False, f"RC data stale ({age:.2f}s > {self._args.max_age:.2f}s)"

        channels = self._last_msg.channels
        needed = max(
            self._args.throttle_channel,
            self._args.ch5_channel,
            self._args.ch6_channel,
        )
        if len(channels) < needed:
            return False, f"RC channel count {len(channels)} < required CH{needed}"

        throttle = channels[self._args.throttle_channel - 1]
        ch5 = channels[self._args.ch5_channel - 1]
        ch6 = channels[self._args.ch6_channel - 1]

        if not (self._args.throttle_min <= throttle <= self._args.throttle_max):
            return (
                False,
                f"CH{self._args.throttle_channel} throttle={throttle} "
                f"not in [{self._args.throttle_min},{self._args.throttle_max}]",
            )

        if not (self._args.ch5_min <= ch5 <= self._args.ch5_max):
            return (
                False,
                f"CH{self._args.ch5_channel}={ch5} "
                f"not in [{self._args.ch5_min},{self._args.ch5_max}]",
            )

        if not (self._args.ch6_min <= ch6 <= self._args.ch6_max):
            return (
                False,
                f"CH{self._args.ch6_channel}={ch6} "
                f"not in [{self._args.ch6_min},{self._args.ch6_max}]",
            )

        return (
            True,
            f"RC safe: CH{self._args.throttle_channel}={throttle}, "
            f"CH{self._args.ch5_channel}={ch5}, CH{self._args.ch6_channel}={ch6}",
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Block px4ctrl startup until RC is safe: throttle centered, "
            "CH5 and CH6 in the required high PWM ranges."
        )
    )
    parser.add_argument("--topic", default="/mavros/rc/in")
    parser.add_argument("--stable-seconds", type=float, default=2.0)
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.0,
        help="Maximum seconds to wait. 0 means wait forever.",
    )
    parser.add_argument("--max-age", type=float, default=1.0)
    parser.add_argument("--log-period", type=float, default=1.0)

    parser.add_argument("--throttle-channel", type=_channel_number, default=3)
    parser.add_argument("--ch5-channel", type=_channel_number, default=5)
    parser.add_argument("--ch6-channel", type=_channel_number, default=6)

    parser.add_argument("--throttle-min", type=int, default=1400)
    parser.add_argument("--throttle-max", type=int, default=1600)
    parser.add_argument("--ch5-min", type=int, default=1900)
    parser.add_argument("--ch5-max", type=int, default=2100)
    parser.add_argument("--ch6-min", type=int, default=1900)
    parser.add_argument("--ch6-max", type=int, default=2100)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main() -> int:
    args = parse_args()
    if args.stable_seconds < 0:
        print("--stable-seconds must be >= 0", file=sys.stderr)
        return 2
    if args.timeout < 0:
        print("--timeout must be >= 0", file=sys.stderr)
        return 2
    if args.max_age <= 0:
        print("--max-age must be > 0", file=sys.stderr)
        return 2

    rospy.init_node("px4ctrl_rc_startup_gate", anonymous=True)
    gate = RcSafetyGate(args)
    rospy.Subscriber(args.topic, RCIn, gate.feed, queue_size=10)

    rospy.loginfo(
        "[px4ctrl_safe] waiting for RC safe state: CH%d in [%d,%d], "
        "CH%d in [%d,%d], CH%d in [%d,%d], stable %.2fs",
        args.throttle_channel,
        args.throttle_min,
        args.throttle_max,
        args.ch5_channel,
        args.ch5_min,
        args.ch5_max,
        args.ch6_channel,
        args.ch6_min,
        args.ch6_max,
        args.stable_seconds,
    )

    start_time = time.monotonic()
    safe_since = None
    last_log_time = 0.0
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        now = time.monotonic()
        ok, reason = gate.check(now)

        if ok:
            if safe_since is None:
                safe_since = now
            stable_for = now - safe_since
            if stable_for >= args.stable_seconds:
                rospy.loginfo("[px4ctrl_safe] %s; startup gate passed", reason)
                return 0
            if now - last_log_time >= args.log_period:
                rospy.loginfo(
                    "[px4ctrl_safe] %s; holding %.2fs / %.2fs",
                    reason,
                    stable_for,
                    args.stable_seconds,
                )
                last_log_time = now
        else:
            safe_since = None
            if now - last_log_time >= args.log_period:
                rospy.logwarn("[px4ctrl_safe] startup blocked: %s", reason)
                last_log_time = now

        if args.timeout > 0 and now - start_time >= args.timeout:
            rospy.logerr("[px4ctrl_safe] timed out waiting for safe RC state")
            return 1

        rate.sleep()

    return 130


if __name__ == "__main__":
    sys.exit(main())
