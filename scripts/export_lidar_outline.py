#!/usr/bin/env python3

import argparse
import csv
import math
import os
import statistics
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def parse_args():
    parser = argparse.ArgumentParser(
        description="Export a stable LaserScan outline as SVG and CSV.")
    parser.add_argument(
        "--topic",
        default="/mi_desktop_48_b0_2d_5f_b6_d0/scan")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--max-forward", type=float, default=10.0)
    parser.add_argument("--half-width", type=float, default=6.0)
    parser.add_argument("--angle-bin-deg", type=float, default=0.5)
    parser.add_argument("--output-prefix", default="/tmp/lidar_outline")
    return parser.parse_args()


def capture_scans(args):
    scans = []

    def callback(msg):
        if len(scans) >= args.frames:
            return
        scans.append({
            "angle_min": float(msg.angle_min),
            "angle_increment": float(msg.angle_increment),
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            "ranges": tuple(float(value) for value in msg.ranges),
        })

    rclpy.init(args=None)
    node = rclpy.create_node("lidar_outline_exporter")
    subscription = node.create_subscription(
        LaserScan, args.topic, callback, qos_profile_sensor_data)

    deadline = time.monotonic() + args.timeout
    try:
        while len(scans) < args.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()

    if not scans:
        raise RuntimeError("no LaserScan messages received before timeout")
    return scans


def aggregate_points(scans, args):
    angle_bin_rad = math.radians(args.angle_bin_deg)
    bins = {}

    for scan in scans:
        frame_bins = {}
        valid_min = max(0.05, scan["range_min"])
        valid_max = min(
            math.hypot(args.max_forward, args.half_width),
            scan["range_max"])

        for index, distance in enumerate(scan["ranges"]):
            if not math.isfinite(distance):
                continue
            if distance < valid_min or distance > valid_max:
                continue

            angle = scan["angle_min"] + index * scan["angle_increment"]
            bin_index = int(round(angle / angle_bin_rad))
            frame_bins.setdefault(bin_index, []).append(distance)

        for bin_index, distances in frame_bins.items():
            bins.setdefault(bin_index, []).append(statistics.median(distances))

    minimum_frames = max(2, int(math.ceil(len(scans) * 0.25)))
    points = []
    for bin_index in sorted(bins):
        distances = bins[bin_index]
        if len(distances) < minimum_frames:
            continue

        angle = bin_index * angle_bin_rad
        distance = statistics.median(distances)
        forward = distance * math.cos(angle)
        left = distance * math.sin(angle)
        if forward < 0.0 or forward > args.max_forward:
            continue
        if abs(left) > args.half_width:
            continue

        points.append({
            "angle_deg": math.degrees(angle),
            "distance_m": distance,
            "forward_m": forward,
            "left_m": left,
            "frame_count": len(distances),
        })
    return points, minimum_frames


def write_csv(path, points):
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "angle_deg",
                "distance_m",
                "forward_m",
                "left_m",
                "frame_count",
            ])
        writer.writeheader()
        for point in points:
            writer.writerow({
                "angle_deg": "{:.3f}".format(point["angle_deg"]),
                "distance_m": "{:.4f}".format(point["distance_m"]),
                "forward_m": "{:.4f}".format(point["forward_m"]),
                "left_m": "{:.4f}".format(point["left_m"]),
                "frame_count": point["frame_count"],
            })


def write_svg(path, points, scan_count, minimum_frames, args):
    width = 1200
    height = 900
    margin_left = 80
    margin_right = 40
    margin_top = 80
    margin_bottom = 70
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom

    def project(point):
        x = margin_left + (
            (args.half_width - point["left_m"]) /
            (2.0 * args.half_width) * plot_width)
        y = margin_top + (
            (args.max_forward - point["forward_m"]) /
            args.max_forward * plot_height)
        return x, y

    distances = sorted(point["distance_m"] for point in points)
    median_distance = statistics.median(distances) if distances else 0.0
    p90_index = int(round(0.9 * (len(distances) - 1))) if distances else 0
    p90_distance = distances[p90_index] if distances else 0.0

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<svg xmlns="http://www.w3.org/2000/svg" '
        'width="{}" height="{}" viewBox="0 0 {} {}">'.format(
            width, height, width, height),
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<text x="{}" y="34" font-family="sans-serif" font-size="22" '
        'fill="#111111">TG30 LaserScan outline</text>'.format(margin_left),
        '<text x="{}" y="60" font-family="monospace" font-size="14" '
        'fill="#444444">frames={} stable_points={} required_frames={} '
        'median={:.3f}m p90={:.3f}m</text>'.format(
            margin_left, scan_count, len(points), minimum_frames,
            median_distance, p90_distance),
    ]

    for lateral in range(-int(args.half_width), int(args.half_width) + 1):
        x = margin_left + (
            (args.half_width - lateral) /
            (2.0 * args.half_width) * plot_width)
        lines.append(
            '<line x1="{0:.1f}" y1="{1}" x2="{0:.1f}" y2="{2}" '
            'stroke="#dddddd" stroke-width="1"/>'.format(
                x, margin_top, margin_top + plot_height))
        lines.append(
            '<text x="{:.1f}" y="{}" text-anchor="middle" '
            'font-family="monospace" font-size="12" fill="#666666">'
            '{:+d}m</text>'.format(
                x, margin_top + plot_height + 22, lateral))

    for forward in range(0, int(args.max_forward) + 1):
        y = margin_top + (
            (args.max_forward - forward) /
            args.max_forward * plot_height)
        lines.append(
            '<line x1="{0}" y1="{1:.1f}" x2="{2}" y2="{1:.1f}" '
            'stroke="#dddddd" stroke-width="1"/>'.format(
                margin_left, y, margin_left + plot_width))
        lines.append(
            '<text x="{}" y="{:.1f}" text-anchor="end" '
            'dominant-baseline="middle" font-family="monospace" '
            'font-size="12" fill="#666666">{}m</text>'.format(
                margin_left - 10, y, forward))

    segments = []
    current_segment = []
    previous = None
    for point in points:
        if previous is not None:
            angle_gap = point["angle_deg"] - previous["angle_deg"]
            coordinate_gap = math.hypot(
                point["forward_m"] - previous["forward_m"],
                point["left_m"] - previous["left_m"])
            if angle_gap > args.angle_bin_deg * 2.5 or coordinate_gap > 0.75:
                if len(current_segment) >= 2:
                    segments.append(current_segment)
                current_segment = []
        current_segment.append(project(point))
        previous = point
    if len(current_segment) >= 2:
        segments.append(current_segment)

    for segment in segments:
        coordinates = " ".join(
            "{:.1f},{:.1f}".format(x, y) for x, y in segment)
        lines.append(
            '<polyline points="{}" fill="none" stroke="#146c94" '
            'stroke-width="2" stroke-linejoin="round"/>'.format(coordinates))

    for point in points:
        x, y = project(point)
        lines.append(
            '<circle cx="{:.1f}" cy="{:.1f}" r="2.2" '
            'fill="#0b4f6c"/>'.format(x, y))

    robot_x = margin_left + plot_width / 2.0
    robot_y = margin_top + plot_height
    lines.extend([
        '<polygon points="{0:.1f},{1:.1f} {2:.1f},{3:.1f} '
        '{4:.1f},{3:.1f}" fill="#c62828"/>'.format(
            robot_x, robot_y - 16,
            robot_x - 10, robot_y + 4,
            robot_x + 10),
        '<text x="{}" y="{}" text-anchor="middle" '
        'font-family="sans-serif" font-size="13" fill="#c62828">'
        'robot / forward up</text>'.format(robot_x, robot_y + 42),
        '</svg>',
    ])

    with open(path, "w") as stream:
        stream.write("\n".join(lines))


def main():
    args = parse_args()
    scans = capture_scans(args)
    points, minimum_frames = aggregate_points(scans, args)

    output_directory = os.path.dirname(args.output_prefix)
    if output_directory:
        os.makedirs(output_directory, exist_ok=True)

    csv_path = args.output_prefix + ".csv"
    svg_path = args.output_prefix + ".svg"
    write_csv(csv_path, points)
    write_svg(svg_path, points, len(scans), minimum_frames, args)

    print("captured_frames:", len(scans))
    print("stable_points:", len(points))
    print("csv:", csv_path)
    print("svg:", svg_path)
    if not points:
        raise RuntimeError("no stable points were available for export")


if __name__ == "__main__":
    main()
