#!/usr/bin/env python3
"""test_realsense.py — RealSense 摄像头实机测试脚本

用法：
  python3 test_realsense.py                  # 假设节点已运行，跳过 lifecycle
  python3 test_realsense.py --launch         # 自动 launch realsense 节点 + lifecycle 激活
  python3 test_realsense.py --depth-only     # 只显示深度图
  python3 test_realsense.py --launch --align # 同时显示 aligned 深度图（需 camera_align 节点）

ESC 或 Ctrl+C 退出。
"""

import argparse
import os
import subprocess
import sys
import threading
import time

# 抑制 OpenCV GStreamer 后端的 GLib 噪音
os.environ.setdefault('GST_DEBUG', '0')
os.environ.setdefault('OPENCV_VIDEOIO_PRIORITY_GSTREAMER', '0')

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

CHANNELS = {
    'left':         '/camera/infra1/image_rect_raw',
    'right':        '/camera/infra2/image_rect_raw',
    'depth':        '/camera/depth/image_rect_raw',
    'aligned':      '/camera/aligned_depth_to_infra1/image_raw',
    'aligned_rgb':  '/camera/aligned_depth_to_rgb/image_raw',
}


# ── lifecycle ─────────────────────────────────────────────────────────────────
def lifecycle_transition(node: Node, lc_node: str, transition_id: int, name: str):
    from lifecycle_msgs.srv import ChangeState
    svc = f'{lc_node}/change_state'
    cli = node.create_client(ChangeState, svc)
    if not cli.wait_for_service(timeout_sec=8.0):
        node.get_logger().warning(f'[lifecycle] service not found: {svc}')
        return False
    req = ChangeState.Request()
    req.transition.id = transition_id
    future = cli.call_async(req)
    deadline = time.time() + 10.0
    while not future.done() and time.time() < deadline:
        time.sleep(0.05)
    if future.done():
        ok = future.result().success
        node.get_logger().info(f'[lifecycle] {name}: {"OK" if ok else "skipped (already in state)"}')
        return ok
    node.get_logger().warning(f'[lifecycle] {name} timeout')
    return False


def activate_lifecycle(node: Node, lc_node: str):
    lifecycle_transition(node, lc_node, 1, 'configure')  # CONFIGURE
    lifecycle_transition(node, lc_node, 3, 'activate')   # ACTIVATE


# ── 图像解码 ──────────────────────────────────────────────────────────────────
def to_bgr(msg: Image) -> np.ndarray:
    enc = msg.encoding
    if enc in ('16UC1', '16SC1'):
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        _, max_val, _, _ = cv2.minMaxLoc(raw)
        norm = (raw.astype(np.float32) * 255.0 / max_val).astype(np.uint8) if max_val > 0 \
               else np.zeros((msg.height, msg.width), dtype=np.uint8)
        return cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    elif enc in ('mono8', '8UC1'):
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        return cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
    elif enc == 'rgb8':
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
    else:
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)


# ── 主节点 ────────────────────────────────────────────────────────────────────
class RealSenseViewer(Node):
    def __init__(self, channels: list):
        super().__init__('realsense_viewer')
        self._frames = {}
        self._lock = threading.Lock()
        self._fps = {ch: {'count': 0, 't': time.time(), 'val': 0.0} for ch in channels}
        self._last_frame_t = {ch: None for ch in channels}

        for ch in channels:
            topic = CHANNELS[ch]
            self.create_subscription(Image, topic,
                                     lambda msg, c=ch: self._cb(c, msg),
                                     SENSOR_QOS)
            self.get_logger().info(f'Subscribed [{ch}]: {topic}')
            cv2.namedWindow(f'RealSense [{ch}]', cv2.WINDOW_NORMAL)
            cv2.resizeWindow(f'RealSense [{ch}]', 640, 480)

        self._channels = channels
        self.create_timer(0.03, self._refresh)
        # 5 秒无帧则提示
        self.create_timer(5.0, self._check_frames)

    def _cb(self, ch: str, msg: Image):
        try:
            frame = to_bgr(msg)
        except Exception as e:
            self.get_logger().warning(f'[{ch}] decode error: {e}')
            return
        s = self._fps[ch]
        s['count'] += 1
        now = time.time()
        elapsed = now - s['t']
        if elapsed >= 1.0:
            s['val'] = s['count'] / elapsed
            s['count'] = 0
            s['t'] = now
        self._last_frame_t[ch] = now
        with self._lock:
            self._frames[ch] = frame

    def _refresh(self):
        with self._lock:
            frames = {k: v.copy() for k, v in self._frames.items()}
        for ch, frame in frames.items():
            label = f"FPS:{self._fps[ch]['val']:.1f}  {frame.shape[1]}x{frame.shape[0]}"
            cv2.putText(frame, label, (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow(f'RealSense [{ch}]', frame)
        if cv2.waitKey(1) == 27:
            rclpy.shutdown()

    def _check_frames(self):
        for ch in self._channels:
            if self._last_frame_t[ch] is None:
                self.get_logger().warning(
                    f'[{ch}] No frames yet on {CHANNELS[ch]}\n'
                    '  → 确认 RealSense 节点已启动: ros2 launch realsense2_camera on_dog.py\n'
                    '  → 确认 lifecycle 已激活:     ros2 lifecycle set /camera/camera activate\n'
                    f'  → 检查 topic:               ros2 topic hz {CHANNELS[ch]}'
                )


# ── 入口 ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--launch', action='store_true',
                        help='自动 launch realsense 节点（ros2 launch realsense2_camera on_dog.py）')
    parser.add_argument('--depth-only', action='store_true', help='只显示深度图')
    parser.add_argument('--align', action='store_true', help='额外显示 aligned 深度图')
    args = parser.parse_args()

    launch_proc = None
    if args.launch:
        print('[launch] Starting realsense2_camera on_dog.py ...')
        launch_proc = subprocess.Popen(
            ['ros2', 'launch', 'realsense2_camera', 'on_dog.py'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        print('[launch] Waiting 5s for node to initialize ...')
        time.sleep(5)

    if args.depth_only:
        channels = ['depth']
    else:
        channels = ['left', 'right', 'depth']
        if args.align:
            channels.append('aligned')

    rclpy.init()
    node = RealSenseViewer(channels)

    if args.launch:
        print('[lifecycle] Activating /camera/camera ...')
        activate_lifecycle(node, '/camera/camera')
        if args.align:
            activate_lifecycle(node, '/camera/camera_align')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        if launch_proc:
            launch_proc.terminate()


if __name__ == '__main__':
    main()
