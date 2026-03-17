"""uran_media_node.py — URAN-media 主节点（T4.1 + T4.2 + T4.3 + T4.4）

职责：
  - 动态订阅视频源 topic（T4.1）
  - WebRTC 通道管理，SDP 信令上报（T4.2）
  - RTSP Server 通道管理（T4.3）
  - 本地录制（T4.4）
  - CyberDog2 camera_service 适配
"""

import asyncio
import json
import os
import queue
import threading
import time
from typing import Dict, Optional

import rclpy
import rclpy.executors
from rclpy.node import Node
import yaml

from uran_msgs.msg import MediaCtrlCmd, MediaSwitchCmd, UplinkPayload, StateField

from .webrtc_channel import WebRTCChannel
from .rtsp_server import RTSPServer
from .cyberdog2_camera import CyberDog2CameraAdapter


def _load_yaml(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


class UranMediaNode(Node):

    def __init__(self):
        super().__init__('uran_media_node')

        self._cfg: dict = {}
        self._sources: Dict[str, dict] = {}       # channel_id -> source config
        self._webrtc_channels: Dict[str, WebRTCChannel] = {}
        self._rtsp_server = RTSPServer()
        self._cyberdog_adapters: Dict[str, CyberDog2CameraAdapter] = {}
        self._image_subs: Dict[str, object] = {}  # channel_id -> subscription
        self._recorders: Dict[str, object] = {}   # channel_id -> cv2.VideoWriter

        # asyncio 事件循环（独立线程，供 aiortc 使用）
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._loop_thread.start()

        self._load_config()

        # ---------- 订阅 ----------
        self.create_subscription(
            MediaCtrlCmd,
            '/uran/core/downlink/media_ctrl',
            self._cb_media_ctrl,
            10,
        )
        self.create_subscription(
            MediaSwitchCmd,
            '/uran/core/switch/media',
            self._cb_media_switch,
            10,
        )

        # ---------- 发布 ----------
        self._uplink_pub = self.create_publisher(UplinkPayload, '/uran/core/uplink/data', 10)
        self._state_pub = self.create_publisher(StateField, '/uran/core/state/write', 10)

        self.get_logger().info('uran_media_node started')

    # ================================================================== 配置加载
    def _load_config(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('uran_media')
        except Exception:
            share = os.path.join(os.path.dirname(__file__), '..', '..')

        cfg_raw = _load_yaml(os.path.join(share, 'config', 'media.yaml'))
        self._cfg = cfg_raw.get('uran_media', {})

        for src in self._cfg.get('video_sources', []):
            cid = src.get('channel_id')
            if cid:
                self._sources[cid] = src

        self.get_logger().info(
            f'Loaded {len(self._sources)} video sources: {list(self._sources.keys())}'
        )

    # ================================================================== asyncio 线程
    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _run_coro(self, coro):
        """在 asyncio 线程中调度协程，返回 Future。"""
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    # ================================================================== 下行回调
    def _cb_media_ctrl(self, cmd: MediaCtrlCmd):
        action = cmd.action
        channel_id = cmd.channel_id
        protocol = cmd.protocol
        signal_json = cmd.signal_json

        self.get_logger().info(
            f'media_ctrl: action={action}, channel={channel_id}, protocol={protocol}'
        )

        if action == 'start':
            if protocol == 'webrtc':
                self._start_webrtc(channel_id)
            elif protocol == 'rtsp':
                self._start_rtsp(channel_id)
            else:
                self.get_logger().warning(f'Unknown protocol: {protocol}')

        elif action == 'stop':
            self._stop_channel(channel_id)

        elif action == 'switch':
            self._stop_all_channels()
            if protocol == 'webrtc':
                self._start_webrtc(channel_id)
            elif protocol == 'rtsp':
                self._start_rtsp(channel_id)

        elif action == 'record_start':
            self._start_record(channel_id)

        elif action == 'record_stop':
            self._stop_record(channel_id)

        elif signal_json and signal_json not in ('', '{}'):
            self._handle_signal(channel_id, signal_json)

    def _cb_media_switch(self, cmd: MediaSwitchCmd):
        action = cmd.action
        protocol = cmd.protocol
        self.get_logger().info(f'media_switch: action={action}, protocol={protocol}')

        if action == 'stop_all':
            self._stop_all_channels()
        elif action == 'start':
            # 对所有已知 source 启动指定协议
            for cid in self._sources:
                if protocol == 'webrtc':
                    self._start_webrtc(cid)
                elif protocol == 'rtsp':
                    self._start_rtsp(cid)

    # ================================================================== WebRTC（T4.2）
    def _start_webrtc(self, channel_id: str):
        if channel_id not in self._sources:
            self.get_logger().warning(f'Unknown channel_id: {channel_id}')
            return

        if channel_id in self._webrtc_channels:
            self.get_logger().info(f'WebRTC channel already active: {channel_id}')
            return

        src = self._sources[channel_id]

        # CyberDog2 camera_service 激活
        if src.get('source_type') == 'camera_service':
            adapter = CyberDog2CameraAdapter(
                node=self,
                service_name=src.get('camera_service_name', 'camera_service'),
                width=src.get('width', 1280),
                height=src.get('height', 960),
                fps=src.get('fps', 10),
            )
            ok = adapter.activate()
            if not ok:
                self.get_logger().error(
                    f'Failed to activate CyberDog2 camera for channel {channel_id}'
                )
                return
            self._cyberdog_adapters[channel_id] = adapter

        stun = self._cfg.get('webrtc', {}).get('stun_server', 'stun:stun.l.google.com:19302')
        channel = WebRTCChannel(
            channel_id=channel_id,
            stun_server=stun,
            on_signal_cb=self._on_webrtc_signal,
        )
        self._webrtc_channels[channel_id] = channel

        # 异步生成 SDP Offer
        future = self._run_coro(channel.start())
        future.add_done_callback(
            lambda f: self._on_offer_ready(channel_id, f)
        )

        # 订阅视频 topic
        self._subscribe_topic(channel_id, src)
        self._update_state()

    def _on_offer_ready(self, channel_id: str, future):
        try:
            offer_json = future.result()
            self._publish_uplink(
                data_type='media_signal',
                payload={
                    'channel_id': channel_id,
                    'signal': json.loads(offer_json),
                },
                urgent=False,
            )
            self.get_logger().info(f'SDP Offer published for channel: {channel_id}')
        except Exception as e:
            self.get_logger().error(f'Failed to generate SDP offer for {channel_id}: {e}')

    def _on_webrtc_signal(self, channel_id: str, signal: dict):
        """WebRTC ICE candidate 回调 → 上报到 uplink。"""
        self._publish_uplink(
            data_type='media_signal',
            payload={'channel_id': channel_id, 'signal': signal},
            urgent=False,
        )

    def _handle_signal(self, channel_id: str, signal_json: str):
        """处理来自云端的 SDP Answer 或 ICE Candidate。"""
        try:
            data = json.loads(signal_json)
        except json.JSONDecodeError:
            self.get_logger().warning(f'Invalid signal_json for {channel_id}')
            return

        sig_type = data.get('type', '')
        channel = self._webrtc_channels.get(channel_id)
        if channel is None:
            self.get_logger().warning(f'No active WebRTC channel for {channel_id}')
            return

        if sig_type == 'answer':
            self._run_coro(channel.set_answer(signal_json))
        elif sig_type == 'candidate':
            self._run_coro(channel.add_ice_candidate(signal_json))
        else:
            self.get_logger().warning(f'Unknown signal type: {sig_type}')

    # ================================================================== RTSP（T4.3）
    def _start_rtsp(self, channel_id: str):
        if channel_id not in self._sources:
            self.get_logger().warning(f'Unknown channel_id: {channel_id}')
            return

        src = self._sources[channel_id]

        # CyberDog2 camera_service 激活
        if src.get('source_type') == 'camera_service' and channel_id not in self._cyberdog_adapters:
            adapter = CyberDog2CameraAdapter(
                node=self,
                service_name=src.get('camera_service_name', 'camera_service'),
                width=src.get('width', 1280),
                height=src.get('height', 960),
                fps=src.get('fps', 10),
            )
            adapter.activate()
            self._cyberdog_adapters[channel_id] = adapter

        url = self._rtsp_server.start(
            channel_id=channel_id,
            width=src.get('width', 640),
            height=src.get('height', 480),
            fps=src.get('fps', 30),
        )

        self._publish_uplink(
            data_type='media_rtsp_url',
            payload={'channel_id': channel_id, 'url': url},
            urgent=False,
        )

        self._subscribe_topic(channel_id, src)
        self._update_state()
        self.get_logger().info(f'RTSP started: {url}')

    # ================================================================== 停止
    def _stop_channel(self, channel_id: str):
        # 停止 WebRTC
        if channel_id in self._webrtc_channels:
            self._run_coro(self._webrtc_channels[channel_id].stop())
            del self._webrtc_channels[channel_id]

        # 停止 RTSP
        self._rtsp_server.stop(channel_id)

        # 停止录制
        self._stop_record(channel_id)

        # 取消订阅
        if channel_id in self._image_subs:
            self.destroy_subscription(self._image_subs.pop(channel_id))

        # 停用 CyberDog2 摄像头
        if channel_id in self._cyberdog_adapters:
            self._cyberdog_adapters[channel_id].deactivate()
            del self._cyberdog_adapters[channel_id]

        self._update_state()
        self.get_logger().info(f'Channel stopped: {channel_id}')

    def _stop_all_channels(self):
        for cid in list(self._webrtc_channels.keys()):
            self._stop_channel(cid)
        self._rtsp_server.stop_all()

    # ================================================================== 视频 topic 订阅
    def _subscribe_topic(self, channel_id: str, src: dict):
        if channel_id in self._image_subs:
            return  # 已订阅

        topic = src.get('ros_topic', '')
        msg_type_str = src.get('msg_type', 'sensor_msgs/Image')

        if not topic:
            return

        try:
            if msg_type_str == 'sensor_msgs/Image':
                from sensor_msgs.msg import Image
                sub = self.create_subscription(
                    Image,
                    topic,
                    lambda msg, cid=channel_id: self._cb_image(cid, msg),
                    10,
                )
            elif msg_type_str == 'sensor_msgs/CompressedImage':
                from sensor_msgs.msg import CompressedImage
                sub = self.create_subscription(
                    CompressedImage,
                    topic,
                    lambda msg, cid=channel_id: self._cb_compressed_image(cid, msg),
                    10,
                )
            else:
                self.get_logger().warning(
                    f'Unsupported msg_type {msg_type_str} for channel {channel_id}, skipping subscription'
                )
                return

            self._image_subs[channel_id] = sub
            self.get_logger().info(f'Subscribed to {topic} for channel {channel_id}')
        except Exception as e:
            self.get_logger().error(f'Failed to subscribe {topic}: {e}')

    # ================================================================== 图像回调
    def _cb_image(self, channel_id: str, msg):
        """sensor_msgs/Image → numpy → push to WebRTC/RTSP/recorder。"""
        try:
            import numpy as np
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1
            )
        except Exception:
            return

        self._dispatch_frame(channel_id, frame, msg)

    def _cb_compressed_image(self, channel_id: str, msg):
        """sensor_msgs/CompressedImage → numpy → push to WebRTC/RTSP/recorder。"""
        try:
            import numpy as np
            import cv2
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if frame is None:
                return
        except Exception:
            return

        self._dispatch_frame(channel_id, frame, msg)

    def _dispatch_frame(self, channel_id: str, frame, msg):
        # WebRTC
        if channel_id in self._webrtc_channels:
            self._webrtc_channels[channel_id].push_frame(frame)

        # RTSP
        self._rtsp_server.push_frame(channel_id, frame)

        # 录制
        if channel_id in self._recorders:
            try:
                self._recorders[channel_id].write(frame)
            except Exception:
                pass

    # ================================================================== 本地录制（T4.4）
    def _start_record(self, channel_id: str):
        if channel_id in self._recorders:
            self.get_logger().info(f'Already recording: {channel_id}')
            return

        record_cfg = self._cfg.get('local_record', {})
        storage_path = record_cfg.get('storage_path', '/tmp/uran_media_record')
        os.makedirs(storage_path, exist_ok=True)

        src = self._sources.get(channel_id, {})
        width = src.get('width', 640)
        height = src.get('height', 480)
        fps = src.get('fps', 30)

        try:
            import cv2
            ts = int(time.time())
            filename = os.path.join(storage_path, f'{channel_id}_{ts}.avi')
            writer = cv2.VideoWriter(
                filename,
                cv2.VideoWriter_fourcc(*'XVID'),
                fps,
                (width, height),
            )
            self._recorders[channel_id] = writer
            self.get_logger().info(f'Recording started: {filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to start recording for {channel_id}: {e}')

    def _stop_record(self, channel_id: str):
        writer = self._recorders.pop(channel_id, None)
        if writer is None:
            return
        try:
            writer.release()
            self.get_logger().info(f'Recording stopped: {channel_id}')
            # 触发上传通知
            self._publish_uplink(
                data_type='media_upload',
                payload={'channel_id': channel_id, 'status': 'ready'},
                urgent=False,
            )
        except Exception as e:
            self.get_logger().error(f'Error stopping recorder for {channel_id}: {e}')

    # ================================================================== 状态写入
    def _update_state(self):
        active_webrtc = len(self._webrtc_channels)
        active_rtsp = len([
            cid for cid in self._sources
            if cid not in self._webrtc_channels
            and self._rtsp_server._channels.get(cid) is not None
        ])
        total = active_webrtc + active_rtsp

        if active_webrtc > 0:
            protocol = 'webrtc'
        elif active_rtsp > 0:
            protocol = 'rtsp'
        else:
            protocol = 'none'

        self._write_state('media_active_protocol', protocol)
        self._write_state('media_channel_count', total)

    # ================================================================== 上报 / 状态写入 helpers
    def _publish_uplink(self, data_type: str, payload: dict, urgent: bool = False):
        msg = UplinkPayload()
        msg.source_pkg = 'uran_media'
        msg.data_type = data_type
        msg.preferred_protocol = ''
        msg.payload_json = json.dumps(payload)
        msg.urgent = urgent
        msg.timestamp_ns = self.get_clock().now().nanoseconds
        self._uplink_pub.publish(msg)

    def _write_state(self, field_name: str, value, persistent: bool = False, urgent: bool = False):
        msg = StateField()
        msg.field_name = field_name
        msg.value_json = json.dumps(value)
        msg.persistent = persistent
        msg.urgent = urgent
        msg.source_pkg = 'uran_media'
        msg.timestamp_ns = self.get_clock().now().nanoseconds
        self._state_pub.publish(msg)

    # ================================================================== 生命周期
    def destroy_node(self):
        self._stop_all_channels()
        self._loop.call_soon_threadsafe(self._loop.stop)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UranMediaNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
