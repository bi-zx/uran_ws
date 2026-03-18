"""uran_media_node.py — URAN-media 主节点（T4.1 + T4.2 + T4.3 + T4.4）

WebRTC 实现策略：
  - source_type=camera_service（CyberDog2 主摄像头）：
      信令中继模式 —— 将云端 SDP/ICE 转发给 image_transmission 节点，
      由 image_transmission 完成实际 WebRTC 推流，无需 aiortc。
  - source_type=ros_topic（通用摄像头）：
      aiortc 模式（若 aiortc 不可用则 stub）。
"""

import asyncio
import json
import os
import queue
import threading
import time
from typing import Dict, Set

import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from uran_msgs.msg import MediaCtrlCmd, MediaSwitchCmd, UplinkPayload, StateField

from .webrtc_channel import WebRTCChannel
from .rtsp_server import RTSPServer
from .cyberdog2_camera import CyberDog2CameraAdapter
from . import cyberdog2_webrtc_bridge as bridge


def _load_yaml(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


class UranMediaNode(Node):

    def __init__(self):
        super().__init__('uran_media_node')

        self._cfg: dict = {}
        self._sources: Dict[str, dict] = {}
        self._cyberdog_ns: str = ''  # e.g. "/mi_desktop_48_b0_2d_5f_b6_d0"

        # WebRTC：桥接通道（camera_service）
        self._bridge_channels: Set[str] = set()

        # WebRTC：aiortc 通道（ros_topic，aiortc 可用时）
        self._webrtc_channels: Dict[str, WebRTCChannel] = {}

        self._rtsp_server = RTSPServer()
        self._cyberdog_adapters: Dict[str, CyberDog2CameraAdapter] = {}
        self._image_subs: Dict[str, object] = {}
        self._recorders: Dict[str, object] = {}

        # asyncio 事件循环（供 aiortc 使用）
        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._run_loop, daemon=True).start()

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
        # image_transmission 信令出口（CyberDog2 WebRTC 应答）
        ns = self._cyberdog_ns
        self.create_subscription(
            String,
            f'{ns}/img_trans_signal_out' if ns else 'img_trans_signal_out',
            self._cb_img_trans_out,
            10,
        )

        # ---------- 发布 ----------
        self._uplink_pub = self.create_publisher(UplinkPayload, '/uran/core/uplink/data', 10)
        self._state_pub = self.create_publisher(StateField, '/uran/core/state/write', 10)
        # image_transmission 信令入口
        _sig_in = f'{ns}/img_trans_signal_in' if ns else 'img_trans_signal_in'
        self._img_trans_pub = self.create_publisher(String, _sig_in, 10)
        self.get_logger().info(
            f'img_trans topics: in={_sig_in}, '
            f'out={ns + "/img_trans_signal_out" if ns else "img_trans_signal_out"}'
        )

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
        self._cyberdog_ns = self._cfg.get('cyberdog_ns', '').rstrip('/')

        for src in self._cfg.get('video_sources', []):
            cid = src.get('channel_id')
            if cid:
                self._sources[cid] = src

        self.get_logger().info(
            f'Loaded {len(self._sources)} video sources: {list(self._sources.keys())}, '
            f'cyberdog_ns="{self._cyberdog_ns}"'
        )

    # ================================================================== asyncio 线程（aiortc）
    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _run_coro(self, coro):
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
            for cid in self._sources:
                if protocol == 'webrtc':
                    self._start_webrtc(cid)
                elif protocol == 'rtsp':
                    self._start_rtsp(cid)

    # ================================================================== WebRTC 启动
    def _start_webrtc(self, channel_id: str):
        if channel_id not in self._sources:
            self.get_logger().warning(f'Unknown channel_id: {channel_id}')
            return

        if channel_id in self._bridge_channels or channel_id in self._webrtc_channels:
            self.get_logger().info(f'WebRTC channel already active: {channel_id}')
            return

        src = self._sources[channel_id]

        # CyberDog2 camera_service → 信令桥接模式
        if src.get('source_type') == 'camera_service':
            self._start_webrtc_bridge(channel_id, src)
        else:
            # 通用 ros_topic → aiortc 模式
            self._start_webrtc_aiortc(channel_id, src)

    def _start_webrtc_bridge(self, channel_id: str, src: dict):
        """CyberDog2 桥接模式：激活摄像头，注册通道，等待云端发来 SDP offer。"""
        svc_name = src.get('camera_service_name', 'camera_service')
        if self._cyberdog_ns and not svc_name.startswith('/'):
            svc_name = f'{self._cyberdog_ns}/{svc_name}'

        adapter = CyberDog2CameraAdapter(
            node=self,
            service_name=svc_name,
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
        self._bridge_channels.add(channel_id)
        self._update_state()

        self.get_logger().info(
            f'[bridge] WebRTC channel ready: {channel_id}. '
            'Waiting for SDP offer from cloud via signal_json.'
        )

    def _start_webrtc_aiortc(self, channel_id: str, src: dict):
        """通用 aiortc 模式：生成 SDP Offer 并上报。"""
        stun = self._cfg.get('webrtc', {}).get('stun_server', 'stun:stun.l.google.com:19302')
        channel = WebRTCChannel(
            channel_id=channel_id,
            stun_server=stun,
            on_signal_cb=self._on_webrtc_signal,
        )
        self._webrtc_channels[channel_id] = channel

        future = self._run_coro(channel.start())
        future.add_done_callback(lambda f: self._on_offer_ready(channel_id, f))

        self._subscribe_topic(channel_id, src)
        self._update_state()

    def _on_offer_ready(self, channel_id: str, future):
        try:
            offer_json = future.result()
            self._publish_uplink(
                data_type='media_signal',
                payload={'channel_id': channel_id, 'signal': json.loads(offer_json)},
            )
            self.get_logger().info(f'SDP Offer published for channel: {channel_id}')
        except Exception as e:
            self.get_logger().error(f'Failed to generate SDP offer for {channel_id}: {e}')

    def _on_webrtc_signal(self, channel_id: str, signal: dict):
        """aiortc ICE candidate 回调 → 上报到 uplink。"""
        self._publish_uplink(
            data_type='media_signal',
            payload={'channel_id': channel_id, 'signal': signal},
        )

    # ================================================================== 信令处理
    def _handle_signal(self, channel_id: str, signal_json: str):
        """处理来自云端的 SDP offer/answer 或 ICE Candidate。"""
        try:
            signal = json.loads(signal_json)
        except json.JSONDecodeError:
            self.get_logger().warning(f'Invalid signal_json for {channel_id}')
            return

        # 桥接模式：转发给 image_transmission
        if channel_id in self._bridge_channels:
            raw = bridge.to_img_trans(channel_id, signal)
            if raw:
                msg = String()
                msg.data = raw
                self._img_trans_pub.publish(msg)
                self.get_logger().debug(
                    f'[bridge] Forwarded signal type={signal.get("type")} for {channel_id}'
                )
            return

        # aiortc 模式
        channel = self._webrtc_channels.get(channel_id)
        if channel is None:
            self.get_logger().warning(f'No active WebRTC channel for {channel_id}')
            return

        sig_type = signal.get('type', '')
        if sig_type == 'answer':
            self._run_coro(channel.set_answer(signal_json))
        elif sig_type == 'candidate':
            self._run_coro(channel.add_ice_candidate(signal_json))
        else:
            self.get_logger().warning(f'Unknown signal type: {sig_type}')

    # ================================================================== image_transmission 应答回调
    def _cb_img_trans_out(self, msg: String):
        """接收 image_transmission 的信令应答，转发给云端。"""
        uid, signal = bridge.from_img_trans(msg.data)
        if uid is None or signal is None:
            return

        # uid 即 channel_id
        if uid not in self._bridge_channels:
            return

        sig_type = signal.get('type', '')

        if sig_type in ('answer', 'candidate', 'offer'):
            self._publish_uplink(
                data_type='media_signal',
                payload={'channel_id': uid, 'signal': signal},
            )
            self.get_logger().debug(
                f'[bridge] Forwarded {sig_type} from image_transmission for {uid}'
            )
        elif sig_type == 'closed':
            self.get_logger().info(f'[bridge] image_transmission closed channel: {uid}')
            self._bridge_channels.discard(uid)
            self._update_state()
        elif sig_type == 'error':
            self.get_logger().error(
                f'[bridge] image_transmission error for {uid}: {signal.get("error")}'
            )
            self._publish_uplink(
                data_type='media_signal',
                payload={'channel_id': uid, 'signal': signal},
            )

    # ================================================================== RTSP（T4.3）
    def _start_rtsp(self, channel_id: str):
        if channel_id not in self._sources:
            self.get_logger().warning(f'Unknown channel_id: {channel_id}')
            return

        src = self._sources[channel_id]

        if src.get('source_type') == 'camera_service' and channel_id not in self._cyberdog_adapters:
            svc_name = src.get('camera_service_name', 'camera_service')
            if self._cyberdog_ns and not svc_name.startswith('/'):
                svc_name = f'{self._cyberdog_ns}/{svc_name}'
            adapter = CyberDog2CameraAdapter(
                node=self,
                service_name=svc_name,
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
        )

        self._subscribe_topic(channel_id, src)
        self._update_state()
        self.get_logger().info(f'RTSP started: {url}')

    # ================================================================== 停止
    def _stop_channel(self, channel_id: str):
        # 桥接模式：发送 is_closed 给 image_transmission
        if channel_id in self._bridge_channels:
            msg = String()
            msg.data = bridge.make_stop(channel_id)
            self._img_trans_pub.publish(msg)
            self._bridge_channels.discard(channel_id)

        # aiortc 模式
        if channel_id in self._webrtc_channels:
            self._run_coro(self._webrtc_channels[channel_id].stop())
            del self._webrtc_channels[channel_id]

        self._rtsp_server.stop(channel_id)
        self._stop_record(channel_id)

        if channel_id in self._image_subs:
            self.destroy_subscription(self._image_subs.pop(channel_id))

        if channel_id in self._cyberdog_adapters:
            self._cyberdog_adapters[channel_id].deactivate()
            del self._cyberdog_adapters[channel_id]

        self._update_state()
        self.get_logger().info(f'Channel stopped: {channel_id}')

    def _stop_all_channels(self):
        for cid in list(self._bridge_channels) + list(self._webrtc_channels.keys()):
            self._stop_channel(cid)
        self._rtsp_server.stop_all()

    # ================================================================== 视频 topic 订阅
    def _subscribe_topic(self, channel_id: str, src: dict):
        if channel_id in self._image_subs:
            return

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
                    f'Unsupported msg_type {msg_type_str} for {channel_id}, skipping'
                )
                return

            self._image_subs[channel_id] = sub
            self.get_logger().info(f'Subscribed to {topic} for channel {channel_id}')
        except Exception as e:
            self.get_logger().error(f'Failed to subscribe {topic}: {e}')

    # ================================================================== 图像回调
    def _cb_image(self, channel_id: str, msg):
        try:
            import numpy as np
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        except Exception:
            return
        self._dispatch_frame(channel_id, frame)

    def _cb_compressed_image(self, channel_id: str, msg):
        try:
            import numpy as np
            import cv2
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if frame is None:
                return
        except Exception:
            return
        self._dispatch_frame(channel_id, frame)

    def _dispatch_frame(self, channel_id: str, frame):
        # aiortc 通道（桥接模式不需要帧，image_transmission 自己读摄像头）
        if channel_id in self._webrtc_channels:
            self._webrtc_channels[channel_id].push_frame(frame)

        self._rtsp_server.push_frame(channel_id, frame)

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
            self._publish_uplink(
                data_type='media_upload',
                payload={'channel_id': channel_id, 'status': 'ready'},
            )
        except Exception as e:
            self.get_logger().error(f'Error stopping recorder for {channel_id}: {e}')

    # ================================================================== 状态写入
    def _update_state(self):
        active_bridge = len(self._bridge_channels)
        active_aiortc = len(self._webrtc_channels)
        active_rtsp = len(self._rtsp_server._channels)
        total = active_bridge + active_aiortc + active_rtsp

        if active_bridge + active_aiortc > 0:
            protocol = 'webrtc'
        elif active_rtsp > 0:
            protocol = 'rtsp'
        else:
            protocol = 'none'

        self._write_state('media_active_protocol', protocol)
        self._write_state('media_channel_count', total)

    # ================================================================== helpers
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
