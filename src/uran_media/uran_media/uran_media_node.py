"""uran_media_node.py — ROS1 版本媒体主节点。"""

import asyncio
import json
import os
import threading
import time
from typing import Dict

import rospy
import rospkg
import yaml
from sensor_msgs.msg import CompressedImage, Image

from uran_msgs.msg import MediaCtrlCmd, MediaSwitchCmd, UplinkPayload, StateField

from .rtsp_server import RTSPServer
from .webrtc_channel import WebRTCChannel


def _load_yaml(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


class UranMediaNode:
    def __init__(self):
        self._cfg = {}
        self._sources: Dict[str, dict] = {}
        self._webrtc_channels: Dict[str, WebRTCChannel] = {}
        self._image_subs: Dict[str, object] = {}
        self._recorders: Dict[str, object] = {}
        self._rtsp_server = None

        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._run_loop, daemon=True).start()

        self._load_config()

        self._uplink_pub = rospy.Publisher('/uran/core/uplink/data', UplinkPayload, queue_size=10)
        self._state_pub = rospy.Publisher('/uran/core/state/write', StateField, queue_size=10)

        rospy.Subscriber('/uran/core/downlink/media_ctrl', MediaCtrlCmd, self._cb_media_ctrl, queue_size=10)
        rospy.Subscriber('/uran/core/switch/media', MediaSwitchCmd, self._cb_media_switch, queue_size=10)

        rospy.loginfo('uran_media_node started')

    def _find_share_dir(self):
        try:
            return rospkg.RosPack().get_path('uran_media')
        except Exception:
            return os.path.join(os.path.dirname(__file__), '..', '..')

    def _load_config(self):
        share = self._find_share_dir()
        cfg_raw = _load_yaml(os.path.join(share, 'config', 'media.yaml'))
        self._cfg = cfg_raw.get('uran_media', {})
        rtsp_cfg = self._cfg.get('rtsp', {})
        self._rtsp_server = RTSPServer(
            port=int(rtsp_cfg.get('port', 8554)),
            encoder_pipeline=rtsp_cfg.get('encoder_pipeline', ''),
        )
        for src in self._cfg.get('video_sources', []):
            cid = src.get('channel_id')
            if cid:
                self._sources[cid] = src
        rospy.loginfo('Loaded %d video sources: %s', len(self._sources), list(self._sources.keys()))

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _run_coro(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    def _cb_media_ctrl(self, cmd: MediaCtrlCmd):
        if cmd.action == 'start':
            if cmd.protocol == 'webrtc':
                self._start_webrtc(cmd.channel_id)
            elif cmd.protocol == 'rtsp':
                self._start_rtsp(cmd.channel_id)
        elif cmd.action == 'stop':
            if cmd.channel_id:
                self._stop_channel(cmd.channel_id)
            else:
                self._stop_all_channels()
        elif cmd.action == 'switch':
            self._stop_all_channels()
            if cmd.protocol == 'webrtc':
                self._start_webrtc(cmd.channel_id)
            elif cmd.protocol == 'rtsp':
                self._start_rtsp(cmd.channel_id)
        elif cmd.action == 'record_start':
            self._start_record(cmd.channel_id)
        elif cmd.action == 'record_stop':
            self._stop_record(cmd.channel_id)
        elif cmd.signal_json and cmd.signal_json not in ('', '{}'):
            self._handle_signal(cmd.channel_id, cmd.signal_json)

    def _cb_media_switch(self, cmd: MediaSwitchCmd):
        if cmd.action == 'stop_all':
            self._stop_all_channels()
        elif cmd.action == 'start':
            for cid in self._sources:
                if cmd.protocol == 'webrtc':
                    self._start_webrtc(cid)
                elif cmd.protocol == 'rtsp':
                    self._start_rtsp(cid)

    def _start_webrtc(self, channel_id: str):
        if channel_id not in self._sources:
            rospy.logwarn('Unknown channel_id: %s', channel_id)
            return
        if channel_id in self._webrtc_channels:
            rospy.loginfo('WebRTC channel already active: %s', channel_id)
            return
        src = self._sources[channel_id]
        if src.get('source_type') != 'ros_topic':
            rospy.logwarn('Unsupported source_type for WebRTC: %s', src.get('source_type'))
            return

        webrtc_cfg = self._cfg.get('webrtc', {})
        channel = WebRTCChannel(
            channel_id=channel_id,
            stun_server=webrtc_cfg.get('stun_server', 'stun:stun.l.google.com:19302'),
            turn_servers=webrtc_cfg.get('turn_servers', []),
            on_signal_cb=self._on_webrtc_signal,
        )
        self._webrtc_channels[channel_id] = channel
        future = self._run_coro(channel.start())
        future.add_done_callback(lambda f, cid=channel_id: self._on_offer_ready(cid, f))
        self._subscribe_topic(channel_id, src)
        self._update_state()

    def _on_offer_ready(self, channel_id: str, future):
        try:
            offer_json = future.result()
            self._publish_uplink('media_signal', {
                'channel_id': channel_id,
                'signal': json.loads(offer_json),
            })
            rospy.loginfo('SDP Offer published for channel: %s', channel_id)
        except Exception as exc:
            rospy.logerr('Failed to generate SDP offer for %s: %s', channel_id, exc)

    def _on_webrtc_signal(self, channel_id: str, signal: dict):
        self._publish_uplink('media_signal', {'channel_id': channel_id, 'signal': signal})

    def _handle_signal(self, channel_id: str, signal_json: str):
        channel = self._webrtc_channels.get(channel_id)
        if channel is None:
            rospy.logwarn('No active WebRTC channel for %s', channel_id)
            return
        try:
            signal = json.loads(signal_json)
        except json.JSONDecodeError:
            rospy.logwarn('Invalid signal_json for %s', channel_id)
            return
        sig_type = signal.get('type', '')
        if sig_type == 'answer':
            self._run_coro(channel.set_answer(signal_json))
        elif sig_type == 'candidate':
            self._run_coro(channel.add_ice_candidate(signal_json))
        else:
            rospy.logwarn('Unknown signal type: %s', sig_type)

    def _start_rtsp(self, channel_id: str):
        if channel_id not in self._sources:
            rospy.logwarn('Unknown channel_id: %s', channel_id)
            return
        src = self._sources[channel_id]
        url = self._rtsp_server.start(
            channel_id=channel_id,
            width=src.get('width', 640),
            height=src.get('height', 480),
            fps=src.get('fps', 30),
        )
        self._publish_uplink('media_rtsp_url', {'channel_id': channel_id, 'url': url})
        self._subscribe_topic(channel_id, src)
        self._update_state()

    def _stop_channel(self, channel_id: str):
        if channel_id in self._webrtc_channels:
            self._run_coro(self._webrtc_channels[channel_id].stop())
            del self._webrtc_channels[channel_id]
        self._rtsp_server.stop(channel_id)
        self._stop_record(channel_id)
        if channel_id in self._image_subs:
            self._image_subs[channel_id].unregister()
            del self._image_subs[channel_id]
        self._update_state()
        rospy.loginfo('Channel stopped: %s', channel_id)

    def _stop_all_channels(self):
        for cid in list(set(list(self._webrtc_channels.keys()) + list(self._rtsp_server._channels.keys()))):
            self._stop_channel(cid)
        self._rtsp_server.stop_all()

    def _subscribe_topic(self, channel_id: str, src: dict):
        if channel_id in self._image_subs:
            return
        topic = src.get('ros_topic', '')
        msg_type_str = src.get('msg_type', 'sensor_msgs/Image')
        if not topic:
            return
        if msg_type_str == 'sensor_msgs/Image':
            sub = rospy.Subscriber(topic, Image, lambda msg, cid=channel_id: self._cb_image(cid, msg), queue_size=1)
        elif msg_type_str == 'sensor_msgs/CompressedImage':
            sub = rospy.Subscriber(topic, CompressedImage, lambda msg, cid=channel_id: self._cb_compressed_image(cid, msg), queue_size=1)
        else:
            rospy.logwarn('Unsupported msg_type %s for %s, skipping', msg_type_str, channel_id)
            return
        self._image_subs[channel_id] = sub
        rospy.loginfo('Subscribed to %s for channel %s', topic, channel_id)

    def _cb_image(self, channel_id: str, msg):
        try:
            import cv2
            import numpy as np
            enc = getattr(msg, 'encoding', 'bgr8')
            if enc in ('16UC1', '16SC1'):
                raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                norm = cv2.normalize(raw, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                frame = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
            elif enc in ('mono8', '8UC1'):
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
                frame = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
            elif enc == 'rgb8':
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                frame = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
            elif enc == 'bgr8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif enc == 'rgba8':
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
                frame = cv2.cvtColor(raw, cv2.COLOR_RGBA2BGR)
            elif enc == 'bgra8':
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
                frame = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
            else:
                rospy.logwarn_throttle(5.0, 'Unsupported image encoding for %s: %s', channel_id, enc)
                return
        except Exception:
            return
        self._dispatch_frame(channel_id, frame)

    def _cb_compressed_image(self, channel_id: str, msg):
        try:
            import cv2
            import numpy as np
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if frame is None:
                return
        except Exception:
            return
        self._dispatch_frame(channel_id, frame)

    def _dispatch_frame(self, channel_id: str, frame):
        if channel_id in self._webrtc_channels:
            self._webrtc_channels[channel_id].push_frame(frame)
        self._rtsp_server.push_frame(channel_id, frame)
        if channel_id in self._recorders:
            try:
                self._recorders[channel_id].write(frame)
            except Exception:
                pass

    def _start_record(self, channel_id: str):
        if channel_id in self._recorders:
            return
        record_cfg = self._cfg.get('local_record', {})
        storage_path = record_cfg.get('storage_path', '/tmp/uran_media_record')
        os.makedirs(storage_path, exist_ok=True)
        src = self._sources.get(channel_id, {})
        try:
            import cv2
            filename = os.path.join(storage_path, '%s_%d.avi' % (channel_id, int(time.time())))
            writer = cv2.VideoWriter(
                filename,
                cv2.VideoWriter_fourcc(*'XVID'),
                src.get('fps', 30),
                (src.get('width', 640), src.get('height', 480)),
            )
            self._recorders[channel_id] = writer
            rospy.loginfo('Recording started: %s', filename)
        except Exception as exc:
            rospy.logerr('Failed to start recording for %s: %s', channel_id, exc)

    def _stop_record(self, channel_id: str):
        writer = self._recorders.pop(channel_id, None)
        if writer is None:
            return
        try:
            writer.release()
            self._publish_uplink('media_upload', {'channel_id': channel_id, 'status': 'ready'})
        except Exception as exc:
            rospy.logerr('Error stopping recorder for %s: %s', channel_id, exc)

    def _update_state(self):
        active_webrtc = len(self._webrtc_channels)
        active_rtsp = len(self._rtsp_server._channels)
        total = active_webrtc + active_rtsp
        protocol = 'webrtc' if active_webrtc > 0 else ('rtsp' if active_rtsp > 0 else 'none')
        self._write_state('media_active_protocol', protocol)
        self._write_state('media_channel_count', total)

    def _publish_uplink(self, data_type: str, payload: dict, urgent: bool = False):
        msg = UplinkPayload()
        msg.source_pkg = 'uran_media'
        msg.data_type = data_type
        msg.preferred_protocol = ''
        msg.payload_json = json.dumps(payload)
        msg.urgent = urgent
        msg.timestamp_ns = int(time.time() * 1_000_000_000)
        self._uplink_pub.publish(msg)

    def _write_state(self, field_name: str, value, persistent: bool = False, urgent: bool = False):
        msg = StateField()
        msg.field_name = field_name
        msg.value_json = json.dumps(value)
        msg.persistent = persistent
        msg.urgent = urgent
        msg.source_pkg = 'uran_media'
        msg.timestamp_ns = int(time.time() * 1_000_000_000)
        self._state_pub.publish(msg)

    def shutdown(self):
        self._stop_all_channels()
        self._loop.call_soon_threadsafe(self._loop.stop)


def main():
    rospy.init_node('uran_media_node')
    node = UranMediaNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()
