"""rtsp_server.py — RTSP Server 封装

使用 GStreamer appsrc pipeline 推送帧到 RTSP Server。
若 GStreamer 不可用，降级为 stub 模式（仅记录日志）。
"""

import logging
import subprocess
import threading
import time
from typing import Dict, Optional

logger = logging.getLogger(__name__)

try:
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstRtspServer', '1.0')
    from gi.repository import Gst, GstRtspServer, GLib
    Gst.init(None)
    _GST_AVAILABLE = True
except Exception:
    _GST_AVAILABLE = False


class _ChannelState:
    def __init__(self, port: int, url: str, fps: int):
        self.port = port
        self.url = url
        self.appsrc = None
        self.pts = 0
        self.fps = fps


class RTSPServer:
    """多通道 RTSP Server，共享一个 GLib 主循环。"""

    def __init__(self):
        self._channels: Dict[str, _ChannelState] = {}
        self._servers: Dict[str, object] = {}
        self._available = _GST_AVAILABLE
        self._loop = None

        if not self._available:
            logger.warning('GStreamer not available; RTSPServer will operate in stub mode')
            return

        # 启动共享 GLib 主循环（必须在任何 server.attach() 之前运行）
        self._loop = GLib.MainLoop()
        t = threading.Thread(target=self._loop.run, daemon=True)
        t.start()
        time.sleep(0.1)  # 确保 loop 已进入 run 状态

    def start(self, channel_id: str, width: int = 640, height: int = 480,
              fps: int = 30, port: int = 8554) -> str:
        """启动 RTSP Server，返回 rtsp URL。"""
        url = f'rtsp://localhost:{port}/{channel_id}'

        if not self._available:
            logger.info(f'[RTSP stub] Would start {url}')
            self._channels[channel_id] = _ChannelState(port, url, fps)
            return url

        if channel_id in self._channels:
            return self._channels[channel_id].url

        try:
            server = GstRtspServer.RTSPServer()
            server.set_service(str(port))

            factory = GstRtspServer.RTSPMediaFactory()
            pipeline_str = (
                f'( appsrc name=src_{channel_id} is-live=true block=true format=time '
                f'caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! '
                f'videoconvert ! video/x-raw,format=I420 ! '
                f'x264enc tune=zerolatency speed-preset=ultrafast ! rtph264pay name=pay0 pt=96 )'
            )
            factory.set_launch(pipeline_str)
            factory.set_shared(True)

            state = _ChannelState(port, url, fps)
            self._channels[channel_id] = state

            def on_media_configure(factory, media, cid=channel_id):
                pipeline = media.get_element()
                appsrc = pipeline.get_by_name(f'src_{cid}')
                if appsrc:
                    self._channels[cid].appsrc = appsrc
                    logger.info(f'[RTSP] appsrc ready for channel: {cid}')

            factory.connect('media-configure', on_media_configure)

            mounts = server.get_mount_points()
            mounts.add_factory(f'/{channel_id}', factory)
            server.attach(None)  # attach 到默认 GLib context（loop 已在运行）

            self._servers[channel_id] = server
            logger.info(f'[RTSP] Started: {url}')
        except Exception as e:
            logger.error(f'[RTSP] Failed to start {channel_id}: {e}')

        return url

    def push_frame(self, channel_id: str, numpy_frame):
        """推送 numpy BGR 帧到对应通道的 appsrc。"""
        if not self._available:
            return

        state = self._channels.get(channel_id)
        if state is None:
            return

        try:
            data = numpy_frame.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            buf.pts = state.pts
            buf.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, state.fps)
            state.pts += buf.duration

            if state.appsrc is not None:
                state.appsrc.emit('push-buffer', buf)
        except Exception as e:
            logger.debug(f'[RTSP] push_frame error ({channel_id}): {e}')

    def stop(self, channel_id: str):
        """停止对应通道。"""
        self._channels.pop(channel_id, None)
        self._servers.pop(channel_id, None)
        logger.info(f'[RTSP] Stopped channel: {channel_id}')

    def stop_all(self):
        for cid in list(self._channels.keys()):
            self.stop(cid)
        if self._loop is not None:
            self._loop.quit()
