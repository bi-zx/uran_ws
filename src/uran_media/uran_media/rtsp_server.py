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
    def __init__(self, port: int, url: str):
        self.port = port
        self.url = url
        self.appsrc = None
        self.pts = 0
        self.fps = 30


class RTSPServer:
    """多通道 RTSP Server，每个 channel_id 独立端口。"""

    def __init__(self):
        self._channels: Dict[str, _ChannelState] = {}
        self._servers: Dict[str, object] = {}  # channel_id -> GstRtspServer
        self._glib_loops: Dict[str, object] = {}
        self._available = _GST_AVAILABLE

        if not self._available:
            logger.warning('GStreamer not available; RTSPServer will operate in stub mode')

    def start(self, channel_id: str, width: int = 640, height: int = 480,
              fps: int = 30, port: int = 8554) -> str:
        """启动 RTSP Server，返回 rtsp URL。"""
        url = f'rtsp://localhost:{port}/{channel_id}'

        if not self._available:
            logger.info(f'[RTSP stub] Would start {url}')
            self._channels[channel_id] = _ChannelState(port, url)
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

            mounts = server.get_mount_points()
            mounts.add_factory(f'/{channel_id}', factory)
            server.attach(None)

            state = _ChannelState(port, url)
            self._channels[channel_id] = state
            self._servers[channel_id] = server

            # GLib 主循环（独立线程）
            loop = GLib.MainLoop()
            self._glib_loops[channel_id] = loop
            t = threading.Thread(target=loop.run, daemon=True)
            t.start()

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
        if channel_id in self._glib_loops:
            try:
                self._glib_loops[channel_id].quit()
            except Exception:
                pass
            del self._glib_loops[channel_id]

        self._channels.pop(channel_id, None)
        self._servers.pop(channel_id, None)
        logger.info(f'[RTSP] Stopped channel: {channel_id}')

    def stop_all(self):
        for cid in list(self._channels.keys()):
            self.stop(cid)
