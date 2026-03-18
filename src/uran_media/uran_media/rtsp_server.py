"""rtsp_server.py — RTSP Server 封装

使用 GStreamer appsrc pipeline 推送帧到 RTSP Server。
通过 media-configure 信号在客户端连接时获取 appsrc 引用。
若 GStreamer 不可用，降级为 stub 模式（仅记录日志）。
"""

import logging
import threading
from typing import Dict

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
    def __init__(self, channel_id: str, port: int, url: str, fps: int):
        self.channel_id = channel_id
        self.port = port
        self.url = url
        self.fps = fps
        self.appsrc = None
        self.pts = 0
        self.frame_duration = (
            Gst.util_uint64_scale_int(1, Gst.SECOND, fps) if _GST_AVAILABLE else 0
        )


class RTSPServer:
    """多通道 RTSP Server，每个 channel_id 独立端口。"""

    def __init__(self):
        self._channels: Dict[str, _ChannelState] = {}
        self._servers: Dict[str, object] = {}
        self._factories: Dict[str, object] = {}
        self._loop: object = None
        self._loop_thread: threading.Thread = None
        self._available = _GST_AVAILABLE

        if not self._available:
            logger.warning('GStreamer not available; RTSPServer will operate in stub mode')

    def _ensure_glib_loop(self):
        """确保 GLib 主循环在运行（所有通道共享一个）。"""
        if self._loop is not None:
            return
        self._loop = GLib.MainLoop()
        self._loop_thread = threading.Thread(target=self._loop.run, daemon=True)
        self._loop_thread.start()

    def start(self, channel_id: str, width: int = 640, height: int = 480,
              fps: int = 30, port: int = 8554) -> str:
        """启动 RTSP Server，返回 rtsp URL。"""
        url = f'rtsp://localhost:{port}/{channel_id}'

        if not self._available:
            logger.info(f'[RTSP stub] Would start {url}')
            self._channels[channel_id] = _ChannelState(channel_id, port, url, fps)
            return url

        if channel_id in self._channels:
            return self._channels[channel_id].url

        try:
            self._ensure_glib_loop()

            server = GstRtspServer.RTSPServer()
            server.set_service(str(port))

            factory = GstRtspServer.RTSPMediaFactory()
            pipeline_str = (
                f'( appsrc name=src is-live=true block=true format=time '
                f'caps=video/x-raw,format=BGR,width={width},height={height},'
                f'framerate={fps}/1 ! '
                f'videoconvert ! video/x-raw,format=I420 ! '
                f'x264enc tune=zerolatency speed-preset=ultrafast ! '
                f'rtph264pay name=pay0 pt=96 )'
            )
            factory.set_launch(pipeline_str)
            factory.set_shared(True)

            state = _ChannelState(channel_id, port, url, fps)
            self._channels[channel_id] = state

            # 客户端连接时通过 media-configure 获取 appsrc 引用
            factory.connect('media-configure',
                            lambda fac, media, cid=channel_id: self._on_media_configure(cid, media))

            mounts = server.get_mount_points()
            mounts.add_factory(f'/{channel_id}', factory)
            server.attach(None)

            self._servers[channel_id] = server
            self._factories[channel_id] = factory

            logger.info(f'[RTSP] Started: {url} (waiting for client connection to acquire appsrc)')
        except Exception as e:
            logger.error(f'[RTSP] Failed to start {channel_id}: {e}')
            self._channels.pop(channel_id, None)

        return url

    def _on_media_configure(self, channel_id: str, media):
        """客户端首次连接时触发，从 pipeline 中获取 appsrc 元素。"""
        state = self._channels.get(channel_id)
        if state is None:
            return

        pipeline = media.get_element()
        appsrc = pipeline.get_by_name('src')
        if appsrc:
            state.appsrc = appsrc
            state.pts = 0
            logger.info(f'[RTSP] appsrc acquired for channel: {channel_id}')
        else:
            logger.error(f'[RTSP] Failed to get appsrc for channel: {channel_id}')

    def push_frame(self, channel_id: str, numpy_frame):
        """推送 numpy BGR 帧到对应通道的 appsrc。"""
        if not self._available:
            return

        state = self._channels.get(channel_id)
        if state is None or state.appsrc is None:
            return

        try:
            data = numpy_frame.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            buf.pts = state.pts
            buf.duration = state.frame_duration
            state.pts += state.frame_duration

            ret = state.appsrc.emit('push-buffer', buf)
            if ret != Gst.FlowReturn.OK:
                logger.debug(f'[RTSP] push-buffer returned {ret} for {channel_id}')
        except Exception as e:
            logger.debug(f'[RTSP] push_frame error ({channel_id}): {e}')

    def stop(self, channel_id: str):
        """停止对应通道。"""
        state = self._channels.pop(channel_id, None)
        if state and state.appsrc:
            try:
                state.appsrc.emit('end-of-stream')
            except Exception:
                pass

        self._servers.pop(channel_id, None)
        self._factories.pop(channel_id, None)
        logger.info(f'[RTSP] Stopped channel: {channel_id}')

    def stop_all(self):
        for cid in list(self._channels.keys()):
            self.stop(cid)
        if self._loop is not None:
            try:
                self._loop.quit()
            except Exception:
                pass
            self._loop = None
