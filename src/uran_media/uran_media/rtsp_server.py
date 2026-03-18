"""rtsp_server.py — RTSP Server 封装

使用 GStreamer appsrc + GstRtspServer 推送帧。
每个 server 实例使用独立 GLib MainContext 避免与 ROS 冲突。
若 GStreamer 不可用，降级为 stub 模式。
"""

import logging
import threading
import time
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
    def __init__(self, port: int, url: str, fps: int):
        self.port = port
        self.url = url
        self.fps = fps
        self.appsrc = None
        self.pts = 0
        self.latest_frame = None  # 缓冲最新帧
        self.lock = threading.Lock()


class RTSPServer:
    """多通道 RTSP Server。"""

    def __init__(self):
        self._channels: Dict[str, _ChannelState] = {}
        self._servers: Dict[str, object] = {}
        self._loops: Dict[str, object] = {}
        self._contexts: Dict[str, object] = {}
        self._available = _GST_AVAILABLE

        if not self._available:
            logger.warning('GStreamer not available; RTSPServer in stub mode')

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

        state = _ChannelState(port, url, fps)
        self._channels[channel_id] = state

        try:
            # 独立 GLib context，避免与 ROS / 其他 GLib 用户冲突
            context = GLib.MainContext.new()
            self._contexts[channel_id] = context

            server = GstRtspServer.RTSPServer()
            server.set_service(str(port))

            factory = GstRtspServer.RTSPMediaFactory()
            pipeline_str = (
                f'( appsrc name=source is-live=true do-timestamp=true format=time '
                f'caps=video/x-raw,format=BGR,width={width},height={height},'
                f'framerate={fps}/1 ! '
                f'videoconvert ! video/x-raw,format=I420 ! '
                f'x264enc tune=zerolatency speed-preset=ultrafast '
                f'key-int-max={fps} ! rtph264pay name=pay0 pt=96 )'
            )
            factory.set_launch(pipeline_str)
            factory.set_shared(True)

            def on_media_configure(factory_obj, media, cid=channel_id):
                element = media.get_element()
                appsrc = element.get_by_name('source')
                if appsrc:
                    with self._channels[cid].lock:
                        self._channels[cid].appsrc = appsrc
                    logger.info(f'[RTSP] appsrc acquired: {cid}')
                    # 立即推送缓冲帧，防止 pipeline preroll 超时
                    cached = self._channels[cid].latest_frame
                    if cached is not None:
                        self._do_push(cid, cached)

            factory.connect('media-configure', on_media_configure)

            mounts = server.get_mount_points()
            mounts.add_factory(f'/{channel_id}', factory)
            server.attach(context)

            self._servers[channel_id] = server

            loop = GLib.MainLoop.new(context, False)
            self._loops[channel_id] = loop
            t = threading.Thread(target=self._run_loop, args=(loop, context),
                                 daemon=True)
            t.start()

            logger.info(f'[RTSP] Started: {url}')
        except Exception as e:
            logger.error(f'[RTSP] Failed to start {channel_id}: {e}')

        return url

    @staticmethod
    def _run_loop(loop, context):
        """在独立线程中运行 GLib MainLoop，手动 acquire context。"""
        context.acquire()
        try:
            loop.run()
        finally:
            context.release()

    def push_frame(self, channel_id: str, numpy_frame):
        """推送 numpy BGR 帧到对应通道。"""
        if not self._available:
            return
        state = self._channels.get(channel_id)
        if state is None:
            return
        # 始终缓存最新帧（供 media-configure 首帧使用）
        state.latest_frame = numpy_frame
        self._do_push(channel_id, numpy_frame)

    def _do_push(self, channel_id: str, numpy_frame):
        state = self._channels.get(channel_id)
        if state is None or state.appsrc is None:
            return
        try:
            data = numpy_frame.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            buf.pts = state.pts
            buf.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, state.fps)
            state.pts += buf.duration
            state.appsrc.emit('push-buffer', buf)
        except Exception as e:
            logger.debug(f'[RTSP] push error ({channel_id}): {e}')

    def stop(self, channel_id: str):
        """停止对应通道。"""
        if channel_id in self._loops:
            try:
                self._loops[channel_id].quit()
            except Exception:
                pass
            del self._loops[channel_id]
        self._contexts.pop(channel_id, None)
        self._channels.pop(channel_id, None)
        self._servers.pop(channel_id, None)
        logger.info(f'[RTSP] Stopped: {channel_id}')

    def stop_all(self):
        for cid in list(self._channels.keys()):
            self.stop(cid)
