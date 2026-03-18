"""rtsp_server.py — RTSP Server 封装（子进程模式）

在独立子进程中运行 GstRtspServer，彻底隔离 GLib 与 ROS 线程冲突。
ROS 主进程通过 stdin pipe 传递帧数据给子进程。
"""

import logging
import os
import struct
import subprocess
import sys
import threading
from typing import Dict

logger = logging.getLogger(__name__)

_GST_AVAILABLE = False
try:
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstRtspServer', '1.0')
    _GST_AVAILABLE = True
except Exception:
    pass

# 子进程入口脚本（内联，写入临时文件运行）
_WORKER_SCRIPT = r'''
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib
import signal, struct, sys, threading
signal.signal(signal.SIGINT, signal.SIG_IGN)
Gst.init(None)

channel_id = sys.argv[1]
width  = int(sys.argv[2])
height = int(sys.argv[3])
fps    = int(sys.argv[4])
port   = int(sys.argv[5])
frame_size = width * height * 3

appsrc = None

def on_media_configure(factory, media):
    global appsrc
    el = media.get_element()
    src = el.get_by_name('source')
    if src:
        appsrc = src
        print(f'[RTSP] appsrc acquired: {channel_id}', flush=True)

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
factory.connect('media-configure', on_media_configure)

mounts = server.get_mount_points()
mounts.add_factory(f'/{channel_id}', factory)
server.attach(None)

loop = GLib.MainLoop()
threading.Thread(target=loop.run, daemon=True).start()

print(f'[RTSP] READY rtsp://localhost:{port}/{channel_id}', flush=True)

pts = 0
duration = Gst.util_uint64_scale_int(1, Gst.SECOND, fps)
stdin = sys.stdin.buffer

while True:
    try:
        raw = stdin.read(4)
        if not raw or len(raw) < 4:
            break
        sz = struct.unpack('<I', raw)[0]
        if sz == 0:
            break
        data = b''
        while len(data) < sz:
            chunk = stdin.read(sz - len(data))
            if not chunk:
                break
            data += chunk
        if len(data) != sz:
            break
        if appsrc is not None:
            buf = Gst.Buffer.new_allocate(None, sz, None)
            buf.fill(0, data)
            buf.pts = pts
            buf.duration = duration
            pts += duration
            appsrc.emit('push-buffer', buf)
    except Exception:
        break

loop.quit()
'''


class _ChannelState:
    def __init__(self, port: int, url: str, fps: int, width: int, height: int):
        self.port = port
        self.url = url
        self.fps = fps
        self.width = width
        self.height = height
        self.proc = None
        self.lock = threading.Lock()


class RTSPServer:
    """多通道 RTSP Server（子进程隔离）。"""

    def __init__(self):
        self._channels: Dict[str, _ChannelState] = {}
        self._available = _GST_AVAILABLE
        self._script_path = None
        if not self._available:
            logger.warning('GStreamer not available; RTSPServer in stub mode')

    def _ensure_script(self) -> str:
        if self._script_path and os.path.exists(self._script_path):
            return self._script_path
        path = '/tmp/_uran_rtsp_worker.py'
        with open(path, 'w') as f:
            f.write(_WORKER_SCRIPT)
        self._script_path = path
        return path

    def start(self, channel_id: str, width: int = 640, height: int = 480,
              fps: int = 30, port: int = 8554) -> str:
        url = f'rtsp://localhost:{port}/{channel_id}'

        if not self._available:
            logger.info(f'[RTSP stub] {url}')
            self._channels[channel_id] = _ChannelState(port, url, fps, width, height)
            return url

        if channel_id in self._channels:
            return self._channels[channel_id].url

        state = _ChannelState(port, url, fps, width, height)

        try:
            script = self._ensure_script()
            proc = subprocess.Popen(
                [sys.executable, script,
                 channel_id, str(width), str(height), str(fps), str(port)],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            state.proc = proc
            self._channels[channel_id] = state

            # 等待子进程 READY 信号
            def _read_stdout():
                for line in iter(proc.stdout.readline, b''):
                    text = line.decode('utf-8', errors='replace').strip()
                    if text:
                        logger.info(text)

            threading.Thread(target=_read_stdout, daemon=True).start()

            # 给子进程一点启动时间
            import time
            time.sleep(0.5)

            logger.info(f'[RTSP] Worker started: {url}')
        except Exception as e:
            logger.error(f'[RTSP] Failed to start {channel_id}: {e}')

        return url

    def push_frame(self, channel_id: str, numpy_frame):
        if not self._available:
            return
        state = self._channels.get(channel_id)
        if state is None or state.proc is None:
            return
        if state.proc.poll() is not None:
            return  # 子进程已退出

        try:
            h, w = numpy_frame.shape[:2]
            if w != state.width or h != state.height:
                import cv2
                numpy_frame = cv2.resize(numpy_frame, (state.width, state.height))

            data = numpy_frame.tobytes()
            header = struct.pack('<I', len(data))
            with state.lock:
                state.proc.stdin.write(header)
                state.proc.stdin.write(data)
                state.proc.stdin.flush()
        except (BrokenPipeError, OSError):
            logger.warning(f'[RTSP] Pipe broken for {channel_id}')
        except Exception as e:
            logger.debug(f'[RTSP] push error ({channel_id}): {e}')

    def stop(self, channel_id: str):
        state = self._channels.pop(channel_id, None)
        if state is None:
            return
        if state.proc and state.proc.poll() is None:
            try:
                state.proc.stdin.write(struct.pack('<I', 0))
                state.proc.stdin.flush()
                state.proc.wait(timeout=3.0)
            except Exception:
                state.proc.kill()
        logger.info(f'[RTSP] Stopped: {channel_id}')

    def stop_all(self):
        for cid in list(self._channels.keys()):
            self.stop(cid)
