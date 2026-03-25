"""webrtc_channel.py — 单路 WebRTC PeerConnection 封装（aiortc）。"""

import asyncio
import json
import queue
from typing import Callable, Optional

try:
    import av
    from aiortc import (
        RTCConfiguration,
        RTCIceCandidate,
        RTCIceServer,
        RTCPeerConnection,
        RTCSessionDescription,
    )
    from aiortc.contrib.media import MediaStreamTrack
    _AIORTC_AVAILABLE = True
except ImportError:
    _AIORTC_AVAILABLE = False


class _RosVideoTrack:
    if _AIORTC_AVAILABLE:
        class _Track(MediaStreamTrack):
            kind = 'video'

            def __init__(self, frame_queue: queue.Queue):
                super().__init__()
                self._queue = frame_queue
                self._pts = 0

            async def recv(self):
                deadline = asyncio.get_event_loop().time() + 1.0
                while True:
                    try:
                        frame_data = self._queue.get_nowait()
                        frame = av.VideoFrame.from_ndarray(frame_data, format='bgr24')
                        frame.pts = self._pts
                        frame.time_base = '1/90000'
                        self._pts += 3000
                        return frame
                    except queue.Empty:
                        if asyncio.get_event_loop().time() > deadline:
                            frame = av.VideoFrame(width=640, height=480, format='bgr24')
                            frame.pts = self._pts
                            frame.time_base = '1/90000'
                            self._pts += 3000
                            return frame
                        await asyncio.sleep(0.01)


class WebRTCChannel:
    def __init__(self, channel_id: str, stun_server: str, on_signal_cb: Callable, turn_servers=None):
        self._channel_id = channel_id
        self._stun_server = stun_server
        self._turn_servers = turn_servers or []
        self._on_signal_cb = on_signal_cb
        self._pc: Optional[object] = None
        self._frame_queue = queue.Queue(maxsize=10)
        self._track = None
        self._available = _AIORTC_AVAILABLE

    async def start(self) -> str:
        if not self._available:
            return json.dumps({'type': 'offer', 'sdp': '', 'stub': True})

        ice_servers = []
        if self._stun_server:
            ice_servers.append(RTCIceServer(urls=[self._stun_server]))
        for server in self._turn_servers:
            ice_servers.append(RTCIceServer(
                urls=server.get('urls') or [server.get('url')],
                username=server.get('username'),
                credential=server.get('credential'),
            ))
        self._pc = RTCPeerConnection(RTCConfiguration(iceServers=ice_servers))
        self._track = _RosVideoTrack._Track(self._frame_queue)
        self._pc.addTrack(self._track)

        @self._pc.on('icecandidate')
        def on_ice(candidate):
            if candidate:
                self._on_signal_cb(self._channel_id, {
                    'type': 'candidate',
                    'candidate': candidate.to_sdp(),
                    'sdpMid': candidate.sdpMid,
                    'sdpMLineIndex': candidate.sdpMLineIndex,
                })

        offer = await self._pc.createOffer()
        await self._pc.setLocalDescription(offer)
        return json.dumps({
            'type': self._pc.localDescription.type,
            'sdp': self._pc.localDescription.sdp,
        })

    async def set_answer(self, answer_json: str):
        if not self._available or self._pc is None:
            return
        data = json.loads(answer_json)
        answer = RTCSessionDescription(sdp=data['sdp'], type=data['type'])
        await self._pc.setRemoteDescription(answer)

    async def add_ice_candidate(self, candidate_json: str):
        if not self._available or self._pc is None:
            return
        data = json.loads(candidate_json)
        candidate = RTCIceCandidate(
            sdp=data.get('candidate', ''),
            sdpMid=data.get('sdpMid'),
            sdpMLineIndex=data.get('sdpMLineIndex'),
        )
        await self._pc.addIceCandidate(candidate)

    def push_frame(self, numpy_frame):
        try:
            self._frame_queue.put_nowait(numpy_frame)
        except queue.Full:
            try:
                self._frame_queue.get_nowait()
                self._frame_queue.put_nowait(numpy_frame)
            except queue.Empty:
                pass

    async def stop(self):
        if self._pc is not None:
            await self._pc.close()
            self._pc = None
