#!/usr/bin/env python3
"""webrtc_signaling_server.py

在机器狗本地运行，同时承担两个角色：
  1. ROS 节点：订阅 img_trans_signal_out，发布 img_trans_signal_in
  2. HTTP 信令服务器：浏览器通过轮询 /api/poll 获取 answer/candidate，
     通过 POST /api/signal 发送 offer/candidate

用法：
  source /opt/ros/humble/setup.bash
  source ~/cyberdog_ws/install/setup.bash
  source ~/uran_ws/install/setup.bash
  python3 webrtc_signaling_server.py [--ns /mi_desktop_48_b0_2d_5f_b6_d0] [--port 8080]

浏览器访问：http://localhost:8080
"""

import argparse
import json
import os
import queue
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── 全局信令队列（ROS → 浏览器） ──────────────────────────────────────────
_to_browser: queue.Queue = queue.Queue()   # image_transmission 发出的信令
_to_ros: queue.Queue = queue.Queue()       # 浏览器发来的信令（offer/candidate）

# ── ROS 节点 ──────────────────────────────────────────────────────────────

class SignalingBridgeNode(Node):
    def __init__(self, ns: str):
        super().__init__('uran_webrtc_signaling')
        ns = ns.rstrip('/')

        sig_in  = f'{ns}/img_trans_signal_in'  if ns else 'img_trans_signal_in'
        sig_out = f'{ns}/img_trans_signal_out' if ns else 'img_trans_signal_out'

        self._pub = self.create_publisher(String, sig_in, 10)
        self.create_subscription(String, sig_out, self._cb_out, 10)

        # 定时器：把浏览器发来的信令转发给 image_transmission
        self.create_timer(0.05, self._timer_forward)

        self.get_logger().info(f'Signaling bridge: {sig_in} <-> {sig_out}')

    def _cb_out(self, msg: String):
        """image_transmission → 浏览器队列"""
        try:
            data = json.loads(msg.data)
            uid = data.get('uid', '')

            if 'answer_sdp' in data:
                sdp = data['answer_sdp']
                _to_browser.put({'type': 'answer', 'sdp': sdp.get('sdp', '')})
            elif 'c_sdp' in data:
                c = data['c_sdp']
                _to_browser.put({
                    'type': 'candidate',
                    'sdpMid': c.get('sdpMid', ''),
                    'sdpMLineIndex': c.get('sdpMLineIndex', 0),
                    'candidate': c.get('candidate', ''),
                })
            elif data.get('is_closed'):
                _to_browser.put({'type': 'closed'})
            elif 'error' in data:
                _to_browser.put({'type': 'error', 'error': data['error']})
        except Exception as e:
            self.get_logger().warning(f'Parse img_trans_signal_out failed: {e}')

    def _timer_forward(self):
        """浏览器队列 → image_transmission"""
        try:
            while True:
                item = _to_ros.get_nowait()
                sig_type = item.get('type', '')
                uid = item.get('uid', 'cyberdog_main')

                if sig_type == 'offer':
                    raw = json.dumps({
                        'uid': uid,
                        'offer_sdp': {'type': 'offer', 'sdp': item.get('sdp', '')},
                        'height': item.get('height', 720),
                        'width': item.get('width', 1280),
                        'alignment': item.get('alignment', 'middle'),
                    })
                elif sig_type == 'candidate':
                    raw = json.dumps({
                        'uid': uid,
                        'c_sdp': {
                            'sdpMid': item.get('sdpMid', ''),
                            'sdpMLineIndex': item.get('sdpMLineIndex', 0),
                            'candidate': item.get('candidate', ''),
                        },
                    })
                elif sig_type == 'stop':
                    raw = json.dumps({'uid': uid, 'is_closed': True})
                else:
                    continue

                msg = String()
                msg.data = raw
                self._pub.publish(msg)
                self.get_logger().info(f'→ img_trans_signal_in: type={sig_type}')
        except queue.Empty:
            pass


# ── HTTP 服务器 ────────────────────────────────────────────────────────────

HTML = r"""<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<title>CyberDog2 WebRTC Viewer</title>
<style>
  body{font-family:monospace;background:#1a1a1a;color:#eee;padding:20px;margin:0}
  h2{color:#4af;margin:0 0 12px}
  video{width:100%;max-width:960px;background:#000;border:1px solid #444;display:block}
  button{padding:8px 18px;margin:4px;background:#2a6;color:#fff;border:none;
         cursor:pointer;font-size:14px;border-radius:4px}
  button:disabled{background:#555;cursor:default}
  button.stop{background:#a33}
  #log{height:180px;overflow-y:auto;background:#111;border:1px solid #333;
       padding:8px;font-size:12px;color:#aaa;margin-top:8px}
  #status{margin-left:12px;font-size:14px}
</style>
</head>
<body>
<h2>CyberDog2 WebRTC Viewer</h2>
<video id="video" autoplay playsinline muted></video>
<div style="margin:10px 0">
  <button id="btnStart" onclick="start()">连接</button>
  <button id="btnStop" class="stop" onclick="stop()" disabled>断开</button>
  <span id="status" style="color:#fa0">未连接</span>
</div>
<div id="log"></div>

<script>
const UID = 'cyberdog_main';
let pc = null, polling = false, pollTimer = null;

function log(msg, color) {
  const d = document.getElementById('log');
  const t = new Date().toTimeString().slice(0,8);
  d.innerHTML += `<span style="color:${color||'#aaa'}">[${t}] ${msg}</span>\n`;
  d.scrollTop = d.scrollHeight;
}
function setStatus(s, c) {
  const el = document.getElementById('status');
  el.textContent = s; el.style.color = c || '#fa0';
}

async function postSignal(data) {
  data.uid = UID;
  await fetch('/api/signal', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(data)
  });
}

async function poll() {
  if (!polling) return;
  try {
    const r = await fetch('/api/poll');
    const msgs = await r.json();
    for (const m of msgs) {
      if (m.type === 'answer') {
        log('收到 Answer，设置远端描述...', '#4af');
        await pc.setRemoteDescription(new RTCSessionDescription(m));
        log('Answer 已设置', '#0f0');
      } else if (m.type === 'candidate' && m.candidate) {
        await pc.addIceCandidate(new RTCIceCandidate(m));
        log('添加 Candidate: ' + m.candidate.slice(0,50) + '...', '#888');
      } else if (m.type === 'closed') {
        log('image_transmission 关闭了连接', '#f44');
        stop();
        return;
      } else if (m.type === 'error') {
        log('错误: ' + JSON.stringify(m.error), '#f44');
      }
    }
  } catch(e) { log('轮询错误: ' + e, '#f44'); }
  if (polling) pollTimer = setTimeout(poll, 200);
}

async function start() {
  document.getElementById('btnStart').disabled = true;
  setStatus('创建连接...', '#fa0');

  pc = new RTCPeerConnection({
    iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
  });

  pc.ontrack = (e) => {
    log('收到视频轨道！', '#0f0');
    setStatus('视频流接收中 ▶', '#0f0');
    document.getElementById('video').srcObject = e.streams[0];
  };

  pc.onicecandidate = (e) => {
    if (!e.candidate) return;
    const c = e.candidate;
    postSignal({
      type: 'candidate',
      sdpMid: c.sdpMid,
      sdpMLineIndex: c.sdpMLineIndex,
      candidate: c.candidate
    });
    log('发送本地 Candidate', '#888');
  };

  pc.oniceconnectionstatechange = () => {
    const s = pc.iceConnectionState;
    log('ICE 状态: ' + s, '#4af');
    if (s === 'connected' || s === 'completed') setStatus('已连接 ✓', '#0f0');
    else if (['failed','disconnected','closed'].includes(s)) {
      setStatus('连接断开: ' + s, '#f44');
    }
  };

  pc.addTransceiver('video', { direction: 'recvonly' });

  const offer = await pc.createOffer();
  await pc.setLocalDescription(offer);

  // 启动轮询
  polling = true;
  poll();

  // 发送 offer
  await postSignal({ type: 'offer', sdp: offer.sdp });
  log('Offer 已发送，等待 Answer...', '#4af');
  setStatus('等待 Answer...', '#fa0');
  document.getElementById('btnStop').disabled = false;
}

function stop() {
  polling = false;
  if (pollTimer) clearTimeout(pollTimer);
  if (pc) { pc.close(); pc = null; }
  postSignal({ type: 'stop' }).catch(()=>{});
  document.getElementById('video').srcObject = null;
  document.getElementById('btnStart').disabled = false;
  document.getElementById('btnStop').disabled = true;
  setStatus('已断开', '#888');
  log('连接已关闭', '#888');
}
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    # 每个请求最多等待 2s 的新消息（长轮询）
    POLL_TIMEOUT = 2.0

    def log_message(self, fmt, *args):
        pass  # 静默 HTTP 日志

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            self._send(200, 'text/html; charset=utf-8', HTML.encode())
        elif self.path == '/api/poll':
            self._handle_poll()
        else:
            self._send(404, 'text/plain', b'Not Found')

    def do_POST(self):
        if self.path == '/api/signal':
            self._handle_signal()
        else:
            self._send(404, 'text/plain', b'Not Found')

    def do_OPTIONS(self):
        self.send_response(200)
        self._cors()
        self.end_headers()

    def _handle_poll(self):
        """收集队列中所有待发消息，最多等待 POLL_TIMEOUT 秒。"""
        msgs = []
        deadline = time.time() + self.POLL_TIMEOUT
        # 先取已有消息
        while True:
            try:
                msgs.append(_to_browser.get_nowait())
            except queue.Empty:
                break
        # 若无消息则短暂等待一条
        if not msgs:
            try:
                remaining = deadline - time.time()
                if remaining > 0:
                    msgs.append(_to_browser.get(timeout=remaining))
            except queue.Empty:
                pass
        # 再取剩余
        while True:
            try:
                msgs.append(_to_browser.get_nowait())
            except queue.Empty:
                break
        self._send(200, 'application/json', json.dumps(msgs).encode())

    def _handle_signal(self):
        length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(length)
        try:
            data = json.loads(body)
            _to_ros.put(data)
            self._send(200, 'application/json', b'{"ok":true}')
        except Exception as e:
            self._send(400, 'application/json', json.dumps({'error': str(e)}).encode())

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _cors(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')


# ── 主程序 ─────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='CyberDog2 WebRTC Signaling Server')
    parser.add_argument('--ns', default='/mi_desktop_48_b0_2d_5f_b6_d0',
                        help='CyberDog2 ROS namespace (default: /mi_desktop_48_b0_2d_5f_b6_d0)')
    parser.add_argument('--port', type=int, default=8080,
                        help='HTTP server port (default: 8080)')
    args = parser.parse_args()

    # 启动 HTTP 服务器（独立线程）
    server = HTTPServer(('0.0.0.0', args.port), Handler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    print(f'[HTTP] Signaling server started: http://localhost:{args.port}')
    print(f'[ROS]  Namespace: {args.ns}')
    print(f'       Press Ctrl+C to stop\n')

    # 启动 ROS 节点（主线程）
    rclpy.init()
    node = SignalingBridgeNode(args.ns)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        server.shutdown()


if __name__ == '__main__':
    main()
