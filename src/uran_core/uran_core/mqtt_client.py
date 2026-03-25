"""T1.3 MQTT 客户端 — 入网、注册、心跳"""
import json
import logging
import threading
import time
from typing import Callable, Optional

logger = logging.getLogger(__name__)

try:
    import paho.mqtt.client as mqtt
    _PAHO_OK = True
except ImportError:
    _PAHO_OK = False
    logger.warning('paho-mqtt not installed; MQTT disabled')


class MqttClient:
    """封装 paho-mqtt，提供连接、注册、发布、订阅能力。"""

    def __init__(self, net_cfg: dict, downlink_cb: Callable[[dict], None]):
        self._net_cfg = net_cfg
        self._downlink_cb = downlink_cb
        self._client: Optional['mqtt.Client'] = None
        self._lock = threading.Lock()
        self._connected = False
        self._registered = False
        self._reg_event = threading.Event()
        self._reg_result = 'timeout'

        device_id = net_cfg.get('device_id', 'device_001')
        tenant_id = net_cfg.get('tenant_id', 'default')
        mqtt_cfg = net_cfg.get('mqtt', {})
        prefix = mqtt_cfg.get('topic_prefix', '/oivs/{tenant_id}/{device_id}')
        prefix = prefix.replace('{tenant_id}', tenant_id).replace('{device_id}', device_id)
        prefix = prefix.replace('{tenant}', tenant_id)  # legacy placeholder

        self._device_id = device_id
        self._template_id = net_cfg.get('template_id', 'template_001')
        auth_cfg = net_cfg.get('auth', {})
        self._mqtt_username = auth_cfg.get('username', '') or self._device_id
        self._token = auth_cfg.get('token', '')
        self._broker_host = mqtt_cfg.get('broker_host', 'localhost')
        self._broker_port = int(mqtt_cfg.get('broker_port', 1883))
        self._keepalive = int(mqtt_cfg.get('keepalive', 60))
        self._uplink_topic = f'{prefix}/up'
        self._downlink_topic = f'{prefix}/down'

        # 通路状态
        self._latency_ms: int = -1
        self._last_check_ts: int = 0

    # ------------------------------------------------------------------ connect
    def connect(self) -> bool:
        if not _PAHO_OK:
            return False
        self._client = mqtt.Client(client_id=self._device_id)
        if self._token:
            self._client.username_pw_set(self._mqtt_username, self._token)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        try:
            self._client.connect(self._broker_host, self._broker_port, self._keepalive)
            self._client.loop_start()
            # 等待连接最多 5s
            for _ in range(50):
                if self._connected:
                    return True
                time.sleep(0.1)
            logger.error('MQTT connect timeout')
            return False
        except Exception as exc:
            logger.error(f'MQTT connect error: {exc}')
            return False

    def disconnect(self):
        if self._client:
            self._client.loop_stop()
            self._client.disconnect()

    # ------------------------------------------------------------------ registration
    def register(self, timeout_s: float = 10.0) -> str:
        """发送注册请求，返回 'registered'/'auto_registered'/'rejected'/'timeout'。"""
        payload = {
            'msg_type': 'register',
            'msg_version': '1.0',
            'device_id': self._device_id,
            'template_id': self._template_id,
            'token': self._token,
            'timestamp_ms': int(time.time() * 1000),
        }
        self._reg_event.clear()
        self._reg_result = 'timeout'
        if not self.publish_raw(payload):
            return 'publish_failed'
        self._reg_event.wait(timeout=timeout_s)
        return self._reg_result

    # ------------------------------------------------------------------ publish
    def publish_uplink(self, payload: dict, qos: int = 1) -> bool:
        return self.publish_raw(payload, qos=qos)

    def publish_raw(self, payload: dict, qos: int = 1) -> bool:
        with self._lock:
            if not self._connected or self._client is None:
                return False
        try:
            data = json.dumps(payload)
            result = self._client.publish(self._uplink_topic, data, qos=qos)
            return result.rc == 0
        except Exception as exc:
            logger.error(f'MQTT publish error: {exc}')
            return False

    # ------------------------------------------------------------------ status
    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    def is_registered(self) -> bool:
        with self._lock:
            return self._registered

    def get_protocol_entry(self) -> dict:
        return {
            'available': self.is_connected(),
            'registered': self.is_registered(),
            'latency_ms': self._latency_ms,
            'last_check_ts': self._last_check_ts,
        }

    # ------------------------------------------------------------------ paho callbacks
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            with self._lock:
                self._connected = True
            self._last_check_ts = int(time.time())
            client.subscribe(self._downlink_topic, qos=1)
            logger.info(f'MQTT connected → subscribed {self._downlink_topic}')
        else:
            logger.error(f'MQTT connect refused rc={rc}')

    def _on_disconnect(self, client, userdata, rc):
        with self._lock:
            self._connected = False
            self._registered = False
        self._last_check_ts = int(time.time())
        logger.warning(f'MQTT disconnected rc={rc}')

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
        except Exception as exc:
            logger.error(f'MQTT message decode error: {exc}')
            return

        msg_type = payload.get('msg_type', '')

        # 处理注册响应
        if msg_type == 'register_response':
            result = payload.get('result', 'rejected')
            with self._lock:
                self._registered = result in ('registered', 'auto_registered', 'accepted', 'ok', 'success')
            self._reg_result = result
            self._reg_event.set()
            return

        # 其余消息交给节点处理
        try:
            self._downlink_cb(payload)
        except Exception as exc:
            logger.error(f'downlink callback error: {exc}')
