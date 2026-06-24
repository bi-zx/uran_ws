import json
import logging
import threading
import time
from typing import Callable, Dict, Optional

from .base import ProtocolAdapter

logger = logging.getLogger(__name__)

try:
    import paho.mqtt.client as mqtt

    _PAHO_OK = True
except ImportError:
    _PAHO_OK = False
    logger.warning('paho-mqtt not installed; MQTT adapter disabled')


class MqttProtocolAdapter(ProtocolAdapter):
    protocol_id = 'mqtt'

    def __init__(
        self,
        protocol_cfg: dict,
        runtime_cfg: dict,
        downlink_cb: Optional[Callable[[dict], None]] = None,
    ):
        super().__init__(protocol_cfg, runtime_cfg, downlink_cb)
        self._client: Optional['mqtt.Client'] = None
        self._lock = threading.Lock()
        self._connected = False
        self._registered = False
        self._reg_event = threading.Event()
        self._reg_result = 'timeout'
        self._latency_ms = -1
        self._last_check_ts = 0

        device_id = runtime_cfg.get('device_id', 'device_001')
        tenant_id = runtime_cfg.get('tenant_id', 'default')
        registration_cfg = runtime_cfg.get('registration', {})
        prefix = protocol_cfg.get('topic_prefix', '/oivs/{tenant_id}/{device_id}')
        prefix = prefix.replace('{tenant_id}', tenant_id).replace('{device_id}', device_id)
        prefix = prefix.replace('{tenant}', tenant_id)

        self._device_id = device_id
        self._template_id = runtime_cfg.get('template_id', 'template_001')
        auth_cfg = runtime_cfg.get('auth', {})
        self._username = auth_cfg.get('username', '') or self._device_id
        self._token = auth_cfg.get('token', '')
        self._broker_host = protocol_cfg.get('broker_host', 'localhost')
        self._broker_port = int(protocol_cfg.get('broker_port', 1883))
        self._keepalive = int(protocol_cfg.get('keepalive', 60))
        self._uplink_topic = f'{prefix}/up'
        self._downlink_topic = f'{prefix}/down'
        self._require_register_response = bool(registration_cfg.get('require_response', False))
        self._legacy_online_on_timeout = bool(
            registration_cfg.get('legacy_online_on_timeout', not self._require_register_response)
        )

    def set_downlink_callback(self, callback: Callable[[dict], None]) -> None:
        self._downlink_cb = callback

    def connect(self) -> bool:
        if not _PAHO_OK:
            return False
        self._client = mqtt.Client(client_id=self._device_id)
        if self._token:
            self._client.username_pw_set(self._username, self._token)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        try:
            self._client.connect(self._broker_host, self._broker_port, self._keepalive)
            self._client.loop_start()
            for _ in range(50):
                if self.is_connected():
                    return True
                time.sleep(0.1)
        except Exception as exc:
            logger.error('MQTT connect error: %s', exc)
        return False

    def disconnect(self) -> None:
        if self._client is not None:
            self._client.loop_stop()
            self._client.disconnect()

    def register_device(self, timeout_s: float = 10.0) -> str:
        payload = {
            'msg_type': 'register',
            'msg_version': '1.0',
            'device_id': self._device_id,
            'template_id': self._template_id,
            'token': self._token,
            'timestamp_ms': int(time.time() * 1000),
        }
        with self._lock:
            self._registered = False
        self._reg_event.clear()
        self._reg_result = 'timeout'
        self.publish(payload)
        self._reg_event.wait(timeout=timeout_s)
        if self._reg_result == 'timeout' and self.is_connected() and self._legacy_online_on_timeout:
            with self._lock:
                self._registered = True
            self._reg_result = 'legacy_connected'
        return self._reg_result

    def publish(self, payload: Dict, qos: int = 1) -> bool:
        with self._lock:
            client = self._client
            connected = self._connected
        if not connected or client is None:
            return False
        try:
            result = client.publish(self._uplink_topic, json.dumps(payload), qos=qos)
            return result.rc == 0
        except Exception as exc:
            logger.error('MQTT publish error: %s', exc)
            return False

    def health_report(self) -> dict:
        return {
            'available': self.is_connected(),
            'latency_ms': self._latency_ms,
            'last_check_ts': self._last_check_ts,
            'registered': self.is_registered(),
        }

    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    def is_registered(self) -> bool:
        with self._lock:
            return self._registered

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            with self._lock:
                self._connected = True
                self._registered = False
            self._last_check_ts = int(time.time())
            client.subscribe(self._downlink_topic, qos=1)
            logger.info('MQTT connected and subscribed to %s', self._downlink_topic)
            return
        logger.error('MQTT connect refused rc=%s', rc)

    def _on_disconnect(self, client, userdata, rc):
        with self._lock:
            self._connected = False
            self._registered = False
        self._last_check_ts = int(time.time())
        logger.warning('MQTT disconnected rc=%s', rc)

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
        except Exception as exc:
            logger.error('MQTT message decode error: %s', exc)
            return
        msg_type = payload.get('msg_type', '')
        if self._is_register_response(payload, msg_type):
            self._reg_result = payload.get('result', 'rejected')
            with self._lock:
                self._registered = self._reg_result in ('registered', 'auto_registered')
            self._reg_event.set()
            return
        if self._downlink_cb is not None:
            try:
                self._downlink_cb(payload)
            except Exception as exc:
                logger.error('MQTT downlink callback error: %s', exc)

    def _is_register_response(self, payload: Dict, msg_type: str) -> bool:
        if msg_type == 'register_response':
            return True
        if msg_type:
            return False
        result = payload.get('result')
        if result not in ('registered', 'auto_registered', 'rejected'):
            return False
        return payload.get('device_id') == self._device_id or payload.get('template_id') == self._template_id
