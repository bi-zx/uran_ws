from typing import Callable, Dict

from .base import ProtocolAdapter


class NullProtocolAdapter(ProtocolAdapter):
    protocol_id = 'null'

    def __init__(
        self,
        protocol_cfg: Dict,
        runtime_cfg: Dict,
        downlink_cb: Callable[[dict], None] = None,
    ):
        super().__init__(protocol_cfg, runtime_cfg, downlink_cb)
        self._connected = False

    def connect(self) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def publish(self, payload: Dict, qos: int = 1) -> bool:
        return self._connected

    def health_report(self) -> dict:
        return {
            'available': self._connected,
            'latency_ms': -1,
            'last_check_ts': 0,
            'registered': self._connected,
        }

    def register_device(self, timeout_s: float = 10.0) -> str:
        return 'registered' if self._connected else 'disconnected'

    def is_connected(self) -> bool:
        return self._connected

    def is_registered(self) -> bool:
        return self._connected

    def set_downlink_callback(self, callback: Callable[[dict], None]) -> None:
        self._downlink_cb = callback
