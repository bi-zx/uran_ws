from abc import ABC, abstractmethod
from typing import Callable, Dict


class ProtocolAdapter(ABC):
    protocol_id = 'unknown'

    def __init__(
        self,
        protocol_cfg: Dict,
        runtime_cfg: Dict,
        downlink_cb: Callable[[dict], None] = None,
    ):
        self._protocol_cfg = protocol_cfg
        self._runtime_cfg = runtime_cfg
        self._downlink_cb = downlink_cb

    @abstractmethod
    def connect(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def disconnect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def publish(self, payload: Dict, qos: int = 1) -> bool:
        raise NotImplementedError

    @abstractmethod
    def health_report(self) -> dict:
        raise NotImplementedError

    @abstractmethod
    def register_device(self, timeout_s: float = 10.0) -> str:
        raise NotImplementedError

    @abstractmethod
    def is_connected(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def is_registered(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def set_downlink_callback(self, callback: Callable[[dict], None]) -> None:
        raise NotImplementedError
