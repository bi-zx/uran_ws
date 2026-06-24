from typing import Callable, Dict, Type

from .protocols import MqttProtocolAdapter, NullProtocolAdapter, ProtocolAdapter


class ProtocolFactory:
    def __init__(self):
        self._builders: Dict[str, Type[ProtocolAdapter]] = {}
        self.register_class(MqttProtocolAdapter.protocol_id, MqttProtocolAdapter)
        self.register_class(NullProtocolAdapter.protocol_id, NullProtocolAdapter)

    def register_class(self, protocol_id: str, adapter_cls: Type[ProtocolAdapter]) -> None:
        self._builders[protocol_id] = adapter_cls

    def supported_protocols(self) -> list:
        return sorted(self._builders.keys())

    def build(
        self,
        protocol_id: str,
        protocol_cfg: dict,
        runtime_cfg: dict,
        downlink_cb: Callable[[dict], None],
    ) -> ProtocolAdapter:
        adapter_cls = self._builders.get(protocol_id)
        if adapter_cls is None:
            raise KeyError(f'unsupported protocol adapter: {protocol_id}')
        return adapter_cls(protocol_cfg, runtime_cfg, downlink_cb)
