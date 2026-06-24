import json
from typing import Any, Dict

from .registries import ProtocolRegistry


class UplinkGateway:
    def __init__(self, protocol_registry: ProtocolRegistry, state_store, default_protocol: str = 'mqtt'):
        self._protocol_registry = protocol_registry
        self._state_store = state_store
        self._default_protocol = default_protocol

    def publish_payload(self, payload: Dict[str, Any], preferred_protocol: str = '', qos: int = 1) -> bool:
        protocol = (
            preferred_protocol
            or self._state_store.get('primary_uplink_protocol', self._default_protocol)
            or self._default_protocol
        )
        ok = self._protocol_registry.publish(protocol, payload, qos=qos)
        if ok:
            return True
        for descriptor in self._protocol_registry.active_protocols():
            if descriptor.protocol_id == protocol:
                continue
            if self._protocol_registry.publish(descriptor.protocol_id, payload, qos=qos):
                return True
        return False

    def publish_uplink_message(self, msg) -> bool:
        payload_value = self._json_value_from_str(msg.payload_json)
        payload = {
            'msg_type': 'uplink_data',
            'msg_version': '1.0',
            'device_id': self._state_store.get('device_id', ''),
            'source_pkg': msg.source_pkg,
            'data_type': msg.data_type,
            'timestamp_ns': msg.timestamp_ns,
            'payload': payload_value,
            'payload_json': msg.payload_json,
        }
        if msg.preferred_protocol:
            payload['preferred_protocol'] = msg.preferred_protocol
        return self.publish_payload(payload, preferred_protocol=msg.preferred_protocol)

    def _json_value_from_str(self, raw: str) -> Any:
        if raw in ('', None):
            return {}
        if not isinstance(raw, str):
            return raw
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return raw
