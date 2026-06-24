from typing import Any, Dict

from .models import DownlinkHandler


class DownlinkRouter:
    def __init__(self):
        self._handlers: Dict[str, DownlinkHandler] = {}

    def register_handler(self, msg_type: str, callback, description: str = '') -> None:
        self._handlers[msg_type] = DownlinkHandler(
            msg_type=msg_type,
            callback=callback,
            description=description,
        )

    def route(self, payload: Dict[str, Any]) -> bool:
        if not isinstance(payload, dict):
            return False
        msg_type = str(payload.get('msg_type', '') or '')
        if not msg_type:
            return False
        handler = self._handlers.get(msg_type)
        if handler is None:
            return False
        handler.callback(payload)
        return True

    def describe(self) -> Dict[str, str]:
        return {
            msg_type: handler.description
            for msg_type, handler in sorted(self._handlers.items())
        }
