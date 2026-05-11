import json
from typing import Any, Dict

from uran_msgs.msg import UnifiedMoveCmd


class UranMoveGateway:
    """Small publisher facade for safety actions handled by uran_move."""

    def __init__(self, node, *, topic_name: str = '/uran/core/downlink/move_cmd'):
        self._node = node
        self._publisher = node.create_publisher(UnifiedMoveCmd, topic_name, 10)

    def stop(self, *, reason: str = ''):
        self._publish_action('stop', reason=reason)

    def emergency_stop(self, *, reason: str = ''):
        self._publish_action('emergency_stop', reason=reason)

    def stand(self, *, reason: str = ''):
        self._publish_action('stand', reason=reason)

    def _publish_action(self, action: str, *, reason: str = '', extra: Dict[str, Any] = None):
        msg = UnifiedMoveCmd()
        msg.msg_version = '1.0'
        msg.timestamp_ns = self._node.get_clock().now().nanoseconds
        msg.controller = 'auto'
        msg.action = str(action)
        payload = dict(extra or {})
        payload['source_pkg'] = 'uran_autotask'
        if reason:
            payload['reason'] = str(reason)
        msg.extra_json = json.dumps(payload, ensure_ascii=False)
        self._publisher.publish(msg)
