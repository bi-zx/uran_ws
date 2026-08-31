import json
from typing import Any, Dict

from uran_msgs.msg import UnifiedMoveCmd


class UranMoveGateway:
    """Small publisher facade for safety actions handled by uran_move."""

    def __init__(self, node, *, topic_name: str = '/uran/core/downlink/move_cmd'):
        self._node = node
        self._publisher = node.create_publisher(UnifiedMoveCmd, topic_name, 10)
        self._command_seq = 0

    def stop(self, *, reason: str = ''):
        self._publish_action('stop', reason=reason)

    def emergency_stop(self, *, reason: str = ''):
        self._publish_action('emergency_stop', reason=reason)

    def stand(self, *, reason: str = ''):
        self._publish_action('stand', reason=reason)

    def velocity(
        self,
        *,
        linear_x: float,
        angular_z: float,
        source: str = '',
        reason: str = '',
        task_id: str = '',
    ):
        msg = UnifiedMoveCmd()
        msg.msg_version = '1.0'
        msg.timestamp_ns = self._node.get_clock().now().nanoseconds
        msg.controller = 'auto'
        msg.linear_vel_x = float(linear_x)
        msg.linear_vel_y = 0.0
        msg.linear_vel_z = 0.0
        msg.angular_vel_z = float(angular_z)
        payload = {
            'source_pkg': 'uran_autotask',
            'source': str(source or 'velocity'),
            'command_seq': self._next_command_seq(),
        }
        if reason:
            payload['reason'] = str(reason)
        if task_id:
            payload['task_id'] = str(task_id)
        msg.extra_json = json.dumps(payload, ensure_ascii=False)
        self._publisher.publish(msg)

    def _publish_action(self, action: str, *, reason: str = '', extra: Dict[str, Any] = None):
        msg = UnifiedMoveCmd()
        msg.msg_version = '1.0'
        msg.timestamp_ns = self._node.get_clock().now().nanoseconds
        msg.controller = 'auto'
        msg.action = str(action)
        payload = dict(extra or {})
        payload['source_pkg'] = 'uran_autotask'
        payload['command_seq'] = self._next_command_seq()
        if reason:
            payload['reason'] = str(reason)
        msg.extra_json = json.dumps(payload, ensure_ascii=False)
        self._publisher.publish(msg)

    def _next_command_seq(self) -> int:
        self._command_seq += 1
        return self._command_seq
