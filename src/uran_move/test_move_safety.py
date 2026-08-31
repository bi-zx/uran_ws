from types import SimpleNamespace

import pytest

pytest.importorskip('uran_msgs')

from uran_move.plugins.cyberdog2_plugin import CyberDog2Plugin
from uran_move.uran_move_node import UranMoveNode


class _Logger:
    def warning(self, _message):
        pass

    warn = warning

    def error(self, _message):
        pass


class _Plugin:
    def __init__(self):
        self.commands = []

    def execute(self, cmd):
        self.commands.append(cmd)
        return True, ''


class _MoveNode:
    def __init__(self):
        self._plugin = _Plugin()
        self._failsafe_active = True
        self._last_received_command_seq = 0
        self._last_executed_command_seq = 0
        self._last_rejected_command_seq = 0
        self.rejections = []
        self.results = []

    def get_logger(self):
        return _Logger()

    def _command_seq(self, cmd):
        return int(getattr(cmd, 'command_seq', 0))

    def _velocity_payload(self, cmd):
        return {
            'linear_x': float(cmd.linear_vel_x),
            'linear_y': float(cmd.linear_vel_y),
            'linear_z': float(cmd.linear_vel_z),
            'angular_z': float(cmd.angular_vel_z),
        }

    def _publish_rejection(self, cmd, *, reason, extra=None):
        self.rejections.append((cmd, reason, extra))

    def _check_motion_control_lock(self, _cmd):
        raise AssertionError('safety action must bypass the control lock')

    def _check_mode(self, _cmd):
        raise AssertionError('safety action must bypass mode filtering')

    def _precheck_cmd(self, _cmd):
        raise AssertionError('safety action must bypass velocity precheck')

    def _report_result(self, cmd, success, result_json, **kwargs):
        self.results.append((cmd, success, result_json, kwargs))


def _command(*, action='', command_seq=1, vx=0.0):
    return SimpleNamespace(
        action=action,
        command_seq=command_seq,
        controller='auto',
        linear_vel_x=vx,
        linear_vel_y=0.0,
        linear_vel_z=0.0,
        angular_vel_z=0.0,
    )


def test_regular_velocity_is_rejected_while_failsafe_is_active():
    node = _MoveNode()
    cmd = _command(command_seq=7, vx=0.4)

    UranMoveNode._cb_move_cmd(node, cmd)

    assert node._plugin.commands == []
    assert node.rejections[0][1] == 'failsafe active'


def test_stop_bypasses_failsafe_mode_and_control_lock():
    node = _MoveNode()
    cmd = _command(action='stop', command_seq=8)

    UranMoveNode._cb_move_cmd(node, cmd)

    assert node._plugin.commands == [cmd]
    assert node._last_executed_command_seq == 8
    assert node.rejections == []
    assert node.results[0][1] is True


def test_cyberdog_stop_publishes_zero_velocity_before_returning():
    plugin = CyberDog2Plugin.__new__(CyberDog2Plugin)
    published = []
    plugin._last_execute_ts = 1.0
    plugin._idle_zero_since = None
    plugin._auto_stand_sent = True
    plugin._auto_stand_retry_after = 1.0
    plugin._publish_servo = lambda vx, vy, wz: published.append((vx, vy, wz))

    success, message = plugin.execute(SimpleNamespace(action='stop', extra_json=''))

    assert success is True
    assert message == ''
    assert published == [(0.0, 0.0, 0.0)]
    assert plugin._last_execute_ts == 0.0
