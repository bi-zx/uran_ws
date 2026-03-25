"""cyberdog2_plugin.py — 小米 CyberDog2 运控适配插件（T2.3）

依赖：
  - protocol.msg.MotionServoCmd
  - protocol.msg.MotionStatus
  - protocol.srv.MotionResultCmd
"""

import json
import time

import rclpy

from uran_move.plugin_base import MovePluginBase

# CyberDog2 motion_id 常量
_MOTION_WALK_ADAPTIVELY = 304
_MOTION_RECOVERYSTAND = 111
_MOTION_GETDOWN = 101
_MOTION_ESTOP = 0

# MotionServoCmd.cmd_type
_SERVO_START = 0

# cmd_source（-1 = 最高调试优先级，绕过 motion_manager 优先级队列）
_SOURCE_APP = -1

# switch_status 含义（来自 MotionStatus.msg）
_SWITCH_STATUS_NORMAL = 0
_SWITCH_STATUS_ESTOP = 2
_SWITCH_STATUS_CHARGING = 14
_SWITCH_STATUS_NAMES = {
    0: 'NORMAL', 1: 'TRANSITIONING', 2: 'ESTOP', 3: 'EDAMP',
    4: 'LIFTED', 5: 'BAN_TRANS', 6: 'OVER_HEAT', 7: 'LOW_BAT',
    8: 'ORI_ERR', 9: 'FOOTPOS_ERR', 10: 'STAND_STUCK',
    11: 'MOTOR_OVER_HEAT', 12: 'MOTOR_OVER_CURR', 13: 'MOTOR_ERR',
    14: 'CHARGING',
}


class CyberDog2Plugin(MovePluginBase):

    def init(self, node, params: dict) -> bool:
        self._node = node
        self._result_timeout = 3.0
        self._recovery_wait_timeout = float(params.get('recovery_wait_timeout_s', 5.0))
        self._auto_recover_from_estop = bool(params.get('auto_recover_from_estop', True))
        self._cmd_timeout_s = 0.5
        self._charging_recover_timeout_s = float(params.get('charging_recover_timeout_s', 1.0))
        self._last_execute_ts = 0.0
        self._switch_status = _SWITCH_STATUS_NORMAL
        self._charger_connected = None
        self._charger_disconnected_since = None

        # cyberdog_ws 节点的 ROS namespace（如 /mi_desktop_48_b0_2d_5f_b6_d0）
        ns = params.get('namespace', '').rstrip('/')

        try:
            from protocol.msg import MotionServoCmd, MotionStatus, BmsStatus
            from protocol.srv import MotionResultCmd
        except ImportError as e:
            node.get_logger().error(f'CyberDog2Plugin: protocol package not found: {e}')
            return False

        self._MotionServoCmd = MotionServoCmd
        self._MotionResultCmd = MotionResultCmd

        # 发布 servo 指令
        self._servo_pub = node.create_publisher(MotionServoCmd, ns + '/motion_servo_cmd', 10)

        # 订阅 motion_status
        self._status_sub = node.create_subscription(
            MotionStatus,
            ns + '/motion_status',
            self._cb_motion_status,
            10,
        )

        bms_status_topic = str(
            params.get('bms_status_topic', params.get('battery_status_topic', 'bms_status'))
        ).strip()
        self._bms_status_sub = None
        if bms_status_topic:
            resolved_bms_status_topic = self._resolve_topic(ns, bms_status_topic)
            self._bms_status_sub = node.create_subscription(
                BmsStatus,
                resolved_bms_status_topic,
                self._cb_bms_status,
                10,
            )
            node.get_logger().info(
                'CyberDog2Plugin charging recovery enabled, '
                f'bms_status_topic="{resolved_bms_status_topic}", '
                f'charging_recover_timeout_s={self._charging_recover_timeout_s}'
            )

        # Result 服务 client
        self._result_client = node.create_client(MotionResultCmd, ns + '/motion_result_cmd')

        # 保活定时器（20Hz）
        self._keepalive_timer = node.create_timer(0.05, self._cb_keepalive)

        node.get_logger().info(f'CyberDog2Plugin initialized, namespace="{ns}"')
        return True

    def device_type(self) -> str:
        return 'cyberdog2'

    def version(self) -> str:
        return '1.0.0'

    def internal_state_json(self) -> str:
        return json.dumps({'switch_status': self._switch_status})

    # ------------------------------------------------------------------ #
    #  执行                                                                #
    # ------------------------------------------------------------------ #

    def execute(self, cmd) -> tuple:
        # 解析 extra_json
        extra = {}
        if cmd.extra_json:
            try:
                extra = json.loads(cmd.extra_json)
            except Exception:
                pass

        action = cmd.action
        if action == 'emergency_stop':
            self._last_execute_ts = 0.0
            return self._call_result_cmd(_MOTION_ESTOP)
        elif action == 'stop':
            self._last_execute_ts = 0.0
            self._publish_servo(0.0, 0.0, 0.0)
            return True, ''

        motion_id = self._extract_motion_id(extra)
        if motion_id is not None:
            ready, error = self._ensure_motion_ready(cmd, motion_id=motion_id)
            if not ready:
                return False, error
            self._last_execute_ts = 0.0
            return self._call_result_cmd(motion_id)

        if action == 'stand':
            ready, error = self._ensure_motion_ready(cmd, motion_id=_MOTION_RECOVERYSTAND)
            if not ready:
                return False, error
            self._last_execute_ts = 0.0
            return self._call_result_cmd(_MOTION_RECOVERYSTAND)
        elif action == 'sit':
            ready, error = self._ensure_motion_ready(cmd, motion_id=_MOTION_GETDOWN)
            if not ready:
                return False, error
            self._last_execute_ts = 0.0
            return self._call_result_cmd(_MOTION_GETDOWN)

        ready, error = self._ensure_motion_ready(cmd)
        if not ready:
            return False, error

        # 速度指令 — 启用保活
        self._last_execute_ts = time.monotonic()
        self._publish_servo(
            cmd.linear_vel_x,
            cmd.linear_vel_y,
            cmd.angular_vel_z,
            roll=cmd.target_roll,
            pitch=cmd.target_pitch,
        )
        return True, ''

    def on_failsafe(self):
        """失控保护：停止运动，禁用保活。"""
        self._last_execute_ts = 0.0
        self._publish_servo(0.0, 0.0, 0.0)
        self._node.get_logger().warn('CyberDog2: failsafe triggered, sending zero-velocity')

    def on_failsafe_recovered(self):
        self._node.get_logger().info('CyberDog2: failsafe recovered')

    def destroy(self):
        if hasattr(self, '_keepalive_timer'):
            self._keepalive_timer.cancel()

    # ------------------------------------------------------------------ #
    #  内部方法                                                             #
    # ------------------------------------------------------------------ #

    def _resolve_topic(self, ns: str, topic: str) -> str:
        topic = str(topic or '').strip()
        if not topic:
            return topic
        if topic.startswith('/'):
            return topic
        if ns:
            return f'{ns}/{topic}'
        return topic

    def _publish_servo(self, vx: float, vy: float, wz: float,
                       roll: float = 0.0, pitch: float = 0.0,
                       step_height: list = None):
        if step_height is None:
            step_height = [0.05, 0.05]
        msg = self._MotionServoCmd()
        msg.motion_id = _MOTION_WALK_ADAPTIVELY
        msg.cmd_type = _SERVO_START
        msg.cmd_source = _SOURCE_APP
        msg.vel_des = [float(vx), float(vy), float(wz)]
        msg.rpy_des = [float(roll), float(pitch), 0.0]
        msg.step_height = step_height
        # 其余数组字段保持默认零值
        self._servo_pub.publish(msg)

    def _call_result_cmd(self, motion_id: int) -> tuple:
        if not self._result_client.service_is_ready():
            return False, 'motion_result_cmd service not ready'

        req = self._MotionResultCmd.Request()
        req.motion_id = motion_id
        req.cmd_source = _SOURCE_APP
        req.vel_des = [0.0, 0.0, 0.0]
        req.rpy_des = [0.0, 0.0, 0.0]
        req.pos_des = [0.0, 0.0, 0.0]
        req.acc_des = [0.0] * 6
        req.ctrl_point = [0.0, 0.0, 0.0]
        req.foot_pose = [0.0] * 6
        req.step_height = [0.0, 0.0]
        req.duration = 0

        future = self._result_client.call_async(req)

        # 轮询等待（主 executor 已在 spin，会处理服务响应回调）
        deadline = time.monotonic() + self._result_timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)

        if not future.done():
            return False, 'timeout'
        resp = future.result()
        return bool(resp.result), f'code={resp.code}'

    def _extract_motion_id(self, extra: dict):
        if 'motion_id' not in extra:
            return None
        try:
            return int(extra['motion_id'])
        except (TypeError, ValueError):
            return None

    def _ensure_motion_ready(self, cmd, motion_id: int = None) -> tuple:
        if self._switch_status == _SWITCH_STATUS_NORMAL:
            return True, ''

        if motion_id == _MOTION_ESTOP:
            return True, ''

        if self._switch_status == _SWITCH_STATUS_ESTOP and motion_id == _MOTION_RECOVERYSTAND:
            return True, ''

        if self._switch_status == _SWITCH_STATUS_ESTOP and self._auto_recover_from_estop:
            if motion_id is None and self._has_motion_request(cmd):
                return self._recover_from_estop()

        return False, self._motion_blocked_reason()

    def _recover_from_estop(self) -> tuple:
        self._node.get_logger().info(
            'CyberDog2: switch_status=ESTOP, sending RECOVERYSTAND before motion command'
        )
        success, result = self._call_result_cmd(_MOTION_RECOVERYSTAND)
        if not success:
            return False, json.dumps({'reason': 'recovery stand failed', 'detail': result})

        deadline = time.monotonic() + self._recovery_wait_timeout
        while self._switch_status != _SWITCH_STATUS_NORMAL and time.monotonic() < deadline:
            time.sleep(0.05)

        if self._switch_status != _SWITCH_STATUS_NORMAL:
            return False, json.dumps({
                'reason': 'recovery stand timeout',
                'switch_status': self._switch_status_name(),
            })

        return True, ''

    def _has_motion_request(self, cmd) -> bool:
        return any(abs(v) > 1e-6 for v in (
            cmd.linear_vel_x,
            cmd.linear_vel_y,
            cmd.angular_vel_z,
            cmd.target_roll,
            cmd.target_pitch,
        ))

    def _motion_blocked_reason(self) -> str:
        return json.dumps({
            'reason': 'motion not normal',
            'switch_status': self._switch_status_name(),
        })

    def _switch_status_name(self) -> str:
        return _SWITCH_STATUS_NAMES.get(self._switch_status, str(self._switch_status))

    def _set_switch_status(self, new_status: int, source: str = 'motion_status') -> bool:
        prev = self._switch_status
        self._switch_status = int(new_status)
        if self._switch_status == prev:
            return False

        self._node._write_state(
            'cyberdog2_switch_status',
            self._switch_status,
            urgent=self._switch_status != _SWITCH_STATUS_NORMAL,
        )

        prev_name = _SWITCH_STATUS_NAMES.get(prev, str(prev))
        name = _SWITCH_STATUS_NAMES.get(self._switch_status, str(self._switch_status))

        if self._switch_status != _SWITCH_STATUS_NORMAL and prev == _SWITCH_STATUS_NORMAL:
            self._node.get_logger().warn(
                f'CyberDog2: motion switch_status {prev_name} -> {name} ({self._switch_status})'
            )
            self._node._publish_uplink(
                data_type='cyberdog2_motion_abnormal',
                payload={'switch_status': self._switch_status},
                urgent=True,
            )
        elif self._switch_status == _SWITCH_STATUS_NORMAL and prev != _SWITCH_STATUS_NORMAL:
            if source == 'bms_status':
                self._node.get_logger().warn(
                    'CyberDog2: '
                    f'bms_status={self._charger_state_name()}, '
                    f'forcing motion switch_status {prev_name} -> NORMAL ({_SWITCH_STATUS_NORMAL})'
                )
            else:
                self._node.get_logger().info(
                    f'CyberDog2: motion switch_status {prev_name} -> NORMAL ({_SWITCH_STATUS_NORMAL})'
                )

        return True

    def _cb_motion_status(self, msg):
        new_status = int(msg.switch_status)
        if new_status == _SWITCH_STATUS_CHARGING and self._charger_disconnected_timed_out():
            if self._switch_status != _SWITCH_STATUS_NORMAL:
                self._set_switch_status(_SWITCH_STATUS_NORMAL, source='bms_status')
            return
        self._set_switch_status(new_status, source='motion_status')

    def _charger_state_name(self) -> str:
        if self._charger_connected is None:
            return 'UNKNOWN'
        return 'CONNECTED' if self._charger_connected else 'DISCONNECTED'

    def _charger_disconnected_timed_out(self) -> bool:
        if self._charger_connected is not False:
            return False
        if self._charger_disconnected_since is None:
            return False
        return (
            time.monotonic() - self._charger_disconnected_since >=
            self._charging_recover_timeout_s
        )

    def _cb_bms_status(self, msg):
        battery_level = max(0.0, min(100.0, float(msg.batt_soc)))
        self._node._write_state('battery_level', battery_level)

        self._charger_connected = any((
            bool(msg.power_wired_charging),
            bool(msg.power_wp_charging),
            bool(msg.power_finished_charging),
            bool(msg.power_expower_supply),
        ))

        if not self._charger_connected:
            if self._charger_disconnected_since is None:
                self._charger_disconnected_since = time.monotonic()
        else:
            self._charger_disconnected_since = None

        if self._switch_status != _SWITCH_STATUS_CHARGING:
            return

    def _maybe_recover_from_stale_charging(self):
        if self._switch_status != _SWITCH_STATUS_CHARGING:
            return
        if not self._charger_disconnected_timed_out():
            return
        self._set_switch_status(_SWITCH_STATUS_NORMAL, source='bms_status')

    def _cb_keepalive(self):
        """保活：若距上次 execute 超过 cmd_timeout_s，发布零速 servo 指令。"""
        if self._last_execute_ts > 0.0:
            elapsed = time.monotonic() - self._last_execute_ts
            if elapsed > self._cmd_timeout_s:
                self._publish_servo(0.0, 0.0, 0.0)
        self._maybe_recover_from_stale_charging()
