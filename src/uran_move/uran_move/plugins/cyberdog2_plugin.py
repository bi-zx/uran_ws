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
_MOTION_WALK_USERTROT = 303
_MOTION_RECOVERYSTAND = 111
_MOTION_GETDOWN = 101
_MOTION_ESTOP = 0

# MotionServoCmd.cmd_type
_SERVO_START = 0

# cmd_source（-1 = 最高调试优先级，绕过 motion_manager 优先级队列）
_SOURCE_APP = -1

# switch_status 含义（来自 MotionStatus.msg）
_SWITCH_STATUS_NORMAL = 0
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
        self._cmd_timeout_s = 0.5
        self._last_execute_ts = 0.0
        self._switch_status = _SWITCH_STATUS_NORMAL

        # cyberdog_ws 节点的 ROS namespace（如 /mi_desktop_48_b0_2d_5f_b6_d0）
        ns = params.get('namespace', '').rstrip('/')

        try:
            from protocol.msg import MotionServoCmd, MotionStatus
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
        # 解析 extra_json（需要提前解析以判断是否为恢复指令）
        extra = {}
        if cmd.extra_json:
            try:
                extra = json.loads(cmd.extra_json)
            except Exception:
                pass

        action = cmd.action
        
        # 恢复性指令（stand）允许在任何状态下执行，用于从异常状态恢复
        if action == 'stand':
            self._last_execute_ts = 0.0
            return self._call_result_cmd(_MOTION_RECOVERYSTAND)
        
        # 检查 motion_status - 非 NORMAL 状态拒绝其他指令
        if self._switch_status != _SWITCH_STATUS_NORMAL:
            name = _SWITCH_STATUS_NAMES.get(self._switch_status, str(self._switch_status))
            return False, json.dumps({'reason': 'motion not normal', 'switch_status': name})

        # 若 extra 含 motion_id → Result 模式（动作指令，禁用保活）
        if 'motion_id' in extra:
            self._last_execute_ts = 0.0
            return self._call_result_cmd(int(extra['motion_id']))

        if action == 'emergency_stop':
            self._last_execute_ts = 0.0
            return self._call_result_cmd(_MOTION_ESTOP)
        elif action == 'sit':
            self._last_execute_ts = 0.0
            return self._call_result_cmd(_MOTION_GETDOWN)
        elif action == 'stop':
            self._last_execute_ts = 0.0
            self._publish_servo(0.0, 0.0, 0.0)
            return True, ''
        else:
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

    def _publish_servo(self, vx: float, vy: float, wz: float,
                       roll: float = 0.0, pitch: float = 0.0,
                       step_height: list = None):
        if step_height is None:
            step_height = [0.05, 0.05]
        msg = self._MotionServoCmd()
        msg.motion_id = _MOTION_WALK_USERTROT
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

    def _cb_motion_status(self, msg):
        prev = self._switch_status
        self._switch_status = msg.switch_status
        if self._switch_status != _SWITCH_STATUS_NORMAL and prev == _SWITCH_STATUS_NORMAL:
            name = _SWITCH_STATUS_NAMES.get(self._switch_status, str(self._switch_status))
            self._node.get_logger().warn(
                f'CyberDog2: motion switch_status -> {name} ({self._switch_status})'
            )
            # 写入状态空间
            self._node._write_state(
                'cyberdog2_switch_status',
                self._switch_status,
                urgent=True,
            )
            # 上报事件
            self._node._publish_uplink(
                data_type='cyberdog2_motion_abnormal',
                payload={'switch_status': self._switch_status},
                urgent=True,
            )

    def _cb_keepalive(self):
        """保活：若距上次 execute 超过 cmd_timeout_s，发布零速 servo 指令。"""
        if self._last_execute_ts > 0.0:
            elapsed = time.monotonic() - self._last_execute_ts
            if elapsed > self._cmd_timeout_s:
                self._publish_servo(0.0, 0.0, 0.0)
