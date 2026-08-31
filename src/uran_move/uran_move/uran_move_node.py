"""uran_move_node.py — URAN-move 主节点（T2.1 + T2.2 + T2.5 + T2.6）

职责：
  - 动态加载运控插件（T2.1）
  - 接收统一运控指令，执行预检限速 + 模式过滤（T2.2）
  - 执行结果上报到 uran_core uplink（T2.2）
  - 失控保护：链路中断超阈值触发保护动作，恢复后延迟退出（T2.5）
  - 提供 /uran/move/switch_plugin 服务，支持运行时热切换插件（T2.6）
"""

import importlib
import json
import math
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from uran_msgs.msg import UnifiedMoveCmd, ModeSwitchCmd, HeartbeatStatus, UplinkPayload, StateField, StateSnapshot
from uran_srvs.srv import SwitchMovePlugin


class UranMoveNode(Node):

    def __init__(self):
        super().__init__('uran_move_node')

        # 当前控制模式（由 ModeSwitchCmd 更新）
        self._control_mode = 'manual'
        self._controller = 'cloud'

        # 最近一次心跳状态（T2.5 失控保护）
        self._last_heartbeat = None  # type: HeartbeatStatus

        # 失控保护状态
        self._failsafe_active = False
        self._failsafe_cfg = {}          # 从 plugins.yaml 加载
        self._link_lost_since = None     # 链路首次中断时间（monotonic）
        self._link_recovered_since = None  # 链路首次恢复时间（monotonic）

        # 限速缓存（由 StateSnapshot 广播更新，避免每条指令同步查服务）
        self._linear_vel_limit = 1.0
        self._angular_vel_limit = 1.0
        self._motion_control_lock = {
            'active': False,
            'owner': '',
            'task_id': '',
            'reason': '',
            'timestamp_ns': 0,
            'expires_at_ns': 0,
        }
        self._control_suspended_by_lock = False
        self._last_received_command_seq = 0
        self._last_executed_command_seq = 0
        self._last_rejected_command_seq = 0

        # 插件相关
        self._plugin = None
        self._active_plugin_id = ''
        self._plugins_cfg: dict = {}  # id -> {module, class}

        # 加载配置并初始化插件
        self._load_config()
        self._load_plugin(self._default_plugin_id)

        # ---------- 失控保护定时器（1Hz 检查）----------
        self.create_timer(1.0, self._cb_failsafe_check)

        # ---------- 订阅 ----------
        self.create_subscription(
            UnifiedMoveCmd,
            '/uran/core/downlink/move_cmd',
            self._cb_move_cmd,
            10,
        )
        self.create_subscription(
            ModeSwitchCmd,
            '/uran/core/switch/mode',
            self._cb_mode_switch,
            10,
        )
        self.create_subscription(
            HeartbeatStatus,
            '/uran/core/heartbeat/status',
            self._cb_heartbeat,
            10,
        )
        self.create_subscription(
            StateSnapshot,
            '/uran/core/state/broadcast',
            self._cb_state_snapshot,
            10,
        )

        # ---------- 发布 ----------
        self._uplink_pub = self.create_publisher(UplinkPayload, '/uran/core/uplink/data', 10)
        self._state_pub = self.create_publisher(StateField, '/uran/core/state/write', 10)

        self._write_state('failsafe_active', False, urgent=True)
        plugin_state = self._plugin_internal_state()
        if 'switch_status' in plugin_state:
            self._write_state(
                'cyberdog2_switch_status',
                plugin_state['switch_status'],
                urgent=plugin_state['switch_status'] not in (0, '0', 'NORMAL'),
            )

        # ---------- 服务 ----------
        self.create_service(SwitchMovePlugin, '/uran/move/switch_plugin', self._srv_switch_plugin)

        self.get_logger().info(
            f'uran_move_node ready, active_plugin={self._active_plugin_id}'
        )

    # ------------------------------------------------------------------ #
    #  配置加载 & 插件动态加载                                              #
    # ------------------------------------------------------------------ #

    def _load_config(self):
        import yaml
        share = get_package_share_directory('uran_move')
        cfg_path = share + '/config/plugins.yaml'
        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)
        node_cfg = cfg.get('uran_move', {})
        self._default_plugin_id = node_cfg.get('active_plugin', '')
        for p in node_cfg.get('plugins', []):
            self._plugins_cfg[p['id']] = {
                'module': p['module'],
                'class': p['class'],
                'params': p.get('params', {}),
            }
        self._failsafe_cfg = node_cfg.get('failsafe', {
            'enabled': True,
            'failsafe_timeout_s': 5.0,
            'failsafe_action': 'stop',
            'failsafe_recover_delay_s': 2.0,
        })

    def _load_plugin(self, plugin_id: str) -> bool:
        if plugin_id not in self._plugins_cfg:
            self.get_logger().error(f'Plugin id not found in config: {plugin_id}')
            return False
        spec = self._plugins_cfg[plugin_id]
        try:
            mod = importlib.import_module(spec['module'])
            cls = getattr(mod, spec['class'])
            plugin = cls()
            ok = plugin.init(self, spec.get('params', {}))
            if not ok:
                self.get_logger().error(f'Plugin {plugin_id} init() returned False')
                return False
        except Exception as e:
            self.get_logger().error(f'Failed to load plugin {plugin_id}: {e}')
            return False

        # 销毁旧插件
        if self._plugin is not None:
            try:
                self._plugin.destroy()
            except Exception:
                pass

        self._plugin = plugin
        self._active_plugin_id = plugin_id
        self.get_logger().info(f'Plugin loaded: {plugin_id} ({plugin.device_type()} v{plugin.version()})')
        return True

    # ------------------------------------------------------------------ #
    #  回调                                                                #
    # ------------------------------------------------------------------ #

    def _cb_mode_switch(self, msg: ModeSwitchCmd):
        self._control_mode = msg.control_mode
        self._controller = msg.controller
        self.get_logger().debug(
            f'Mode switch: control_mode={self._control_mode}, controller={self._controller}'
        )

    def _cb_heartbeat(self, msg: HeartbeatStatus):
        self._last_heartbeat = msg
        now = time.monotonic()
        link_ok = msg.success

        if not link_ok:
            # 链路中断：记录首次中断时间，清除恢复计时
            if self._link_lost_since is None:
                self._link_lost_since = now
            self._link_recovered_since = None
        else:
            # 链路恢复：记录首次恢复时间，清除中断计时
            if self._link_recovered_since is None and self._link_lost_since is not None:
                self._link_recovered_since = now
            self._link_lost_since = None

    def _cb_state_snapshot(self, msg: StateSnapshot):
        """缓存状态快照中的限速和控制锁，供运控过滤使用。"""
        try:
            fields = json.loads(msg.fields_json)
            if 'linear_vel_limit' in fields:
                self._linear_vel_limit = float(fields['linear_vel_limit'])
            if 'angular_vel_limit' in fields:
                self._angular_vel_limit = float(fields['angular_vel_limit'])
            if 'motion_control_lock' in fields:
                self._update_motion_control_lock(fields.get('motion_control_lock') or {})
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    #  失控保护（T2.5）                                                    #
    # ------------------------------------------------------------------ #

    def _cb_failsafe_check(self):
        """1Hz 定时检查：链路中断超阈值触发保护，恢复后延迟退出。"""
        if not self._failsafe_cfg.get('enabled', True):
            return

        timeout_s = float(self._failsafe_cfg.get('failsafe_timeout_s', 5.0))
        recover_delay_s = float(self._failsafe_cfg.get('failsafe_recover_delay_s', 2.0))
        now = time.monotonic()

        if not self._failsafe_active:
            # 检查是否需要触发
            if (self._link_lost_since is not None and
                    now - self._link_lost_since >= timeout_s):
                self._trigger_failsafe(duration_s=now - self._link_lost_since)
        else:
            # 已在保护模式，检查是否可以恢复
            if (self._link_recovered_since is not None and
                    now - self._link_recovered_since >= recover_delay_s):
                self._recover_failsafe()

    def _trigger_failsafe(self, duration_s: float):
        """触发失控保护。"""
        self._failsafe_active = True
        action = self._failsafe_cfg.get('failsafe_action', 'stop')

        self.get_logger().warn(
            f'[FAILSAFE] Triggered! Link lost for {duration_s:.1f}s, action={action}'
        )

        # 执行保护动作
        if self._plugin is not None:
            try:
                if action == 'custom':
                    self._plugin.on_failsafe()
                else:
                    # stop / return_home / land / hold_position 均通过 execute 下发
                    from uran_msgs.msg import UnifiedMoveCmd
                    cmd = UnifiedMoveCmd()
                    cmd.timestamp_ns = self.get_clock().now().nanoseconds
                    cmd.controller = 'failsafe'
                    if action == 'stop':
                        cmd.action = 'stop'
                    elif action == 'return_home':
                        cmd.action = 'return_home'
                    elif action == 'land':
                        cmd.action = 'land'
                    elif action == 'hold_position':
                        cmd.action = 'stop'  # 通用平台用 stop 近似
                    else:
                        cmd.action = 'stop'
                    self._plugin.execute(cmd)
                    # 无论哪种动作，都额外调用 on_failsafe 钩子
                    self._plugin.on_failsafe()
            except Exception as e:
                self.get_logger().error(f'[FAILSAFE] Plugin action failed: {e}')

        # 写入状态空间
        self._write_state('failsafe_active', True, urgent=True)
        self._write_state('failsafe_action_executed', action)

        # 上报事件
        self._publish_uplink(
            data_type='failsafe_event',
            payload={
                'event': 'triggered',
                'failsafe_action': action,
                'duration_s': round(duration_s, 2),
                'timestamp_ns': self.get_clock().now().nanoseconds,
            },
            urgent=True,
        )

    def _recover_failsafe(self):
        """退出失控保护模式。"""
        self._failsafe_active = False
        self._link_recovered_since = None

        self.get_logger().info('[FAILSAFE] Recovered, resuming normal operation')

        if self._plugin is not None:
            try:
                self._plugin.on_failsafe_recovered()
            except Exception as e:
                self.get_logger().error(f'[FAILSAFE] on_failsafe_recovered error: {e}')

        self._write_state('failsafe_active', False, urgent=True)

        self._publish_uplink(
            data_type='failsafe_event',
            payload={
                'event': 'recovered',
                'failsafe_action': self._failsafe_cfg.get('failsafe_action', 'stop'),
                'timestamp_ns': self.get_clock().now().nanoseconds,
            },
            urgent=True,
        )

    def _cb_move_cmd(self, cmd: UnifiedMoveCmd):
        command_seq = self._command_seq(cmd)
        self._last_received_command_seq = command_seq
        safety_action = str(cmd.action or '').strip().lower() in {
            'stop',
            'emergency_stop',
        }
        requested_velocity = self._velocity_payload(cmd)

        if self._plugin is None:
            self.get_logger().warn('No active plugin, dropping move_cmd')
            self._publish_rejection(cmd, reason='no active move plugin')
            return

        # 停车类指令必须能穿过所有普通控制拦截，否则异常状态下反而
        # 无法把已经下发到底层的运动速度清零。
        if self._failsafe_active and not safety_action:
            self.get_logger().warn(
                f'[FAILSAFE] Active, dropping move_cmd from controller={cmd.controller}'
            )
            self._publish_rejection(cmd, reason='failsafe active')
            return

        if not safety_action and not self._check_motion_control_lock(cmd):
            return

        # 模式过滤
        if not safety_action and not self._check_mode(cmd):
            return

        # 预检限速
        if not safety_action:
            cmd = self._precheck_cmd(cmd)

        # 执行
        try:
            success, result_json = self._plugin.execute(cmd)
        except Exception as e:
            success, result_json = False, str(e)
            self.get_logger().error(f'Plugin execute error: {e}')

        if success:
            self._last_executed_command_seq = command_seq
        else:
            self._last_rejected_command_seq = command_seq
        self._report_result(
            cmd,
            success,
            result_json,
            requested_velocity=requested_velocity,
        )

    # ------------------------------------------------------------------ #
    #  预检限速（T2.2）                                                    #
    # ------------------------------------------------------------------ #

    def _precheck_cmd(self, cmd: UnifiedMoveCmd) -> UnifiedMoveCmd:
        """用缓存的速度限制截断指令速度（限速参数由 StateSnapshot 广播更新）。"""
        linear_limit = max(0.0, self._linear_vel_limit)
        angular_limit = max(0.0, self._angular_vel_limit)
        requested_velocity = self._velocity_payload(cmd)

        clamped = False
        vx, vy, vz = cmd.linear_vel_x, cmd.linear_vel_y, cmd.linear_vel_z
        v_norm = math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
        if v_norm > linear_limit and v_norm > 0.0:
            scale = linear_limit / v_norm
            cmd.linear_vel_x = vx * scale
            cmd.linear_vel_y = vy * scale
            cmd.linear_vel_z = vz * scale
            clamped = True

        wz = cmd.angular_vel_z
        if abs(wz) > angular_limit:
            cmd.angular_vel_z = math.copysign(angular_limit, wz)
            clamped = True

        if clamped:
            self._publish_uplink(
                data_type='move_clamp_event',
                payload={
                    'command_seq': self._command_seq(cmd),
                    'original_v_norm': v_norm,
                    'linear_limit': linear_limit,
                    'angular_limit': angular_limit,
                    'requested_velocity': requested_velocity,
                    'clamped_velocity': self._velocity_payload(cmd),
                },
                urgent=False,
            )

        return cmd

    # ------------------------------------------------------------------ #
    #  模式过滤（T2.2）                                                    #
    # ------------------------------------------------------------------ #

    def _check_mode(self, cmd: UnifiedMoveCmd) -> bool:
        """检查当前控制模式是否允许该指令来源。"""
        if self._control_mode == 'manual' and cmd.controller == 'auto':
            self._publish_rejection(
                cmd,
                reason='manual mode rejects auto controller',
            )
            return False
        if self._control_mode == 'auto' and cmd.controller in ('cloud', 'field'):
            self._publish_rejection(
                cmd,
                reason='auto mode rejects manual controller',
            )
            return False
        return True

    def _update_motion_control_lock(self, lock):
        if not isinstance(lock, dict):
            lock = {}
        previous_active = bool(self._motion_control_lock.get('active', False))
        previous_owner = str(self._motion_control_lock.get('owner') or '')

        normalized = {
            'active': bool(lock.get('active', False)),
            'owner': str(lock.get('owner') or ''),
            'task_id': str(lock.get('task_id') or ''),
            'reason': str(lock.get('reason') or ''),
            'timestamp_ns': int(lock.get('timestamp_ns') or 0),
            'expires_at_ns': int(lock.get('expires_at_ns') or 0),
        }
        self._motion_control_lock = normalized

        current_active = bool(normalized.get('active', False))
        current_owner = str(normalized.get('owner') or '')
        if current_active and (not previous_active or current_owner != previous_owner):
            self._suspend_plugin_for_control_lock(normalized)
            self._write_state('motion_control_blocked_by', current_owner, urgent=True)
            self._publish_uplink(
                data_type='move_control_lock_event',
                payload={
                    'event': 'acquired',
                    'owner': current_owner,
                    'task_id': normalized.get('task_id', ''),
                    'reason': normalized.get('reason', ''),
                    'timestamp_ns': self.get_clock().now().nanoseconds,
                },
                urgent=False,
            )
            return

        if previous_active and not current_active:
            self._resume_plugin_after_control_lock(previous_owner, normalized)
            self._write_state('motion_control_blocked_by', '', urgent=True)
            self._publish_uplink(
                data_type='move_control_lock_event',
                payload={
                    'event': 'released',
                    'owner': previous_owner,
                    'reason': normalized.get('reason', ''),
                    'timestamp_ns': self.get_clock().now().nanoseconds,
                },
                urgent=False,
            )

    def _suspend_plugin_for_control_lock(self, lock: dict):
        if self._plugin is None or self._control_suspended_by_lock:
            self._control_suspended_by_lock = True
            return
        try:
            self._plugin.on_control_suspended(
                owner=str(lock.get('owner') or ''),
                reason=str(lock.get('reason') or ''),
            )
        except Exception as exc:
            self.get_logger().error(f'Plugin suspend for control lock failed: {exc}')
        self._control_suspended_by_lock = True

    def _resume_plugin_after_control_lock(self, owner: str, lock: dict):
        if self._plugin is None or not self._control_suspended_by_lock:
            self._control_suspended_by_lock = False
            return
        try:
            self._plugin.on_control_resumed(
                owner=str(owner or ''),
                reason=str(lock.get('reason') or ''),
            )
        except Exception as exc:
            self.get_logger().error(f'Plugin resume after control lock failed: {exc}')
        self._control_suspended_by_lock = False

    def _check_motion_control_lock(self, cmd: UnifiedMoveCmd) -> bool:
        lock = self._motion_control_lock
        if not bool(lock.get('active', False)):
            return True

        owner = str(lock.get('owner') or '')
        controller = str(cmd.controller or '')
        source_pkg = self._cmd_source_pkg(cmd)
        if owner and controller == 'auto' and source_pkg == owner:
            return True

        self._publish_rejection(
            cmd,
            reason='motion control locked',
            extra={
                'source_pkg': source_pkg,
                'lock_owner': owner,
                'lock_task_id': str(lock.get('task_id') or ''),
            },
        )
        return False

    def _cmd_source_pkg(self, cmd: UnifiedMoveCmd) -> str:
        extra = self._cmd_extra(cmd)
        return str(extra.get('source_pkg') or '')

    def _cmd_extra(self, cmd: UnifiedMoveCmd) -> dict:
        try:
            extra = json.loads(cmd.extra_json or '{}')
        except Exception:
            extra = {}
        return extra if isinstance(extra, dict) else {}

    def _command_seq(self, cmd: UnifiedMoveCmd) -> int:
        try:
            return int(self._cmd_extra(cmd).get('command_seq') or 0)
        except (TypeError, ValueError):
            return 0

    def _velocity_payload(self, cmd: UnifiedMoveCmd) -> dict:
        return {
            'linear_x': float(cmd.linear_vel_x),
            'linear_y': float(cmd.linear_vel_y),
            'linear_z': float(cmd.linear_vel_z),
            'angular_z': float(cmd.angular_vel_z),
        }

    def _plugin_internal_state(self) -> dict:
        if self._plugin is None:
            return {}
        try:
            state = json.loads(self._plugin.internal_state_json() or '{}')
        except Exception as exc:
            return {'parse_error': str(exc)}
        return state if isinstance(state, dict) else {'raw_state': state}

    def _publish_rejection(
        self,
        cmd: UnifiedMoveCmd,
        *,
        reason: str,
        extra: dict = None,
    ):
        command_seq = self._command_seq(cmd)
        self._last_rejected_command_seq = command_seq
        plugin_state = self._plugin_internal_state()
        payload = {
            'reason': str(reason),
            'command_seq': command_seq,
            'received_command_seq': self._last_received_command_seq,
            'executed_command_seq': self._last_executed_command_seq,
            'rejected_command_seq': self._last_rejected_command_seq,
            'controller': str(cmd.controller or ''),
            'control_mode': self._control_mode,
            'action': str(cmd.action or ''),
            'requested_velocity': self._velocity_payload(cmd),
            'failsafe_active': self._failsafe_active,
            'switch_status': plugin_state.get('switch_status'),
            'switch_status_name': plugin_state.get('switch_status_name'),
        }
        payload.update(dict(extra or {}))
        self._publish_uplink(
            data_type='move_reject_event',
            payload=payload,
            urgent=False,
        )

    # ------------------------------------------------------------------ #
    #  上报                                                                #
    # ------------------------------------------------------------------ #

    def _report_result(
        self,
        cmd: UnifiedMoveCmd,
        success: bool,
        result_json: str,
        *,
        requested_velocity: dict = None,
    ):
        plugin_state = self._plugin_internal_state()
        try:
            result_detail = json.loads(result_json) if result_json else {}
        except Exception:
            result_detail = {'message': str(result_json or '')}
        payload = {
            'cmd_timestamp_ns': cmd.timestamp_ns,
            'command_seq': self._command_seq(cmd),
            'received_command_seq': self._last_received_command_seq,
            'executed_command_seq': self._last_executed_command_seq,
            'rejected_command_seq': self._last_rejected_command_seq,
            'success': success,
            'error_code': 0 if success else 1,
            'error_msg': '' if success else result_json,
            'result_detail': result_detail,
            'current_control_mode': self._control_mode,
            'controller': str(cmd.controller or ''),
            'action': str(cmd.action or ''),
            'requested_velocity': dict(requested_velocity or self._velocity_payload(cmd)),
            'clamped_velocity': self._velocity_payload(cmd),
            'actual_velocity': dict(plugin_state.get('last_published_velocity') or {}),
            'failsafe_active': self._failsafe_active,
            'switch_status': plugin_state.get('switch_status'),
            'switch_status_name': plugin_state.get('switch_status_name'),
            'plugin_id': self._active_plugin_id,
            'plugin_internal_state': plugin_state,
        }
        self._publish_uplink(data_type='move_result', payload=payload, urgent=False)

    def _publish_uplink(self, data_type: str, payload: dict, urgent: bool = False):
        msg = UplinkPayload()
        msg.source_pkg = 'uran_move'
        msg.data_type = data_type
        msg.preferred_protocol = ''
        msg.payload_json = json.dumps(payload)
        msg.urgent = urgent
        msg.timestamp_ns = self.get_clock().now().nanoseconds
        self._uplink_pub.publish(msg)

    def _write_state(self, field_name: str, value, persistent: bool = False, urgent: bool = False):
        msg = StateField()
        msg.field_name = field_name
        msg.value_json = json.dumps(value)
        msg.persistent = persistent
        msg.urgent = urgent
        msg.source_pkg = 'uran_move'
        msg.timestamp_ns = self.get_clock().now().nanoseconds
        self._state_pub.publish(msg)

    # ------------------------------------------------------------------ #
    #  服务：切换插件                                                       #
    # ------------------------------------------------------------------ #

    def _srv_switch_plugin(self, request: SwitchMovePlugin.Request, response: SwitchMovePlugin.Response):
        prev_plugin = self._active_plugin_id
        ok = self._load_plugin(request.plugin_id)
        response.success = ok
        response.message = 'ok' if ok else f'failed to load plugin: {request.plugin_id}'
        response.current_plugin = self._active_plugin_id

        if ok:
            self._write_state('active_move_plugin', self._active_plugin_id)
            self._publish_uplink(
                data_type='plugin_switch_event',
                payload={
                    'prev_plugin': prev_plugin,
                    'new_plugin': self._active_plugin_id,
                    'timestamp_ns': self.get_clock().now().nanoseconds,
                },
                urgent=False,
            )
        return response

    # ------------------------------------------------------------------ #
    #  生命周期                                                             #
    # ------------------------------------------------------------------ #

    def destroy_node(self):
        if self._plugin is not None:
            try:
                self._plugin.destroy()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UranMoveNode()
    # MultiThreadedExecutor 允许 _call_result_cmd 在回调中阻塞等待服务响应
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
