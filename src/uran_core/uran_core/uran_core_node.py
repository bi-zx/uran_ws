"""uran_core_node — T1.1~T1.5 主节点"""
import json
import os
import threading
import time
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import yaml

from uran_msgs.msg import (
    StateField, StateSnapshot, HeartbeatStatus,
    ModeSwitchCmd, MediaSwitchCmd, UplinkProtocolCmd,
    UnifiedMoveCmd, TaskCtrlCmd, MediaCtrlCmd, FrpcCtrlCmd, ParamUpdateCmd,
    UplinkPayload,
)
from uran_srvs.srv import (
    GetStateField, SetStateField,
    ConnectProtocol, GetNetworkStatus,
    TriggerStateReport, ConfigureStateReport,
)

from .state_manager import StateManager
from .mqtt_client import MqttClient


def _load_yaml(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


class UranCoreNode(Node):
    def __init__(self):
        super().__init__('uran_core_node')

        # ── 加载配置 ──────────────────────────────────────────────────────
        share = self._find_share_dir()
        net_cfg_raw = _load_yaml(os.path.join(share, 'config', 'network.yaml'))
        core_cfg_raw = _load_yaml(os.path.join(share, 'config', 'core.yaml'))

        self._net_cfg: dict = net_cfg_raw.get('network', {})
        self._core_cfg: dict = core_cfg_raw.get('uran_core', {})

        db_path = self._core_cfg.get('db_path', '/tmp/uran_core_state.db')
        self._broadcast_ms = int(self._core_cfg.get('state_broadcast_interval_ms', 1000))
        self._report_ms = int(self._core_cfg.get('state_report_interval_ms', 10000))
        self._hb_ms = int(self._net_cfg.get('heartbeat_interval_ms', 5000))
        self._report_on_change = bool(self._core_cfg.get('state_report_on_change', True))
        self._change_fields = set(self._core_cfg.get('state_report_change_fields', []))

        # ── 状态空间 ──────────────────────────────────────────────────────
        self._state = StateManager(db_path)
        # 从配置覆写 device_id / template_id
        self._state.set('device_id', self._net_cfg.get('device_id', 'device_001'), persistent=True)
        self._state.set('template_id', self._net_cfg.get('template_id', 'template_001'), persistent=True)
        self._state.register_change_callback(
            list(self._change_fields), self._on_state_change
        )

        # ── MQTT 客户端 ───────────────────────────────────────────────────
        self._mqtt: MqttClient = MqttClient(self._net_cfg, self._on_mqtt_downlink)
        self._mqtt_enabled = self._net_cfg.get('mqtt', {}).get('enabled', True)

        # ── ROS 发布者 ────────────────────────────────────────────────────
        latch = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_broadcast = self.create_publisher(StateSnapshot, '/uran/core/state/broadcast', 10)
        self._pub_hb = self.create_publisher(HeartbeatStatus, '/uran/core/heartbeat/status', 10)
        self._pub_mode = self.create_publisher(ModeSwitchCmd, '/uran/core/switch/mode', 10)
        self._pub_media = self.create_publisher(MediaSwitchCmd, '/uran/core/switch/media', 10)
        self._pub_uplink_proto = self.create_publisher(UplinkProtocolCmd, '/uran/core/switch/uplink_protocol', 10)
        self._pub_move = self.create_publisher(UnifiedMoveCmd, '/uran/core/downlink/move_cmd', 10)
        self._pub_task = self.create_publisher(TaskCtrlCmd, '/uran/core/downlink/task_ctrl', 10)
        self._pub_media_ctrl = self.create_publisher(MediaCtrlCmd, '/uran/core/downlink/media_ctrl', 10)
        self._pub_frpc = self.create_publisher(FrpcCtrlCmd, '/uran/core/downlink/frpc_ctrl', 10)
        self._pub_param = self.create_publisher(ParamUpdateCmd, '/uran/core/downlink/param_update', 10)

        # ── ROS 订阅者 ────────────────────────────────────────────────────
        self.create_subscription(StateField, '/uran/core/state/write', self._cb_state_write, 10)
        self.create_subscription(UplinkPayload, '/uran/core/uplink/data', self._cb_uplink, 10)

        # ── ROS 服务 ──────────────────────────────────────────────────────
        self.create_service(GetStateField, '/uran/core/state/get', self._srv_state_get)
        self.create_service(SetStateField, '/uran/core/state/set', self._srv_state_set)
        self.create_service(ConnectProtocol, '/uran/core/network/connect', self._srv_net_connect)
        self.create_service(GetNetworkStatus, '/uran/core/network/status', self._srv_net_status)
        self.create_service(TriggerStateReport, '/uran/core/state_report/trigger', self._srv_report_trigger)
        self.create_service(ConfigureStateReport, '/uran/core/state_report/configure', self._srv_report_configure)

        # ── 定时器 ────────────────────────────────────────────────────────
        self._start_ts = time.time()
        self._last_report_ts = 0.0          # T1.6: 支持动态修改上报周期
        self.create_timer(self._broadcast_ms / 1000.0, self._timer_broadcast)
        self.create_timer(1.0, self._timer_report_check)  # T1.6: 每秒检查是否到上报周期
        self.create_timer(self._hb_ms / 1000.0, self._timer_heartbeat)
        self.create_timer(1.0, self._timer_uptime)

        # ── 启动 MQTT ─────────────────────────────────────────────────────
        if self._mqtt_enabled:
            threading.Thread(target=self._mqtt_connect_thread, daemon=True).start()

        self.get_logger().info('uran_core_node started')

    # ================================================================== helpers
    def _find_share_dir(self) -> str:
        """优先用 ament 路径，回退到包源码目录。"""
        try:
            from ament_index_python.packages import get_package_share_directory
            return get_package_share_directory('uran_core')
        except Exception:
            return os.path.join(os.path.dirname(__file__), '..', '..')

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    # ================================================================== MQTT 连接线程
    def _mqtt_connect_thread(self):
        ok = self._mqtt.connect()
        if not ok:
            self.get_logger().error('MQTT connect failed')
            return
        self.get_logger().info('MQTT connected, sending registration...')
        result = self._mqtt.register(timeout_s=10.0)
        self.get_logger().info(f'Registration result: {result}')
        if result == 'rejected':
            self.get_logger().error('Registration rejected — node will continue without cloud link')
            return
        self._state.set('online_status', True)
        self._update_protocol_table()

    def _update_protocol_table(self):
        table = {
            'mqtt': self._mqtt.get_protocol_entry(),
            'websocket': {'available': False, 'latency_ms': -1, 'last_check_ts': 0},
            'tcp': {'available': False, 'latency_ms': -1, 'last_check_ts': 0},
            'udp': {'available': False, 'latency_ms': -1, 'last_check_ts': 0},
            'lora': {'available': False, 'latency_ms': -1, 'last_check_ts': 0},
        }
        self._state.set('protocol_table', table)

    # ================================================================== MQTT 下行回调（MQTT 线程）
    def _on_mqtt_downlink(self, payload: dict):
        # 从 paho 后台线程直接调度到 ROS executor 线程，避免队列轮询延时
        self.executor.create_task(lambda: self._handle_downlink(payload))

    def _handle_downlink(self, payload: dict):
        msg_type = payload.get('msg_type', '')
        if msg_type == 'control_switch':
            self._handle_control_switch(payload)
        elif msg_type == 'move_cmd':
            self._route_move_cmd(payload)
        elif msg_type == 'task_ctrl':
            self._route_task_ctrl(payload)
        elif msg_type == 'media_ctrl':
            self._route_media_ctrl(payload)
        elif msg_type == 'frpc_ctrl':
            self._route_frpc_ctrl(payload)
        elif msg_type == 'param_update':
            self._route_param_update(payload)
        elif msg_type == 'state_query':
            self._handle_state_query(payload)
        else:
            self.get_logger().debug(f'Unknown downlink msg_type: {msg_type}')

    # ================================================================== T1.4 控制切换
    def _handle_control_switch(self, payload: dict):
        sw = payload.get('switch', {})
        changed_mode = False
        changed_proto = False

        control_mode = sw.get('control_mode')
        controller = sw.get('controller')
        uplink_proto = sw.get('primary_uplink_protocol')
        media = sw.get('media', {})

        if control_mode:
            self._state.set('control_mode', control_mode)
            changed_mode = True
        if controller:
            self._state.set('current_controller', controller)
            changed_mode = True
        if uplink_proto:
            self._state.set('primary_uplink_protocol', uplink_proto)
            changed_proto = True

        if changed_mode:
            msg = ModeSwitchCmd()
            msg.control_mode = self._state.get('control_mode') or ''
            msg.controller = self._state.get('current_controller') or ''
            msg.timestamp_ns = self._now_ns()
            self._pub_mode.publish(msg)

        if media.get('action'):
            msg = MediaSwitchCmd()
            msg.action = media.get('action', '')
            msg.protocol = media.get('protocol', '')
            msg.timestamp_ns = self._now_ns()
            self._pub_media.publish(msg)

        if changed_proto:
            msg = UplinkProtocolCmd()
            msg.protocol = uplink_proto
            msg.timestamp_ns = self._now_ns()
            self._pub_uplink_proto.publish(msg)

        # 控制切换完成后触发即时上报
        self._do_state_report(trigger='switch')

    # ================================================================== T1.5 下行路由
    def _route_move_cmd(self, payload: dict):
        msg = UnifiedMoveCmd()
        msg.msg_version = payload.get('msg_version', '1.0')
        msg.device_id = payload.get('device_id', '')
        msg.timestamp_ns = int(payload.get('timestamp_ms', 0)) * 1_000_000
        msg.controller = payload.get('controller', '')
        msg.linear_vel_x = float(payload.get('linear_vel_x', 0.0))
        msg.linear_vel_y = float(payload.get('linear_vel_y', 0.0))
        msg.linear_vel_z = float(payload.get('linear_vel_z', 0.0))
        msg.angular_vel_z = float(payload.get('angular_vel_z', 0.0))
        msg.target_roll = float(payload.get('target_roll', 0.0))
        msg.target_pitch = float(payload.get('target_pitch', 0.0))
        msg.target_yaw = float(payload.get('target_yaw', 0.0))
        msg.action = payload.get('action', '')
        msg.extra_json = payload.get('extra_json', '{}')
        self._pub_move.publish(msg)

    def _route_task_ctrl(self, payload: dict):
        msg = TaskCtrlCmd()
        msg.msg_version = payload.get('msg_version', '1.0')
        msg.task_id = payload.get('task_id', '')
        msg.action = payload.get('action', '')
        msg.task_type = payload.get('task_type', '')
        msg.task_params_json = payload.get('task_params_json', '{}')
        msg.timestamp_ns = int(payload.get('timestamp_ms', 0)) * 1_000_000
        self._pub_task.publish(msg)

    def _route_media_ctrl(self, payload: dict):
        msg = MediaCtrlCmd()
        msg.action = payload.get('action', '')
        msg.protocol = payload.get('protocol', '')
        msg.channel_id = payload.get('channel_id', '')
        msg.signal_json = payload.get('signal_json', '{}')
        msg.timestamp_ns = int(payload.get('timestamp_ms', 0)) * 1_000_000
        self._pub_media_ctrl.publish(msg)

    def _route_frpc_ctrl(self, payload: dict):
        msg = FrpcCtrlCmd()
        msg.action = payload.get('action', '')
        msg.frps_host = payload.get('frps_host', '')
        msg.frps_port = int(payload.get('frps_port', 0))
        msg.service_name = payload.get('service_name', '')
        msg.local_port = int(payload.get('local_port', 0))
        msg.remote_port = int(payload.get('remote_port', 0))
        msg.auth_token = payload.get('auth_token', '')
        msg.timestamp_ns = int(payload.get('timestamp_ms', 0)) * 1_000_000
        self._pub_frpc.publish(msg)

    def _route_param_update(self, payload: dict):
        msg = ParamUpdateCmd()
        msg.params_json = payload.get('params_json', '{}')
        msg.timestamp_ns = int(payload.get('timestamp_ms', 0)) * 1_000_000
        self._pub_param.publish(msg)
        # 同时更新本地状态空间
        try:
            params = json.loads(msg.params_json)
            for k, v in params.items():
                self._state.set(k, v)
        except Exception:
            pass

    def _handle_state_query(self, payload: dict):
        field_names = payload.get('field_names', [])
        if field_names:
            fields = self._state.get_fields(field_names)
        else:
            fields = self._state.get_all()
        response = {
            'msg_type': 'state_query_response',
            'msg_version': '1.0',
            'device_id': self._state.get('device_id'),
            'timestamp_ms': int(time.time() * 1000),
            'fields': fields,
        }
        self._mqtt.publish_uplink(response)

    # ================================================================== T1.5 上行通路
    def _cb_uplink(self, msg: UplinkPayload):
        """功能包通过 /uran/core/uplink/data 上报数据，转发至 MQTT。"""
        preferred = msg.preferred_protocol or self._state.get('primary_uplink_protocol') or 'mqtt'
        # 当前仅实现 MQTT 通路；后续可按 preferred 选择协议
        payload = {
            'msg_type': 'uplink_data',
            'msg_version': '1.0',
            'device_id': self._state.get('device_id'),
            'source_pkg': msg.source_pkg,
            'data_type': msg.data_type,
            'timestamp_ns': msg.timestamp_ns,
            'payload': msg.payload_json,
        }
        ok = self._mqtt.publish_uplink(payload)
        if not ok and msg.urgent:
            # urgent 数据：枚举所有可用通路重试（当前仅 MQTT）
            self.get_logger().warning(f'Urgent uplink failed for {msg.data_type}')

    # ================================================================== 状态写入订阅
    def _cb_state_write(self, msg: StateField):
        try:
            value = json.loads(msg.value_json)
        except json.JSONDecodeError:
            value = msg.value_json
        self._state.set(msg.field_name, value, persistent=msg.persistent)
        # T1.6: urgent StateField 触发即时上报
        if msg.urgent:
            self.get_logger().debug(f'urgent StateField {msg.field_name} → immediate report')
            self._do_state_report(trigger='change')

    # ================================================================== 状态变更回调
    def _on_state_change(self, field_name: str, new_value: Any):
        if self._report_on_change:
            self._do_state_report(trigger='change')

    # ================================================================== 定时器
    def _timer_broadcast(self):
        """对内广播状态快照。"""
        msg = StateSnapshot()
        msg.msg_version = '1.0'
        msg.timestamp_ns = self._now_ns()
        msg.device_id = self._state.get('device_id') or ''
        msg.fields_json = self._state.get_snapshot_json()
        self._pub_broadcast.publish(msg)

    def _timer_report_check(self):
        """T1.6: 每秒检查，支持动态修改上报周期。"""
        now = time.time()
        if now - self._last_report_ts >= self._report_ms / 1000.0:
            self._last_report_ts = now
            self._do_state_report(trigger='periodic')

    def _timer_heartbeat(self):
        """发送心跳包并发布 HeartbeatStatus。"""
        self._update_protocol_table()
        connected = self._mqtt.is_connected()
        hb = {
            'msg_type': 'heartbeat',
            'msg_version': '1.0',
            'device_id': self._state.get('device_id'),
            'timestamp_ms': int(time.time() * 1000),
            'online': connected,
            'uptime_seconds': int(self._state.get('uptime_seconds') or 0),
            'battery_level': self._state.get('battery_level') or 0.0,
            'control_mode': self._state.get('control_mode') or 'manual',
            'current_controller': self._state.get('current_controller') or 'cloud',
            'primary_uplink_protocol': self._state.get('primary_uplink_protocol') or 'mqtt',
            'protocol_table': {
                k: v.get('available', False)
                for k, v in (self._state.get('protocol_table') or {}).items()
            },
            'position': self._state.get('position') or {},
            'error_code': self._state.get('error_code') or 0,
        }
        self._mqtt.publish_uplink(hb)

        ros_hb = HeartbeatStatus()
        ros_hb.timestamp_ns = self._now_ns()
        ros_hb.protocol = 'mqtt'
        ros_hb.last_sent_ts = self._now_ns()
        ros_hb.success = connected
        self._pub_hb.publish(ros_hb)

    def _timer_uptime(self):
        self._state.set('uptime_seconds', int(time.time() - self._start_ts))

    def _do_state_report(self, trigger: str = 'periodic'):
        """构建 state_snapshot 并通过 MQTT 上报。"""
        fields = self._state.get_all()
        payload = {
            'msg_type': 'state_snapshot',
            'msg_version': '1.0',
            'device_id': fields.get('device_id', ''),
            'timestamp_ns': self._now_ns(),
            'trigger': trigger,
            'fields': fields,
        }
        self._mqtt.publish_uplink(payload)

    # ================================================================== ROS 服务
    def _srv_state_get(self, req: GetStateField.Request, res: GetStateField.Response):
        fields = self._state.get_fields(list(req.field_names))
        res.fields_json = json.dumps(fields, default=str)
        res.success = True
        return res

    def _srv_state_set(self, req: SetStateField.Request, res: SetStateField.Response):
        try:
            value = json.loads(req.value_json)
            self._state.set(req.field_name, value, persistent=req.persistent)
            res.success = True
            res.message = 'ok'
        except Exception as exc:
            res.success = False
            res.message = str(exc)
        return res

    def _srv_net_connect(self, req: ConnectProtocol.Request, res: ConnectProtocol.Response):
        if req.protocol == 'mqtt':
            if req.action == 'connect':
                if not self._mqtt.is_connected():
                    threading.Thread(target=self._mqtt_connect_thread, daemon=True).start()
                res.success = True
                res.message = 'connecting'
            elif req.action == 'disconnect':
                self._mqtt.disconnect()
                self._state.set('online_status', False)
                res.success = True
                res.message = 'disconnected'
            else:
                res.success = False
                res.message = f'unknown action: {req.action}'
        else:
            res.success = False
            res.message = f'protocol {req.protocol} not implemented'
        return res

    def _srv_net_status(self, req: GetNetworkStatus.Request, res: GetNetworkStatus.Response):
        table = self._state.get('protocol_table') or {}
        res.protocol_table_json = json.dumps(table, default=str)
        return res

    def _srv_report_trigger(self, req: TriggerStateReport.Request, res: TriggerStateReport.Response):
        self._do_state_report(trigger='manual')
        res.success = True
        res.message = f'triggered (reason={req.reason})'
        return res

    def _srv_report_configure(self, req: ConfigureStateReport.Request, res: ConfigureStateReport.Response):
        if req.interval_ms > 0:
            self._report_ms = req.interval_ms
            self._last_report_ts = 0.0  # 立即触发一次，让新周期从现在开始计算
        if req.protocol:
            self._core_cfg['state_report_protocol'] = req.protocol
        self._report_on_change = req.report_on_change
        res.success = True
        res.message = 'ok'
        res.current_interval_ms = self._report_ms
        res.current_protocol = self._core_cfg.get('state_report_protocol', 'mqtt')
        return res


def main(args=None):
    rclpy.init(args=args)
    node = UranCoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._mqtt.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

