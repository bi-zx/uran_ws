"""uran_core_node — ROS1 版本主节点。"""
import json
import os
import queue
import threading
import time
from typing import Any

import rospy
import rospkg
import yaml

from uran_msgs.msg import (
    StateField, StateSnapshot, HeartbeatStatus,
    ModeSwitchCmd, MediaSwitchCmd, UplinkProtocolCmd,
    UnifiedMoveCmd, TaskCtrlCmd, MediaCtrlCmd, FrpcCtrlCmd, ParamUpdateCmd,
    UplinkPayload,
)
from uran_srvs.srv import (
    GetStateField, GetStateFieldResponse,
    SetStateField, SetStateFieldResponse,
    ConnectProtocol, ConnectProtocolResponse,
    GetNetworkStatus, GetNetworkStatusResponse,
    TriggerStateReport, TriggerStateReportResponse,
    ConfigureStateReport, ConfigureStateReportResponse,
)

from .state_manager import StateManager
from .mqtt_client import MqttClient


def _load_yaml(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


class UranCoreNode:
    def __init__(self):
        share = self._find_share_dir()
        net_cfg_raw = _load_yaml(os.path.join(share, 'config', 'network.yaml'))
        core_cfg_raw = _load_yaml(os.path.join(share, 'config', 'core.yaml'))

        self._net_cfg = net_cfg_raw.get('network', {})
        self._core_cfg = core_cfg_raw.get('uran_core', {})

        db_path = self._core_cfg.get('db_path', '/tmp/uran_core_state.db')
        self._broadcast_ms = int(self._core_cfg.get('state_broadcast_interval_ms', 1000))
        self._report_ms = int(self._core_cfg.get('state_report_interval_ms', 10000))
        self._hb_ms = int(self._net_cfg.get('heartbeat_interval_ms', 5000))
        self._report_on_change = bool(self._core_cfg.get('state_report_on_change', True))
        self._change_fields = set(self._core_cfg.get('state_report_change_fields', []))

        self._state = StateManager(db_path)
        self._state.set('device_id', self._net_cfg.get('device_id', 'device_001'), persistent=True)
        self._state.set('template_id', self._net_cfg.get('template_id', 'template_001'), persistent=True)
        self._state.register_change_callback(list(self._change_fields), self._on_state_change)

        self._mqtt_queue = queue.Queue()
        self._mqtt = MqttClient(self._net_cfg, self._on_mqtt_downlink)
        self._mqtt_enabled = self._net_cfg.get('mqtt', {}).get('enabled', True)

        self._pub_broadcast = rospy.Publisher('/uran/core/state/broadcast', StateSnapshot, queue_size=10, latch=True)
        self._pub_hb = rospy.Publisher('/uran/core/heartbeat/status', HeartbeatStatus, queue_size=10)
        self._pub_mode = rospy.Publisher('/uran/core/switch/mode', ModeSwitchCmd, queue_size=10)
        self._pub_media = rospy.Publisher('/uran/core/switch/media', MediaSwitchCmd, queue_size=10)
        self._pub_uplink_proto = rospy.Publisher('/uran/core/switch/uplink_protocol', UplinkProtocolCmd, queue_size=10)
        self._pub_move = rospy.Publisher('/uran/core/downlink/move_cmd', UnifiedMoveCmd, queue_size=10)
        self._pub_task = rospy.Publisher('/uran/core/downlink/task_ctrl', TaskCtrlCmd, queue_size=10)
        self._pub_media_ctrl = rospy.Publisher('/uran/core/downlink/media_ctrl', MediaCtrlCmd, queue_size=10)
        self._pub_frpc = rospy.Publisher('/uran/core/downlink/frpc_ctrl', FrpcCtrlCmd, queue_size=10)
        self._pub_param = rospy.Publisher('/uran/core/downlink/param_update', ParamUpdateCmd, queue_size=10)

        rospy.Subscriber('/uran/core/state/write', StateField, self._cb_state_write, queue_size=10)
        rospy.Subscriber('/uran/core/uplink/data', UplinkPayload, self._cb_uplink, queue_size=10)

        rospy.Service('/uran/core/state/get', GetStateField, self._srv_state_get)
        rospy.Service('/uran/core/state/set', SetStateField, self._srv_state_set)
        rospy.Service('/uran/core/network/connect', ConnectProtocol, self._srv_net_connect)
        rospy.Service('/uran/core/network/status', GetNetworkStatus, self._srv_net_status)
        rospy.Service('/uran/core/state_report/trigger', TriggerStateReport, self._srv_report_trigger)
        rospy.Service('/uran/core/state_report/configure', ConfigureStateReport, self._srv_report_configure)

        self._start_ts = time.time()
        self._last_report_ts = 0.0
        rospy.Timer(rospy.Duration(self._broadcast_ms / 1000.0), self._timer_broadcast)
        rospy.Timer(rospy.Duration(1.0), self._timer_report_check)
        rospy.Timer(rospy.Duration(self._hb_ms / 1000.0), self._timer_heartbeat)
        rospy.Timer(rospy.Duration(1.0), self._timer_uptime)
        rospy.Timer(rospy.Duration(0.05), self._timer_mqtt_queue)

        if self._mqtt_enabled:
            threading.Thread(target=self._mqtt_connect_thread, daemon=True).start()

        rospy.loginfo('uran_core_node started')

    def _find_share_dir(self) -> str:
        try:
            return rospkg.RosPack().get_path('uran_core')
        except Exception:
            return os.path.join(os.path.dirname(__file__), '..', '..')

    def _now_ns(self) -> int:
        return int(time.time() * 1_000_000_000)

    def _mqtt_connect_thread(self):
        ok = self._mqtt.connect()
        if not ok:
            rospy.logerr('MQTT connect failed')
            return
        rospy.loginfo('MQTT connected, sending registration...')
        result = self._mqtt.register(timeout_s=10.0)
        rospy.loginfo('Registration result: %s', result)
        if result == 'rejected':
            rospy.logerr('Registration rejected — node will continue without cloud link')
            self._state.set('online_status', False)
            self._update_protocol_table()
            return
        if result in ('timeout', 'publish_failed'):
            rospy.logwarn('Registration did not complete (%s) — keeping local ROS bring-up active', result)
        self._state.set('online_status', self._mqtt.is_connected())
        self._update_protocol_table()

    def _update_protocol_table(self):
        table = {
            'mqtt': self._mqtt.get_protocol_entry(),
            'websocket': {'available': False, 'registered': False, 'latency_ms': -1, 'last_check_ts': 0},
            'tcp': {'available': False, 'registered': False, 'latency_ms': -1, 'last_check_ts': 0},
            'udp': {'available': False, 'registered': False, 'latency_ms': -1, 'last_check_ts': 0},
            'lora': {'available': False, 'registered': False, 'latency_ms': -1, 'last_check_ts': 0},
        }
        self._state.set('protocol_table', table)

    def _on_mqtt_downlink(self, payload: dict):
        self._mqtt_queue.put(payload)

    def _timer_mqtt_queue(self, _event):
        try:
            while True:
                payload = self._mqtt_queue.get_nowait()
                self._handle_downlink(payload)
        except queue.Empty:
            pass

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
            rospy.logdebug('Unknown downlink msg_type: %s', msg_type)

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

        self._do_state_report(trigger='switch')

    def _route_move_cmd(self, payload: dict):
        msg = UnifiedMoveCmd()
        msg.msg_version = payload.get('msg_version', '1.0')
        msg.device_id = payload.get('device_id', '')
        msg.timestamp_ns = int(payload.get('timestamp_ms', 0)) * 1_000_000
        msg.controller = payload.get('controller', '')
        msg.linear_vel_x = float(payload.get('linear_vel_x') or 0.0)
        msg.linear_vel_y = float(payload.get('linear_vel_y') or 0.0)
        msg.linear_vel_z = float(payload.get('linear_vel_z') or 0.0)
        msg.angular_vel_z = float(payload.get('angular_vel_z') or 0.0)
        msg.target_roll = float(payload.get('target_roll') or 0.0)
        msg.target_pitch = float(payload.get('target_pitch') or 0.0)
        _raw_yaw = payload.get('target_yaw')
        msg.target_yaw = float('nan') if _raw_yaw is None else float(_raw_yaw)
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
        try:
            params = json.loads(msg.params_json)
            for k, v in params.items():
                self._state.set(k, v)
        except Exception:
            pass

    def _handle_state_query(self, payload: dict):
        field_names = payload.get('field_names', [])
        fields = self._state.get_fields(field_names) if field_names else self._state.get_all()
        response = {
            'msg_type': 'state_query_response',
            'msg_version': '1.0',
            'device_id': self._state.get('device_id'),
            'timestamp_ms': int(time.time() * 1000),
            'fields': fields,
        }
        self._mqtt.publish_uplink(response)

    def _cb_uplink(self, msg: UplinkPayload):
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
            rospy.logwarn('Urgent uplink failed for %s', msg.data_type)

    def _cb_state_write(self, msg: StateField):
        try:
            value = json.loads(msg.value_json)
        except json.JSONDecodeError:
            value = msg.value_json
        self._state.set(msg.field_name, value, persistent=msg.persistent)
        if msg.urgent:
            rospy.logdebug('urgent StateField %s → immediate report', msg.field_name)
            self._do_state_report(trigger='change')

    def _on_state_change(self, _field_name: str, _new_value: Any):
        if self._report_on_change:
            self._do_state_report(trigger='change')

    def _timer_broadcast(self, _event):
        msg = StateSnapshot()
        msg.msg_version = '1.0'
        msg.timestamp_ns = self._now_ns()
        msg.device_id = self._state.get('device_id') or ''
        msg.fields_json = self._state.get_snapshot_json()
        self._pub_broadcast.publish(msg)

    def _timer_report_check(self, _event):
        now = time.time()
        if now - self._last_report_ts >= self._report_ms / 1000.0:
            self._last_report_ts = now
            self._do_state_report(trigger='periodic')

    def _timer_heartbeat(self, _event):
        self._state.set('online_status', self._mqtt.is_connected())
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

    def _timer_uptime(self, _event):
        self._state.set('uptime_seconds', int(time.time() - self._start_ts))

    def _do_state_report(self, trigger: str = 'periodic'):
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

    def _srv_state_get(self, req):
        fields = self._state.get_fields(list(req.field_names))
        return GetStateFieldResponse(fields_json=json.dumps(fields, default=str), success=True)

    def _srv_state_set(self, req):
        try:
            value = json.loads(req.value_json)
            self._state.set(req.field_name, value, persistent=req.persistent)
            return SetStateFieldResponse(success=True, message='ok')
        except Exception as exc:
            return SetStateFieldResponse(success=False, message=str(exc))

    def _srv_net_connect(self, req):
        if req.protocol == 'mqtt':
            if req.action == 'connect':
                if not self._mqtt.is_connected():
                    threading.Thread(target=self._mqtt_connect_thread, daemon=True).start()
                return ConnectProtocolResponse(success=True, message='connecting')
            if req.action == 'disconnect':
                self._mqtt.disconnect()
                self._state.set('online_status', False)
                return ConnectProtocolResponse(success=True, message='disconnected')
            return ConnectProtocolResponse(success=False, message='unknown action: %s' % req.action)
        return ConnectProtocolResponse(success=False, message='protocol %s not implemented' % req.protocol)

    def _srv_net_status(self, _req):
        table = self._state.get('protocol_table') or {}
        return GetNetworkStatusResponse(protocol_table_json=json.dumps(table, default=str))

    def _srv_report_trigger(self, req):
        self._do_state_report(trigger='manual')
        return TriggerStateReportResponse(success=True, message='triggered (reason=%s)' % req.reason)

    def _srv_report_configure(self, req):
        if req.interval_ms > 0:
            self._report_ms = req.interval_ms
            self._last_report_ts = 0.0
        if req.protocol:
            self._core_cfg['state_report_protocol'] = req.protocol
        self._report_on_change = req.report_on_change
        return ConfigureStateReportResponse(
            success=True,
            message='ok',
            current_interval_ms=self._report_ms,
            current_protocol=self._core_cfg.get('state_report_protocol', 'mqtt'),
        )

    def shutdown(self):
        self._mqtt.disconnect()


def main():
    rospy.init_node('uran_core_node')
    node = UranCoreNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()
