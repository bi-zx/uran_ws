import json
import os
import threading
import time
from typing import Any, Dict, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from uran_msgs.msg import (
    FrpcCtrlCmd,
    HeartbeatStatus,
    MediaCtrlCmd,
    MediaSwitchCmd,
    ModeSwitchCmd,
    ParamUpdateCmd,
    StateField,
    StateSnapshot,
    TaskCtrlCmd,
    UnifiedMoveCmd,
    UplinkPayload,
    UplinkProtocolCmd,
)
from uran_srvs.srv import (
    ConfigureStateReport,
    ConnectProtocol,
    GetNetworkStatus,
    GetStateField,
    ListSensors,
    ReloadConfig,
    SetStateField,
    TriggerStateReport,
)

from .config_loader import load_base_config
from .downlink_router import DownlinkRouter
from .handlers import (
    handle_control_switch,
    handle_frpc_ctrl,
    handle_media_ctrl,
    handle_move_cmd,
    handle_param_update,
    handle_state_query,
    handle_task_ctrl,
)
from .models import DownlinkContext, ProtocolDescriptor, StateReportPolicy, descriptor_from_dict
from .protocol_factory import ProtocolFactory
from .registries import PackageRegistry, ProtocolRegistry
from .route_catalog import RouteCatalog
from .runtime_components import (
    run_broadcast_cycle,
    run_heartbeat_cycle,
    run_report_cycle,
    run_uptime_cycle,
)
from .service_handlers import (
    handle_configure_report_service,
    handle_connect_protocol_service,
    handle_list_sensors_service,
    handle_network_status_service,
    handle_reload_config_service,
    handle_state_get_service,
    handle_state_set_service,
    handle_trigger_report_service,
)
from .state_store import StateStore
from .uplink_gateway import UplinkGateway


class StructuredUranCoreNode(Node):

    ROUTE_CATALOG_PARAMETER = 'uran_base.route_catalog'

    def __init__(self):
        super().__init__('uran_base_core_node')
        self._share_dir = self._find_share_dir()
        self._config_path_override = ''
        self._protocol_factory = ProtocolFactory()
        self._package_registry = PackageRegistry()
        self._protocol_registry = ProtocolRegistry()
        self._downlink_router = DownlinkRouter()

        config = self._load_runtime_config()
        self._core_cfg = config['core'].get('uran_base_core', {})
        self._net_cfg = config['network'].get('network', {})
        self._route_catalog = RouteCatalog(config['routes'])
        self._state = StateStore(
            self._core_cfg.get('db_path', '/tmp/uran_base_state.db'),
            warning_callback=self.get_logger().warning,
        )
        self._default_protocol = str(self._core_cfg.get('default_protocol', 'mqtt'))
        self._uplink_gateway = UplinkGateway(
            self._protocol_registry,
            self._state,
            default_protocol=self._default_protocol,
        )
        self._report_policy = StateReportPolicy()
        self._broadcast_ms = 1000
        self._heartbeat_ms = 5000
        self._start_ts = time.time()
        self._last_report_ts = 0.0

        self._apply_runtime_config(config, initial=True)
        self._initialize_ros_interfaces()
        self._initialize_downlink_context()
        self._initialize_downlink_routes()
        self._initialize_timers()

        threading.Thread(target=self._connect_enabled_protocols, daemon=True).start()
        self.get_logger().info('Structured URAN core node started')

    def _find_share_dir(self) -> str:
        try:
            return get_package_share_directory('uran_base')
        except Exception:
            return os.path.join(os.path.dirname(__file__), '..')

    def _load_runtime_config(self, config_path: str = '') -> Dict[str, Dict[str, object]]:
        if config_path:
            config_dir = os.path.abspath(config_path)
            return load_base_config(config_dir)
        return load_base_config(self._share_dir)

    def _apply_runtime_config(self, config: Dict[str, Dict[str, object]], *, initial: bool) -> None:
        self._core_cfg = config['core'].get('uran_base_core', {})
        self._net_cfg = config['network'].get('network', {})
        self._route_catalog = RouteCatalog(config['routes'])
        self._default_protocol = str(self._core_cfg.get('default_protocol', 'mqtt'))
        self._broadcast_ms = int(self._core_cfg.get('state_broadcast_interval_ms', 1000))
        self._heartbeat_ms = int(self._net_cfg.get('heartbeat_interval_ms', 5000))
        self._report_policy = StateReportPolicy(
            interval_ms=int(self._core_cfg.get('state_report_interval_ms', 10000)),
            protocol=str(self._core_cfg.get('state_report_protocol', self._default_protocol)),
            report_on_change=bool(self._core_cfg.get('state_report_on_change', True)),
            watched_fields=list(self._core_cfg.get('state_report_change_fields', [])),
        )
        self._state.clear_change_callbacks()
        self._state.register_change_callback(self._report_policy.watched_fields, self._on_state_change)
        self._initialize_state(initial=initial)
        self._initialize_package_registry()
        self._initialize_protocol_registry()
        if hasattr(self, '_downlink_context'):
            self._initialize_downlink_context()
        self._publish_route_catalog_snapshot()

    def _initialize_state(self, *, initial: bool) -> None:
        self._state.set('device_id', self._net_cfg.get('device_id', 'device_001'), persistent=True)
        self._state.set('template_id', self._net_cfg.get('template_id', 'template_001'), persistent=True)
        self._state.set('primary_uplink_protocol', self._default_protocol)
        if initial:
            self._state.set('node_registry', {})

    def _initialize_package_registry(self) -> None:
        self._package_registry = PackageRegistry()
        descriptors = [
            descriptor_from_dict(item)
            for item in self._core_cfg.get('package_registry', [])
        ]
        self._package_registry.bulk_register(descriptors)
        self._state.set('package_registry', self._package_registry.as_dict())
        self._state.set('route_registry', self._route_catalog.snapshot())

    def _initialize_protocol_registry(self) -> None:
        protocols_cfg = dict(self._core_cfg.get('protocols', {}))
        if not protocols_cfg:
            protocols_cfg = {
                'mqtt': {
                    'enabled': bool(self._net_cfg.get('mqtt', {}).get('enabled', True)),
                    'priority': 10,
                    'config': dict(self._net_cfg.get('mqtt', {})),
                }
            }
        self._protocol_registry = ProtocolRegistry()
        self._uplink_gateway = UplinkGateway(
            self._protocol_registry,
            self._state,
            default_protocol=self._default_protocol,
        )
        for protocol_id, payload in protocols_cfg.items():
            try:
                self._protocol_registry.register(self._build_protocol_descriptor(protocol_id, payload))
            except Exception as exc:
                self.get_logger().error('Failed to build protocol %s: %s', protocol_id, exc)
        self._refresh_protocol_table()

    def _build_protocol_descriptor(self, protocol_id: str, payload: dict) -> ProtocolDescriptor:
        adapter = self._protocol_factory.build(
            protocol_id,
            dict(payload.get('config', {})),
            self._net_cfg,
            self._on_protocol_downlink,
        )
        return ProtocolDescriptor(
            protocol_id=protocol_id,
            adapter=adapter,
            enabled=bool(payload.get('enabled', True)),
            priority=int(payload.get('priority', 100)),
            metadata=dict(payload.get('metadata', {})),
        )

    def _initialize_ros_interfaces(self) -> None:
        self._pub_broadcast = self.create_publisher(
            StateSnapshot, self._route_catalog.topic('state_broadcast_topic'), 10
        )
        self._pub_heartbeat = self.create_publisher(
            HeartbeatStatus, self._route_catalog.topic('heartbeat_topic'), 10
        )
        self._pub_mode_switch = self.create_publisher(
            ModeSwitchCmd, self._route_catalog.topic('mode_switch_topic'), 10
        )
        self._pub_media_switch = self.create_publisher(
            MediaSwitchCmd, self._route_catalog.topic('media_switch_topic'), 10
        )
        self._pub_protocol_switch = self.create_publisher(
            UplinkProtocolCmd, self._route_catalog.topic('uplink_protocol_topic'), 10
        )
        self._pub_move = self.create_publisher(
            UnifiedMoveCmd, self._route_catalog.topic('move_cmd_topic'), 10
        )
        self._pub_task = self.create_publisher(
            TaskCtrlCmd, self._route_catalog.topic('task_ctrl_topic'), 10
        )
        self._pub_media_ctrl = self.create_publisher(
            MediaCtrlCmd, self._route_catalog.topic('media_ctrl_topic'), 10
        )
        self._pub_frpc = self.create_publisher(
            FrpcCtrlCmd, self._route_catalog.topic('frpc_ctrl_topic'), 10
        )
        self._pub_param = self.create_publisher(
            ParamUpdateCmd, self._route_catalog.topic('param_update_topic'), 10
        )

        self.create_subscription(
            StateField,
            self._route_catalog.topic('state_write_topic'),
            self._cb_state_write,
            10,
        )
        self.create_subscription(
            UplinkPayload,
            self._route_catalog.topic('uplink_topic'),
            self._cb_uplink,
            10,
        )

        self.create_service(GetStateField, self._route_catalog.service('get_state'), self._srv_state_get)
        self.create_service(SetStateField, self._route_catalog.service('set_state'), self._srv_state_set)
        self.create_service(
            ConnectProtocol,
            self._route_catalog.service('connect_protocol'),
            self._srv_connect_protocol,
        )
        self.create_service(
            GetNetworkStatus,
            self._route_catalog.service('get_network_status'),
            self._srv_network_status,
        )
        self.create_service(
            TriggerStateReport,
            self._route_catalog.service('trigger_report'),
            self._srv_trigger_report,
        )
        self.create_service(
            ConfigureStateReport,
            self._route_catalog.service('configure_report'),
            self._srv_configure_report,
        )
        reload_service = self._route_catalog.service('reload_config') if self._has_service('reload_config') else '/uran/core/config/reload'
        list_sensors_service = self._route_catalog.service('list_sensors') if self._has_service('list_sensors') else '/uran/core/sensors/list'
        self.create_service(ReloadConfig, reload_service, self._srv_reload_config)
        self.create_service(ListSensors, list_sensors_service, self._srv_list_sensors)

    def _initialize_downlink_context(self) -> None:
        self._downlink_context = DownlinkContext(
            state_store=self._state,
            uplink_gateway=self._uplink_gateway,
            package_registry=self._package_registry,
            route_catalog=self._route_catalog,
            publishers={
                'mode_switch': self._pub_mode_switch,
                'media_switch': self._pub_media_switch,
                'uplink_protocol_switch': self._pub_protocol_switch,
                'move_cmd': self._pub_move,
                'task_ctrl': self._pub_task,
                'media_ctrl': self._pub_media_ctrl,
                'frpc_ctrl': self._pub_frpc,
                'param_update': self._pub_param,
            },
            logger=self.get_logger(),
            now_ns=self._now_ns,
            payload_str=self._payload_str,
            payload_int=self._payload_int,
            payload_float=self._payload_float,
            payload_dict=self._payload_dict,
            payload_list=self._payload_list,
            payload_json_str=self._payload_json_str,
            payload_timestamp_ns=self._payload_timestamp_ns,
            resolve_task_type=self._resolve_task_type,
            do_state_report=self._do_state_report,
        )

    def _initialize_downlink_routes(self) -> None:
        self._register_downlink_handler('control_switch', handle_control_switch, 'Mode/media/protocol switch')
        self._register_downlink_handler('move_cmd', handle_move_cmd, 'Unified move command')
        self._register_downlink_handler('task_ctrl', handle_task_ctrl, 'Task control command')
        self._register_downlink_handler('media_ctrl', handle_media_ctrl, 'Media control command')
        self._register_downlink_handler('frpc_ctrl', handle_frpc_ctrl, 'FRPC control command')
        self._register_downlink_handler('param_update', handle_param_update, 'Parameter update command')
        self._register_downlink_handler('state_query', handle_state_query, 'State query request')

    def _register_downlink_handler(self, msg_type: str, handler, description: str) -> None:
        self._downlink_router.register_handler(
            msg_type,
            lambda payload, fn=handler: fn(self._downlink_context, payload),
            description,
        )

    def _initialize_timers(self) -> None:
        self.create_timer(self._broadcast_ms / 1000.0, self._timer_broadcast)
        self.create_timer(self._heartbeat_ms / 1000.0, self._timer_heartbeat)
        self.create_timer(1.0, self._timer_report_check)
        self.create_timer(1.0, self._timer_uptime)

    def _publish_route_catalog_snapshot(self) -> None:
        snapshot_json = json.dumps(self._route_catalog.snapshot(), ensure_ascii=False)
        if not self.has_parameter(self.ROUTE_CATALOG_PARAMETER):
            self.declare_parameter(self.ROUTE_CATALOG_PARAMETER, snapshot_json)
        self.set_parameters([
            Parameter(name=self.ROUTE_CATALOG_PARAMETER, value=snapshot_json)
        ])

    def _connect_enabled_protocols(self) -> None:
        online = False
        for descriptor in self._protocol_registry.active_protocols():
            online = self._connect_protocol(descriptor.protocol_id) or online
        self._state.set('online_status', online)
        self._refresh_protocol_table()

    def _connect_protocol(self, protocol_id: str) -> bool:
        descriptor = self._protocol_registry.get(protocol_id)
        if descriptor is None:
            return False
        adapter = descriptor.adapter
        if not adapter.connect():
            self.get_logger().error('Protocol connect failed: %s', protocol_id)
            self._refresh_protocol_table()
            return False
        result = adapter.register_device(timeout_s=10.0)
        self.get_logger().info('Protocol %s registration result: %s', protocol_id, result)
        self._refresh_protocol_table()
        return result in ('registered', 'auto_registered', 'legacy_connected')

    def _refresh_protocol_table(self) -> None:
        self._state.set('protocol_table', self._protocol_registry.protocol_table())

    def _on_protocol_downlink(self, payload: dict) -> None:
        try:
            self.executor.create_task(lambda: self._handle_downlink(payload))
        except Exception:
            self._handle_downlink(payload)

    def _handle_downlink(self, payload: dict) -> None:
        try:
            routed = self._downlink_router.route(payload)
            if not routed:
                self.get_logger().debug('No downlink handler for payload: %s', payload.get('msg_type', ''))
        except Exception as exc:
            self.get_logger().error('Downlink handling failed: %s', exc)

    def _cb_uplink(self, msg: UplinkPayload) -> None:
        ok = self._uplink_gateway.publish_uplink_message(msg)
        if not ok and msg.urgent:
            self.get_logger().warning('Urgent uplink failed for data_type=%s', msg.data_type)

    def _cb_state_write(self, msg: StateField) -> None:
        try:
            value = json.loads(msg.value_json)
        except json.JSONDecodeError:
            value = msg.value_json
        if msg.field_name.startswith('node_registry.'):
            self._merge_node_registry(msg.field_name, value, persistent=msg.persistent)
        else:
            success = self._state.set(msg.field_name, value, persistent=msg.persistent)
            if not success:
                self.get_logger().warning('Rejected state write: %s', msg.field_name)
        if msg.field_name == 'motion_control_lock':
            self._publish_mode_switch()
        if msg.urgent:
            self._do_state_report(trigger='change')

    def _merge_node_registry(self, field_name: str, value: Any, persistent: bool = False) -> None:
        parts = field_name.split('.')
        if len(parts) < 3:
            self._state.set(field_name, value, persistent=persistent)
            return
        package_name = parts[1]
        node_name = '.'.join(parts[2:])
        registry = self._state.get('node_registry', {}) or {}
        package_bucket = dict(registry.get(package_name, {}))
        package_bucket[node_name] = value
        registry[package_name] = package_bucket
        self._state.set('node_registry', registry, persistent=persistent)

    def _on_state_change(self, field_name: str, new_value: Any) -> None:
        del field_name, new_value
        if self._report_policy.report_on_change:
            self._do_state_report(trigger='change')

    def _timer_broadcast(self) -> None:
        run_broadcast_cycle(self)

    def _timer_heartbeat(self) -> None:
        run_heartbeat_cycle(self)

    def _select_heartbeat_protocol(self) -> Optional[ProtocolDescriptor]:
        primary = self._state.get('primary_uplink_protocol', self._default_protocol) or self._default_protocol
        descriptor = self._protocol_registry.get(primary)
        if descriptor is not None and descriptor.enabled:
            return descriptor
        active = self._protocol_registry.active_protocols()
        return active[0] if active else None

    def _timer_report_check(self) -> None:
        run_report_cycle(self)

    def _timer_uptime(self) -> None:
        run_uptime_cycle(self)

    def _do_state_report(self, trigger: str) -> None:
        payload = {
            'msg_type': 'state_snapshot',
            'msg_version': '1.0',
            'device_id': self._state.get('device_id', ''),
            'timestamp_ns': self._now_ns(),
            'trigger': trigger,
            'fields': self._state.get_all(),
            'package_registry': self._package_registry.as_dict(),
            'node_registry': self._state.get('node_registry', {}),
            'route_registry': self._route_catalog.snapshot(),
            'schema': self._state.describe_schema(),
        }
        self._uplink_gateway.publish_payload(payload, preferred_protocol=self._report_policy.protocol)

    def _publish_mode_switch(self) -> None:
        msg = ModeSwitchCmd()
        msg.control_mode = self._state.get('control_mode', '') or ''
        msg.controller = self._state.get('current_controller', '') or ''
        msg.timestamp_ns = self._now_ns()
        self._pub_mode_switch.publish(msg)

    def _expire_motion_control_lock_if_needed(self) -> None:
        lock = self._state.get('motion_control_lock', {}) or {}
        if not isinstance(lock, dict) or not bool(lock.get('active', False)):
            return
        expires_at_ns = int(lock.get('expires_at_ns', 0) or 0)
        if expires_at_ns <= 0 or self._now_ns() <= expires_at_ns:
            return
        updated = dict(lock)
        updated.update({
            'active': False,
            'reason': 'expired',
            'timestamp_ns': self._now_ns(),
        })
        self._state.set('motion_control_lock', updated)
        self._publish_mode_switch()

    def _srv_state_get(self, req: GetStateField.Request, res: GetStateField.Response):
        return handle_state_get_service(self, req, res)

    def _srv_state_set(self, req: SetStateField.Request, res: SetStateField.Response):
        return handle_state_set_service(self, req, res)

    def _srv_connect_protocol(self, req: ConnectProtocol.Request, res: ConnectProtocol.Response):
        return handle_connect_protocol_service(self, req, res)

    def _srv_network_status(self, req: GetNetworkStatus.Request, res: GetNetworkStatus.Response):
        return handle_network_status_service(self, req, res)

    def _srv_trigger_report(self, req: TriggerStateReport.Request, res: TriggerStateReport.Response):
        return handle_trigger_report_service(self, req, res)

    def _srv_configure_report(self, req: ConfigureStateReport.Request, res: ConfigureStateReport.Response):
        return handle_configure_report_service(self, req, res)

    def _srv_reload_config(self, req: ReloadConfig.Request, res: ReloadConfig.Response):
        return handle_reload_config_service(self, req, res)

    def _srv_list_sensors(self, req: ListSensors.Request, res: ListSensors.Response):
        return handle_list_sensors_service(self, req, res)

    def reload_runtime_config(self, config_path: str = '') -> tuple:
        try:
            config = self._load_runtime_config(config_path)
            self._apply_runtime_config(config, initial=False)
            details = 'reloaded core/network/routes; existing publishers and subscribers were not hot-rebound'
            return True, f'partial success: {details}'
        except Exception as exc:
            return False, str(exc)

    def list_registered_sensors(self) -> list:
        registry = self._state.get('node_registry', {}) or {}
        sensors = []
        for package_name, nodes in registry.items():
            if not isinstance(nodes, dict):
                continue
            for node_name, payload in nodes.items():
                if not isinstance(payload, dict):
                    continue
                capabilities = list(payload.get('capabilities', []))
                metadata = dict(payload.get('metadata', {}))
                uplink_types = list(payload.get('uplink_data_types', []))
                if not self._looks_like_sensor(capabilities, metadata, uplink_types):
                    continue
                sensors.append(
                    {
                        'package_name': package_name,
                        'node_name': node_name,
                        'capabilities': capabilities,
                        'metadata': metadata,
                        'uplink_data_types': uplink_types,
                    }
                )
        sensors.sort(key=lambda item: (item['package_name'], item['node_name']))
        return sensors

    def destroy_node(self):
        self._protocol_registry.disconnect_all()
        super().destroy_node()

    def _has_service(self, service_name: str) -> bool:
        try:
            self._route_catalog.service(service_name)
            return True
        except Exception:
            return False

    @staticmethod
    def _looks_like_sensor(capabilities: list, metadata: dict, uplink_types: list) -> bool:
        if any('sensor' in str(item).lower() for item in capabilities):
            return True
        if any('camera' in str(item).lower() for item in capabilities):
            return True
        if bool(metadata.get('sensor')) or bool(metadata.get('sensor_type')):
            return True
        return any(
            token in str(item).lower()
            for item in uplink_types
            for token in ('sensor', 'pose', 'image', 'imu', 'gps', 'camera')
        )

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _payload_str(self, payload: Dict[str, Any], key: str, default: str = '') -> str:
        value = payload.get(key, default)
        if value is None:
            return default
        return str(value)

    def _payload_int(self, payload: Dict[str, Any], key: str, default: int = 0) -> int:
        value = payload.get(key, default)
        try:
            return int(value)
        except (TypeError, ValueError):
            try:
                return int(float(value))
            except (TypeError, ValueError):
                return default

    def _payload_float(self, payload: Dict[str, Any], key: str, default: float = 0.0) -> float:
        value = payload.get(key, default)
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def _payload_dict(self, payload: Dict[str, Any], key: str) -> Dict[str, Any]:
        value = payload.get(key)
        return dict(value) if isinstance(value, dict) else {}

    def _payload_list(self, payload: Dict[str, Any], key: str) -> list:
        value = payload.get(key)
        return list(value) if isinstance(value, list) else []

    def _payload_json_str(self, payload: Dict[str, Any], key: str, default: str = '{}') -> str:
        value = payload.get(key, default)
        if isinstance(value, str):
            return value
        try:
            return json.dumps(value, ensure_ascii=False)
        except (TypeError, ValueError):
            return default

    def _payload_timestamp_ns(self, payload: Dict[str, Any], default: int = 0) -> int:
        timestamp_ns = self._payload_int(payload, 'timestamp_ns', 0)
        if timestamp_ns > 0:
            return timestamp_ns
        timestamp_ms = self._payload_int(payload, 'timestamp_ms', 0)
        if timestamp_ms > 0:
            return timestamp_ms * 1_000_000
        return int(default)

    @staticmethod
    def _resolve_task_type(payload: Dict[str, Any], task_params_json: str) -> str:
        raw_task_type = payload.get('task_type', '')
        task_type = '' if raw_task_type is None else str(raw_task_type).strip()
        if task_type and task_type != 'default_task':
            return task_type
        try:
            parsed = json.loads(task_params_json)
        except (TypeError, ValueError):
            return task_type
        if not isinstance(parsed, dict):
            return task_type
        return str(parsed.get('task_type') or parsed.get('type') or task_type).strip()


def main(args=None):
    rclpy.init(args=args)
    node = StructuredUranCoreNode()
    executor = MultiThreadedExecutor()
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
