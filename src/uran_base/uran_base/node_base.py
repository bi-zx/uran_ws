import json
from typing import Callable, Dict, List, Optional, Type

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

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

from .models import (
    CoreRouteBindings,
    DownlinkPublicationSpec,
    DownlinkSubscriptionSpec,
    NodeRegistration,
)
from .route_catalog import RouteCatalog


class UranBaseNode(Node):
    """URAN package base node with built-in core access, registration, and hooks."""

    ROUTE_CATALOG_PARAMETER = 'uran_base.route_catalog'
    DEFAULT_CORE_NODE_NAME = 'uran_base_core_node'

    def __init__(
        self,
        node_name: str,
        *,
        package_name: str,
        route_catalog: Optional[RouteCatalog] = None,
        capabilities: Optional[List[str]] = None,
        metadata: Optional[Dict[str, object]] = None,
        core_node_name: str = DEFAULT_CORE_NODE_NAME,
    ):
        super().__init__(node_name)
        self.package_name = package_name
        self._core_node_name = str(core_node_name or self.DEFAULT_CORE_NODE_NAME)
        self._route_catalog = self._resolve_route_catalog(route_catalog)
        self._bindings = CoreRouteBindings(
            state_write_topic=self._route_catalog.topic('state_write_topic'),
            state_broadcast_topic=self._route_catalog.topic('state_broadcast_topic'),
            uplink_topic=self._route_catalog.topic('uplink_topic'),
            heartbeat_topic=self._route_catalog.topic('heartbeat_topic'),
            mode_switch_topic=self._route_catalog.topic('mode_switch_topic'),
            media_switch_topic=self._route_catalog.topic('media_switch_topic'),
            uplink_protocol_topic=self._route_catalog.topic('uplink_protocol_topic'),
            move_cmd_topic=self._route_catalog.topic('move_cmd_topic'),
            task_ctrl_topic=self._route_catalog.topic('task_ctrl_topic'),
            media_ctrl_topic=self._route_catalog.topic('media_ctrl_topic'),
            frpc_ctrl_topic=self._route_catalog.topic('frpc_ctrl_topic'),
            param_update_topic=self._route_catalog.topic('param_update_topic'),
        )
        self._state_pub = self.create_publisher(StateField, self._bindings.state_write_topic, 10)
        self._uplink_pub = self.create_publisher(UplinkPayload, self._bindings.uplink_topic, 10)
        self._registration = NodeRegistration(
            package_name=self.package_name,
            node_name=node_name,
            capabilities=list(capabilities or []),
            metadata=dict(metadata or {}),
        )
        self._downlink_subscriptions: Dict[str, object] = {}
        self._downlink_publishers: Dict[str, object] = {}

    def core_routes(self) -> CoreRouteBindings:
        return self._bindings

    def route_catalog(self) -> RouteCatalog:
        return self._route_catalog

    def registration_snapshot(self) -> dict:
        return {
            'package_name': self._registration.package_name,
            'node_name': self._registration.node_name,
            'capabilities': list(self._registration.capabilities),
            'downlink_subscriptions': [
                {
                    'route_name': item.route_name,
                    'message_type': item.message_type,
                    'description': item.description,
                }
                for item in self._registration.downlink_subscriptions
            ],
            'downlink_publications': [
                {
                    'route_name': item.route_name,
                    'message_type': item.message_type,
                    'description': item.description,
                }
                for item in self._registration.downlink_publications
            ],
            'uplink_data_types': list(self._registration.uplink_data_types),
            'state_fields_written': list(self._registration.state_fields_written),
            'service_exports': list(self._registration.service_exports),
            'metadata': dict(self._registration.metadata),
        }

    def register_with_core(self, *, urgent: bool = True) -> None:
        registry = self.registration_snapshot()
        self.publish_state_field(
            field_name=f'node_registry.{self.package_name}.{self.get_name()}',
            value=registry,
            urgent=urgent,
        )
        self.publish_uplink(
            data_type='node_registration',
            payload=registry,
            urgent=urgent,
        )

    def declare_capability(self, capability: str) -> None:
        if capability and capability not in self._registration.capabilities:
            self._registration.capabilities.append(capability)

    def declare_uplink_type(self, data_type: str) -> None:
        if data_type and data_type not in self._registration.uplink_data_types:
            self._registration.uplink_data_types.append(data_type)

    def declare_service_export(self, service_name: str) -> None:
        if service_name and service_name not in self._registration.service_exports:
            self._registration.service_exports.append(service_name)

    def publish_state_field(
        self,
        *,
        field_name: str,
        value,
        persistent: bool = False,
        urgent: bool = False,
    ) -> None:
        self._record_state_field(field_name)
        msg = StateField()
        msg.field_name = field_name
        msg.value_json = json.dumps(value, ensure_ascii=False)
        msg.persistent = persistent
        msg.urgent = urgent
        msg.source_pkg = self.package_name
        msg.timestamp_ns = self.get_clock().now().nanoseconds
        self._state_pub.publish(msg)

    def publish_uplink(
        self,
        *,
        data_type: str,
        payload: dict,
        preferred_protocol: str = '',
        urgent: bool = False,
    ) -> None:
        self.declare_uplink_type(data_type)
        msg = UplinkPayload()
        msg.source_pkg = self.package_name
        msg.data_type = data_type
        msg.preferred_protocol = preferred_protocol
        msg.payload_json = json.dumps(payload, ensure_ascii=False)
        msg.urgent = urgent
        msg.timestamp_ns = self.get_clock().now().nanoseconds
        self._uplink_pub.publish(msg)

    def publish_downlink_move_cmd(
        self,
        *,
        controller: str = 'auto',
        linear_vel_x: float = 0.0,
        linear_vel_y: float = 0.0,
        linear_vel_z: float = 0.0,
        angular_vel_z: float = 0.0,
        target_roll: float = 0.0,
        target_pitch: float = 0.0,
        target_yaw: float = 0.0,
        action: str = '',
        device_id: str = '',
        extra_json: str = '',
        extra: Optional[Dict[str, object]] = None,
    ) -> UnifiedMoveCmd:
        msg = UnifiedMoveCmd()
        msg.msg_version = '1.0'
        msg.device_id = str(device_id or '')
        msg.timestamp_ns = self.get_clock().now().nanoseconds
        msg.controller = str(controller or 'auto')
        msg.linear_vel_x = float(linear_vel_x)
        msg.linear_vel_y = float(linear_vel_y)
        msg.linear_vel_z = float(linear_vel_z)
        msg.angular_vel_z = float(angular_vel_z)
        msg.target_roll = float(target_roll)
        msg.target_pitch = float(target_pitch)
        msg.target_yaw = float(target_yaw)
        msg.action = str(action or '')
        msg.extra_json = self._normalize_extra_json(extra_json=extra_json, extra=extra)
        self._publish_downlink_message(
            route_name='move_cmd_topic',
            message_type=UnifiedMoveCmd,
            message=msg,
            description='move command producer',
        )
        return msg

    def publish_downlink_media_ctrl(
        self,
        *,
        action: str,
        protocol: str = '',
        channel_id: str = '',
        signal_json: str = '',
        signal: Optional[Dict[str, object]] = None,
    ) -> MediaCtrlCmd:
        msg = MediaCtrlCmd()
        msg.action = str(action)
        msg.protocol = str(protocol or '')
        msg.channel_id = str(channel_id or '')
        msg.signal_json = self._normalize_json_payload(signal_json=signal_json, payload=signal)
        msg.timestamp_ns = self.get_clock().now().nanoseconds
        self._publish_downlink_message(
            route_name='media_ctrl_topic',
            message_type=MediaCtrlCmd,
            message=msg,
            description='media control producer',
        )
        return msg

    def publish_downlink_task_ctrl(
        self,
        *,
        task_id: str = '',
        action: str,
        task_type: str = '',
        task_params_json: str = '',
        task_params: Optional[Dict[str, object]] = None,
    ) -> TaskCtrlCmd:
        msg = TaskCtrlCmd()
        msg.msg_version = '1.0'
        msg.task_id = str(task_id or '')
        msg.action = str(action)
        msg.task_type = str(task_type or '')
        msg.task_params_json = self._normalize_json_payload(
            signal_json=task_params_json,
            payload=task_params,
        )
        msg.timestamp_ns = self.get_clock().now().nanoseconds
        self._publish_downlink_message(
            route_name='task_ctrl_topic',
            message_type=TaskCtrlCmd,
            message=msg,
            description='task control producer',
        )
        return msg

    def publish_downlink_param_update(
        self,
        *,
        params_json: str = '',
        params: Optional[Dict[str, object]] = None,
    ) -> ParamUpdateCmd:
        msg = ParamUpdateCmd()
        msg.params_json = self._normalize_json_payload(signal_json=params_json, payload=params)
        msg.timestamp_ns = self.get_clock().now().nanoseconds
        self._publish_downlink_message(
            route_name='param_update_topic',
            message_type=ParamUpdateCmd,
            message=msg,
            description='parameter update producer',
        )
        return msg

    def register_downlink_hook(
        self,
        *,
        route_name: str,
        message_type: Type,
        callback: Callable,
        description: str = '',
    ):
        topic = self._route_catalog.topic(route_name)
        subscription = self.create_subscription(message_type, topic, callback, 10)
        self._downlink_subscriptions[route_name] = subscription
        self._record_downlink_subscription(
            route_name=route_name,
            message_type=f'{message_type.__module__}.{message_type.__name__}',
            description=description,
        )
        return subscription

    def subscribe_core_state_snapshot(self, callback: Callable[[StateSnapshot], None]):
        return self.register_downlink_hook(
            route_name='state_broadcast_topic',
            message_type=StateSnapshot,
            callback=callback,
            description='core state snapshot',
        )

    def subscribe_core_heartbeat(self, callback: Callable[[HeartbeatStatus], None]):
        return self.register_downlink_hook(
            route_name='heartbeat_topic',
            message_type=HeartbeatStatus,
            callback=callback,
            description='core heartbeat',
        )

    def subscribe_mode_switch(self, callback: Callable[[ModeSwitchCmd], None]):
        return self.register_downlink_hook(
            route_name='mode_switch_topic',
            message_type=ModeSwitchCmd,
            callback=callback,
            description='mode switch',
        )

    def subscribe_media_switch(self, callback: Callable[[MediaSwitchCmd], None]):
        return self.register_downlink_hook(
            route_name='media_switch_topic',
            message_type=MediaSwitchCmd,
            callback=callback,
            description='media switch',
        )

    def subscribe_uplink_protocol_switch(self, callback: Callable[[UplinkProtocolCmd], None]):
        return self.register_downlink_hook(
            route_name='uplink_protocol_topic',
            message_type=UplinkProtocolCmd,
            callback=callback,
            description='uplink protocol switch',
        )

    def subscribe_move_cmd(self, callback: Callable[[UnifiedMoveCmd], None]):
        return self.register_downlink_hook(
            route_name='move_cmd_topic',
            message_type=UnifiedMoveCmd,
            callback=callback,
            description='move command',
        )

    def subscribe_task_ctrl(self, callback: Callable[[TaskCtrlCmd], None]):
        return self.register_downlink_hook(
            route_name='task_ctrl_topic',
            message_type=TaskCtrlCmd,
            callback=callback,
            description='task control',
        )

    def subscribe_media_ctrl(self, callback: Callable[[MediaCtrlCmd], None]):
        return self.register_downlink_hook(
            route_name='media_ctrl_topic',
            message_type=MediaCtrlCmd,
            callback=callback,
            description='media control',
        )

    def subscribe_param_update(self, callback: Callable[[ParamUpdateCmd], None]):
        return self.register_downlink_hook(
            route_name='param_update_topic',
            message_type=ParamUpdateCmd,
            callback=callback,
            description='param update',
        )

    def subscribe_frpc_ctrl(self, callback: Callable[[FrpcCtrlCmd], None]):
        return self.register_downlink_hook(
            route_name='frpc_ctrl_topic',
            message_type=FrpcCtrlCmd,
            callback=callback,
            description='frpc control',
        )

    def on_core_ready(self) -> None:
        """Hook for subclasses after they finish declaring subscriptions/capabilities."""
        self.register_with_core()

    def _resolve_route_catalog(self, route_catalog: Optional[RouteCatalog]) -> RouteCatalog:
        if route_catalog is not None:
            return route_catalog

        shared_catalog = self._route_catalog_from_parameter()
        if shared_catalog is not None:
            return shared_catalog

        shared_catalog = self._route_catalog_from_core_node()
        if shared_catalog is not None:
            return shared_catalog

        self.get_logger().warning(
            'Route catalog parameter %s not found, falling back to local defaults',
            self.ROUTE_CATALOG_PARAMETER,
        )
        return RouteCatalog({'routes': self._default_routes()})

    def _route_catalog_from_parameter(self) -> Optional[RouteCatalog]:
        self.declare_parameter(
            self.ROUTE_CATALOG_PARAMETER,
            Parameter.Type.STRING,
        )
        raw_value = self.get_parameter(self.ROUTE_CATALOG_PARAMETER).value
        if not raw_value:
            return None
        if not isinstance(raw_value, str):
            raw_value = str(raw_value)
        try:
            snapshot = json.loads(raw_value)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(
                'Failed to parse route catalog parameter %s: %s',
                self.ROUTE_CATALOG_PARAMETER,
                exc,
            )
            return None
        try:
            return RouteCatalog.from_snapshot(snapshot)
        except Exception as exc:
            self.get_logger().warning('Invalid route catalog snapshot: %s', exc)
            return None

    def _route_catalog_from_core_node(self) -> Optional[RouteCatalog]:
        if not self._core_node_name or self._core_node_name == self.get_name():
            return None
        try:
            client = AsyncParameterClient(self, self._core_node_name)
            future = client.get_parameters([self.ROUTE_CATALOG_PARAMETER])
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if not future.done():
                return None
            result = future.result()
        except Exception as exc:
            self.get_logger().debug('Route catalog remote lookup failed: %s', exc)
            return None
        if not result or not getattr(result, 'values', None):
            return None
        raw_value = getattr(result.values[0], 'string_value', '')
        if not raw_value:
            return None
        try:
            return RouteCatalog.from_snapshot(json.loads(raw_value))
        except Exception as exc:
            self.get_logger().warning('Invalid remote route catalog snapshot: %s', exc)
            return None

    def _publisher_for_route(self, route_name: str, message_type: Type, description: str):
        publisher = self._downlink_publishers.get(route_name)
        if publisher is not None:
            return publisher
        topic = self._route_catalog.topic(route_name)
        publisher = self.create_publisher(message_type, topic, 10)
        self._downlink_publishers[route_name] = publisher
        self._record_downlink_publication(
            route_name=route_name,
            message_type=f'{message_type.__module__}.{message_type.__name__}',
            description=description,
        )
        return publisher

    def _publish_downlink_message(self, *, route_name: str, message_type: Type, message, description: str) -> None:
        publisher = self._publisher_for_route(route_name, message_type, description)
        publisher.publish(message)

    def _record_downlink_subscription(
        self,
        *,
        route_name: str,
        message_type: str,
        description: str,
    ) -> None:
        current = [
            item for item in self._registration.downlink_subscriptions
            if item.route_name != route_name
        ]
        current.append(
            DownlinkSubscriptionSpec(
                route_name=route_name,
                message_type=message_type,
                description=description,
            )
        )
        self._registration.downlink_subscriptions = current

    def _record_downlink_publication(
        self,
        *,
        route_name: str,
        message_type: str,
        description: str,
    ) -> None:
        current = [
            item for item in self._registration.downlink_publications
            if item.route_name != route_name
        ]
        current.append(
            DownlinkPublicationSpec(
                route_name=route_name,
                message_type=message_type,
                description=description,
            )
        )
        self._registration.downlink_publications = current

    def _record_state_field(self, field_name: str) -> None:
        if field_name and field_name not in self._registration.state_fields_written:
            self._registration.state_fields_written.append(field_name)

    @staticmethod
    def _normalize_extra_json(
        *,
        extra_json: str = '',
        extra: Optional[Dict[str, object]] = None,
    ) -> str:
        if extra_json:
            return str(extra_json)
        if extra is None:
            return ''
        return json.dumps(extra, ensure_ascii=False)

    @staticmethod
    def _normalize_json_payload(
        *,
        signal_json: str = '',
        payload: Optional[Dict[str, object]] = None,
    ) -> str:
        if signal_json:
            return str(signal_json)
        if payload is None:
            return ''
        return json.dumps(payload, ensure_ascii=False)

    @staticmethod
    def _default_routes() -> Dict[str, object]:
        return {
            'state_write_topic': '/uran/core/state/write',
            'state_broadcast_topic': '/uran/core/state/broadcast',
            'uplink_topic': '/uran/core/uplink/data',
            'heartbeat_topic': '/uran/core/heartbeat/status',
            'mode_switch_topic': '/uran/core/switch/mode',
            'media_switch_topic': '/uran/core/switch/media',
            'uplink_protocol_topic': '/uran/core/switch/uplink_protocol',
            'move_cmd_topic': '/uran/core/downlink/move_cmd',
            'task_ctrl_topic': '/uran/core/downlink/task_ctrl',
            'media_ctrl_topic': '/uran/core/downlink/media_ctrl',
            'frpc_ctrl_topic': '/uran/core/downlink/frpc_ctrl',
            'param_update_topic': '/uran/core/downlink/param_update',
            'services': {},
        }
