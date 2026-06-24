from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional


@dataclass(frozen=True)
class RouteDefinition:
    name: str
    ros_name: str
    kind: str
    description: str = ''


@dataclass
class PackageDescriptor:
    package_id: str
    category: str
    version: str = '1.0.0'
    enabled: bool = True
    provides: List[str] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ProtocolDescriptor:
    protocol_id: str
    adapter: Any
    enabled: bool = True
    priority: int = 100
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class CoreRouteBindings:
    state_write_topic: str
    state_broadcast_topic: str
    uplink_topic: str
    heartbeat_topic: str
    mode_switch_topic: str
    media_switch_topic: str
    uplink_protocol_topic: str
    move_cmd_topic: str
    task_ctrl_topic: str
    media_ctrl_topic: str
    frpc_ctrl_topic: str
    param_update_topic: str


@dataclass
class DownlinkSubscriptionSpec:
    route_name: str
    message_type: str
    description: str = ''


@dataclass
class DownlinkPublicationSpec:
    route_name: str
    message_type: str
    description: str = ''


@dataclass
class NodeRegistration:
    package_name: str
    node_name: str
    capabilities: List[str] = field(default_factory=list)
    downlink_subscriptions: List[DownlinkSubscriptionSpec] = field(default_factory=list)
    downlink_publications: List[DownlinkPublicationSpec] = field(default_factory=list)
    uplink_data_types: List[str] = field(default_factory=list)
    state_fields_written: List[str] = field(default_factory=list)
    service_exports: List[str] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class DownlinkHandler:
    msg_type: str
    callback: Callable[[Dict[str, Any]], None]
    description: str = ''


@dataclass
class DownlinkContext:
    state_store: Any
    uplink_gateway: Any
    package_registry: Any
    route_catalog: Any
    publishers: Dict[str, Any]
    logger: Any
    now_ns: Callable[[], int]
    payload_str: Callable[[Dict[str, Any], str, str], str]
    payload_int: Callable[[Dict[str, Any], str, int], int]
    payload_float: Callable[[Dict[str, Any], str, float], float]
    payload_dict: Callable[[Dict[str, Any], str], Dict[str, Any]]
    payload_list: Callable[[Dict[str, Any], str], list]
    payload_json_str: Callable[[Dict[str, Any], str, str], str]
    payload_timestamp_ns: Callable[[Dict[str, Any], int], int]
    resolve_task_type: Callable[[Dict[str, Any], str], str]
    do_state_report: Callable[[str], None]


@dataclass
class StateFieldSpec:
    name: str
    default: Any
    persistent: bool = False
    description: str = ''
    tags: List[str] = field(default_factory=list)


@dataclass
class PublishResult:
    protocol: str
    success: bool
    detail: str = ''


@dataclass
class StateReportPolicy:
    interval_ms: int = 10000
    protocol: str = 'mqtt'
    report_on_change: bool = True
    watched_fields: List[str] = field(default_factory=list)


@dataclass
class DownlinkEnvelope:
    msg_type: str
    payload: Dict[str, Any]
    source_protocol: str = ''
    source_topic: str = ''
    correlation_id: str = ''
    received_ts_ms: int = 0


@dataclass
class ProtocolHealth:
    available: bool = False
    latency_ms: int = -1
    last_check_ts: int = 0
    registered: bool = False
    details: Dict[str, Any] = field(default_factory=dict)


def descriptor_from_dict(payload: Dict[str, Any]) -> PackageDescriptor:
    return PackageDescriptor(
        package_id=str(payload.get('package_id', '')),
        category=str(payload.get('category', 'misc')),
        version=str(payload.get('version', '1.0.0')),
        enabled=bool(payload.get('enabled', True)),
        provides=[str(item) for item in payload.get('provides', [])],
        metadata=dict(payload.get('metadata', {})),
    )
