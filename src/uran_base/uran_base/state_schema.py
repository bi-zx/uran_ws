from typing import Dict

from .models import StateFieldSpec


ALLOWED_EXTENSION_PREFIXES = (
    'ext.',
    'pkg.',
)

CORE_MANAGED_PREFIXES = (
    'node_registry.',
)


STATE_SCHEMA: Dict[str, StateFieldSpec] = {
    'device_id': StateFieldSpec('device_id', 'device_001', persistent=True, tags=['identity']),
    'template_id': StateFieldSpec('template_id', 'template_001', persistent=True, tags=['identity']),
    'device_type': StateFieldSpec('device_type', 'unknown', persistent=True, tags=['identity']),
    'online_status': StateFieldSpec('online_status', False, tags=['network']),
    'current_controller': StateFieldSpec('current_controller', 'cloud', tags=['control']),
    'control_mode': StateFieldSpec('control_mode', 'manual', tags=['control']),
    'primary_uplink_protocol': StateFieldSpec('primary_uplink_protocol', 'mqtt', tags=['network']),
    'protocol_table': StateFieldSpec('protocol_table', {}, tags=['network']),
    'package_registry': StateFieldSpec('package_registry', {}, tags=['registry']),
    'node_registry': StateFieldSpec('node_registry', {}, tags=['registry']),
    'route_registry': StateFieldSpec('route_registry', {}, tags=['registry']),
    'battery_level': StateFieldSpec('battery_level', 0.0, tags=['telemetry']),
    'position': StateFieldSpec('position', {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}, tags=['telemetry']),
    'velocity': StateFieldSpec('velocity', {'vx': 0.0, 'vy': 0.0, 'vz': 0.0}, tags=['telemetry']),
    'attitude': StateFieldSpec('attitude', {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}, tags=['telemetry']),
    'linear_vel_limit': StateFieldSpec('linear_vel_limit', 1.0, persistent=True, tags=['control']),
    'angular_vel_limit': StateFieldSpec('angular_vel_limit', 1.0, persistent=True, tags=['control']),
    'error_code': StateFieldSpec('error_code', 0, tags=['health']),
    'task_id': StateFieldSpec('task_id', '', tags=['task']),
    'task_stage': StateFieldSpec('task_stage', '', tags=['task']),
    'failsafe_active': StateFieldSpec('failsafe_active', False, tags=['health']),
    'failsafe_action_executed': StateFieldSpec('failsafe_action_executed', '', tags=['health']),
    'motion_control_blocked_by': StateFieldSpec('motion_control_blocked_by', '', tags=['control']),
    'active_move_plugin': StateFieldSpec('active_move_plugin', '', tags=['control']),
    'cyberdog2_switch_status': StateFieldSpec('cyberdog2_switch_status', '', tags=['device']),
    'motion_control_lock': StateFieldSpec(
        'motion_control_lock',
        {
            'active': False,
            'owner': '',
            'task_id': '',
            'reason': '',
            'timestamp_ns': 0,
            'expires_at_ns': 0,
        },
        tags=['control'],
    ),
    'uptime_seconds': StateFieldSpec('uptime_seconds', 0, tags=['health']),
    'media_camera_list': StateFieldSpec('media_camera_list', [], tags=['media']),
    'media_active_protocol': StateFieldSpec('media_active_protocol', 'none', tags=['media']),
    'media_channel_count': StateFieldSpec('media_channel_count', 0, tags=['media']),
}


def default_state() -> Dict[str, object]:
    return {name: spec.default for name, spec in STATE_SCHEMA.items()}


def persistent_fields() -> set:
    return {name for name, spec in STATE_SCHEMA.items() if spec.persistent}


def known_fields() -> set:
    return set(STATE_SCHEMA.keys())


def describe_schema() -> Dict[str, object]:
    return {
        'core_fields': sorted(STATE_SCHEMA.keys()),
        'extension_namespaces': list(ALLOWED_EXTENSION_PREFIXES),
        'core_managed_namespaces': list(CORE_MANAGED_PREFIXES),
        'persistent_fields': sorted(persistent_fields()),
    }
