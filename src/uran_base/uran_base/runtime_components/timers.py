import time

from uran_msgs.msg import HeartbeatStatus, ModeSwitchCmd, StateSnapshot


def run_broadcast_cycle(node) -> None:
    node._expire_motion_control_lock_if_needed()
    msg = StateSnapshot()
    msg.msg_version = '1.0'
    msg.timestamp_ns = node._now_ns()
    msg.device_id = node._state.get('device_id', '')
    msg.fields_json = node._state.get_snapshot_json()
    node._pub_broadcast.publish(msg)


def run_heartbeat_cycle(node) -> None:
    node._refresh_protocol_table()
    active_protocol = node._select_heartbeat_protocol()
    connected = active_protocol.adapter.is_connected() if active_protocol else False
    registered = active_protocol.adapter.is_registered() if active_protocol else False
    heartbeat_protocol = active_protocol.protocol_id if active_protocol else node._default_protocol
    node._state.set('online_status', bool(registered))

    payload = {
        'msg_type': 'heartbeat',
        'msg_version': '1.0',
        'device_id': node._state.get('device_id', ''),
        'timestamp_ms': int(time.time() * 1000),
        'online': bool(registered),
        'uptime_seconds': int(node._state.get('uptime_seconds', 0) or 0),
        'battery_level': node._state.get('battery_level', 0.0) or 0.0,
        'control_mode': node._state.get('control_mode', 'manual') or 'manual',
        'current_controller': node._state.get('current_controller', 'cloud') or 'cloud',
        'primary_uplink_protocol': node._state.get('primary_uplink_protocol', node._default_protocol) or node._default_protocol,
        'protocol_table': {
            key: value.get('available', False)
            for key, value in (node._state.get('protocol_table', {}) or {}).items()
        },
        'position': node._state.get('position', {}) or {},
        'error_code': node._state.get('error_code', 0) or 0,
    }
    node._uplink_gateway.publish_payload(payload, preferred_protocol=heartbeat_protocol)

    hb = HeartbeatStatus()
    hb.timestamp_ns = node._now_ns()
    hb.protocol = heartbeat_protocol
    hb.last_sent_ts = node._now_ns()
    hb.success = connected
    node._pub_heartbeat.publish(hb)


def run_report_cycle(node) -> None:
    now = time.time()
    if now - node._last_report_ts >= node._report_policy.interval_ms / 1000.0:
        node._last_report_ts = now
        node._do_state_report(trigger='periodic')


def run_uptime_cycle(node) -> None:
    node._state.set('uptime_seconds', int(time.time() - node._start_ts))


def build_mode_switch_message(node) -> ModeSwitchCmd:
    msg = ModeSwitchCmd()
    msg.control_mode = node._state.get('control_mode', '') or ''
    msg.controller = node._state.get('current_controller', '') or ''
    msg.timestamp_ns = node._now_ns()
    return msg
