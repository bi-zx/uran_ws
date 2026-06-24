from uran_msgs.msg import MediaSwitchCmd, ModeSwitchCmd, UplinkProtocolCmd


def handle_control_switch(context, payload: dict) -> None:
    switch = context.payload_dict(payload, 'switch')
    control_mode = context.payload_str(switch, 'control_mode')
    controller = context.payload_str(switch, 'controller')
    uplink_proto = context.payload_str(switch, 'primary_uplink_protocol')
    media = switch.get('media') or {}

    if control_mode:
        context.state_store.set('control_mode', control_mode)
    if controller:
        context.state_store.set('current_controller', controller)
    if uplink_proto:
        context.state_store.set('primary_uplink_protocol', uplink_proto)
        protocol_msg = UplinkProtocolCmd()
        protocol_msg.protocol = uplink_proto
        protocol_msg.timestamp_ns = context.now_ns()
        context.publishers['uplink_protocol_switch'].publish(protocol_msg)

    mode_msg = ModeSwitchCmd()
    mode_msg.control_mode = context.state_store.get('control_mode', '')
    mode_msg.controller = context.state_store.get('current_controller', '')
    mode_msg.timestamp_ns = context.now_ns()
    context.publishers['mode_switch'].publish(mode_msg)

    if isinstance(media, dict) and media.get('action'):
        media_msg = MediaSwitchCmd()
        media_msg.action = context.payload_str(media, 'action')
        media_msg.protocol = context.payload_str(media, 'protocol')
        media_msg.timestamp_ns = context.now_ns()
        context.publishers['media_switch'].publish(media_msg)

    context.do_state_report('switch')
