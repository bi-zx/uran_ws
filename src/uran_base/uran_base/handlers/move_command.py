from uran_msgs.msg import UnifiedMoveCmd


def handle_move_cmd(context, payload: dict) -> None:
    msg = UnifiedMoveCmd()
    msg.msg_version = context.payload_str(payload, 'msg_version', '1.0')
    msg.device_id = context.payload_str(payload, 'device_id', '')
    msg.timestamp_ns = context.payload_timestamp_ns(payload, 0)
    msg.controller = context.payload_str(payload, 'controller', '')
    msg.linear_vel_x = context.payload_float(payload, 'linear_vel_x', 0.0)
    msg.linear_vel_y = context.payload_float(payload, 'linear_vel_y', 0.0)
    msg.linear_vel_z = context.payload_float(payload, 'linear_vel_z', 0.0)
    msg.angular_vel_z = context.payload_float(payload, 'angular_vel_z', 0.0)
    msg.target_roll = context.payload_float(payload, 'target_roll', 0.0)
    msg.target_pitch = context.payload_float(payload, 'target_pitch', 0.0)
    msg.target_yaw = context.payload_float(payload, 'target_yaw', 0.0)
    msg.action = context.payload_str(payload, 'action', '')
    msg.extra_json = context.payload_json_str(payload, 'extra_json', '{}')
    context.publishers['move_cmd'].publish(msg)
