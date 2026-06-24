from uran_msgs.msg import FrpcCtrlCmd


def handle_frpc_ctrl(context, payload: dict) -> None:
    msg = FrpcCtrlCmd()
    msg.action = context.payload_str(payload, 'action', '')
    msg.frps_host = context.payload_str(payload, 'frps_host', '')
    msg.frps_port = context.payload_int(payload, 'frps_port', 0)
    msg.service_name = context.payload_str(payload, 'service_name', '')
    msg.local_port = context.payload_int(payload, 'local_port', 0)
    msg.remote_port = context.payload_int(payload, 'remote_port', 0)
    msg.auth_token = context.payload_str(payload, 'auth_token', '')
    msg.timestamp_ns = context.payload_timestamp_ns(payload, 0)
    context.publishers['frpc_ctrl'].publish(msg)
