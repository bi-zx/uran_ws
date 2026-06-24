from uran_msgs.msg import MediaCtrlCmd


def handle_media_ctrl(context, payload: dict) -> None:
    msg = MediaCtrlCmd()
    msg.action = context.payload_str(payload, 'action', '')
    msg.protocol = context.payload_str(payload, 'protocol', '')
    msg.channel_id = context.payload_str(payload, 'channel_id', '')
    msg.signal_json = context.payload_json_str(payload, 'signal_json', '{}')
    msg.timestamp_ns = context.payload_timestamp_ns(payload, 0)
    context.publishers['media_ctrl'].publish(msg)
