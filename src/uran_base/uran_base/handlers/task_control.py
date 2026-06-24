from uran_msgs.msg import TaskCtrlCmd


def handle_task_ctrl(context, payload: dict) -> None:
    msg = TaskCtrlCmd()
    msg.msg_version = context.payload_str(payload, 'msg_version', '1.0')
    msg.task_id = context.payload_str(payload, 'task_id', '')
    msg.action = context.payload_str(payload, 'action', '')
    msg.task_params_json = context.payload_json_str(payload, 'task_params_json', '{}')
    msg.task_type = context.resolve_task_type(payload, msg.task_params_json)
    msg.timestamp_ns = context.payload_timestamp_ns(payload, 0)
    context.publishers['task_ctrl'].publish(msg)
