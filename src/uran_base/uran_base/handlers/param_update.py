import json

from uran_msgs.msg import ParamUpdateCmd


def handle_param_update(context, payload: dict) -> None:
    msg = ParamUpdateCmd()
    msg.params_json = context.payload_json_str(payload, 'params_json', '{}')
    msg.timestamp_ns = context.payload_timestamp_ns(payload, 0)
    context.publishers['param_update'].publish(msg)
    try:
        parsed = json.loads(msg.params_json)
        if isinstance(parsed, dict):
            context.state_store.update_many(parsed)
    except Exception:
        pass
