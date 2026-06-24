import json


def handle_state_get_service(node, req, res):
    res.fields_json = json.dumps(node._state.get_fields(list(req.field_names)), default=str, ensure_ascii=False)
    res.success = True
    return res


def handle_state_set_service(node, req, res):
    try:
        success = node._state.set(req.field_name, json.loads(req.value_json), persistent=req.persistent)
        res.success = bool(success)
        res.message = 'ok' if success else f'rejected: {req.field_name}'
    except Exception as exc:
        res.success = False
        res.message = str(exc)
    return res
