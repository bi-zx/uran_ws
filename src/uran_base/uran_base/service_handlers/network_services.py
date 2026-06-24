import json
import threading


def handle_connect_protocol_service(node, req, res):
    descriptor = node._protocol_registry.get(req.protocol)
    if descriptor is None:
        res.success = False
        res.message = f'unknown protocol: {req.protocol}'
        return res
    if req.action == 'connect':
        threading.Thread(target=node._connect_protocol, args=(req.protocol,), daemon=True).start()
        res.success = True
        res.message = 'connecting'
        return res
    if req.action == 'disconnect':
        descriptor.adapter.disconnect()
        node._refresh_protocol_table()
        protocol_table = node._state.get('protocol_table', {}) or {}
        node._state.set(
            'online_status',
            any(item.get('registered', False) for item in protocol_table.values()),
        )
        res.success = True
        res.message = 'disconnected'
        return res
    res.success = False
    res.message = f'unknown action: {req.action}'
    return res


def handle_network_status_service(node, req, res):
    node._refresh_protocol_table()
    res.protocol_table_json = json.dumps(node._state.get('protocol_table', {}), default=str, ensure_ascii=False)
    return res
