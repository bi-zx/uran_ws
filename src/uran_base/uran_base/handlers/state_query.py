def handle_state_query(context, payload: dict) -> None:
    field_names = context.payload_list(payload, 'field_names')
    fields = context.state_store.get_fields(field_names) if field_names else context.state_store.get_all()
    response = {
        'msg_type': 'state_query_response',
        'msg_version': '1.0',
        'device_id': context.state_store.get('device_id', ''),
        'timestamp_ms': int(context.now_ns() / 1_000_000),
        'fields': fields,
        'package_registry': context.package_registry.as_dict(),
        'route_registry': context.route_catalog.snapshot(),
        'node_registry': context.state_store.get('node_registry', {}),
    }
    context.uplink_gateway.publish_payload(response)
