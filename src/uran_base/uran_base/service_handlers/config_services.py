def handle_trigger_report_service(node, req, res):
    node._do_state_report(trigger='manual')
    res.success = True
    res.message = f'triggered (reason={req.reason})'
    return res


def handle_configure_report_service(node, req, res):
    if req.interval_ms > 0:
        node._report_policy.interval_ms = int(req.interval_ms)
        node._last_report_ts = 0.0
    if req.protocol:
        node._report_policy.protocol = str(req.protocol)
    node._report_policy.report_on_change = bool(req.report_on_change)
    res.success = True
    res.message = 'ok'
    res.current_interval_ms = node._report_policy.interval_ms
    res.current_protocol = node._report_policy.protocol
    return res


def handle_reload_config_service(node, req, res):
    success, message = node.reload_runtime_config(config_path=str(req.config_path or ''))
    res.success = bool(success)
    res.message = str(message)
    return res
