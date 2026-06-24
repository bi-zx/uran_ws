import json


def handle_list_sensors_service(node, req, res):
    del req
    sensors = node.list_registered_sensors()
    res.sensors_json = json.dumps(sensors, default=str, ensure_ascii=False)
    return res
