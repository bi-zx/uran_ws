import json

from uran_core.uran_core_node import UranCoreNode


def _resolve(payload):
    task_params_json = UranCoreNode._payload_json_str(None, payload, 'task_params_json', '{}')
    return UranCoreNode._resolve_task_type(payload, task_params_json)


def test_resolve_task_type_uses_params_when_top_level_is_default_task():
    payload = {
        'task_type': 'default_task',
        'task_params_json': {
            'task_type': 'mission_planner_route',
        },
    }

    assert _resolve(payload) == 'mission_planner_route'


def test_resolve_task_type_uses_params_when_top_level_missing():
    payload = {
        'task_params_json': json.dumps({
            'task_type': 'straight_drive',
        }),
    }

    assert _resolve(payload) == 'straight_drive'


def test_resolve_task_type_keeps_explicit_top_level_value():
    payload = {
        'task_type': 'patrol',
        'task_params_json': {
            'task_type': 'mission_planner_route',
        },
    }

    assert _resolve(payload) == 'patrol'
