import os
from typing import Any, Dict

import yaml


def _safe_load_yaml(path: str) -> Dict[str, Any]:
    if not os.path.exists(path):
        return {}
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def load_base_config(share_dir: str) -> Dict[str, Any]:
    config_dir = share_dir
    if os.path.isfile(config_dir):
        config_dir = os.path.dirname(config_dir)
    if os.path.isdir(os.path.join(share_dir, 'config')):
        config_dir = os.path.join(share_dir, 'config')
    return {
        'core': _safe_load_yaml(os.path.join(config_dir, 'core.yaml')),
        'network': _safe_load_yaml(os.path.join(config_dir, 'network.yaml')),
        'routes': _safe_load_yaml(os.path.join(config_dir, 'routes.yaml')),
    }
