"""T1.2 状态空间管理 — 内存 + SQLite 持久化"""
import json
import sqlite3
import threading
from typing import Any, Dict, List, Optional

# §4.2.3 持久化字段集合
PERSISTENT_FIELDS = {
    'device_id', 'template_id', 'device_type',
    'linear_vel_limit', 'angular_vel_limit',
}

# §4.2.3 标准字段默认值
DEFAULT_STATE: Dict[str, Any] = {
    'device_id': 'device_001',
    'template_id': 'template_001',
    'device_type': 'unknown',
    'online_status': False,
    'current_controller': 'cloud',
    'control_mode': 'manual',
    'primary_uplink_protocol': 'mqtt',
    'protocol_table': {},
    'battery_level': 0.0,
    'position': {'lat': 0.0, 'lon': 0.0, 'alt': 0.0},
    'velocity': {'vx': 0.0, 'vy': 0.0, 'vz': 0.0},
    'attitude': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    'linear_vel_limit': 1.0,
    'angular_vel_limit': 1.0,
    'error_code': 0,
    'task_id': '',
    'task_stage': '',
    'uptime_seconds': 0,
    'media_active_protocol': 'none',
    'media_channel_count': 0,
    'media_camera_list': [],
}


class StateManager:
    """线程安全的状态空间，持久化字段写入 SQLite。"""

    def __init__(self, db_path: str = '/tmp/uran_core_state.db'):
        self._lock = threading.Lock()
        self._state: Dict[str, Any] = dict(DEFAULT_STATE)
        self._db_path = db_path
        self._change_callbacks: List[Any] = []  # (fields_to_watch, callback)
        self._init_db()
        self._load_persistent()

    # ------------------------------------------------------------------ DB
    def _init_db(self):
        with sqlite3.connect(self._db_path) as conn:
            conn.execute(
                'CREATE TABLE IF NOT EXISTS persistent_state '
                '(field_name TEXT PRIMARY KEY, value_json TEXT)'
            )

    def _load_persistent(self):
        with sqlite3.connect(self._db_path) as conn:
            rows = conn.execute(
                'SELECT field_name, value_json FROM persistent_state'
            ).fetchall()
        with self._lock:
            for field_name, value_json in rows:
                try:
                    self._state[field_name] = json.loads(value_json)
                except (json.JSONDecodeError, TypeError):
                    pass

    def _save_persistent(self, field_name: str, value: Any):
        with sqlite3.connect(self._db_path) as conn:
            conn.execute(
                'INSERT OR REPLACE INTO persistent_state (field_name, value_json) VALUES (?, ?)',
                (field_name, json.dumps(value)),
            )

    # ------------------------------------------------------------------ API
    def get(self, field_name: str) -> Optional[Any]:
        with self._lock:
            return self._state.get(field_name)

    def set(self, field_name: str, value: Any, persistent: bool = False) -> bool:
        old_value = None
        with self._lock:
            old_value = self._state.get(field_name)
            self._state[field_name] = value
        if persistent or field_name in PERSISTENT_FIELDS:
            self._save_persistent(field_name, value)
        # Fire change callbacks
        if old_value != value:
            self._fire_change(field_name, value)
        return True

    def get_fields(self, field_names: List[str]) -> Dict[str, Any]:
        with self._lock:
            return {k: self._state.get(k) for k in field_names}

    def get_all(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._state)

    def get_snapshot_json(self) -> str:
        return json.dumps(self.get_all(), default=str)

    # ------------------------------------------------------------------ Change watch
    def register_change_callback(self, watch_fields: List[str], callback):
        """当 watch_fields 中任意字段变更时调用 callback(field_name, new_value)。"""
        self._change_callbacks.append((set(watch_fields), callback))

    def _fire_change(self, field_name: str, new_value: Any):
        for watch_set, cb in self._change_callbacks:
            if not watch_set or field_name in watch_set:
                try:
                    cb(field_name, new_value)
                except Exception:
                    pass
