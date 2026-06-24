import copy
import json
import sqlite3
import threading
from typing import Any, Callable, Dict, Iterable, List, Optional

from .state_schema import (
    ALLOWED_EXTENSION_PREFIXES,
    CORE_MANAGED_PREFIXES,
    default_state,
    describe_schema as describe_state_schema,
    known_fields,
    persistent_fields,
)


class StateStore:

    def __init__(
        self,
        db_path: str,
        *,
        allow_extension_namespaces: bool = True,
        reject_unknown_fields: bool = True,
        warning_callback: Optional[Callable[[str], None]] = None,
    ):
        self._db_path = db_path
        self._lock = threading.Lock()
        self._state: Dict[str, Any] = default_state()
        self._persistent_fields = persistent_fields()
        self._known_fields = known_fields()
        self._allow_extension_namespaces = bool(allow_extension_namespaces)
        self._reject_unknown_fields = bool(reject_unknown_fields)
        self._warning_callback = warning_callback
        self._change_callbacks: List[tuple] = []
        self._init_db()
        self._load_persistent()

    def _init_db(self) -> None:
        with sqlite3.connect(self._db_path) as conn:
            conn.execute(
                'CREATE TABLE IF NOT EXISTS persistent_state '
                '(field_name TEXT PRIMARY KEY, value_json TEXT)'
            )

    def _load_persistent(self) -> None:
        with sqlite3.connect(self._db_path) as conn:
            rows = conn.execute(
                'SELECT field_name, value_json FROM persistent_state'
            ).fetchall()
        with self._lock:
            for field_name, value_json in rows:
                try:
                    self._state[field_name] = json.loads(value_json)
                except (TypeError, ValueError):
                    continue

    def _save_persistent(self, field_name: str, value: Any) -> None:
        with sqlite3.connect(self._db_path) as conn:
            conn.execute(
                'INSERT OR REPLACE INTO persistent_state (field_name, value_json) VALUES (?, ?)',
                (field_name, json.dumps(value, ensure_ascii=False)),
            )

    def get(self, field_name: str, default: Optional[Any] = None) -> Any:
        with self._lock:
            value = self._state.get(field_name, default)
        return copy.deepcopy(value)

    def set(self, field_name: str, value: Any, persistent: bool = False) -> bool:
        allowed, reason = self._validate_field_name(field_name)
        if not allowed:
            self._warn(reason)
            return False
        with self._lock:
            old_value = self._state.get(field_name)
            self._state[field_name] = copy.deepcopy(value)
        if persistent or field_name in self._persistent_fields:
            self._save_persistent(field_name, value)
        if old_value != value:
            self._fire_change(field_name, value)
        return True

    def update_many(self, values: Dict[str, Any], persistent: bool = False) -> Dict[str, bool]:
        results: Dict[str, bool] = {}
        for key, value in values.items():
            results[key] = self.set(key, value, persistent=persistent)
        return results

    def get_fields(self, field_names: Iterable[str]) -> Dict[str, Any]:
        with self._lock:
            data = {name: self._state.get(name) for name in field_names}
        return copy.deepcopy(data)

    def get_all(self) -> Dict[str, Any]:
        with self._lock:
            data = dict(self._state)
        return copy.deepcopy(data)

    def get_snapshot_json(self) -> str:
        return json.dumps(self.get_all(), default=str, ensure_ascii=False)

    def register_change_callback(
        self,
        watch_fields: Iterable[str],
        callback: Callable[[str, Any], None],
    ) -> None:
        self._change_callbacks.append((set(watch_fields), callback))

    def clear_change_callbacks(self) -> None:
        self._change_callbacks = []

    def describe_schema(self) -> Dict[str, Any]:
        return {
            **describe_state_schema(),
            'allow_extension_namespaces': self._allow_extension_namespaces,
            'reject_unknown_fields': self._reject_unknown_fields,
        }

    def _validate_field_name(self, field_name: str) -> tuple:
        if field_name in self._known_fields:
            return True, ''
        if any(field_name.startswith(prefix) for prefix in CORE_MANAGED_PREFIXES):
            return True, ''
        if self._allow_extension_namespaces and self._is_extension_field(field_name):
            return True, ''
        if self._reject_unknown_fields:
            return False, f'reject unknown state field: {field_name}'
        return True, ''

    def _is_extension_field(self, field_name: str) -> bool:
        if not field_name:
            return False
        if field_name.startswith('pkg.'):
            parts = field_name.split('.')
            return len(parts) >= 3 and bool(parts[1])
        return any(field_name.startswith(prefix) for prefix in ALLOWED_EXTENSION_PREFIXES)

    def _warn(self, message: str) -> None:
        if not message:
            return
        if self._warning_callback is None:
            return
        try:
            self._warning_callback(message)
        except Exception:
            pass

    def _fire_change(self, field_name: str, new_value: Any) -> None:
        for watch_set, callback in self._change_callbacks:
            if watch_set and field_name not in watch_set:
                continue
            try:
                callback(field_name, copy.deepcopy(new_value))
            except Exception:
                continue
