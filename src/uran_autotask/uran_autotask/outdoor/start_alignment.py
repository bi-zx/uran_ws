import math
from typing import Any, Dict, Iterable, Optional, Set

from .mission_contract import OutdoorMissionPlan


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {'1', 'true', 'yes', 'on'}:
        return True
    if text in {'0', 'false', 'no', 'off'}:
        return False
    return default


def _as_float(value: Any, default: float) -> float:
    if value in (None, ''):
        return float(default)
    return float(value)


def _as_kind_set(value: Any) -> Set[str]:
    if isinstance(value, str):
        items: Iterable[Any] = value.split(',')
    elif isinstance(value, (list, tuple, set)):
        items = value
    else:
        items = ('start',)
    return {str(item).strip().lower() for item in items if str(item).strip()}


class OutdoorStartAligner:
    """Align or reject an outdoor route before the first goal is dispatched."""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        config = dict(config or {})
        self._enabled = _as_bool(config.get('enabled'), True)
        self._aligned_tolerance_m = max(
            0.0,
            _as_float(config.get('aligned_tolerance_m'), 1.0),
        )
        self._skip_start_tolerance_m = max(
            self._aligned_tolerance_m,
            _as_float(config.get('skip_start_tolerance_m'), 3.0),
        )
        allow_start_correction = config.get('allow_start_correction')
        if allow_start_correction is None:
            allow_start_correction = config.get('allow_route_shift')
        max_start_correction_m = config.get('max_start_correction_m')
        if max_start_correction_m is None:
            max_start_correction_m = config.get('max_route_shift_m')
        self._allow_start_correction = _as_bool(allow_start_correction, True)
        self._max_start_correction_m = max(
            0.0,
            _as_float(max_start_correction_m, 10.0),
        )
        self._max_pose_age_s = max(
            0.0,
            _as_float(config.get('max_pose_age_s'), 3.0),
        )
        self._skip_explicit_start = _as_bool(config.get('skip_explicit_start'), True)
        self._start_kinds = _as_kind_set(config.get('start_kinds'))

    def config_snapshot(self) -> Dict[str, Any]:
        return {
            'enabled': self._enabled,
            'aligned_tolerance_m': self._aligned_tolerance_m,
            'skip_start_tolerance_m': self._skip_start_tolerance_m,
            'allow_start_correction': self._allow_start_correction,
            'max_start_correction_m': self._max_start_correction_m,
            'max_pose_age_s': self._max_pose_age_s,
            'skip_explicit_start': self._skip_explicit_start,
            'start_kinds': sorted(self._start_kinds),
        }

    def evaluate_and_apply(
        self,
        *,
        mission: Optional[OutdoorMissionPlan],
        current_pose: Optional[Dict[str, Any]],
        now_ns: int = 0,
    ) -> Dict[str, Any]:
        if not self._enabled:
            return self._result(status='disabled', action='none', message='start alignment is disabled')

        if mission is None or not mission.execution_points:
            return self._result(
                status='passed',
                action='none',
                message='outdoor mission has no execution point to align',
            )

        if not current_pose or 'x' not in current_pose or 'y' not in current_pose:
            return self._result(
                status='unavailable',
                action='wait',
                message='current map pose is unavailable',
            )
        pose_timestamp_ns = int(current_pose.get('timestamp_ns', 0))
        pose_age_s = None
        if now_ns and pose_timestamp_ns:
            pose_age_s = max(0.0, float(int(now_ns) - pose_timestamp_ns) / 1_000_000_000.0)
            if self._max_pose_age_s > 0.0 and pose_age_s > self._max_pose_age_s:
                return self._result(
                    status='stale_pose',
                    action='wait',
                    message=(
                        f'current map pose is stale: age {pose_age_s:.2f}s exceeds '
                        f'{self._max_pose_age_s:.2f}s'
                    ),
                    pose_age_s=pose_age_s,
                    pose_timestamp_ns=pose_timestamp_ns,
                )

        start_point = mission.execution_points[0]
        current_x = float(current_pose['x'])
        current_y = float(current_pose['y'])
        dx = current_x - float(start_point.x)
        dy = current_y - float(start_point.y)
        distance_m = math.hypot(dx, dy)
        is_explicit_start = str(start_point.kind or '').strip().lower() in self._start_kinds
        base = {
            'distance_m': distance_m,
            'dx': dx,
            'dy': dy,
            'current_pose': {
                'x': current_x,
                'y': current_y,
                'z': float(current_pose.get('z', 0.0)),
                'frame_id': str(current_pose.get('frame_id', '')),
                'timestamp_ns': pose_timestamp_ns,
                'age_s': pose_age_s,
            },
            'planned_start': start_point.to_dict(),
            'explicit_start': is_explicit_start,
        }

        if distance_m <= self._aligned_tolerance_m:
            if is_explicit_start and self._skip_explicit_start:
                return self._result(
                    status='start_skipped',
                    action='skip_start',
                    next_current_waypoint_index=0,
                    message='current pose is already at the planned start point',
                    **base,
                )
            return self._result(
                status='passed',
                action='none',
                message='current pose is already aligned with the planned start point',
                **base,
            )

        if (
            is_explicit_start and
            self._skip_explicit_start and
            distance_m <= self._skip_start_tolerance_m
        ):
            return self._result(
                status='start_skipped',
                action='skip_start',
                next_current_waypoint_index=0,
                message='current pose is close enough; explicit start point is skipped',
                **base,
            )

        if self._allow_start_correction and distance_m <= self._max_start_correction_m:
            action = (
                'apply_start_correction_and_skip_start'
                if is_explicit_start and self._skip_explicit_start else
                'apply_start_correction'
            )
            next_index = 0 if action == 'apply_start_correction_and_skip_start' else -1
            return self._result(
                status='start_correction_applied',
                action=action,
                next_current_waypoint_index=next_index,
                goal_correction={
                    'enabled': True,
                    'applied': True,
                    'dx': dx,
                    'dy': dy,
                    'reason': 'startup map pose correction',
                },
                message='startup correction will be applied while dispatching map goals',
                **base,
            )

        return self._result(
            status='failed',
            action='reject',
            message=(
                f'current pose is {distance_m:.2f}m from planned start, '
                f'exceeding allowed startup correction {self._max_start_correction_m:.2f}m'
            ),
            **base,
        )

    def _result(self, *, status: str, action: str, message: str, **extra) -> Dict[str, Any]:
        result = {
            'enabled': self._enabled,
            'status': status,
            'action': action,
            'message': message,
        }
        result.update(extra)
        return result
