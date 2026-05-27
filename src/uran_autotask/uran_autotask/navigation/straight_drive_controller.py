import math
import time
from typing import Any, Dict, Optional

from .obstacle_avoidance import GapAvoidancePlanner


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class StraightDriveController:
    """Runs one straight-drive command with reactive 2D LiDAR obstacle avoidance."""

    def __init__(
        self,
        *,
        config: Optional[Dict[str, Any]] = None,
        move_gateway,
        pose_getter,
        map_pose_getter=None,
        now_ns_getter,
        logger,
    ):
        cfg = dict(config or {})
        self.enabled = bool(cfg.get('enabled', True))
        self.control_hz = max(5.0, float(cfg.get('control_hz', 20.0)))
        self.default_speed_mps = float(cfg.get('default_speed_mps', 0.18))
        self.max_speed_mps = float(cfg.get('max_speed_mps', 0.25))
        self.min_avoid_speed_mps = float(cfg.get('min_avoid_speed_mps', 0.06))
        self.max_angular_speed_radps = float(cfg.get('max_angular_speed_radps', 0.7))
        self.heading_kp = float(cfg.get('heading_kp', 1.4))
        self.slowdown_heading_rad = math.radians(float(cfg.get('slowdown_heading_deg', 55.0)))
        self.turn_in_place_heading_rad = math.radians(
            float(cfg.get('turn_in_place_heading_deg', 85.0))
        )
        self.scan_timeout_s = float(cfg.get('scan_timeout_s', 0.35))
        self.blocked_timeout_s = float(cfg.get('blocked_timeout_s', 4.0))
        self.recover_heading_tolerance_rad = math.radians(
            float(cfg.get('recover_heading_tolerance_deg', 10.0))
        )
        self.distance_tolerance_m = float(cfg.get('distance_tolerance_m', 0.15))
        self.target_tolerance_m = float(cfg.get('target_tolerance_m', 0.8))
        self.target_tolerance_relax_after_s = float(
            cfg.get('target_tolerance_relax_after_s', 30.0)
        )
        self.target_tolerance_relaxed_m = float(cfg.get('target_tolerance_relaxed_m', 5.0))
        self.target_tolerance_final_m = float(cfg.get('target_tolerance_final_m', 10.0))
        self.command_source = str(cfg.get('command_source', 'straight_drive'))
        self._planner = GapAvoidancePlanner(cfg)
        self._move_gateway = move_gateway
        self._pose_getter = pose_getter
        self._map_pose_getter = map_pose_getter
        self._now_ns_getter = now_ns_getter
        self._logger = logger

        self._latest_scan = None
        self._latest_scan_monotonic_s = 0.0
        self._active = False
        self._task_id = ''
        self._started_monotonic_s = 0.0
        self._last_command_monotonic_s = 0.0
        self._blocked_since_s: Optional[float] = None
        self._yaw_ref_rad: Optional[float] = None
        self._start_pose: Optional[Dict[str, Any]] = None
        self._target_distance_m: Optional[float] = None
        self._target_duration_s: Optional[float] = None
        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None
        self._target_tolerance_base_m: float = self.target_tolerance_m
        self._target_tolerance_max_m: float = self.target_tolerance_m
        self._pose_source = 'local'
        self._active_command_source = self.command_source
        self._speed_mps = self.default_speed_mps
        self._last_vx = 0.0
        self._last_wz = 0.0
        self._last_decision: Dict[str, Any] = {
            'active': False,
            'state': 'idle',
            'reason': 'not started',
        }

    def update_scan(self, scan):
        self._latest_scan = scan
        self._latest_scan_monotonic_s = time.monotonic()

    def start(
        self,
        *,
        task_id: str,
        distance_m: Optional[float] = None,
        duration_s: Optional[float] = None,
        speed_mps: Optional[float] = None,
        target_x: Optional[float] = None,
        target_y: Optional[float] = None,
        target_tolerance_m: Optional[float] = None,
        command_source: Optional[str] = None,
    ) -> Dict[str, Any]:
        if not self.enabled:
            return {
                'accepted': False,
                'code': 'E_STRAIGHT_DRIVE_DISABLED',
                'message': 'straight drive controller is disabled',
            }
        if self._move_gateway is None:
            return {
                'accepted': False,
                'code': 'E_MOVE_GATEWAY_UNAVAILABLE',
                'message': 'uran_move gateway is unavailable',
            }
        distance = None if distance_m is None else float(distance_m)
        duration = None if duration_s is None else float(duration_s)
        has_target = target_x is not None and target_y is not None
        target_x_value = None if target_x is None else float(target_x)
        target_y_value = None if target_y is None else float(target_y)
        if (
            (distance is None or distance <= 0.0) and
            (duration is None or duration <= 0.0) and
            not has_target
        ):
            return {
                'accepted': False,
                'code': 'E_STRAIGHT_DRIVE_TARGET_EMPTY',
                'message': 'distance_m, duration_s, or target_x/target_y must be provided',
            }

        now = time.monotonic()
        pose_source = 'map' if has_target else 'local'
        pose = self._current_pose(source=pose_source)
        if (
            (distance is not None and distance > 0.0) or has_target
        ) and not self._pose_has_xy(pose):
            required = 'map pose' if has_target else 'local odometry pose'
            return {
                'accepted': False,
                'code': 'E_STRAIGHT_DRIVE_POSE_UNAVAILABLE',
                'message': f'{required} is required for this straight drive command',
            }
        yaw = self._pose_yaw_rad(pose)
        self._active = True
        self._task_id = str(task_id or '')
        self._started_monotonic_s = now
        self._last_command_monotonic_s = 0.0
        self._blocked_since_s = None
        self._yaw_ref_rad = yaw
        self._start_pose = pose
        self._target_distance_m = distance if distance is not None and distance > 0.0 else None
        self._target_duration_s = duration if duration is not None and duration > 0.0 else None
        self._target_x = target_x_value if has_target else None
        self._target_y = target_y_value if has_target else None
        self._target_tolerance_base_m = self.target_tolerance_m
        requested_target_tolerance_m = (
            self.target_tolerance_m
            if target_tolerance_m is None
            else max(0.05, float(target_tolerance_m))
        )
        self._target_tolerance_max_m = max(
            self._target_tolerance_base_m,
            requested_target_tolerance_m,
        )
        self._pose_source = pose_source
        self._active_command_source = str(command_source or self.command_source)
        requested_speed = self.default_speed_mps if speed_mps is None else float(speed_mps)
        self._speed_mps = _clamp(abs(requested_speed), 0.02, self.max_speed_mps)
        self._last_vx = 0.0
        self._last_wz = 0.0
        self._last_decision = {
            'active': True,
            'state': 'starting',
            'reason': 'straight drive accepted',
            'task_id': self._task_id,
            'target_distance_m': self._target_distance_m,
            'target_duration_s': self._target_duration_s,
            'target_x': self._target_x,
            'target_y': self._target_y,
            'target_tolerance_m': self._effective_target_tolerance_m(now) if has_target else None,
            'target_tolerance_base_m': self._target_tolerance_base_m if has_target else None,
            'target_tolerance_max_m': self._target_tolerance_max_m if has_target else None,
            'pose_source': self._pose_source,
            'command_source': self._active_command_source,
            'speed_mps': self._speed_mps,
            'yaw_ref_deg': math.degrees(yaw) if yaw is not None else None,
        }
        return {'accepted': True}

    def stop(self, *, reason: str = ''):
        if self._active and self._move_gateway is not None:
            self._publish_velocity(0.0, 0.0, reason=reason or 'straight_drive_stop')
        self._active = False
        self._blocked_since_s = None
        self._last_decision = {
            'active': False,
            'state': 'idle',
            'reason': reason or 'stopped',
            'task_id': self._task_id,
        }

    def tick(self, *, monotonic_s: Optional[float] = None) -> Dict[str, Any]:
        now = time.monotonic() if monotonic_s is None else float(monotonic_s)
        if not self._active:
            return {'state': 'idle', 'active': False}

        if self._last_command_monotonic_s > 0.0:
            min_interval = 1.0 / self.control_hz
            if now - self._last_command_monotonic_s < min_interval:
                return {'state': 'running', 'active': True, 'throttled': True}

        if self._target_x is not None and not self._pose_has_xy(
            self._current_pose(source=self._pose_source)
        ):
            self._publish_velocity(0.0, 0.0, reason='target_pose_unavailable')
            self._active = False
            result = {
                'state': 'blocked',
                'active': False,
                'terminal': True,
                'reason': 'target pose unavailable',
                'distance_traveled_m': self.distance_traveled_m(),
                'target_remaining_m': None,
                'elapsed_s': self.elapsed_s(now),
            }
            self._last_decision = dict(result)
            return result

        completion = self._completion_state(now)
        if completion is not None:
            self._publish_velocity(0.0, 0.0, reason=completion)
            self._active = False
            self._last_decision = {
                'active': False,
                'state': 'completed',
                'reason': completion,
                'distance_traveled_m': self.distance_traveled_m(),
                'target_remaining_m': self.target_remaining_m(),
                'elapsed_s': self.elapsed_s(now),
            }
            return {'state': 'completed', 'active': False, 'reason': completion}

        if self._latest_scan is None or now - self._latest_scan_monotonic_s > self.scan_timeout_s:
            self._publish_velocity(0.0, 0.0, reason='scan_timeout')
            self._mark_blocked(now)
            result = self._blocked_or_waiting(now, reason='scan_timeout')
            self._last_decision = dict(result)
            return result

        theta_goal = self._theta_goal()
        decision = self._planner.plan(self._latest_scan, theta_goal=theta_goal)
        vx, wz = self._velocity_from_decision(decision)
        self._publish_velocity(vx, wz, reason=decision.state)

        if decision.state in {'blocked', 'stop'} and abs(vx) < 1e-4:
            self._mark_blocked(now)
        else:
            self._blocked_since_s = None

        result = {
            'state': decision.state,
            'active': True,
            'task_id': self._task_id,
            'theta_goal_deg': math.degrees(theta_goal),
            'vx': vx,
            'wz': wz,
            'distance_traveled_m': self.distance_traveled_m(),
            'target_remaining_m': self.target_remaining_m(),
            'elapsed_s': self.elapsed_s(now),
            'decision': decision.to_dict(),
        }
        if self._blocked_since_s is not None:
            blocked_for = now - self._blocked_since_s
            result['blocked_for_s'] = blocked_for
            if blocked_for >= self.blocked_timeout_s:
                self._publish_velocity(0.0, 0.0, reason='blocked_timeout')
                self._active = False
                result['state'] = 'blocked'
                result['active'] = False
                result['terminal'] = True
                result['reason'] = 'blocked timeout'
        self._last_decision = dict(result)
        return result

    def snapshot(self) -> Dict[str, Any]:
        data = dict(self._last_decision)
        data.setdefault('active', self._active)
        data['scan_age_s'] = self.scan_age_s()
        data['distance_traveled_m'] = self.distance_traveled_m()
        data['target_distance_m'] = self._target_distance_m
        data['target_duration_s'] = self._target_duration_s
        data['target_x'] = self._target_x
        data['target_y'] = self._target_y
        data['target_tolerance_m'] = (
            self._effective_target_tolerance_m() if self._target_x is not None else None
        )
        data['target_tolerance_base_m'] = (
            self._target_tolerance_base_m if self._target_x is not None else None
        )
        data['target_tolerance_max_m'] = (
            self._target_tolerance_max_m if self._target_x is not None else None
        )
        data['target_remaining_m'] = self.target_remaining_m()
        data['pose_source'] = self._pose_source
        data['command_source'] = self._active_command_source
        data['elapsed_s'] = self.elapsed_s()
        data['control_hz'] = self.control_hz
        return data

    def is_active(self) -> bool:
        return self._active

    def elapsed_s(self, now: Optional[float] = None) -> float:
        if self._started_monotonic_s <= 0.0:
            return 0.0
        now = time.monotonic() if now is None else float(now)
        return max(0.0, now - self._started_monotonic_s)

    def scan_age_s(self) -> Optional[float]:
        if self._latest_scan_monotonic_s <= 0.0:
            return None
        return max(0.0, time.monotonic() - self._latest_scan_monotonic_s)

    def distance_traveled_m(self) -> float:
        if self._start_pose is None:
            return 0.0
        pose = self._current_pose(source=self._pose_source)
        if not pose:
            return 0.0
        return math.hypot(
            float(pose.get('x', 0.0)) - float(self._start_pose.get('x', 0.0)),
            float(pose.get('y', 0.0)) - float(self._start_pose.get('y', 0.0)),
        )

    def target_remaining_m(self) -> Optional[float]:
        if self._target_x is None or self._target_y is None:
            return None
        pose = self._current_pose(source=self._pose_source)
        if not self._pose_has_xy(pose):
            return None
        return math.hypot(
            float(self._target_x) - float(pose.get('x', 0.0)),
            float(self._target_y) - float(pose.get('y', 0.0)),
        )

    def _completion_state(self, now: float) -> Optional[str]:
        if self._target_duration_s is not None and self.elapsed_s(now) >= self._target_duration_s:
            return 'duration reached'
        if (
            self._target_distance_m is not None and
            self.distance_traveled_m() >= max(0.0, self._target_distance_m - self.distance_tolerance_m)
        ):
            return 'distance reached'
        remaining = self.target_remaining_m()
        if remaining is not None and remaining <= self._effective_target_tolerance_m(now):
            return 'target reached'
        return None

    def _effective_target_tolerance_m(self, now: Optional[float] = None) -> float:
        if self._target_x is None or self._target_y is None:
            return self._target_tolerance_base_m
        if self.target_tolerance_relax_after_s <= 0.0:
            return self._target_tolerance_max_m

        now = time.monotonic() if now is None else float(now)
        elapsed = self.elapsed_s(now)
        tolerance = self._target_tolerance_base_m
        if elapsed >= self.target_tolerance_relax_after_s:
            tolerance = max(tolerance, self.target_tolerance_relaxed_m)
        if elapsed >= 2.0 * self.target_tolerance_relax_after_s:
            tolerance = max(tolerance, self.target_tolerance_final_m)
        return min(tolerance, self._target_tolerance_max_m)

    def _velocity_from_decision(self, decision) -> tuple:
        theta = _normalize_angle(float(decision.theta_target))
        abs_theta = abs(theta)
        wz = _clamp(
            self.heading_kp * theta,
            -self.max_angular_speed_radps,
            self.max_angular_speed_radps,
        )

        if decision.state == 'blocked':
            return 0.0, 0.0
        if decision.state == 'stop':
            return 0.0, wz
        if (
            self._target_x is not None and
            abs_theta >= self.turn_in_place_heading_rad
        ):
            return 0.0, wz

        heading_scale = 1.0 - _clamp(abs_theta / max(self.slowdown_heading_rad, 0.1), 0.0, 0.85)
        clearance_scale = _clamp(
            (float(decision.front_clearance_m) - self._planner.min_stop_distance_m) /
            max(0.1, self._planner.clear_path_distance_m - self._planner.min_stop_distance_m),
            0.0,
            1.0,
        )
        vx = self._speed_mps * min(heading_scale, max(0.25, clearance_scale))
        if decision.state == 'avoid':
            vx = max(self.min_avoid_speed_mps, vx * 0.75)
        return _clamp(vx, 0.0, self.max_speed_mps), wz

    def _theta_goal(self) -> float:
        if self._target_x is not None and self._target_y is not None:
            pose = self._current_pose(source=self._pose_source)
            if not self._pose_has_xy(pose):
                return 0.0
            current_yaw = self._pose_yaw_rad(pose)
            if current_yaw is None:
                return 0.0
            target_yaw = math.atan2(
                float(self._target_y) - float(pose.get('y', 0.0)),
                float(self._target_x) - float(pose.get('x', 0.0)),
            )
            return _normalize_angle(target_yaw - current_yaw)

        yaw_ref = self._yaw_ref_rad
        current_yaw = self._pose_yaw_rad(self._current_pose(source=self._pose_source))
        if yaw_ref is None or current_yaw is None:
            return 0.0
        theta = _normalize_angle(yaw_ref - current_yaw)
        if abs(theta) <= self.recover_heading_tolerance_rad:
            return theta
        return theta

    def _current_pose(self, *, source: str = '') -> Dict[str, Any]:
        getter = self._pose_getter
        if source == 'map' and self._map_pose_getter is not None:
            getter = self._map_pose_getter
        try:
            pose = getter() or {}
        except Exception:
            pose = {}
        return dict(pose)

    def _pose_yaw_rad(self, pose: Optional[Dict[str, Any]]) -> Optional[float]:
        if not pose:
            return None
        if 'yaw_deg' in pose:
            return math.radians(float(pose.get('yaw_deg', 0.0)))
        return None

    def _pose_has_xy(self, pose: Optional[Dict[str, Any]]) -> bool:
        if not pose:
            return False
        try:
            float(pose['x'])
            float(pose['y'])
            return True
        except Exception:
            return False

    def _mark_blocked(self, now: float):
        if self._blocked_since_s is None:
            self._blocked_since_s = now

    def _blocked_or_waiting(self, now: float, *, reason: str) -> Dict[str, Any]:
        blocked_for = 0.0 if self._blocked_since_s is None else now - self._blocked_since_s
        result = {
            'state': 'blocked',
            'active': True,
            'reason': reason,
            'blocked_for_s': blocked_for,
            'distance_traveled_m': self.distance_traveled_m(),
            'elapsed_s': self.elapsed_s(now),
        }
        if blocked_for >= self.blocked_timeout_s:
            self._active = False
            result['active'] = False
            result['terminal'] = True
            result['reason'] = f'{reason} timeout'
        return result

    def _publish_velocity(self, vx: float, wz: float, *, reason: str):
        self._last_vx = float(vx)
        self._last_wz = float(wz)
        self._last_command_monotonic_s = time.monotonic()
        self._move_gateway.velocity(
            linear_x=float(vx),
            angular_z=float(wz),
            source=self._active_command_source,
            reason=reason,
            task_id=self._task_id,
        )
