import json
import math
import time
from typing import Any, Dict, Optional

from .waypoint_action_runner import WaypointActionRunner
from .waypoint_dispatcher import WaypointDispatcher
from ..task_models import MissionTask, TaskRuntime, TERMINAL_TASK_STAGES
from ..task_parser import parse_task_definition
from ..geo_utils import EARTH_RADIUS_M
from ..localization import OutdoorPoseAligner
from ..outdoor import (
    GpsVoGate,
    GoalResolution,
    OutdoorMissionPlan,
    OutdoorGoalResolver,
    is_outdoor_task_payload,
    parse_outdoor_task_definition,
)


class MissionManager:
    def __init__(
        self,
        *,
        logger,
        backend,
        camera_capture,
        projector,
        closed_loop_manager,
        pose_registry,
        control_mode_getter,
        controller_getter,
        battery_level_getter,
        now_ns_getter,
        position_getter=None,
        stamp_getter,
        publish_uplink,
        write_state,
        publish_media_control,
        set_map_pose_reporting_enabled,
        move_gateway=None,
        extra_status_getter=None,
        mission_defaults,
        outdoor_goal_resolver_cfg: Optional[Dict[str, Any]] = None,
        outdoor_pose_aligner_cfg: Optional[Dict[str, Any]] = None,
        progress_report_interval_s: float,
        require_auto_mode: bool,
        media_actions_cfg: Optional[Dict[str, Any]] = None,
    ):
        self._logger = logger
        self._backend = backend
        self._projector = projector
        self._closed_loop_manager = closed_loop_manager
        self._pose_registry = pose_registry
        self._control_mode_getter = control_mode_getter
        self._controller_getter = controller_getter
        self._battery_level_getter = battery_level_getter
        self._now_ns_getter = now_ns_getter
        self._position_getter = position_getter or self._pose_registry.effective_position
        self._publish_uplink = publish_uplink
        self._write_state = write_state
        self._publish_media_control = publish_media_control
        self._set_map_pose_reporting_enabled = set_map_pose_reporting_enabled
        self._move_gateway = move_gateway
        self._extra_status_getter = extra_status_getter
        self._mission_defaults = dict(mission_defaults or {})
        self._outdoor_goal_resolver = OutdoorGoalResolver(outdoor_goal_resolver_cfg)
        self._outdoor_pose_aligner_cfg = dict(outdoor_pose_aligner_cfg or {})
        self._outdoor_pose_aligner = OutdoorPoseAligner(self._outdoor_pose_aligner_cfg)
        self._progress_report_interval_s = float(progress_report_interval_s)
        self._require_auto_mode = bool(require_auto_mode)
        self._media_actions_cfg = dict(media_actions_cfg or {})

        self._dispatcher = WaypointDispatcher(
            logger=logger,
            backend=backend,
            projector=projector,
            stamp_getter=stamp_getter,
        )
        self._action_runner = WaypointActionRunner(
            camera_capture=camera_capture,
            backend_snapshot_getter=self._dispatcher.backend_snapshot,
            battery_level_getter=self._battery_level_getter,
            position_getter=self._position_getter,
            gps_monitor_state_getter=self._closed_loop_manager.gps_state_snapshot,
            now_ns_getter=self._now_ns_getter,
            publish_event=self._publish_task_action_event,
            set_runtime_event=self._set_runtime_event,
        )

        self._task: Optional[MissionTask] = None
        self._outdoor_mission: Optional[OutdoorMissionPlan] = None
        self._runtime = TaskRuntime()
        self._active_waypoint_index: Optional[int] = None
        self._active_outdoor_point_index: Optional[int] = None
        self._active_outdoor_goal_resolution: Optional[GoalResolution] = None
        self._active_outdoor_candidate_index: Optional[int] = None
        self._hover_until: Optional[float] = None
        self._last_periodic_report_ts = 0.0
        self._recording_active = False
        self._outdoor_calibration_records = []
        self._outdoor_goal_records = []
        self._outdoor_gps_vo_gate = GpsVoGate()

    @property
    def task(self) -> Optional[MissionTask]:
        return self._task

    @property
    def runtime(self) -> TaskRuntime:
        return self._runtime

    def has_active_task(self) -> bool:
        if self._task is None and self._outdoor_mission is None:
            return False
        return self._runtime.stage not in TERMINAL_TASK_STAGES

    def handle_task_command(
        self,
        *,
        action: str,
        task_id: str = '',
        task_type: str = '',
        task_params_json: str = '',
    ):
        normalized_action = str(action or '').strip().lower()
        if normalized_action == 'start':
            self._handle_start(
                task_id=task_id,
                task_type=task_type,
                task_params_json=task_params_json,
            )
            return
        if normalized_action == 'pause':
            self.pause_current_operation(reason='paused_by_command')
            return
        if normalized_action == 'resume':
            self.resume_current_operation()
            return
        if normalized_action == 'stop':
            self.stop_current_operation(reason='stopped_by_command')
            return
        if normalized_action == 'update_params':
            self._logger.warning('update_params is reserved but not implemented yet')
            self._set_runtime_event('update_params_ignored', publish_now=True)
            return
        self._logger.warning(f'unknown task action: {action}')

    def pause_current_operation(self, *, reason: str = 'paused_by_command') -> Dict[str, Any]:
        if self.has_active_task():
            self._handle_pause(reason=reason)
            return {'accepted': True, 'message': 'task paused'}
        return {'accepted': False, 'message': 'no active task'}

    def resume_current_operation(self) -> Dict[str, Any]:
        if self.has_active_task() and self._runtime.stage == 'paused':
            self._handle_resume()
            return {'accepted': True, 'message': 'task resumed'}

        return {'accepted': False, 'message': 'no paused task'}

    def stop_current_operation(self, *, reason: str = 'stopped_by_command') -> Dict[str, Any]:
        if self.has_active_task():
            self._handle_stop(reason=reason)
            return {'accepted': True, 'message': 'task stopped'}

        return {'accepted': False, 'message': 'no active task'}

    def tick(self, *, monotonic_s: Optional[float] = None):
        monotonic_s = time.monotonic() if monotonic_s is None else float(monotonic_s)

        if self._backend is not None and self.has_active_task():
            self._backend.refresh_status()

        backend_event = self._backend.take_terminal_event() if self._backend is not None else None

        if self.has_active_task():
            self._tick_task(monotonic_s=monotonic_s, backend_event=backend_event)
            return

    def build_status_payload(self) -> Dict[str, Any]:
        payload = self._runtime.to_status_dict(
            task=self._task,
            position=self._position_getter(),
            battery_level=self._battery_level_getter(),
            control_mode=self._control_mode_getter(),
            controller=self._controller_getter(),
            backend_state=self._dispatcher.backend_snapshot(),
        )
        payload['timestamp_ns'] = self._now_ns_getter()
        payload['geo_reference'] = self._projector.reference_summary()
        payload['gps_monitor'] = self._closed_loop_manager.gps_state_snapshot()
        payload['closed_loop'] = self._closed_loop_manager.state_snapshot()
        payload['recording_active'] = self._recording_active
        payload['map_pose'] = self._pose_registry.map_pose_dict()
        payload['visual_pose'] = self._pose_registry.visual_pose_dict()
        payload['outdoor_navigation'] = self._outdoor_status_snapshot()
        if self._extra_status_getter is not None:
            payload.update(dict(self._extra_status_getter() or {}))
        return payload

    def outdoor_pose_alignment_state(self) -> Dict[str, Any]:
        return self._outdoor_pose_aligner.state_snapshot()

    def _outdoor_status_snapshot(self) -> Dict[str, Any]:
        if self._outdoor_mission is None:
            return {'active': False}

        current_point = None
        if 0 <= self._runtime.current_waypoint_index < len(self._outdoor_mission.execution_points):
            current_point = self._outdoor_mission.execution_points[
                self._runtime.current_waypoint_index
            ].to_dict()

        active_goal = None
        if (
            self._active_outdoor_point_index is not None and
            0 <= self._active_outdoor_point_index < len(self._outdoor_mission.execution_points)
        ):
            active_goal = self._outdoor_mission.execution_points[
                self._active_outdoor_point_index
            ].to_dict()

        active_candidate = None
        if (
            self._active_outdoor_goal_resolution is not None and
            self._active_outdoor_candidate_index is not None
        ):
            candidate = self._active_outdoor_goal_resolution.candidate(
                self._active_outdoor_candidate_index
            )
            active_candidate = candidate.to_dict() if candidate is not None else None

        return {
            'active': self._runtime.stage not in TERMINAL_TASK_STAGES,
            'mission': self._outdoor_mission.to_status_dict(),
            'current_point': current_point,
            'active_goal': active_goal,
            'active_candidate': active_candidate,
            'active_resolution': (
                self._active_outdoor_goal_resolution.to_dict()
                if self._active_outdoor_goal_resolution is not None else None
            ),
            'goal_resolver': self._outdoor_goal_resolver.config_snapshot(),
            'goal_records': list(self._outdoor_goal_records[-20:]),
            'calibration_records': list(self._outdoor_calibration_records[-20:]),
            'gps_vo_gate': self._outdoor_gps_vo_gate.state_snapshot(),
            'pose_alignment': self._outdoor_pose_aligner.state_snapshot(),
        }

    def status_json(self, *, task_id: str = '') -> str:
        if task_id and task_id != self._runtime.task_id:
            return json.dumps(
                {
                    'task_id': task_id,
                    'stage': 'not_found',
                    'status': 'unknown',
                },
                ensure_ascii=False,
            )
        return json.dumps(self.build_status_payload(), ensure_ascii=False)

    def destroy(self):
        self._stop_recording_if_needed()
        self._set_map_pose_reporting_enabled(False)

    def _tick_task(self, *, monotonic_s: float, backend_event):
        if self._outdoor_mission is not None:
            self._tick_outdoor_task(monotonic_s=monotonic_s, backend_event=backend_event)
            return

        if self._require_auto_mode and self._control_mode_getter() != 'auto':
            self._pause_with_error(
                code='E_TASK_CONFLICT',
                description='current control_mode is not auto',
                suggested_action='switch_to_auto_mode',
            )
            return

        if (
            self._task is not None and
            self._task.abort_on_low_battery and
            self._battery_level_getter() is not None and
            self._battery_level_getter() <= self._task.low_battery_threshold
        ):
            self._abort_task(
                code='E_LOW_BATTERY',
                description=(
                    f'battery level {self._battery_level_getter():.1f} is below threshold '
                    f'{self._task.low_battery_threshold:.1f}'
                ),
                suggested_action=self._task.abort_action or 'stop',
            )
            return

        if self._runtime.stage == 'executing':
            violation = self._closed_loop_manager.evaluate(
                projector=self._projector,
                pose_registry=self._pose_registry,
                now_ns=self._now_ns_getter(),
                monotonic_s=monotonic_s,
            )
            if violation is not None:
                if violation.get('action') == 'abort':
                    self._abort_task(
                        code=violation['code'],
                        description=violation['description'],
                        suggested_action=violation['suggested_action'],
                    )
                else:
                    self._pause_with_error(
                        code=violation['code'],
                        description=violation['description'],
                        suggested_action=violation['suggested_action'],
                    )
                return

        if backend_event is not None:
            self._handle_backend_event(backend_event)

        if self._runtime.stage == 'paused':
            self._publish_periodic_progress(monotonic_s=monotonic_s)
            return

        if self._hover_until is not None:
            if monotonic_s < self._hover_until:
                self._publish_periodic_progress(monotonic_s=monotonic_s)
                return
            self._hover_until = None
            if self._task is not None and self._runtime.current_waypoint_index >= 0:
                completed_waypoint = self._task.waypoints[self._runtime.current_waypoint_index]
                self._action_runner.queue(completed_waypoint, task_id=self._runtime.task_id)

        if self._action_runner.has_pending():
            if not self._action_runner.tick():
                self._publish_periodic_progress(monotonic_s=monotonic_s)
                return

        if self._active_waypoint_index is None and self._hover_until is None:
            self._dispatch_next_waypoint_if_needed()

        self._publish_periodic_progress(monotonic_s=monotonic_s)

    def _handle_start(self, *, task_id: str, task_type: str, task_params_json: str):
        if is_outdoor_task_payload(task_type=task_type, task_params_json=task_params_json):
            self._handle_outdoor_start(
                task_id=task_id,
                task_type=task_type,
                task_params_json=task_params_json,
            )
            return

        try:
            task = parse_task_definition(
                task_id=task_id,
                task_type=task_type,
                task_params_json=task_params_json,
                defaults=self._mission_defaults,
            )
        except Exception as exc:
            self._runtime = TaskRuntime(
                task_id=task_id,
                stage='aborted',
                status='exception',
                error=self._make_error(
                    code='E_TASK_PARSE',
                    description=f'failed to parse task params: {exc}',
                    suggested_action='fix_task_payload',
                ),
            )
            self._task = None
            self._outdoor_mission = None
            self._publish_task_progress()
            self._write_state_fields()
            return

        self._task = task
        self._outdoor_mission = None
        self._runtime = TaskRuntime(
            task_id=task.task_id,
            stage='preparing',
            status='normal',
            current_waypoint_index=-1,
            current_waypoint_seq=-1,
            total_waypoints=len(task.waypoints),
            progress_percent=0.0,
            event='task_received',
            error=None,
        )
        self._active_waypoint_index = None
        self._hover_until = None
        self._action_runner.reset()
        self._closed_loop_manager.reset()

        if self._require_auto_mode and self._control_mode_getter() != 'auto':
            self._pause_with_error(
                code='E_TASK_CONFLICT',
                description='current control_mode is not auto; switch to auto before executing task',
                suggested_action='switch_to_auto_mode',
            )
            return

        if not self._projector.is_ready():
            self._pause_with_error(
                code='E_GEOREF_NOT_READY',
                description='geo reference is disabled or not configured',
                suggested_action='configure_geo_reference',
            )
            return

        self._set_map_pose_reporting_enabled(True)
        self._start_recording_if_needed()
        self._publish_task_progress()
        self._write_state_fields()

    def _handle_outdoor_start(self, *, task_id: str, task_type: str, task_params_json: str):
        try:
            outdoor_mission = parse_outdoor_task_definition(
                task_id=task_id,
                task_type=task_type,
                task_params_json=task_params_json,
                defaults=self._mission_defaults,
            )
        except Exception as exc:
            self._runtime = TaskRuntime(
                task_id=task_id,
                stage='aborted',
                status='exception',
                error=self._make_error(
                    code='E_OUTDOOR_TASK_PARSE',
                    description=f'failed to parse outdoor task params: {exc}',
                    suggested_action='fix_mission_planner_payload',
                ),
            )
            self._task = None
            self._outdoor_mission = None
            self._publish_task_progress()
            self._write_state_fields()
            return

        self._task = None
        self._outdoor_mission = outdoor_mission
        self._runtime = TaskRuntime(
            task_id=outdoor_mission.task_id,
            stage='preparing',
            status='normal',
            current_waypoint_index=-1,
            current_waypoint_seq=-1,
            total_waypoints=len(outdoor_mission.execution_points),
            progress_percent=0.0,
            event='outdoor_task_received',
            error=None,
        )
        self._active_waypoint_index = None
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._hover_until = None
        self._action_runner.reset()
        self._closed_loop_manager.reset()
        self._outdoor_calibration_records = []
        self._outdoor_goal_records = []
        self._outdoor_gps_vo_gate = GpsVoGate(
            stable_required_count=outdoor_mission.stable_offset_required_count
        )
        self._outdoor_pose_aligner.reset()

        if self._require_auto_mode and self._control_mode_getter() != 'auto':
            self._pause_with_error(
                code='E_TASK_CONFLICT',
                description='current control_mode is not auto; switch to auto before executing outdoor task',
                suggested_action='switch_to_auto_mode',
            )
            return

        self._set_map_pose_reporting_enabled(True)
        self._publish_task_progress()
        self._write_state_fields()

    def _handle_pause(self, *, reason: str):
        if (self._task is None and self._outdoor_mission is None) or self._runtime.stage in TERMINAL_TASK_STAGES:
            return

        self._stop_backend_navigation_if_needed()
        self._publish_move_stop(reason=reason)
        self._active_waypoint_index = None
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._hover_until = None
        self._action_runner.reset()
        self._runtime.stage = 'paused'
        self._runtime.status = 'paused'
        self._runtime.event = reason
        self._publish_task_progress()
        self._write_state_fields()

    def _handle_resume(self):
        if self._task is None and self._outdoor_mission is None:
            return
        if self._runtime.stage != 'paused':
            return
        if self._require_auto_mode and self._control_mode_getter() != 'auto':
            self._pause_with_error(
                code='E_TASK_CONFLICT',
                description='cannot resume task because control_mode is not auto',
                suggested_action='switch_to_auto_mode',
            )
            return
        self._runtime.stage = 'executing'
        self._runtime.status = 'normal'
        self._runtime.event = 'task_resumed'
        self._runtime.error = None
        self._publish_task_progress()
        self._write_state_fields()

    def _handle_stop(self, *, reason: str):
        self._stop_backend_navigation_if_needed()
        self._publish_move_stop(reason=reason)
        self._active_waypoint_index = None
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._hover_until = None
        self._action_runner.reset()
        self._runtime.stage = 'aborted'
        self._runtime.status = 'exception'
        self._runtime.event = reason
        self._runtime.error = self._make_error(
            code='E_TASK_STOPPED',
            description='task stopped by external command',
            suggested_action='restart_task',
            severity='info',
        )
        self._stop_recording_if_needed()
        self._set_map_pose_reporting_enabled(False)
        self._publish_task_progress()
        self._write_state_fields()

    def _dispatch_next_waypoint_if_needed(self):
        if self._task is None:
            return

        next_index = self._runtime.current_waypoint_index + 1
        if next_index >= len(self._task.waypoints):
            self._complete_task()
            return

        waypoint = self._task.waypoints[next_index]
        result = self._dispatcher.dispatch_task_waypoint(task=self._task, waypoint=waypoint)
        if not result.get('success', False):
            self._pause_with_error(
                code=result.get('error_code', 'E_MOVE_FAIL'),
                description=result.get('description', 'failed to dispatch waypoint'),
                suggested_action=result.get('suggested_action', 'check_navigation_backend'),
            )
            return

        pose = result['pose']
        self._active_waypoint_index = next_index
        self._runtime.stage = 'executing'
        self._runtime.status = 'normal'
        self._runtime.event = 'waypoint_dispatched'
        self._runtime.current_waypoint_seq = waypoint.seq
        self._runtime.last_goal_map_pose = {
            'x': float(pose.pose.position.x),
            'y': float(pose.pose.position.y),
            'z': float(pose.pose.position.z),
        }
        self._publish_task_progress()
        self._write_state_fields()

    def _tick_outdoor_task(self, *, monotonic_s: float, backend_event):
        if self._outdoor_mission is None:
            return

        if self._require_auto_mode and self._control_mode_getter() != 'auto':
            self._pause_with_error(
                code='E_TASK_CONFLICT',
                description='current control_mode is not auto',
                suggested_action='switch_to_auto_mode',
            )
            return

        if backend_event is not None:
            self._handle_outdoor_backend_event(backend_event)

        if self._runtime.stage == 'paused':
            self._publish_periodic_progress(monotonic_s=monotonic_s)
            return

        if self._active_outdoor_point_index is None:
            self._dispatch_next_outdoor_point_if_needed()

        self._publish_periodic_progress(monotonic_s=monotonic_s)

    def _dispatch_next_outdoor_point_if_needed(self):
        mission = self._outdoor_mission
        if mission is None:
            return

        next_index = self._runtime.current_waypoint_index + 1
        if next_index >= len(mission.execution_points):
            self._complete_task()
            return

        point = mission.execution_points[next_index]
        resolution = self._outdoor_goal_resolver.resolve(
            point,
            previous_point=self._outdoor_point_at(next_index - 1),
            next_point=self._outdoor_point_at(next_index + 1),
            current_pose=self._pose_registry.map_pose_dict(),
        )
        if not resolution.candidates:
            self._pause_with_error(
                code='E_OUTDOOR_GOAL_RESOLVE_EMPTY',
                description='outdoor goal resolver produced no candidate target',
                suggested_action='check_goal_resolver_config',
            )
            return

        self._active_outdoor_goal_resolution = resolution
        self._active_outdoor_candidate_index = 0
        if not self._dispatch_outdoor_candidate(next_index=next_index, candidate_index=0):
            self._active_outdoor_goal_resolution = None
            self._active_outdoor_candidate_index = None

    def _outdoor_point_at(self, index: int):
        mission = self._outdoor_mission
        if mission is None:
            return None
        if index < 0 or index >= len(mission.execution_points):
            return None
        return mission.execution_points[index]

    def _dispatch_outdoor_candidate(self, *, next_index: int, candidate_index: int) -> bool:
        mission = self._outdoor_mission
        resolution = self._active_outdoor_goal_resolution
        if mission is None or resolution is None:
            return False

        candidate = resolution.candidate(candidate_index)
        if candidate is None:
            self._pause_with_error(
                code='E_OUTDOOR_GOAL_CANDIDATE_EMPTY',
                description='outdoor goal resolver candidate index is out of range',
                suggested_action='check_goal_resolver_config',
            )
            return False

        point = mission.execution_points[next_index]
        goal_correction = self._current_outdoor_goal_correction()
        corrected_x = float(candidate.x) + float(goal_correction.get('dx', 0.0))
        corrected_y = float(candidate.y) + float(goal_correction.get('dy', 0.0))
        result = self._dispatcher.dispatch_map_goal(
            map_x=corrected_x,
            map_y=corrected_y,
            map_name=mission.map_name,
            outdoor=True,
            frame_id=mission.frame_id,
            yaw_deg=candidate.yaw_deg,
        )
        if not result.get('success', False):
            self._pause_with_error(
                code=result.get('error_code', 'E_MOVE_FAIL'),
                description=result.get('description', 'failed to dispatch outdoor route point'),
                suggested_action=result.get('suggested_action', 'check_navigation_backend'),
            )
            return False

        pose = result['pose']
        self._active_outdoor_point_index = next_index
        self._active_outdoor_candidate_index = candidate_index
        self._runtime.stage = 'executing'
        self._runtime.status = 'normal'
        self._runtime.event = (
            'outdoor_point_dispatched'
            if candidate_index == 0 else
            'outdoor_point_retry_candidate_dispatched'
        )
        self._runtime.current_waypoint_seq = point.seq
        self._runtime.last_goal_map_pose = {
            'x': float(pose.pose.position.x),
            'y': float(pose.pose.position.y),
            'z': float(pose.pose.position.z),
            'original_x': float(point.x),
            'original_y': float(point.y),
            'candidate_x': float(candidate.x),
            'candidate_y': float(candidate.y),
            'goal_correction': goal_correction,
            'candidate_index': int(candidate_index),
            'candidate_source': candidate.source,
            'offset_kind': candidate.offset_kind,
            'offset_distance_m': candidate.offset_distance_m,
        }
        self._publish_task_progress()
        self._write_state_fields()
        return True

    def _current_outdoor_goal_correction(self) -> Dict[str, Any]:
        alignment_state = self._outdoor_pose_aligner.state_snapshot()
        correction = dict((alignment_state.get('last_result') or {}).get('goal_correction') or {})
        if not correction.get('enabled', False):
            return {
                'enabled': False,
                'applied': False,
                'dx': 0.0,
                'dy': 0.0,
                'reason': correction.get('reason', 'goal correction is disabled'),
            }
        if not correction.get('applied', False):
            return {
                'enabled': True,
                'applied': False,
                'dx': 0.0,
                'dy': 0.0,
                'reason': correction.get('reason', 'alignment estimate is not stable'),
            }
        return {
            'enabled': True,
            'applied': True,
            'dx': float(correction.get('dx', 0.0)),
            'dy': float(correction.get('dy', 0.0)),
            'reason': correction.get('reason', 'stable gps/map offset correction'),
        }

    def _handle_outdoor_backend_event(self, backend_event: Dict[str, Any]):
        state = backend_event.get('state')
        if state == 'succeeded':
            self._handle_outdoor_point_success()
            return

        if state in {'stopped', 'canceled'}:
            if self._runtime.stage == 'paused':
                self._runtime.event = 'navigation_stopped'
                self._publish_task_progress()
            return

        if self._retry_next_outdoor_candidate(backend_event):
            return

        if self._skip_current_outdoor_point_if_allowed(backend_event):
            return

        self._record_active_outdoor_goal(status='failed_exhausted', backend_event=backend_event)
        self._pause_with_error(
            code='E_MOVE_FAIL',
            description=(
                'algorithm_manager returned terminal state '
                f'{state}, feedback={backend_event.get("feedback_msg", "")}'
            ),
            suggested_action='retry_or_relocalize',
        )

    def _retry_next_outdoor_candidate(self, backend_event: Dict[str, Any]) -> bool:
        if (
            self._active_outdoor_point_index is None or
            self._active_outdoor_goal_resolution is None or
            self._active_outdoor_candidate_index is None
        ):
            return False

        current_index = int(self._active_outdoor_candidate_index)
        if not self._active_outdoor_goal_resolution.has_next_after(current_index):
            return False

        self._record_active_outdoor_goal(status='candidate_failed', backend_event=backend_event)
        next_candidate_index = current_index + 1
        self._runtime.event = 'outdoor_point_retrying_with_resolved_candidate'
        dispatched = self._dispatch_outdoor_candidate(
            next_index=self._active_outdoor_point_index,
            candidate_index=next_candidate_index,
        )
        return True if dispatched or self._runtime.stage == 'paused' else False

    def _skip_current_outdoor_point_if_allowed(self, backend_event: Dict[str, Any]) -> bool:
        mission = self._outdoor_mission
        if mission is None or self._active_outdoor_point_index is None:
            return False

        point = mission.execution_points[self._active_outdoor_point_index]
        if not self._outdoor_goal_resolver.should_skip_after_exhausted(point.kind):
            return False

        self._record_active_outdoor_goal(status='skipped_unreachable', backend_event=backend_event)
        self._runtime.current_waypoint_index = self._active_outdoor_point_index
        self._runtime.current_waypoint_seq = point.seq
        self._runtime.progress_percent = (
            100.0 * (self._runtime.current_waypoint_index + 1) /
            max(1, len(mission.execution_points))
        )
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._runtime.event = 'outdoor_transit_point_skipped_unreachable'
        self._runtime.error = None
        self._runtime.stage = 'executing'
        self._runtime.status = 'normal'
        self._publish_task_progress()
        self._write_state_fields()
        return True

    def _handle_outdoor_point_success(self):
        mission = self._outdoor_mission
        if mission is None or self._active_outdoor_point_index is None:
            return

        point = mission.execution_points[self._active_outdoor_point_index]
        self._record_active_outdoor_goal(status='reached', backend_event=None)
        self._runtime.current_waypoint_index = self._active_outdoor_point_index
        self._runtime.current_waypoint_seq = point.seq
        self._runtime.progress_percent = (
            100.0 * (self._runtime.current_waypoint_index + 1) /
            max(1, len(mission.execution_points))
        )
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._runtime.event = 'outdoor_point_reached'
        self._runtime.error = None
        self._runtime.stage = 'executing'
        self._runtime.status = 'normal'

        if mission.calibrate_at_nav_points and point.kind in {'calibration', 'inspection', 'home'}:
            calibration = self._record_outdoor_calibration(point)
            if calibration.get('status') == 'failed':
                self._pause_with_error(
                    code='E_OUTDOOR_CALIBRATION_ERROR',
                    description=calibration.get('message', 'outdoor calibration failed'),
                    suggested_action='check_gps_or_relocalize',
                )
                return
            self._runtime.event = (
                'outdoor_calibration_checked'
                if calibration.get('status') == 'passed' else
                'outdoor_calibration_observing'
            )

        self._publish_task_progress()
        self._write_state_fields()

    def _record_active_outdoor_goal(
        self,
        *,
        status: str,
        backend_event: Optional[Dict[str, Any]],
    ):
        mission = self._outdoor_mission
        if (
            mission is None or
            self._active_outdoor_point_index is None or
            self._active_outdoor_goal_resolution is None or
            self._active_outdoor_candidate_index is None
        ):
            return

        point = mission.execution_points[self._active_outdoor_point_index]
        candidate = self._active_outdoor_goal_resolution.candidate(
            self._active_outdoor_candidate_index
        )
        record = {
            'timestamp_ns': self._now_ns_getter(),
            'status': str(status),
            'point': point.to_dict(),
            'candidate': candidate.to_dict() if candidate is not None else None,
            'candidate_index': int(self._active_outdoor_candidate_index),
            'candidate_count': len(self._active_outdoor_goal_resolution.candidates),
            'resolution': {
                'strategy': self._active_outdoor_goal_resolution.strategy,
                'validator_status': self._active_outdoor_goal_resolution.validator_status,
            },
            'backend_event': dict(backend_event or {}),
        }
        self._outdoor_goal_records.append(record)

    def _record_outdoor_calibration(self, point) -> Dict[str, Any]:
        mission = self._outdoor_mission
        tolerance_m = float(point.tolerance_m or (mission.position_tolerance_m if mission else 10.0))
        gps_position = self._pose_registry.gps_position()
        visual_pose = self._pose_registry.latest_visual_pose()
        record = {
            'seq': point.seq,
            'kind': point.kind,
            'nav_point_id': point.nav_point_id,
            'name': point.name,
            'target': point.to_dict(),
            'gps_position': gps_position,
            'visual_pose': visual_pose or {},
            'tolerance_m': tolerance_m,
            'timestamp_ns': self._now_ns_getter(),
            'status': 'skipped',
            'message': '',
        }

        if point.lat is None or point.lon is None:
            record['message'] = 'target nav point has no lat/lon; map-frame navigation accepted'
            self._outdoor_calibration_records.append(record)
            return record

        if not gps_position or 'lat' not in gps_position or 'lon' not in gps_position:
            record['status'] = 'pending_gps'
            record['message'] = 'gps position is unavailable at calibration point'
            self._outdoor_calibration_records.append(record)
            return record

        error_m = self._latlon_distance_m(
            float(point.lat),
            float(point.lon),
            float(gps_position['lat']),
            float(gps_position['lon']),
        )
        record['error_m'] = error_m
        record['gps_fix_type'] = self._pose_registry.gps_fix_type()
        record['gps_num_sv'] = self._pose_registry.gps_num_sv()

        gps_delta_from_prev_m = None
        visual_delta_from_prev_m = None
        if self._outdoor_calibration_records:
            prev = self._outdoor_calibration_records[-1].get('gps_position') or {}
            if 'lat' in prev and 'lon' in prev:
                gps_delta_from_prev_m = self._latlon_distance_m(
                    float(prev['lat']),
                    float(prev['lon']),
                    float(gps_position['lat']),
                    float(gps_position['lon']),
                )
                record['gps_delta_from_prev_m'] = gps_delta_from_prev_m
            prev_visual = self._outdoor_calibration_records[-1].get('visual_pose') or {}
            if visual_pose is not None and 'x' in prev_visual and 'y' in prev_visual:
                visual_delta_from_prev_m = self._xy_distance_m(
                    float(prev_visual['x']),
                    float(prev_visual['y']),
                    float(visual_pose['x']),
                    float(visual_pose['y']),
                )
                record['visual_delta_from_prev_m'] = visual_delta_from_prev_m

        gate = self._outdoor_gps_vo_gate.evaluate(
            gps_error_m=error_m,
            tolerance_m=tolerance_m,
            jump_reject_m=mission.gps_jump_reject_m if mission is not None else 15.0,
            gps_delta_from_prev_m=gps_delta_from_prev_m,
            visual_delta_from_prev_m=visual_delta_from_prev_m,
            gps_fix_type=self._pose_registry.gps_fix_type(),
            gps_num_sv=self._pose_registry.gps_num_sv(),
            min_fix_type=mission.min_gps_fix_type if mission is not None else 2,
        )
        record['gate'] = gate
        alignment = self._record_outdoor_pose_alignment(
            point=point,
            gps_position=gps_position,
            visual_pose=visual_pose,
        )
        if alignment:
            record['alignment'] = alignment
        record['status'] = gate.get('status', 'pending')
        record['message'] = gate.get('reason', '')
        if record['status'] == 'passed':
            record['message'] = f'gps error {error_m:.2f}m within tolerance {tolerance_m:.2f}m'
        elif record['status'] == 'failed':
            record['message'] = f'gps error {error_m:.2f}m exceeds tolerance {tolerance_m:.2f}m'

        self._outdoor_calibration_records.append(record)
        return record

    def _record_outdoor_pose_alignment(
        self,
        *,
        point,
        gps_position: Dict[str, Any],
        visual_pose: Optional[Dict[str, Any]],
    ) -> Dict[str, Any]:
        if point.lat is None or point.lon is None:
            return {}
        if not gps_position or 'lat' not in gps_position or 'lon' not in gps_position:
            return {}

        try:
            alignment = self._outdoor_pose_aligner.evaluate(
                projector=self._projector,
                target_lat=float(point.lat),
                target_lon=float(point.lon),
                target_x=float(point.x),
                target_y=float(point.y),
                gps_position=gps_position,
                map_pose=self._pose_registry.latest_map_pose(),
                visual_pose=visual_pose,
                gps_fix_type=self._pose_registry.gps_fix_type(),
                gps_num_sv=self._pose_registry.gps_num_sv(),
                timestamp_ns=self._now_ns_getter(),
            )
        except Exception as exc:
            alignment = {
                'status': 'error',
                'message': f'failed to evaluate outdoor pose alignment: {exc}',
                'stable': False,
            }
        return alignment

    def _xy_distance_m(self, x_a: float, y_a: float, x_b: float, y_b: float) -> float:
        return math.hypot(float(x_b) - float(x_a), float(y_b) - float(y_a))

    def _latlon_distance_m(self, lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
        lat_a_rad = math.radians(lat_a)
        lat_b_rad = math.radians(lat_b)
        dlat = lat_b_rad - lat_a_rad
        dlon = math.radians(lon_b - lon_a)
        a = (
            math.sin(dlat / 2.0) ** 2 +
            math.cos(lat_a_rad) * math.cos(lat_b_rad) * math.sin(dlon / 2.0) ** 2
        )
        return 2.0 * EARTH_RADIUS_M * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))

    def _handle_backend_event(self, backend_event: Dict[str, Any]):
        if self._task is None:
            return

        state = backend_event.get('state')
        if state == 'succeeded':
            self._handle_waypoint_success()
            return

        if state in {'stopped', 'canceled'}:
            if self._runtime.stage == 'paused':
                self._runtime.event = 'navigation_stopped'
                self._publish_task_progress()
            return

        self._pause_with_error(
            code='E_MOVE_FAIL',
            description=(
                'algorithm_manager returned terminal state '
                f'{state}, feedback={backend_event.get("feedback_msg", "")}'
            ),
            suggested_action='retry_or_relocalize',
        )

    def _handle_waypoint_success(self):
        if self._task is None or self._active_waypoint_index is None:
            return

        waypoint = self._task.waypoints[self._active_waypoint_index]
        self._runtime.current_waypoint_index = self._active_waypoint_index
        self._runtime.current_waypoint_seq = waypoint.seq
        self._runtime.progress_percent = (
            100.0 * (self._runtime.current_waypoint_index + 1) / max(1, len(self._task.waypoints))
        )
        self._active_waypoint_index = None
        self._runtime.event = 'waypoint_reached'
        self._runtime.error = None
        self._runtime.stage = 'executing'
        self._runtime.status = 'normal'
        if waypoint.hover_time_s > 0.0:
            self._hover_until = time.monotonic() + waypoint.hover_time_s
        else:
            self._action_runner.queue(waypoint, task_id=self._runtime.task_id)
        self._publish_task_progress()
        self._write_state_fields()

    def _complete_task(self):
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._runtime.stage = 'completed'
        self._runtime.status = 'normal'
        self._runtime.progress_percent = 100.0
        self._runtime.event = 'task_completed'
        self._runtime.error = None
        self._action_runner.reset()
        self._stop_recording_if_needed()
        self._set_map_pose_reporting_enabled(False)
        self._publish_task_progress()
        self._write_state_fields()

    def _abort_task(self, *, code: str, description: str, suggested_action: str):
        self._stop_backend_navigation_if_needed()
        self._publish_move_stop(reason='task_aborted')
        self._active_waypoint_index = None
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._hover_until = None
        self._action_runner.reset()
        self._runtime.stage = 'aborted'
        self._runtime.status = 'exception'
        self._runtime.event = 'task_aborted'
        self._runtime.error = self._make_error(
            code=code,
            description=description,
            suggested_action=suggested_action,
        )
        self._stop_recording_if_needed()
        self._set_map_pose_reporting_enabled(False)
        self._publish_task_progress()
        self._write_state_fields()

    def _pause_with_error(self, *, code: str, description: str, suggested_action: str):
        self._stop_backend_navigation_if_needed()
        self._publish_move_stop(reason=code)
        self._active_waypoint_index = None
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._hover_until = None
        self._action_runner.reset()
        self._runtime.stage = 'paused'
        self._runtime.status = 'exception'
        self._runtime.event = 'task_paused'
        self._runtime.error = self._make_error(
            code=code,
            description=description,
            suggested_action=suggested_action,
        )
        self._publish_task_progress()
        self._write_state_fields()

    def _stop_backend_navigation_if_needed(self):
        if self._task is None and self._outdoor_mission is None:
            return
        if not self._dispatcher.has_active_goal():
            return
        map_name = self._task.map_name if self._task is not None else self._outdoor_mission.map_name
        self._dispatcher.stop_all_tasks(map_name=map_name)

    def _publish_move_stop(self, *, reason: str):
        if self._move_gateway is None:
            return
        try:
            self._move_gateway.stop(reason=reason)
        except Exception as exc:
            self._logger.warning(f'failed to publish uran_move stop action: {exc}')

    def _publish_task_action_event(self, action_name: str, payload: Dict[str, Any]):
        self._publish_uplink(
            data_type='task_action_result',
            payload=payload,
            urgent=not bool(payload.get('success', False)),
        )

    def _start_recording_if_needed(self):
        if self._task is None or not self._task.media_record or self._recording_active:
            return
        channel_id = str(self._media_actions_cfg.get('record_channel_id', 'cyberdog_main'))
        self._recording_active = True
        self._publish_media_control(
            action='record_start',
            channel_id=channel_id,
        )

    def _stop_recording_if_needed(self):
        if not self._recording_active:
            return
        self._recording_active = False
        channel_id = str(self._media_actions_cfg.get('record_channel_id', 'cyberdog_main'))
        self._publish_media_control(
            action='record_stop',
            channel_id=channel_id,
        )

    def _set_runtime_event(self, event: str, *, publish_now: bool):
        self._runtime.event = event
        if publish_now:
            self._publish_task_progress()
            self._write_state_fields()

    def _make_error(
        self,
        *,
        code: str,
        description: str,
        suggested_action: str,
        severity: str = 'warning',
    ) -> Dict[str, Any]:
        return {
            'code': code,
            'severity': severity,
            'description': description,
            'suggested_action': suggested_action,
        }

    def _publish_periodic_progress(self, *, monotonic_s: float):
        if monotonic_s - self._last_periodic_report_ts < self._progress_report_interval_s:
            return
        self._publish_task_progress()

    def _publish_task_progress(self):
        self._publish_uplink(
            data_type='task_progress',
            payload=self.build_status_payload(),
            urgent=(
                self._runtime.stage in TERMINAL_TASK_STAGES or
                self._runtime.status == 'exception'
            ),
        )
        self._last_periodic_report_ts = time.monotonic()

    def _write_state_fields(self):
        task_id = self._runtime.task_id if self._runtime.task_id else ''
        self._write_state('task_id', task_id)
        self._write_state('task_stage', self._runtime.stage)
