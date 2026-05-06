from collections import deque
from typing import Any, Dict, Optional


class CyberdogAlgorithmManagerAdapter:
    """Thin adapter around CyberDog2 algorithm_manager interfaces."""

    def __init__(self, node, config: Dict[str, Any]):
        try:
            from protocol.action import Navigation
            from protocol.srv import AlgoTaskStatus, StopAlgoTask
            from rclpy.action import ActionClient
        except ImportError as exc:
            raise RuntimeError(
                'protocol / action interfaces are unavailable. '
                'Source cyberdog_ws/install/setup.bash before starting uran_autotask.'
            ) from exc

        self._node = node
        self._Navigation = Navigation
        self._AlgoTaskStatus = AlgoTaskStatus
        self._StopAlgoTask = StopAlgoTask
        self._ActionClient = ActionClient
        self._ready_timeout_s = float(config.get('ready_timeout_s', 5.0))
        self._status_poll_interval_s = float(config.get('status_poll_interval_s', 1.0))
        raw_namespace = str(config.get('namespace', '')).strip()
        self._auto_namespace = raw_namespace == '' or raw_namespace.lower() == 'auto'
        self._namespace = '' if raw_namespace.lower() == 'auto' else raw_namespace.strip('/')
        self._start_action_relative_name = str(config.get('start_action_name', 'start_algo_task'))
        self._stop_service_relative_name = str(config.get('stop_service_name', 'stop_algo_task'))
        self._status_service_relative_name = str(config.get('status_service_name', 'algo_task_status'))

        if self._auto_namespace and not self._namespace:
            self._namespace = self._detect_namespace_from_graph()

        self._action_name = ''
        self._stop_service_name = ''
        self._status_service_name = ''
        self._action_client = None
        self._stop_client = None
        self._status_client = None
        self._create_clients()

        self._goal_state = 'idle'
        self._goal_result_code: Optional[int] = None
        self._goal_status_code: Optional[int] = None
        self._last_feedback_code: Optional[int] = None
        self._last_feedback_msg: str = ''
        self._active_goal_handle = None
        self._terminal_events = deque()
        self._manager_status_code: Optional[int] = None
        self._manager_status_name: str = ''
        self._manager_code: Optional[int] = None
        self._last_status_poll_ts = 0.0
        self._status_request_pending = False

    def _resolve_name(self, relative_name: str) -> str:
        relative_name = str(relative_name).strip().lstrip('/')
        if not self._namespace:
            return '/' + relative_name
        return '/' + self._namespace + '/' + relative_name

    def _create_clients(self):
        self._action_name = self._resolve_name(self._start_action_relative_name)
        self._stop_service_name = self._resolve_name(self._stop_service_relative_name)
        self._status_service_name = self._resolve_name(self._status_service_relative_name)
        self._action_client = self._ActionClient(self._node, self._Navigation, self._action_name)
        self._stop_client = self._node.create_client(self._StopAlgoTask, self._stop_service_name)
        self._status_client = self._node.create_client(self._AlgoTaskStatus, self._status_service_name)

    def _destroy_clients(self):
        for client in (self._action_client, self._stop_client, self._status_client):
            if client is None:
                continue
            try:
                client.destroy()
            except Exception:
                pass
        self._action_client = None
        self._stop_client = None
        self._status_client = None

    def _detect_namespace_from_graph(self) -> str:
        """Infer the CyberDog namespace from algorithm_manager services."""
        try:
            services = self._node.get_service_names_and_types()
        except Exception:
            return ''

        expected = {
            self._stop_service_relative_name.strip('/'): 'protocol/srv/StopAlgoTask',
            self._status_service_relative_name.strip('/'): 'protocol/srv/AlgoTaskStatus',
        }
        scores: Dict[str, int] = {}
        for service_name, service_types in services:
            service_name = str(service_name)
            service_types = set(service_types or [])
            for suffix, expected_type in expected.items():
                marker = '/' + suffix
                if not service_name.endswith(marker):
                    continue
                if expected_type not in service_types:
                    continue
                namespace = service_name[:-len(marker)].strip('/')
                if not namespace:
                    continue
                scores[namespace] = scores.get(namespace, 0) + 1

        if not scores:
            return ''
        namespace = max(scores.items(), key=lambda item: (item[1], len(item[0])))[0]
        if namespace:
            self._node.get_logger().info(
                f'auto-detected CyberDog algorithm namespace: /{namespace}'
            )
        return namespace

    def _refresh_auto_namespace_if_needed(self):
        if not self._auto_namespace or self._namespace:
            return
        namespace = self._detect_namespace_from_graph()
        if not namespace:
            return
        self._namespace = namespace
        self._destroy_clients()
        self._create_clients()

    def _status_name_from_value(self, value: Optional[int]) -> str:
        if value is None:
            return ''
        for attr in dir(self._AlgoTaskStatus.Response):
            if not attr.isupper():
                continue
            raw = getattr(self._AlgoTaskStatus.Response, attr)
            if not isinstance(raw, int):
                continue
            if int(raw) == int(value):
                return attr.lower()
        return f'unknown_{value}'

    def wait_until_ready(self) -> bool:
        self._refresh_auto_namespace_if_needed()
        return self._action_client.wait_for_server(timeout_sec=self._ready_timeout_s)

    def has_active_goal(self) -> bool:
        return self._goal_state in {'waiting_accept', 'active'}

    def navigate_to_pose(self, pose_stamped, *, map_name: str, outdoor: bool) -> bool:
        return self.send_raw_navigation_goal(
            nav_type=self._Navigation.Goal.NAVIGATION_TYPE_START_AB,
            poses=[pose_stamped],
            map_name=map_name,
            outdoor=outdoor,
        )

    def send_raw_navigation_goal(self, *, nav_type: int, poses, map_name: str, outdoor: bool) -> bool:
        if self.has_active_goal():
            self._node.get_logger().warning('algorithm_manager already has an active goal')
            return False
        if not self.wait_until_ready():
            self._node.get_logger().error(
                f'algorithm_manager action server not ready: {self._action_name}'
            )
            return False

        goal = self._Navigation.Goal()
        goal.nav_type = int(nav_type)
        goal.poses = list(poses or [])
        goal.label_id = ''
        goal.map_name = map_name or ''
        goal.relative_pos = getattr(self._Navigation.Goal, 'TRACING_AUTO', 200)
        goal.keep_distance = 0.0
        goal.outdoor = bool(outdoor)
        goal.object_tracking = False

        self._goal_state = 'waiting_accept'
        self._goal_result_code = None
        self._goal_status_code = None
        self._last_feedback_code = None
        self._last_feedback_msg = ''

        future = self._action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)
        return True

    def stop_all_tasks(self, *, map_name: str = '') -> bool:
        if not self._stop_client.wait_for_service(timeout_sec=self._ready_timeout_s):
            self._node.get_logger().error(
                f'stop_algo_task service not ready: {self._stop_service_name}'
            )
            return False

        request = self._StopAlgoTask.Request()
        request.task_id = self._StopAlgoTask.Request.ALGO_TASK_ALL
        request.map_name = map_name or ''

        future = self._stop_client.call_async(request)
        future.add_done_callback(self._on_stop_response)
        self._goal_state = 'stopping'
        return True

    def refresh_status(self, *, force: bool = False):
        now = self._node.get_clock().now().nanoseconds / 1e9
        if not force and now - self._last_status_poll_ts < self._status_poll_interval_s:
            return
        if self._status_request_pending:
            return
        if not self._status_client.wait_for_service(timeout_sec=0.0):
            return

        request = self._AlgoTaskStatus.Request()
        future = self._status_client.call_async(request)
        future.add_done_callback(self._on_status_response)
        self._status_request_pending = True
        self._last_status_poll_ts = now

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._goal_state = 'failed'
            self._terminal_events.append({
                'state': 'failed',
                'reason': 'goal_request_error',
                'description': str(exc),
            })
            return

        if goal_handle is None or not goal_handle.accepted:
            self._goal_state = 'rejected'
            self._terminal_events.append({
                'state': 'rejected',
                'reason': 'goal_rejected',
                'description': 'algorithm_manager rejected the navigation goal',
            })
            return

        self._active_goal_handle = goal_handle
        self._goal_state = 'active'
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self._last_feedback_code = int(feedback.feedback_code)
        self._last_feedback_msg = str(feedback.feedback_msg)

    def _on_goal_result(self, future):
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._goal_state = 'failed'
            self._terminal_events.append({
                'state': 'failed',
                'reason': 'result_error',
                'description': str(exc),
            })
            self._active_goal_handle = None
            return

        result = wrapped_result.result
        self._goal_status_code = int(wrapped_result.status)
        self._goal_result_code = int(result.result)

        success_code = self._Navigation.Result.NAVIGATION_RESULT_TYPE_SUCCESS
        cancel_code = self._Navigation.Result.NAVIGATION_RESULT_TYPE_CANCEL

        if self._goal_result_code == success_code:
            self._goal_state = 'succeeded'
        elif self._goal_result_code == cancel_code:
            self._goal_state = 'canceled'
        else:
            self._goal_state = 'failed'

        self._terminal_events.append({
            'state': self._goal_state,
            'result_code': self._goal_result_code,
            'status_code': self._goal_status_code,
            'feedback_code': self._last_feedback_code,
            'feedback_msg': self._last_feedback_msg,
        })
        self._active_goal_handle = None

    def _on_stop_response(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self._terminal_events.append({
                'state': 'failed',
                'reason': 'stop_service_error',
                'description': str(exc),
            })
            self._goal_state = 'failed'
            self._active_goal_handle = None
            return

        if int(response.result) == int(response.SUCCESS):
            self._goal_state = 'stopped'
            self._terminal_events.append({
                'state': 'stopped',
                'reason': 'stop_service_success',
            })
        else:
            self._goal_state = 'failed'
            self._terminal_events.append({
                'state': 'failed',
                'reason': 'stop_service_failed',
                'code': int(response.code),
            })
        self._active_goal_handle = None

    def _on_status_response(self, future):
        self._status_request_pending = False
        try:
            response = future.result()
        except Exception:
            return

        self._manager_status_code = int(response.status)
        self._manager_status_name = self._status_name_from_value(self._manager_status_code)
        self._manager_code = int(response.code)

    def take_terminal_event(self) -> Optional[Dict[str, Any]]:
        if not self._terminal_events:
            return None
        return self._terminal_events.popleft()

    def get_execution_snapshot(self) -> Dict[str, Any]:
        return {
            'namespace': self._namespace,
            'auto_namespace': self._auto_namespace,
            'action_name': self._action_name,
            'stop_service_name': self._stop_service_name,
            'status_service_name': self._status_service_name,
            'goal_state': self._goal_state,
            'goal_result_code': self._goal_result_code,
            'goal_status_code': self._goal_status_code,
            'last_feedback_code': self._last_feedback_code,
            'last_feedback_msg': self._last_feedback_msg,
            'manager_status_code': self._manager_status_code,
            'manager_status_name': self._manager_status_name,
            'manager_code': self._manager_code,
        }

    def destroy(self):
        self._destroy_clients()
