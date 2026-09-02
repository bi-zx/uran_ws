import json
import os
import time
from typing import Any, Dict, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from uran_msgs.msg import MediaCtrlCmd, ModeSwitchCmd, StateField, StateSnapshot, TaskCtrlCmd, UplinkPayload
from uran_srvs.srv import GetTaskStatus

from .adapters import (
    CyberdogAlgorithmManagerAdapter,
    CyberdogCameraCaptureAdapter,
    UranMoveGateway,
)
from .geo_utils import GeoReferenceConfig, MapProjector
from .localization import (
    ClosedLoopManager,
    GeoPoseFuser,
    GpsStatusTracker,
    GpsSupervisor,
    PoseRegistry,
    VisualPoseSupervisor,
    pose_stamped_to_dict,
)
from .mission import MissionManager
from .navigation import StraightDriveController
from .navigation.depth_scan_adapter import DepthPointCloudScanAdapter, RigidTransform
from .task_models import TERMINAL_TASK_STAGES


def _load_yaml(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as file:
        return yaml.safe_load(file) or {}


def _nested_attr(obj, path: str, default=None):
    value = obj
    for name in str(path).split('.'):
        if value is None or not hasattr(value, name):
            return default
        value = getattr(value, name)
    return value


def _first_attr(obj, paths, default=None):
    for path in paths:
        value = _nested_attr(obj, path, None)
        if value is not None:
            return value
    return default


def _first_float_attr(obj, paths, default=None):
    value = _first_attr(obj, paths, None)
    if value in (None, ''):
        return default
    return float(value)


def _first_int_attr(obj, paths, default=0) -> int:
    value = _first_attr(obj, paths, None)
    if value in (None, ''):
        return int(default)
    return int(value)


def _message_timestamp_ns(msg, fallback_ns: int = 0) -> int:
    sec = _first_attr(msg, ('sec', 'stamp.sec', 'header.stamp.sec', 'timestamp.sec'), None)
    nanosec = _first_attr(
        msg,
        ('nanosec', 'nsec', 'stamp.nanosec', 'stamp.nsec', 'header.stamp.nanosec',
         'header.stamp.nsec', 'timestamp.nanosec', 'timestamp.nsec'),
        None,
    )
    if sec is not None:
        return int(sec) * 1_000_000_000 + int(nanosec or 0)

    timestamp_ns = _first_attr(msg, ('timestamp_ns', 'time_ns'), None)
    if timestamp_ns not in (None, ''):
        return int(timestamp_ns)

    timestamp_ms = _first_attr(msg, ('timestamp_ms', 'time_ms'), None)
    if timestamp_ms not in (None, ''):
        return int(timestamp_ms) * 1_000_000

    return int(fallback_ns)


_POSE_KIND_TO_TOPIC_TYPE = {
    'pose_stamped': 'geometry_msgs/msg/PoseStamped',
    'odometry': 'nav_msgs/msg/Odometry',
    'se3_pose': 'motion_msgs/msg/SE3Pose',
}

_POSE_TOPIC_TYPE_TO_KIND = {
    topic_type: kind for kind, topic_type in _POSE_KIND_TO_TOPIC_TYPE.items()
}


class UranAutotaskNode(Node):
    def __init__(self):
        super().__init__('uran_autotask_node')

        self._share_dir = self._resolve_share_dir()
        node_cfg = self._load_config().get('uran_autotask', {})

        self._loop_interval_s = float(node_cfg.get('loop_interval_s', 0.5))
        self._progress_report_interval_s = float(node_cfg.get('progress_report_interval_s', 2.0))
        self._require_auto_mode = bool(node_cfg.get('require_auto_mode', True))
        self._mission_defaults = dict(node_cfg.get('mission_defaults', {}))
        self._outdoor_goal_resolver_cfg = dict(node_cfg.get('outdoor_goal_resolver', {}))
        self._outdoor_start_alignment_cfg = dict(node_cfg.get('outdoor_start_alignment', {}))
        self._outdoor_pose_aligner_cfg = dict(node_cfg.get('outdoor_pose_aligner', {}))
        self._gps_vo_yaw_aligner_cfg = dict(node_cfg.get('gps_vo_yaw_aligner', {}))
        self._pose_report_cfg = dict(node_cfg.get('pose_report', {}))
        self._pose_report_enabled = bool(self._pose_report_cfg.get('enabled', True))
        self._pose_report_interval_s = max(
            0.1,
            float(self._pose_report_cfg.get('interval_s', 1.0)),
        )
        self._gps_monitor_cfg = dict(node_cfg.get('gps_monitor', {}))
        self._gps_monitor_enabled = bool(self._gps_monitor_cfg.get('enabled', False))
        self._visual_pose_monitor_cfg = dict(node_cfg.get('visual_pose_monitor', {}))
        self._visual_pose_monitor_enabled = bool(
            self._visual_pose_monitor_cfg.get('enabled', False)
        )
        self._media_actions_cfg = dict(node_cfg.get('media_actions', {}))
        self._straight_drive_cfg = dict(node_cfg.get('straight_drive', {}))
        self._obstacle_source = str(
            self._straight_drive_cfg.get('obstacle_source', 'laser_scan') or 'laser_scan'
        ).strip().lower()
        if self._obstacle_source not in {'laser_scan', 'depth_pointcloud'}:
            self.get_logger().warning(
                f'unknown obstacle_source {self._obstacle_source!r}; using laser_scan'
            )
            self._obstacle_source = 'laser_scan'
        self._obstacle_topic = ''
        self._depth_scan_adapter = None
        self._depth_scan_pub = None
        self._last_depth_process_monotonic_s = None
        depth_processing_hz = max(
            1.0,
            float(self._straight_drive_cfg.get('depth_processing_hz', 15.0)),
        )
        self._depth_process_interval_s = 1.0 / depth_processing_hz
        if self._obstacle_source == 'depth_pointcloud':
            self._depth_scan_adapter = DepthPointCloudScanAdapter(
                self._straight_drive_cfg,
                monotonic_getter=time.monotonic,
            )

        self._control_mode = 'manual'
        self._controller = 'cloud'
        self._battery_level: Optional[float] = None
        self._cyberdog2_switch_status = None
        self._failsafe_active = False
        self._device_namespace = ''
        self._protocol_available = False

        self._pose_registry = PoseRegistry()
        self._geo_pose_fuser = GeoPoseFuser(self._pose_report_cfg)
        self._gps_status_tracker = GpsStatusTracker(self._gps_monitor_cfg)
        self._gps_status_topic = ''
        self._tf_buffer = None
        self._tf_listener = None
        self._map_tf_fallback_cfg = dict(self._pose_report_cfg.get('map_tf_fallback') or {})
        self._map_tf_fallback_enabled = bool(self._map_tf_fallback_cfg.get('enabled', False))
        self._visual_tf_fallback_cfg = dict(self._visual_pose_monitor_cfg.get('tf_fallback') or {})
        self._visual_tf_fallback_enabled = bool(self._visual_tf_fallback_cfg.get('enabled', False))
        self._pose_sample_timeout_s = max(
            1.0,
            float(self._pose_report_cfg.get('pose_sample_timeout_s', 3.0)),
        )
        if (
            self._map_tf_fallback_enabled or
            (self._visual_pose_monitor_enabled and self._visual_tf_fallback_enabled) or
            self._obstacle_source == 'depth_pointcloud'
        ):
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pose_subscriptions = []
        self._pose_subscription_keys = set()
        self._pose_channel_state: Dict[str, Dict[str, Any]] = {
            'map': {
                'role': 'map',
                'configured_topic': '',
                'configured_message_type': 'auto',
                'selected_topic': '',
                'selected_message_type': '',
                'source': 'unbound',
                'health': 'unbound',
                'bound': False,
                'attempts': 0,
                'publisher_count': 0,
                'sample_count': 0,
                'bound_since_ns': 0,
                'last_sample_timestamp_ns': 0,
                'last_sample_age_s': None,
                'last_sample_frame_id': '',
            },
            'visual': {
                'role': 'local_odom',
                'configured_topic': '',
                'configured_message_type': 'auto',
                'selected_topic': '',
                'selected_message_type': '',
                'source': 'unbound',
                'health': 'unbound',
                'bound': False,
                'attempts': 0,
                'publisher_count': 0,
                'sample_count': 0,
                'bound_since_ns': 0,
                'last_sample_timestamp_ns': 0,
                'last_sample_age_s': None,
                'last_sample_frame_id': '',
            },
        }
        self._projector = MapProjector(
            GeoReferenceConfig.from_dict(node_cfg.get('geo_reference', {}))
        )

        self._backend: Optional[CyberdogAlgorithmManagerAdapter] = None
        backend_cfg = dict(node_cfg.get('cyberdog_backend', {}))
        if backend_cfg.get('enabled', True):
            try:
                self._backend = CyberdogAlgorithmManagerAdapter(self, backend_cfg)
            except RuntimeError as exc:
                self.get_logger().warning(str(exc))

        raw_device_namespace = str(
            self._gps_monitor_cfg.get('namespace') or
            node_cfg.get('camera_capture', {}).get('namespace') or
            backend_cfg.get('namespace', '')
        ).strip()
        if raw_device_namespace == '' or raw_device_namespace.lower() == 'auto':
            self._device_namespace = self._auto_detect_device_namespace()
        else:
            self._device_namespace = raw_device_namespace.strip('/')

        self._camera_capture = None
        camera_cfg = dict(node_cfg.get('camera_capture', {}))
        if 'namespace' not in camera_cfg or not str(camera_cfg.get('namespace', '')).strip():
            camera_cfg['namespace'] = self._device_namespace
        if camera_cfg.get('enabled', True):
            try:
                self._camera_capture = CyberdogCameraCaptureAdapter(self, camera_cfg)
            except RuntimeError as exc:
                self.get_logger().warning(str(exc))

        self._gps_supervisor = GpsSupervisor(self._gps_monitor_cfg) if self._gps_monitor_enabled else None
        self._visual_pose_supervisor = (
            VisualPoseSupervisor(self._visual_pose_monitor_cfg)
            if self._visual_pose_monitor_enabled else None
        )
        self._closed_loop_manager = ClosedLoopManager(
            gps_supervisor=self._gps_supervisor,
            visual_pose_supervisor=self._visual_pose_supervisor,
        )

        self._try_import_protocol_interfaces()
        self._try_import_motion_interfaces()

        self.create_subscription(TaskCtrlCmd, '/uran/core/downlink/task_ctrl', self._cb_task_ctrl, 10)
        self.create_subscription(ModeSwitchCmd, '/uran/core/switch/mode', self._cb_mode_switch, 10)
        self.create_subscription(StateSnapshot, '/uran/core/state/broadcast', self._cb_state_snapshot, 10)

        self._uplink_pub = self.create_publisher(UplinkPayload, '/uran/core/uplink/data', 10)
        self._state_pub = self.create_publisher(StateField, '/uran/core/state/write', 10)
        self._media_ctrl_pub = self.create_publisher(MediaCtrlCmd, '/uran/core/downlink/media_ctrl', 10)
        self._pose_enable_pub = None
        self._move_gateway = UranMoveGateway(self)
        self._straight_drive_controller = StraightDriveController(
            config=self._straight_drive_cfg,
            move_gateway=self._move_gateway,
            pose_getter=self._pose_registry.latest_visual_pose,
            map_pose_getter=self._local_navigation_pose,
            now_ns_getter=self._now_ns,
            logger=self.get_logger(),
            control_state_getter=self._motion_control_state,
            monotonic_getter=time.monotonic,
        )

        self._configure_pose_related_subscriptions()
        self._configure_straight_drive_subscriptions()

        self._mission_manager = MissionManager(
            logger=self.get_logger(),
            backend=self._backend,
            camera_capture=self._camera_capture,
            projector=self._projector,
            closed_loop_manager=self._closed_loop_manager,
            pose_registry=self._pose_registry,
            control_mode_getter=lambda: self._control_mode,
            controller_getter=lambda: self._controller,
            battery_level_getter=lambda: self._battery_level,
            now_ns_getter=self._now_ns,
            position_getter=self._current_fused_position,
            stamp_getter=lambda: self.get_clock().now().to_msg(),
            publish_uplink=self._publish_uplink_message,
            write_state=self._write_state,
            write_motion_control_lock=self._write_motion_control_lock,
            publish_media_control=self._publish_media_control,
            set_map_pose_reporting_enabled=self._set_map_pose_reporting_enabled,
            move_gateway=self._move_gateway,
            straight_drive_controller=self._straight_drive_controller,
            extra_status_getter=self._build_extra_status_payload,
            mission_defaults=self._mission_defaults,
            outdoor_goal_resolver_cfg=self._outdoor_goal_resolver_cfg,
            outdoor_pose_aligner_cfg=self._outdoor_pose_aligner_cfg,
            gps_vo_yaw_aligner_cfg=self._gps_vo_yaw_aligner_cfg,
            outdoor_start_alignment_cfg=self._outdoor_start_alignment_cfg,
            progress_report_interval_s=self._progress_report_interval_s,
            require_auto_mode=self._require_auto_mode,
            media_actions_cfg=self._media_actions_cfg,
        )

        self.create_service(GetTaskStatus, '/uran/autotask/status', self._srv_get_task_status)
        self.create_timer(self._loop_interval_s, self._cb_task_tick)
        if bool(self._straight_drive_cfg.get('enabled', True)):
            control_hz = max(5.0, float(self._straight_drive_cfg.get('control_hz', 20.0)))
            self.create_timer(1.0 / control_hz, self._cb_straight_drive_tick)
        if self._pose_report_enabled:
            self.create_timer(self._pose_report_interval_s, self._cb_pose_report_tick)

        self.get_logger().info('uran_autotask_node ready')

    def _resolve_share_dir(self) -> str:
        try:
            return get_package_share_directory('uran_autotask')
        except Exception:
            return os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

    def _load_config(self) -> dict:
        return _load_yaml(os.path.join(self._share_dir, 'config', 'autotask.yaml'))

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _auto_detect_device_namespace(self) -> str:
        """Infer the CyberDog namespace from known robot topics/services."""
        scores: Dict[str, int] = {}

        try:
            topic_names_and_types = self.get_topic_names_and_types()
        except Exception:
            topic_names_and_types = []
        expected_topic_types = {
            'dog_pose': {
                'geometry_msgs/msg/PoseStamped',
                'motion_msgs/msg/SE3Pose',
            },
            'gps_payload': {'protocol/msg/GpsPayload'},
            'odom_out': {'nav_msgs/msg/Odometry'},
            'odometry': {'nav_msgs/msg/Odometry'},
            'pose_filtered': {'geometry_msgs/msg/PoseStamped'},
        }
        for topic_name, topic_types in topic_names_and_types:
            topic_name = str(topic_name)
            type_set = set(topic_types or [])
            for suffix, expected_types in expected_topic_types.items():
                marker = '/' + suffix
                if not topic_name.endswith(marker):
                    continue
                if expected_types and not type_set.intersection(expected_types):
                    continue
                namespace = topic_name[:-len(marker)].strip('/')
                if namespace:
                    scores[namespace] = scores.get(namespace, 0) + 1

        try:
            service_names_and_types = self.get_service_names_and_types()
        except Exception:
            service_names_and_types = []
        expected_service_types = {
            'stop_algo_task': {'protocol/srv/StopAlgoTask'},
            'algo_task_status': {'protocol/srv/AlgoTaskStatus'},
            'motion_result_cmd': {'protocol/srv/MotionResultCmd'},
        }
        for service_name, service_types in service_names_and_types:
            service_name = str(service_name)
            type_set = set(service_types or [])
            for suffix, expected_types in expected_service_types.items():
                marker = '/' + suffix
                if not service_name.endswith(marker):
                    continue
                if expected_types and not type_set.intersection(expected_types):
                    continue
                namespace = service_name[:-len(marker)].strip('/')
                if namespace:
                    scores[namespace] = scores.get(namespace, 0) + 2

        if not scores:
            return ''

        namespace = max(scores.items(), key=lambda item: (item[1], len(item[0])))[0]
        if namespace:
            self.get_logger().info(f'auto-detected CyberDog device namespace: /{namespace}')
        return namespace

    def _resolve_name(self, relative_name: str) -> str:
        relative_name = str(relative_name or '').strip()
        if not relative_name:
            return ''
        if relative_name.startswith('/'):
            return relative_name
        if not self._device_namespace:
            return '/' + relative_name.lstrip('/')
        return '/' + self._device_namespace + '/' + relative_name.lstrip('/')

    def _try_import_protocol_interfaces(self):
        try:
            from protocol.msg import GpsPayload
            self._GpsPayload = GpsPayload
            self._protocol_available = True
        except ImportError as exc:
            if self._gps_monitor_enabled or self._pose_report_enabled:
                self.get_logger().warning(
                    f'protocol.msg not available ({exc}), GPS input unavailable'
                )

    def _try_import_motion_interfaces(self):
        self._SE3Pose = None
        try:
            from motion_msgs.msg import SE3Pose
            self._SE3Pose = SE3Pose
        except ImportError:
            self._SE3Pose = None

    def _configure_pose_related_subscriptions(self):
        needs_map_pose = (
            self._gps_monitor_enabled or
            self._pose_report_enabled or
            self._visual_pose_monitor_enabled
        )
        needs_gps = self._gps_monitor_enabled or self._pose_report_enabled
        map_pose_value = self._pose_report_cfg.get('map_pose_topic', None)
        if map_pose_value is None:
            map_pose_value = self._gps_monitor_cfg.get('map_pose_topic', 'dog_pose')
        map_pose_relative = str(map_pose_value or '').strip()
        map_pose_message_type = str(
            self._pose_report_cfg.get('map_pose_message_type') or
            self._gps_monitor_cfg.get('map_pose_message_type', 'auto')
        )
        self._pose_channel_state['map']['configured_topic'] = map_pose_relative
        self._pose_channel_state['map']['configured_message_type'] = map_pose_message_type

        if needs_gps and self._protocol_available:
            gps_topic = self._resolve_name(self._gps_monitor_cfg.get('gps_topic', 'gps_payload'))
            self.create_subscription(self._GpsPayload, gps_topic, self._cb_gps_payload, 10)
        if needs_gps:
            status_topic = self._resolve_name(self._gps_monitor_cfg.get('status_topic', '/ubx/status'))
            if status_topic:
                self._gps_status_topic = status_topic
                self.create_subscription(String, status_topic, self._cb_gps_status, 10)

        if needs_map_pose:
            self._ensure_pose_subscription(
                role='map',
                relative_topic=map_pose_relative,
                message_type=map_pose_message_type,
                candidates=['dog_pose'],
            )

        if self._gps_monitor_enabled:
            pose_enable_topic = self._resolve_name(
                self._gps_monitor_cfg.get('pose_enable_topic', 'pose_enable')
            )
            self._pose_enable_pub = self.create_publisher(Bool, pose_enable_topic, 10)

        if self._visual_pose_monitor_enabled:
            visual_pose_topic = str(self._visual_pose_monitor_cfg.get('pose_topic', '')).strip()
            visual_pose_message_type = str(
                self._visual_pose_monitor_cfg.get('pose_message_type', 'auto')
            )
            self._pose_channel_state['visual']['configured_topic'] = visual_pose_topic
            self._pose_channel_state['visual']['configured_message_type'] = visual_pose_message_type
            self._ensure_pose_subscription(
                role='visual',
                relative_topic=visual_pose_topic,
                message_type=visual_pose_message_type,
                candidates=[
                    'odom_out',
                    'odometry',
                    'pose_filtered',
                    'odom_slam',
                    'mivins/odometry',
                    'mivins/imuodom_slam',
                    'mivins/reloc_odom',
                ],
            )

    def _configure_straight_drive_subscriptions(self):
        if not bool(self._straight_drive_cfg.get('enabled', True)):
            return
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        if self._obstacle_source == 'depth_pointcloud':
            pointcloud_topic = self._resolve_name(str(
                self._straight_drive_cfg.get(
                    'depth_pointcloud_topic',
                    '/camera/depth/color/points',
                )
            ))
            if not pointcloud_topic:
                return
            self._obstacle_topic = pointcloud_topic
            self.create_subscription(
                PointCloud2,
                pointcloud_topic,
                self._cb_straight_drive_pointcloud,
                qos,
            )
            virtual_scan_topic = str(
                self._straight_drive_cfg.get(
                    'depth_virtual_scan_topic',
                    '/uran/autotask/depth_scan',
                ) or ''
            ).strip()
            if virtual_scan_topic:
                self._depth_scan_pub = self.create_publisher(
                    LaserScan,
                    virtual_scan_topic,
                    qos,
                )
            self.get_logger().info(
                'straight drive obstacle avoidance listening depth point cloud: '
                f'{pointcloud_topic}'
            )
            return

        scan_topic = self._resolve_name(str(self._straight_drive_cfg.get('scan_topic', '/scan')))
        if not scan_topic:
            return
        self._obstacle_topic = scan_topic
        self.create_subscription(LaserScan, scan_topic, self._cb_straight_drive_scan, qos)
        self.get_logger().info(f'straight drive obstacle avoidance listening scan: {scan_topic}')

    def _ensure_pose_subscription(
        self,
        *,
        role: str,
        relative_topic: str,
        message_type: str,
        candidates,
    ):
        state = self._pose_channel_state[role]
        if state.get('bound', False):
            return

        self._reset_pose_channel_binding(role)
        state['attempts'] = int(state.get('attempts', 0)) + 1
        chosen_topic = ''
        source = 'configured'
        if relative_topic:
            chosen_topic = relative_topic
            publisher_count = self._pose_publisher_count(chosen_topic, message_type)
            if publisher_count <= 0:
                state['selected_topic'] = self._resolve_name(chosen_topic)
                state['selected_message_type'] = ''
                state['source'] = 'awaiting_publisher'
                state['health'] = 'awaiting_publisher'
                state['publisher_count'] = 0
                state['bound'] = False
                return
        else:
            chosen_topic = self._auto_detect_pose_topic(
                candidates=candidates,
                preferred_message_type=message_type,
            )
            source = 'auto_detected' if chosen_topic else 'unbound'

        if not chosen_topic:
            state['selected_topic'] = ''
            state['selected_message_type'] = ''
            state['source'] = source
            state['health'] = source
            state['bound'] = False
            return

        actual_kind = self._resolve_message_kind_for_topic(
            relative_topic=chosen_topic,
            preferred_message_type=message_type,
        )
        if not actual_kind:
            state['selected_topic'] = self._resolve_name(chosen_topic)
            state['selected_message_type'] = ''
            state['source'] = 'awaiting_type'
            state['health'] = 'awaiting_type'
            state['bound'] = False
            return
        if not self._create_pose_subscriptions(
            relative_topic=chosen_topic,
            message_type=actual_kind or message_type,
            role=role,
        ):
            state['selected_topic'] = ''
            state['selected_message_type'] = ''
            state['source'] = 'unbound'
            state['health'] = 'unbound'
            state['bound'] = False
            return

        state['selected_topic'] = self._resolve_name(chosen_topic)
        state['selected_message_type'] = actual_kind or str(message_type or 'auto')
        state['source'] = source
        state['health'] = 'awaiting_sample'
        state['bound'] = True
        state['publisher_count'] = self._pose_publisher_count(chosen_topic, actual_kind)
        state['bound_since_ns'] = self._now_ns()
        self.get_logger().info(
            f'bound {role} pose channel: topic={state["selected_topic"]}, '
            f'type={state["selected_message_type"]}, source={state["source"]}'
        )

    def _auto_detect_pose_topic(self, *, candidates, preferred_message_type: str) -> str:
        available_topics = self._topic_type_map()
        for candidate in candidates:
            resolved = self._resolve_name(candidate)
            topic_types = available_topics.get(resolved, [])
            if self._pose_publisher_count(candidate, preferred_message_type) <= 0:
                continue
            if self._topic_matches_message_type(topic_types, preferred_message_type):
                return candidate
        return ''

    def _pose_publisher_count(self, relative_topic: str, preferred_message_type: str) -> int:
        topic_name = self._resolve_name(relative_topic)
        try:
            publisher_infos = self.get_publishers_info_by_topic(topic_name)
        except Exception:
            return 0

        normalized = str(preferred_message_type or 'auto').strip().lower()
        expected_type = _POSE_KIND_TO_TOPIC_TYPE.get(normalized)
        count = 0
        for info in publisher_infos:
            topic_type = str(getattr(info, 'topic_type', '') or '')
            if normalized == 'auto' and topic_type in _POSE_TOPIC_TYPE_TO_KIND:
                count += 1
            if expected_type and topic_type == expected_type:
                count += 1
        return count

    def _topic_type_map(self) -> Dict[str, Any]:
        return {
            str(topic_name): list(topic_types)
            for topic_name, topic_types in self.get_topic_names_and_types()
        }

    def _topic_matches_message_type(self, topic_types, preferred_message_type: str) -> bool:
        if not topic_types:
            return False
        normalized = str(preferred_message_type or 'auto').strip().lower()
        if normalized == 'auto':
            return any(item in topic_types for item in _POSE_TOPIC_TYPE_TO_KIND)
        expected = _POSE_KIND_TO_TOPIC_TYPE.get(normalized)
        return bool(expected and expected in topic_types)

    def _resolve_message_kind_for_topic(self, *, relative_topic: str, preferred_message_type: str) -> str:
        available_topics = self._topic_type_map()
        topic_types = available_topics.get(self._resolve_name(relative_topic), [])
        normalized = str(preferred_message_type or 'auto').strip().lower()
        if not topic_types:
            if normalized == 'auto':
                return ''
            return normalized
        if normalized != 'auto' and self._topic_matches_message_type(topic_types, preferred_message_type):
            return normalized
        for topic_type, kind in _POSE_TOPIC_TYPE_TO_KIND.items():
            if topic_type in topic_types:
                return kind
        return ''

    def _create_pose_subscriptions(self, *, relative_topic: str, message_type: str, role: str):
        topic_name = self._resolve_name(relative_topic)
        normalized = str(message_type or 'auto').strip().lower()
        requested = []
        if normalized == 'pose_stamped':
            requested = ['pose_stamped']
        elif normalized == 'odometry':
            requested = ['odometry']
        elif normalized == 'se3_pose':
            requested = ['se3_pose']
        else:
            requested = ['pose_stamped', 'odometry', 'se3_pose']

        created = False
        for kind in requested:
            subscription_key = (role, topic_name, kind)
            if subscription_key in self._pose_subscription_keys:
                created = True
                continue
            subscription = self._create_pose_subscription(topic_name=topic_name, kind=kind, role=role)
            if subscription is not None:
                self._pose_subscriptions.append(subscription)
                self._pose_subscription_keys.add(subscription_key)
                created = True
        return created

    def _create_pose_subscription(self, *, topic_name: str, kind: str, role: str):
        if kind == 'pose_stamped':
            try:
                return self.create_subscription(
                    PoseStamped,
                    topic_name,
                    lambda msg, role=role: self._cb_pose_stamped(role, msg),
                    self._pose_qos_for_topic(topic_name=topic_name, kind=kind),
                )
            except Exception as exc:
                self.get_logger().warning(
                    f'failed to subscribe {topic_name} as PoseStamped: {exc}'
                )
                return None
        if kind == 'odometry':
            try:
                return self.create_subscription(
                    Odometry,
                    topic_name,
                    lambda msg, role=role: self._cb_odometry(role, msg),
                    self._pose_qos_for_topic(topic_name=topic_name, kind=kind),
                )
            except Exception as exc:
                self.get_logger().warning(
                    f'failed to subscribe {topic_name} as Odometry: {exc}'
                )
                return None
        if kind == 'se3_pose':
            if self._SE3Pose is None:
                self.get_logger().warning(
                    f'motion_msgs/SE3Pose unavailable, skip subscription for {topic_name}'
                )
                return None
            try:
                return self.create_subscription(
                    self._SE3Pose,
                    topic_name,
                    lambda msg, role=role: self._cb_se3_pose(role, msg),
                    self._pose_qos_for_topic(topic_name=topic_name, kind=kind),
                )
            except Exception as exc:
                self.get_logger().warning(
                    f'failed to subscribe {topic_name} as SE3Pose: {exc}'
                )
                return None
        return None

    def _pose_qos_for_topic(self, *, topic_name: str, kind: str):
        reliability = ReliabilityPolicy.RELIABLE
        durability = DurabilityPolicy.VOLATILE
        expected_topic_type = _POSE_KIND_TO_TOPIC_TYPE.get(kind, '')

        try:
            publisher_infos = self.get_publishers_info_by_topic(topic_name)
        except Exception:
            publisher_infos = []

        for info in publisher_infos:
            topic_type = str(getattr(info, 'topic_type', '') or '')
            if expected_topic_type and topic_type != expected_topic_type:
                continue
            offered_reliability = getattr(info, 'qos_profile', None)
            offered_reliability = getattr(offered_reliability, 'reliability', None)
            if offered_reliability == ReliabilityPolicy.BEST_EFFORT:
                reliability = ReliabilityPolicy.BEST_EFFORT
                break

        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=reliability,
            durability=durability,
        )

    def _cb_mode_switch(self, msg: ModeSwitchCmd):
        self._control_mode = msg.control_mode
        self._controller = msg.controller
        if hasattr(self, '_mission_manager'):
            self._mission_manager.handle_control_state_change()

    def _cb_state_snapshot(self, msg: StateSnapshot):
        try:
            fields = json.loads(msg.fields_json)
        except Exception as exc:
            self.get_logger().warning(f'failed to parse state snapshot: {exc}')
            return

        try:
            if 'battery_level' in fields:
                self._battery_level = float(fields['battery_level'])
        except Exception:
            pass

        if 'cyberdog2_switch_status' in fields:
            value = fields.get('cyberdog2_switch_status')
            try:
                self._cyberdog2_switch_status = int(value)
            except (TypeError, ValueError):
                self._cyberdog2_switch_status = value
        if 'failsafe_active' in fields:
            value = fields.get('failsafe_active')
            self._failsafe_active = (
                str(value).strip().lower() in {'1', 'true', 'yes', 'on'}
                if isinstance(value, str) else bool(value)
            )

        mode_changed = False
        if 'control_mode' in fields:
            control_mode = str(fields.get('control_mode') or '')
            if control_mode and control_mode != self._control_mode:
                self._control_mode = control_mode
                mode_changed = True
        if 'current_controller' in fields:
            controller = str(fields.get('current_controller') or '')
            if controller and controller != self._controller:
                self._controller = controller
                mode_changed = True
        if mode_changed and hasattr(self, '_mission_manager'):
            self._mission_manager.handle_control_state_change()

        position = fields.get('position')
        if isinstance(position, dict):
            self._pose_registry.update_cloud_position(position)

    def _cb_gps_payload(self, msg):
        try:
            lat = _first_float_attr(msg, ('lat', 'latitude'))
            lon = _first_float_attr(msg, ('lon', 'lng', 'longitude'))
            if lat is None or lon is None:
                self.get_logger().warning(
                    'gps payload has no lat/lon fields; ignoring this sample'
                )
                return
            self._pose_registry.update_gps_payload(
                lat=lat,
                lon=lon,
                alt=_first_float_attr(msg, ('alt', 'height', 'altitude'), 0.0),
                fix_type=_first_int_attr(msg, ('fix_type', 'fix', 'status'), 0),
                num_sv=_first_int_attr(msg, ('num_sv', 'satellites_used', 'satellite_num'), 0),
                timestamp_ns=_message_timestamp_ns(msg, fallback_ns=self._now_ns()),
            )
        except Exception as exc:
            self.get_logger().warning(f'failed to parse gps payload: {exc}')

    def _cb_gps_status(self, msg: String):
        self._gps_status_tracker.update_from_json(
            msg.data,
            received_timestamp_ns=self._now_ns(),
        )

    def _cb_straight_drive_scan(self, msg: LaserScan):
        if self._straight_drive_controller is not None:
            self._straight_drive_controller.update_scan(
                msg,
                monotonic_s=time.monotonic(),
            )

    def _cb_straight_drive_pointcloud(self, msg: PointCloud2):
        adapter = self._depth_scan_adapter
        if adapter is None or self._straight_drive_controller is None:
            return

        now = time.monotonic()
        if (
            self._last_depth_process_monotonic_s is not None and
            now - self._last_depth_process_monotonic_s < self._depth_process_interval_s
        ):
            adapter.record_skipped()
            return
        self._last_depth_process_monotonic_s = now

        source_frame = str(msg.header.frame_id or '').strip('/')
        target_frame = str(adapter.target_frame or '').strip('/')
        if not source_frame:
            adapter.record_received('')
            adapter.record_error(
                'point cloud frame_id is empty',
                status='frame_id_missing',
            )
            return

        if source_frame == target_frame:
            transform = RigidTransform.identity()
        else:
            if self._tf_buffer is None:
                adapter.record_received(source_frame)
                adapter.record_error(
                    f'TF buffer is unavailable for {source_frame} -> {target_frame}',
                    status='tf_unavailable',
                )
                return
            try:
                transform_msg = self._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                )
                transform = RigidTransform.from_ros(transform_msg)
            except Exception as exc:
                adapter.record_received(source_frame)
                adapter.record_error(
                    f'TF {source_frame} -> {target_frame} unavailable: {exc}',
                    status='tf_unavailable',
                )
                return

        try:
            scan = adapter.convert(
                msg,
                transform,
                monotonic_s=now,
            )
        except Exception as exc:
            self.get_logger().warning(f'failed to convert depth point cloud: {exc}')
            return

        self._straight_drive_controller.update_scan(
            scan,
            monotonic_s=now,
        )
        self._publish_depth_virtual_scan(scan, msg)

    def _publish_depth_virtual_scan(self, scan, source_msg: PointCloud2):
        if self._depth_scan_pub is None:
            return
        msg = LaserScan()
        msg.header.stamp = source_msg.header.stamp
        msg.header.frame_id = str(
            self._depth_scan_adapter.target_frame if self._depth_scan_adapter else 'base_link'
        )
        msg.angle_min = float(scan.angle_min)
        msg.angle_max = float(scan.angle_max)
        msg.angle_increment = float(scan.angle_increment)
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = float(scan.range_min)
        msg.range_max = float(scan.range_max)
        msg.ranges = list(scan.ranges)
        msg.intensities = []
        self._depth_scan_pub.publish(msg)

    def _cb_pose_stamped(self, role: str, msg: PoseStamped):
        payload = pose_stamped_to_dict(msg)
        self._update_pose_by_role(role, payload)

    def _cb_odometry(self, role: str, msg: Odometry):
        pose = msg.pose.pose
        payload = {
            'frame_id': msg.header.frame_id,
            'stamp_sec': int(msg.header.stamp.sec),
            'stamp_nanosec': int(msg.header.stamp.nanosec),
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'z': float(pose.position.z),
            'qx': float(pose.orientation.x),
            'qy': float(pose.orientation.y),
            'qz': float(pose.orientation.z),
            'qw': float(pose.orientation.w),
        }
        self._update_pose_by_role(role, payload)

    def _cb_se3_pose(self, role: str, msg):
        frame_id = ''
        if hasattr(msg, 'frameid'):
            frame = getattr(msg, 'frameid')
            frame_id = str(
                getattr(frame, 'id', '') or
                getattr(frame, 'frame_id', '') or
                getattr(frame, 'name', '')
            )
        stamp = getattr(msg, 'timestamp', None)
        payload = {
            'frame_id': frame_id,
            'stamp_sec': int(getattr(stamp, 'sec', 0)),
            'stamp_nanosec': int(getattr(stamp, 'nanosec', 0)),
            'x': float(getattr(msg, 'position_x', 0.0)),
            'y': float(getattr(msg, 'position_y', 0.0)),
            'z': float(getattr(msg, 'position_z', 0.0)),
            'qx': float(getattr(msg, 'rotation_x', 0.0)),
            'qy': float(getattr(msg, 'rotation_y', 0.0)),
            'qz': float(getattr(msg, 'rotation_z', 0.0)),
            'qw': float(getattr(msg, 'rotation_w', 1.0)),
        }
        self._update_pose_by_role(role, payload)

    def _update_pose_by_role(self, role: str, payload: Dict[str, Any]):
        self._mark_pose_sample(role, payload)
        if role == 'visual':
            self._pose_registry.update_visual_pose(payload)
            return
        self._pose_registry.update_map_pose(payload)

    def _mark_pose_sample(self, role: str, payload: Dict[str, Any]):
        state = self._pose_channel_state.get(role)
        if state is None:
            return
        state['sample_count'] = int(state.get('sample_count', 0)) + 1
        state['last_sample_timestamp_ns'] = int(payload.get('timestamp_ns', 0)) or (
            int(payload.get('stamp_sec', 0)) * 1_000_000_000 +
            int(payload.get('stamp_nanosec', 0))
        )
        state['last_sample_age_s'] = 0.0
        state['last_sample_frame_id'] = str(payload.get('frame_id', ''))
        state['health'] = 'receiving'

    def _cb_task_ctrl(self, msg: TaskCtrlCmd):
        self._mission_manager.handle_task_command(
            action=msg.action,
            task_id=msg.task_id,
            task_type=msg.task_type,
            task_params_json=msg.task_params_json,
        )

    def _cb_task_tick(self):
        self._refresh_pose_channel_health()
        self._retry_unbound_pose_channels()
        self._update_map_pose_from_tf_fallback()
        self._update_visual_pose_from_tf_fallback()
        self._mission_manager.tick(monotonic_s=time.monotonic())

    def _cb_straight_drive_tick(self):
        self._mission_manager.tick_control(monotonic_s=time.monotonic())

    def _cb_pose_report_tick(self):
        now_ns = self._now_ns()
        position = self._current_fused_position(timestamp_ns=now_ns)
        gps_status = self._gps_status_tracker.snapshot(now_ns=now_ns)
        position = dict(position)
        position['gps_status'] = gps_status
        data_type = str(self._pose_report_cfg.get('data_type', 'robot_pose'))
        packet_type = str(self._pose_report_cfg.get('packet_type') or data_type)
        payload = {
            'schema_version': 1,
            'packet_type': packet_type,
            'packet_label': str(
                self._pose_report_cfg.get('packet_label', '机器狗实时位置')
            ),
            'data_type': data_type,
            'robot_id': str(self._pose_report_cfg.get('robot_id', '')),
            'coordinate_system': 'WGS84',
            'position': position,
            'gps_status': gps_status,
            'control_mode': self._control_mode,
            'controller': self._controller,
            'battery_level': self._battery_level,
            'task_id': self._mission_manager.runtime.task_id,
            'task_stage': self._mission_manager.runtime.stage,
            'task_status': self._mission_manager.runtime.status,
            'timestamp_ns': now_ns,
        }
        self._publish_uplink_message(
            data_type=data_type,
            payload=payload,
            urgent=False,
        )

    def _current_fused_position(self, *, timestamp_ns: Optional[int] = None) -> Dict[str, Any]:
        timestamp_ns = self._now_ns() if timestamp_ns is None else int(timestamp_ns)
        alignment_state = (
            self._mission_manager.outdoor_pose_alignment_state()
            if hasattr(self, '_mission_manager') else {}
        )
        return self._geo_pose_fuser.fuse(
            pose_registry=self._pose_registry,
            projector=self._projector,
            alignment_state=alignment_state,
            timestamp_ns=timestamp_ns,
        )

    def _local_navigation_pose(self) -> Optional[Dict[str, Any]]:
        map_pose = self._pose_registry.latest_map_pose()
        if map_pose:
            return map_pose
        if hasattr(self, '_mission_manager'):
            return self._mission_manager.local_navigation_pose()
        return None

    def _refresh_pose_channel_health(self):
        now_ns = self._now_ns()
        for role, state in self._pose_channel_state.items():
            selected_topic = str(state.get('selected_topic', '') or '')
            selected_type = str(state.get('selected_message_type', '') or '')
            if not selected_topic or selected_topic.startswith('tf:'):
                continue

            publisher_count = self._pose_publisher_count(selected_topic, selected_type or 'auto')
            state['publisher_count'] = publisher_count
            last_sample_ns = int(state.get('last_sample_timestamp_ns', 0) or 0)
            if last_sample_ns > 0:
                state['last_sample_age_s'] = max(0.0, float(now_ns - last_sample_ns) / 1_000_000_000.0)

            if publisher_count <= 0:
                state['health'] = 'awaiting_publisher'
                state['bound'] = False
                continue

            if int(state.get('sample_count', 0)) <= 0:
                bound_since_ns = int(state.get('bound_since_ns', 0) or 0)
                wait_s = max(0.0, float(now_ns - bound_since_ns) / 1_000_000_000.0) if bound_since_ns else 0.0
                state['health'] = 'awaiting_sample'
                if wait_s > self._pose_sample_timeout_s:
                    state['bound'] = False
                continue

            state['health'] = 'receiving'

    def _reset_pose_channel_binding(self, role: str):
        state = self._pose_channel_state[role]
        state['selected_topic'] = ''
        state['selected_message_type'] = ''
        state['source'] = 'unbound'
        state['health'] = 'unbound'
        state['bound'] = False
        state['publisher_count'] = 0
        state['sample_count'] = 0
        state['bound_since_ns'] = 0
        state['last_sample_timestamp_ns'] = 0
        state['last_sample_age_s'] = None
        state['last_sample_frame_id'] = ''

    def _retry_unbound_pose_channels(self):
        if not self._pose_channel_state['map'].get('bound', False):
            self._ensure_pose_subscription(
                role='map',
                relative_topic=str(self._pose_channel_state['map'].get('configured_topic', '')),
                message_type=str(self._pose_channel_state['map'].get('configured_message_type', 'auto')),
                candidates=['dog_pose'],
            )
        if self._visual_pose_monitor_enabled and not self._pose_channel_state['visual'].get('bound', False):
            self._ensure_pose_subscription(
                role='visual',
                relative_topic=str(self._pose_channel_state['visual'].get('configured_topic', '')),
                message_type=str(self._pose_channel_state['visual'].get('configured_message_type', 'auto')),
                candidates=[
                    'odom_out',
                    'odometry',
                    'pose_filtered',
                    'odom_slam',
                    'mivins/odometry',
                    'mivins/imuodom_slam',
                    'mivins/reloc_odom',
                ],
            )

    def _lookup_tf_pose(self, *, target_frame: str, source_frame: str) -> Optional[Dict[str, Any]]:
        if self._tf_buffer is None:
            return None

        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            'frame_id': transform.header.frame_id,
            'stamp_sec': int(transform.header.stamp.sec),
            'stamp_nanosec': int(transform.header.stamp.nanosec),
            'x': float(translation.x),
            'y': float(translation.y),
            'z': float(translation.z),
            'qx': float(rotation.x),
            'qy': float(rotation.y),
            'qz': float(rotation.z),
            'qw': float(rotation.w),
        }

    def _update_map_pose_from_tf_fallback(self):
        if not self._map_tf_fallback_enabled:
            return
        if self._pose_channel_state['map'].get('bound', False):
            return

        target_frame = str(self._map_tf_fallback_cfg.get('target_frame', 'map') or 'map')
        source_frame = str(self._map_tf_fallback_cfg.get('source_frame', 'base_link') or 'base_link')
        payload = self._lookup_tf_pose(target_frame=target_frame, source_frame=source_frame)
        if payload is None:
            return

        self._pose_registry.update_map_pose(payload)
        state = self._pose_channel_state['map']
        state['selected_topic'] = f'tf:{target_frame}->{source_frame}'
        state['selected_message_type'] = 'tf2_msgs/msg/TFMessage'
        state['source'] = 'tf_fallback'
        state['health'] = 'receiving'
        state['publisher_count'] = 1
        state['sample_count'] = int(state.get('sample_count', 0)) + 1
        state['last_sample_timestamp_ns'] = int(payload.get('timestamp_ns', 0)) or (
            int(payload.get('stamp_sec', 0)) * 1_000_000_000 +
            int(payload.get('stamp_nanosec', 0))
        )
        state['last_sample_age_s'] = 0.0
        state['last_sample_frame_id'] = str(payload.get('frame_id', ''))

    def _update_visual_pose_from_tf_fallback(self):
        if not self._visual_pose_monitor_enabled or not self._visual_tf_fallback_enabled:
            return
        if self._pose_channel_state['visual'].get('bound', False):
            return

        target_frame = str(self._visual_tf_fallback_cfg.get('target_frame', 'map') or 'map')
        source_frame = str(self._visual_tf_fallback_cfg.get('source_frame', 'base_link') or 'base_link')
        payload = self._lookup_tf_pose(target_frame=target_frame, source_frame=source_frame)
        if payload is None:
            return

        self._pose_registry.update_visual_pose(payload)
        state = self._pose_channel_state['visual']
        state['selected_topic'] = f'tf:{target_frame}->{source_frame}'
        state['selected_message_type'] = 'tf2_msgs/msg/TFMessage'
        state['source'] = 'tf_fallback'
        state['health'] = 'receiving'
        state['publisher_count'] = 1
        state['sample_count'] = int(state.get('sample_count', 0)) + 1
        state['last_sample_timestamp_ns'] = int(payload.get('timestamp_ns', 0)) or (
            int(payload.get('stamp_sec', 0)) * 1_000_000_000 +
            int(payload.get('stamp_nanosec', 0))
        )
        state['last_sample_age_s'] = 0.0
        state['last_sample_frame_id'] = str(payload.get('frame_id', ''))

    def _build_extra_status_payload(self) -> Dict[str, Any]:
        payload = {
            'pose_channels': {
                'map': dict(self._pose_channel_state['map']),
                'visual': dict(self._pose_channel_state['visual']),
            },
            'geo_pose_fuser': self._geo_pose_fuser.state_snapshot(),
            'gps_status': self._gps_status_tracker.snapshot(now_ns=self._now_ns()),
            'gps_status_topic': self._gps_status_topic,
            'pose_history': self._pose_registry.history_summary(),
            'motion_control': self._motion_control_state(),
            'local_odometry_note': (
                'The visual channel stores the selected local odometry source. '
                'It can be visual odometry, leg odometry, fused odometry, or TF fallback.'
            ),
        }
        obstacle_sensor = {
            'source': self._obstacle_source,
            'topic': self._obstacle_topic,
        }
        if self._depth_scan_adapter is not None:
            obstacle_sensor.update(self._depth_scan_adapter.snapshot())
        payload['obstacle_sensor'] = obstacle_sensor
        return payload

    def _motion_control_state(self) -> Dict[str, Any]:
        return {
            'control_mode': self._control_mode,
            'controller': self._controller,
            'bottom_status': self._cyberdog2_switch_status,
            'cyberdog2_switch_status': self._cyberdog2_switch_status,
            'failsafe_active': self._failsafe_active,
            'battery_level': self._battery_level,
        }

    def _set_map_pose_reporting_enabled(self, enabled: bool):
        if not self._gps_monitor_enabled:
            return
        if not self._gps_monitor_cfg.get('auto_enable_map_pose_reporting', False):
            return
        if self._pose_enable_pub is None:
            return
        msg = Bool()
        msg.data = bool(enabled)
        self._pose_enable_pub.publish(msg)

    def _publish_uplink_message(self, *, data_type: str, payload: Dict[str, Any], urgent: bool = False):
        msg = UplinkPayload()
        msg.source_pkg = 'uran_autotask'
        msg.data_type = data_type
        msg.preferred_protocol = ''
        msg.payload_json = json.dumps(payload, ensure_ascii=False)
        msg.urgent = bool(urgent)
        msg.timestamp_ns = int(payload.get('timestamp_ns', self._now_ns()))
        self._uplink_pub.publish(msg)

    def _publish_media_control(self, *, action: str, channel_id: str):
        msg = MediaCtrlCmd()
        msg.action = action
        msg.protocol = ''
        msg.channel_id = channel_id
        msg.signal_json = ''
        msg.timestamp_ns = self._now_ns()
        self._media_ctrl_pub.publish(msg)

    def _write_state(self, field_name: str, value: Any):
        msg = StateField()
        msg.field_name = field_name
        msg.value_json = json.dumps(value, ensure_ascii=False)
        msg.persistent = False
        msg.urgent = (
            self._mission_manager.runtime.stage in TERMINAL_TASK_STAGES or
            self._mission_manager.runtime.status == 'exception'
        )
        msg.source_pkg = 'uran_autotask'
        msg.timestamp_ns = self._now_ns()
        self._state_pub.publish(msg)

    def _write_motion_control_lock(self, *, active: bool, task_id: str = '', reason: str = ''):
        now_ns = self._now_ns()
        payload = {
            'active': bool(active),
            'owner': 'uran_autotask' if active else '',
            'task_id': str(task_id or ''),
            'reason': str(reason or ''),
            'timestamp_ns': now_ns,
            # 兜底：节点异常退出时，core 会自动释放这个锁。
            'expires_at_ns': now_ns + 24 * 60 * 60 * 1_000_000_000 if active else 0,
        }
        msg = StateField()
        msg.field_name = 'motion_control_lock'
        msg.value_json = json.dumps(payload, ensure_ascii=False)
        msg.persistent = False
        msg.urgent = True
        msg.source_pkg = 'uran_autotask'
        msg.timestamp_ns = now_ns
        self._state_pub.publish(msg)

    def _srv_get_task_status(self, request, response):
        response.status_json = self._mission_manager.status_json(task_id=request.task_id)
        return response

    def destroy_node(self):
        self._mission_manager.destroy()
        if self._backend is not None:
            self._backend.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UranAutotaskNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
