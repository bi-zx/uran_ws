# uran_autotask Architecture

`uran_autotask` is the robot-side mission node. It runs on the CyberDog/URAN ROS
environment and talks to the cloud through `uran_core`.

The package should stay narrow:

- receive task packets from `/uran/core/downlink/task_ctrl`;
- parse mission-planner route payloads;
- dispatch navigation goals through the CyberDog algorithm manager;
- supervise outdoor GPS / local odometry alignment;
- publish task progress and robot pose through `/uran/core/uplink/data`;
- avoid local debug frontend, local map preview pages, and duplicated UI logic.

## Runtime Data Flow

Cloud command flow:

1. cloud backend sends a task packet to `uran_core`;
2. `uran_core` publishes `uran_msgs/msg/TaskCtrlCmd` on
   `/uran/core/downlink/task_ctrl`;
3. `uran_autotask` parses and executes the task;
4. `uran_autotask` publishes progress and robot pose on
   `/uran/core/uplink/data`;
5. `uran_core` forwards those payloads to the cloud backend;
6. the cloud frontend renders the robot pose from the backend stream.

There is no local HTTP server, local click-goal page, or local map preview in
`uran_autotask`.

## Package Layout

```text
uran_autotask/
├── config/
│   └── autotask.yaml
├── launch/
│   └── uran_autotask.launch.py
├── uran_autotask/
│   ├── uran_autotask_node.py
│   ├── geo_utils.py
│   ├── task_models.py
│   ├── task_parser.py
│   ├── adapters/
│   │   ├── cyberdog_algorithm_manager.py
│   │   ├── cyberdog_camera_capture.py
│   │   └── uran_move_gateway.py
│   ├── localization/
│   │   ├── closed_loop_manager.py
│   │   ├── gps_supervisor.py
│   │   ├── outdoor_pose_aligner.py
│   │   ├── pose_msg_utils.py
│   │   ├── pose_registry.py
│   │   └── vo_supervisor.py
│   ├── mission/
│   │   ├── mission_manager.py
│   │   ├── waypoint_action_runner.py
│   │   └── waypoint_dispatcher.py
│   └── outdoor/
│       ├── goal_resolver.py
│       ├── gps_vo_gate.py
│       ├── mission_contract.py
│       ├── planner_result_parser.py
│       └── route_sampler.py
├── package.xml
└── setup.py
```

## Robot Pose Uplink

Robot pose is published through the existing topic:

```text
/uran/core/uplink/data
uran_msgs/msg/UplinkPayload
```

Default `data_type`:

```text
robot_pose
```

Payload JSON:

```json
{
  "schema_version": 1,
  "robot_id": "",
  "coordinate_system": "WGS84",
  "position": {
    "available": true,
    "fusion_state": "raw_gps",
    "lat": 39.0,
    "lon": 116.0,
    "alt": 0.0,
    "fix_type": 2,
    "num_sv": 10,
    "timestamp_ns": 0,
    "map_pose": {},
    "alignment": {}
  },
  "control_mode": "auto",
  "controller": "cloud",
  "battery_level": 80.0,
  "task_id": "task_001",
  "task_stage": "executing",
  "task_status": "normal",
  "timestamp_ns": 0
}
```

`fusion_state` values:

- `aligned_map_pose`: GPS-map offset is stable, so map pose is converted back to
  WGS84 with the current alignment estimate.
- `raw_gps`: no stable alignment is available, so raw GPS is reported.
- `local_odom_dead_reckoning`: GPS is stale, so the last good GPS anchor plus
  map/local odometry delta is converted back to WGS84.
  `visual_dead_reckoning` is still emitted as a legacy alias for older
  dashboards.
- `unavailable`: neither GPS nor aligned map pose is available.

The cloud frontend should render `position.lat`, `position.lon`, and
`position.alt` when `position.available` is true.

## Outdoor Alignment

`localization/outdoor_pose_aligner.py` estimates the stable offset between GPS
projection and Nav2 `map` pose.

Current implementation:

- GPS latitude and longitude are projected into the map frame;
- GPS motion is compared with local odometry motion;
- GPS jumps are rejected when local odometry does not support the same motion;
- accepted offsets are stored in a weighted sliding window;
- the stable offset is exposed in task status and used by pose reporting.

Goal correction is enabled, but bounded by `max_goal_correction_m`. It is only
applied after the sliding-window estimate becomes stable. Before that, the
planner point is sent unchanged.

This is not a full Kalman filter. The current fusion rule is:

- use stable aligned map pose when GPS-map offset is stable;
- otherwise use fresh GPS;
- otherwise use short-term dead reckoning from the last reliable GPS anchor;
- otherwise report `unavailable`.

After enough logs are collected, the internal estimator can be upgraded from a
sliding window to a Kalman filter without changing the cloud protocol.

## Configuration

Important groups in [autotask.yaml](/home/techno/uran_ws/src/uran_autotask/config/autotask.yaml):

- `pose_report`: frequency and data type for robot pose uplink.
- `gps_monitor`: CyberDog GPS topic and GPS quality thresholds.
- `visual_pose_monitor`: legacy name for local odometry topic and drift
  thresholds. The selected source can be visual odometry, leg odometry, fused
  odometry, or TF.
- `outdoor_pose_aligner`: sliding-window alignment parameters.
- `cyberdog_backend`: CyberDog algorithm manager action/service names.

## Field Data Needed Before High-Frequency Fusion

The following facts cannot be safely guessed from the code alone:

- exact local odometry topic and message type on the deployed CyberDog image;
- GPS and local odometry publish rates;
- whether message timestamps share the same ROS clock source;
- GPS noise under open sky, near buildings, and under trees;
- local odometry drift after 10 m, 30 m, and 100 m movement;
- fixed yaw/scale relation between the outdoor map and the local odometry frame.

Use these commands on the robot:

```bash
ros2 topic info -v /<namespace>/gps_payload
ros2 topic hz /<namespace>/gps_payload
ros2 topic info -v /<namespace>/dog_pose
ros2 topic info -v /<namespace>/pose_filtered
ros2 topic info -v /<namespace>/odom_out
ros2 topic hz /<namespace>/odom_out
ros2 topic list | grep -Ei 'dog_pose|odom|mivins|vio|vins|visual'
```

Observed CyberDog behavior on the current robot image:

- `/dog_pose` may exist as `geometry_msgs/msg/PoseStamped` with no publisher.
- `/pose_filtered` may be `BEST_EFFORT`; subscribers must match that reliability.
- `/odom_out` is the stable leg odometry source seen so far.
- `/gps_payload` with `fix_type=0`, `num_sv=0`, and zero latitude/longitude is not usable GPS.

`uran_autotask` therefore treats `/odom_out` as local odometry, not as a global
map pose. For global map pose it first uses a real pose topic, then falls back
to TF `map -> base_link` when that transform is available.

## What Should Not Be Reintroduced

- local HTTP debug frontend;
- local click-goal control page;
- local map image preview assets;
- a second local obstacle avoidance controller;
- direct frontend access to ROS topics.

The product frontend belongs to the cloud backend. `uran_autotask` should only
publish stable JSON payloads through `uran_core`.
