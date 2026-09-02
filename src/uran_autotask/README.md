# uran_autotask

`uran_autotask` 是机器狗 ROS 端的自动任务节点。

当前职责：

- 接收 `uran_core` 转发的任务控制消息；
- 解析 `mission_planner` 生成的室外路径任务包；
- 将经纬度航点转换为 CyberDog/Nav2 可执行的 `map` 坐标目标；
- 在包内执行室外路线跟随和近场绕障控制；
- 支持 `straight_drive` 单步直线行驶任务，使用 D430I 深度点云或 2D 激光雷达做近场避障；
- 兼容保留 CyberDog 现有 `algorithm_manager` 的过渡接口；
- 在户外任务中做 GPS 与本地里程计的对齐监督；
- 按固定频率通过 `/uran/core/uplink/data` 上报机器狗当前位置；
- 通过 `/uran/core/uplink/data` 上报任务进度和异常。

## 运行链路

```text
总后台 -> uran_core -> /uran/core/downlink/task_ctrl -> uran_autotask
uran_autotask -> /uran/core/uplink/data -> uran_core -> 总后台
```

总后台前端实时渲染机器狗位置时，应消费总后台收到的 `robot_pose` 上行包，而不是访问机器狗上的本地网页。

`uran_autotask` 不直接集成 MQTT 客户端。MQTT 连接、注册、下行路由和上行转发
统一由 `uran_core` 负责。`uran_autotask` 只使用下面列出的 ROS 话题与
`uran_core` 通信。

## 主要 Topic

下行任务：

```text
/uran/core/downlink/task_ctrl
uran_msgs/msg/TaskCtrlCmd
```

上行数据：

```text
/uran/core/uplink/data
uran_msgs/msg/UplinkPayload
```

任务状态查询：

```text
/uran/autotask/status
uran_srvs/srv/GetTaskStatus
```

## 总后台 MQTT 包边界

总后台下发任务时，仍然发给 `uran_core` 的 MQTT 下行 topic。顶层
`msg_type` 必须是 `task_ctrl`：

```json
{
  "msg_type": "task_ctrl",
  "msg_version": "1.0",
  "device_id": "device_001",
  "task_id": "mp_task_001",
  "action": "start",
  "task_type": "mission_planner_route",
  "task_params_json": "{\"schema_version\":\"1.1.0\",\"task_type\":\"mission_planner_route\",\"route\":{\"points\":[]}}",
  "timestamp_ns": 1741564800000000000
}
```

`uran_core` 会把该 MQTT 包转换为 `uran_msgs/msg/TaskCtrlCmd` 并发布到
`/uran/core/downlink/task_ctrl`。`timestamp_ns` 是推荐字段；如果旧总后台只发
`timestamp_ms`，`uran_core` 会兼容换算。

`uran_autotask` 上报时发布 `uran_msgs/msg/UplinkPayload`。`uran_core` 转成
MQTT 后，顶层包形如：

```json
{
  "msg_type": "uplink_data",
  "msg_version": "1.0",
  "device_id": "device_001",
  "source_pkg": "uran_autotask",
  "data_type": "robot_pose",
  "timestamp_ns": 1741564800000000000,
  "preferred_protocol": "mqtt",
  "payload": {},
  "payload_json": "{}"
}
```

总后台新逻辑应优先读取 `payload` 对象；`payload_json` 保留用于兼容和排查。

后台判断位置包类型时，优先使用 MQTT 顶层 `data_type`。如果后台只消费
`payload` 对象，也可以使用 `payload.packet_type`。这两个字段默认都是
`robot_pose`。

## robot_pose 上行包

`pose_report` 默认每 1 秒发布一次。

`UplinkPayload.data_type`：

```text
robot_pose
```

`UplinkPayload.payload_json` 示例：

```json
{
  "schema_version": 1,
  "packet_type": "robot_pose",
  "packet_label": "机器狗实时位置",
  "data_type": "robot_pose",
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

## straight_drive 单步任务

`straight_drive` 只表示“向前直线走一段距离或一段时间”。它不是未知环境自主导航，
不使用地图，不做 SLAM。当前默认把 `/camera/depth/color/points` 转换到
`base_link`，过滤地面、机身和过高点，再生成 `/uran/autotask/depth_scan`
供现有绕障控制器使用；绕不开、点云过期或坐标变换缺失时会停车并把任务置为暂停异常。

`straight_drive.obstacle_source` 可选 `depth_pointcloud` 或 `laser_scan`。当前损坏的
TG30 不参与避障；更换雷达后才能把配置切回 `laser_scan`。

第一版默认 `scan` 的角度 0 就是机器狗正前方。如果实机雷达坐标和 `base_link`
不一致，需要在 `straight_drive.angle_offset_deg` 里补偿。

下行任务示例：

```json
{
  "action": "start",
  "task_id": "straight_001",
  "task_type": "straight_drive",
  "task_params_json": "{\"task_type\":\"straight_drive\",\"distance_m\":3.0,\"speed_mps\":0.8}"
}
```

也可以用持续时间：

```json
{
  "task_type": "straight_drive",
  "duration_s": 8.0,
  "speed_mps": 0.8
}
```

状态会出现在 `task_progress.payload.straight_drive` 里。重点看：

- `state`：`clear`、`avoid`、`stop`、`blocked`、`completed`。
- `scan_age_s`：最近一帧雷达数据的年龄。
- `distance_traveled_m`：按本地里程计估算的已走距离。
- `decision.front_clearance_m`：正前方最近障碍距离。
- `decision.theta_target_deg`：当前避障选择的目标方向。
- `route`：当前路线段投影、横向误差、前视点和通行点越过状态。
- `safety`：自动模式、位姿、雷达、底层状态和失控保护的检查结果。
- `goal_relaxation_age_s`：进入目标最终容差范围后的计时；为空表示尚未开始放宽容差。

默认巡航速度和最高速度均为 `0.8 m/s`。转向、接近停车点、障碍物靠近、
传感器质量下降或安全条件不满足时，控制器会降速或停车，所以底层实际速度不一定恒为
`0.8 m/s`。

`fusion_state` 含义：

- `aligned_map_pose`：GPS-map 偏移稳定，使用 map 位姿加偏移后反投影为经纬度。
- `raw_gps`：偏移尚不稳定，直接上报原始 GPS。
- `local_odom_dead_reckoning`：GPS 短时过期，使用最后一次可靠 GPS 加本地里程计增量外推经纬度。
- `unavailable`：当前没有可用位置。

说明：当前版本不是完整卡尔曼滤波。它是“GPS 优先 + 稳定偏移对齐 + GPS 丢失时短时本地里程计外推”的第一版融合逻辑。这里的本地里程计可以是视觉里程计、腿式里程计、融合里程计或 TF。完整高频融合需要现场确认 GPS 频率、本地里程计话题、时间戳来源、噪声水平和坐标系关系。

## 配置入口

配置文件：

```text
config/autotask.yaml
```

关键配置组：

- `pose_report`：控制位置上报频率、上报 `data_type` 和位姿 topic。
- `gps_monitor`：配置 GPS topic 和 GPS 质量阈值。
- `visual_pose_monitor`：历史名称，实际用于配置本地里程计 topic 和漂移阈值。
- `outdoor_pose_aligner`：配置 GPS/视觉滑动窗口对齐器。
- `cyberdog_backend`：配置 CyberDog `algorithm_manager` action/service 名称。

## 现场需要确认的定位信息

真机上至少需要记录以下信息：

```bash
ros2 topic info -v /<namespace>/gps_payload
ros2 topic echo /<namespace>/gps_payload
ros2 topic list | grep -Ei 'dog_pose|odom|mivins|vio|vins|visual'
ros2 topic info -v /<namespace>/dog_pose
ros2 topic info -v /<namespace>/pose_filtered
ros2 topic info -v /<namespace>/odom_out
ros2 topic hz /<namespace>/gps_payload
ros2 topic hz /<namespace>/odom_out
```

如果自动探测不到本地里程计，还需要对候选话题分别执行：

```bash
ros2 topic info -v /<namespace>/<local_odom_topic>
ros2 topic echo /<namespace>/<local_odom_topic>
ros2 topic hz /<namespace>/<local_odom_topic>
```

当前默认候选本地里程计话题优先级为 `odom_out`、`odometry`、`pose_filtered`、`odom_slam`、`mivins/odometry`、`mivins/imuodom_slam`、`mivins/reloc_odom`。其中你已经在实机上确认 `/odom_out` 是腿式里程计，类型为 `nav_msgs/msg/Odometry`，频率约 43-45 Hz。

当前实机结论：

- `/dog_pose` 的类型是 `geometry_msgs/msg/PoseStamped`，但发布者数量为 0，所以不能作为默认全局位姿输入。
- `/pose_filtered` 的类型是 `geometry_msgs/msg/PoseStamped`，发布端是 `BEST_EFFORT`。节点会按发布端服务质量策略兼容订阅，但如果 `ros2 topic echo` 没有输出，它仍然不能提供实际位姿。
- `/odom_out` 的类型是 `nav_msgs/msg/Odometry`，当前可稳定输出腿式里程计，适合作为第一版本地连续运动输入。
- `/gps_payload` 连通不等于 GPS 有效。`fix_type: 0`、`num_sv: 0`、`lat/lon: 0.0` 表示当前没有可用 GPS 定位，`uran_autotask` 不会把它上报为有效经纬度。

全局 `map_pose` 默认先尝试真实位姿话题；如果没有可用发布者，会读取 TF 的 `map -> base_link` 作为回退。这个回退依赖机器狗系统实际发布 `/tf`。
