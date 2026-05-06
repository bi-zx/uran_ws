# uran_autotask

`uran_autotask` 是机器狗 ROS 端的自动任务节点。

当前职责：

- 接收 `uran_core` 转发的任务控制消息；
- 解析 `mission_planner` 生成的室外路径任务包；
- 将经纬度航点转换为 CyberDog/Nav2 可执行的 `map` 坐标目标；
- 调用 CyberDog 现有 `algorithm_manager` 执行导航；
- 在户外任务中做 GPS 与本地里程计的对齐监督；
- 按固定频率通过 `/uran/core/uplink/data` 上报机器狗当前位置；
- 通过 `/uran/core/uplink/data` 上报任务进度和异常。

## 运行链路

```text
总后台 -> uran_core -> /uran/core/downlink/task_ctrl -> uran_autotask
uran_autotask -> /uran/core/uplink/data -> uran_core -> 总后台
```

总后台前端实时渲染机器狗位置时，应消费总后台收到的 `robot_pose` 上行包，而不是访问机器狗上的本地网页。

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
ros2 topic hz /<namespace>/gps_payload
ros2 topic hz /<namespace>/dog_pose
```

如果自动探测不到本地里程计，还需要对候选话题分别执行：

```bash
ros2 topic info -v /<namespace>/<local_odom_topic>
ros2 topic echo /<namespace>/<local_odom_topic>
ros2 topic hz /<namespace>/<local_odom_topic>
```

当前默认候选本地里程计话题为 `odometry`、`pose_filtered`、`odom_slam`、`mivins/odometry`、`mivins/imuodom_slam`、`mivins/reloc_odom`、`odom_out`。其中你已经在实机上确认 `/odom_out` 是腿式里程计，类型为 `nav_msgs/msg/Odometry`，频率约 43-45 Hz。最终以真机 `ros2 topic info -v` 的结果为准。
