# `uran_autotask` 调试清单

这份文档给后续排查用。目标是把问题分成三层：

1. 外接 GPS 本身是否正常；
2. `uran_autotask` 是否把 GPS、里程计、任务状态正确拼起来；
3. `uran_core` 是否把上行数据按 MQTT 发出去。

## 1. 先看外接 GPS

先看 `/ubx/status`，不要只看 `/ubx/gps_payload`。

重点字段：

- `connected`：串口是否打开成功。
- `stale`：串口数据是否已经过期。
- `valid_fix`：当前是否真的拿到了可用定位。
- `fix_type`：UBX 定位类型。
- `num_sv`：当前可见卫星数。
- `active_port`：当前打开的是哪个串口。
- `last_error`：最近一次错误。

判断顺序：

- `connected=false`，先查权限、串口是否存在、是否被别的进程占用。
- `connected=true` 但 `stale=true`，说明串口还开着，但数据没有持续进来。
- `connected=true`、`stale=false`、`valid_fix=false`、`num_sv=0`，说明设备连上了，但还没有可用卫星信号。
- `valid_fix=true` 且 `fix_type` 达标，才算外接 GPS 可用。

## 1.1 这里说的 2D 地图是什么

这里的 2D 地图，不是 `mission_planner` 生成的路线数据。

- `mission_planner` 主要给的是路线、点位、投影坐标和任务参数。
- `map_name` 只是一个地图名字字段，用来选用哪张图，不是地图内容本身。
- 真正给 `NavAB` 用的是 `nav2_map_server` 载入的 `map.yaml` 对应的占据栅格地图。
- 这张地图通常来自以前扫描好的 SLAM 地图，或者来自外部提前准备好的静态地图。
- 如果机器上没有这张地图，当前这条导航链路不能只靠 `mission_planner` 自己补出来。

## 2. 再看 `uran_autotask` 自己

任务状态服务：

```bash
ros2 service call /uran/autotask/status uran_srvs/srv/GetTaskStatus "{task_id: ''}"
```

或者直接看上报状态：

```bash
ros2 topic echo /uran/core/uplink/data
```

`/uran/autotask/status` 返回的是一整段 JSON，重点看这些字段：

- `stage`：`idle`、`preparing`、`executing`、`paused`、`completed`、`aborted`。
- `status`：`normal` 或 `exception`。
- `current_waypoint_index`：当前执行到第几个点。
- `current_waypoint_seq`：当前点的序号。
- `total_waypoints`：总点数。
- `progress_percent`：进度百分比。
- `last_goal_map_pose`：最后一次发给底层的 map 目标。
- `position`：当前融合位置。
- `gps_monitor`：GPS 监督状态。
- `closed_loop`：GPS 和本地里程计的联动状态。
- `outdoor_navigation`：室外路线执行细节。

室外任务里重点看：

- `outdoor_navigation.current_point`
- `outdoor_navigation.active_goal`
- `outdoor_navigation.active_candidate`
- `outdoor_navigation.goal_records`
- `outdoor_navigation.gps_vo_gate`
- `outdoor_navigation.pose_alignment`

## 3. 看位置上报

`uran_autotask` 默认每 1 秒上报一次位置到：

```text
/uran/core/uplink/data
```

真正发到云端时，`uran_core` 会转成 MQTT 上行包。当前 MQTT 目标前缀是：

```text
/oivs/{tenant_id}/{device_id}/up
```

这表示：

- ROS 侧固定先发 `/uran/core/uplink/data`。
- `uran_core` 再转发到 MQTT。
- `robot_pose` 也是走这条链路。

`robot_pose` 的 `payload_json` 里，常看这些字段：

- `packet_type`
- `packet_label`
- `data_type`
- `position.available`
- `position.fusion_state`
- `position.lat`
- `position.lon`
- `position.fix_type`
- `position.num_sv`
- `gps_status.connected`
- `gps_status.stale`
- `gps_status.valid_fix`
- `task_stage`
- `task_status`

## 4. 区分问题在哪一层

### 4.1 GPS 层

看 `/ubx/status`。

## 5. 调试 straight_drive 直线避障

先确认雷达有数据。实机常见话题不是 `/scan`，而是带机器狗 namespace：

```bash
ros2 topic list -t | grep -E 'scan|LaserScan|lidar'
ros2 topic hz /mi_desktop_48_b0_2d_5f_b6_d0/scan
ros2 topic echo /mi_desktop_48_b0_2d_5f_b6_d0/scan --once
```

`autotask.yaml` 默认写 `straight_drive.scan_topic: "scan"`。这是相对名，节点启动后会
自动加机器狗 namespace；不要写成 `/scan`，除非现场真的发布了全局 `/scan`。

发送一个低速测试任务：

```bash
ros2 topic pub --once /uran/core/downlink/task_ctrl uran_msgs/msg/TaskCtrlCmd "{action: start, task_id: straight_test_001, task_type: straight_drive, task_params_json: '{\"task_type\":\"straight_drive\",\"distance_m\":2.0,\"speed_mps\":0.12}'}"
```

监控任务状态：

```bash
ros2 topic echo /uran/core/uplink/data
ros2 service call /uran/autotask/status uran_srvs/srv/GetTaskStatus "{task_id: ''}"
```

重点看：

- `straight_drive.state`
- `straight_drive.scan_age_s`
- `straight_drive.decision.front_clearance_m`
- `straight_drive.decision.theta_target_deg`
- `straight_drive.distance_traveled_m`

如果状态变成 `blocked`，含义是局部避障认为当前没有安全可通行方向，或 `/scan`
超时。这个功能不会继续尝试全局找路，正确处理方式是清理障碍或下发停止/新任务。

### 4.2 `uran_autotask` 融合层

看 `/uran/autotask/status` 里的：

- `gps_monitor`
- `closed_loop`
- `position`
- `outdoor_navigation`

### 4.3 上行链路层

看 `/uran/core/uplink/data` 有没有持续发出。

如果 ROS 里有数据，但云端没有数据，再看 `uran_core` 的 MQTT 配置和 broker 状态。

## 5. 现场常用命令

```bash
ros2 topic echo /ubx/status --field data --full-length
ros2 topic echo /ubx/gps_payload
ros2 service call /uran/autotask/status uran_srvs/srv/GetTaskStatus "{task_id: ''}"
ros2 topic echo /uran/core/uplink/data
ros2 topic hz /ubx/gps_payload
ros2 topic hz /uran/core/uplink/data
```

## 6. 关键结论

- `/ubx/status` 决定外接 GPS 有没有真正可用。
- `/uran/autotask/status` 决定任务有没有进入执行态。
- `/uran/core/uplink/data` 决定云端能不能收到机器狗当前坐标。
- `robot_pose` 不是单独一条云端 topic，它也是通过 `/uran/core/uplink/data` 进 `uran_core` 后再转到 MQTT。

## 7. 现场流程

这部分按“开机后先确认基础链路，再下发任务，再看失败点”的顺序来。

### 7.1 开机后先看三件事

1. 先确认工作区环境已经 source。
   - `uran_autotask` 需要它自己的 ROS 环境。
   - 调用 CyberDog 底层导航时，还需要能找到 `cyberdog_ws` 里的接口包。
2. 先看 `/ubx/status`。
   - 重点不是“串口有没有打开”，而是“有没有真正可用的定位”。
3. 再看 `/uran/core/uplink/data` 有没有持续上报。
   - 这能先确认 `uran_autotask` 没有卡死。

建议先执行：

```bash
ros2 topic echo /ubx/status --field data --full-length
ros2 topic hz /ubx/gps_payload
ros2 service call /uran/autotask/status uran_srvs/srv/GetTaskStatus "{task_id: ''}"
ros2 topic hz /uran/core/uplink/data
```

### 7.2 发任务后看什么

任务刚收到时，`stage` 一般会先变成 `preparing`，然后进入 `executing`。

如果是室外路线任务，再重点看：

- `outdoor_navigation.start_alignment`
- `outdoor_navigation.current_point`
- `outdoor_navigation.active_goal`
- `outdoor_navigation.active_candidate`
- `outdoor_navigation.goal_records`

如果任务已经发给底层，但没有继续走，先分三类看：

- `stage=paused`，通常是任务被挂起了，去看 `error.code` 和 `suggested_action`。
- `stage=aborted`，通常是底层或参数校验失败，去看 `task_status` 和 `backend_state`。
- `stage=executing` 但 `active_goal` 长时间不变，优先看底层导航和局部避障，而不是先怀疑 GPS。

### 7.3 出问题时的最短判断路径

1. 先看 `/ubx/status`。
   - 如果这里已经不对，就先修 GPS。
2. 再看 `/uran/autotask/status`。
   - 如果 `gps_monitor` 或 `position.gps_status` 异常，说明是融合层问题。
3. 再看 `outdoor_navigation.goal_records`。
   - 如果这里在反复切换 `candidate_failed`，说明是目标点试探失败，不是 GPS 本身坏了。
4. 最后看 `algorithm_manager` 和 Nav2。
   - 如果导航目标已经发出，但一直 `ABORTED` 或反复 recovery，问题通常在局部规划、代价地图、TF、或传感器输入。

### 7.4 任务执行时建议盯的状态

- `/ubx/status`
- `/uran/autotask/status`
- `/uran/core/uplink/data`
- `ros2 action list | grep -E 'start_algo_task|navigate_to_pose'`
- `ros2 service list | grep -E 'algo_task_status|stop_algo_task'`

## 8. 避障原理

这里说的是 `uran_autotask -> algorithm_manager -> Nav2` 这条链路里的避障，不是 `uran_autotask` 自己做避障。

### 8.1 `uran_autotask` 做什么，不做什么

它做的是：

- 把任务点变成 map 坐标目标；
- 把目标发给 `algorithm_manager`；
- 记录任务状态、目标点、候选点和失败原因；
- 在目标发不出去或底层返回失败时暂停或中止任务。

它不做的是：

- 不直接算局部避障轨迹；
- 不自己读代价地图；
- 不自己判断“前面有没有墙”。

室外任务里的 `goal_resolver` 只是生成几个附近候选点，给 Nav2 失败后重试用，不是障碍物识别模块。

### 8.2 底层导航怎么跑

`algorithm_manager` 里的 `NavAB` 任务，最终会发 `NavigateToPose`。

它的执行链路是：

1. `algorithm_manager` 检查当前状态、地图和依赖节点；
2. 把目标送给 `bt_navigator_ab`；
3. 行为树每秒重规划一次全局路径；
4. `FollowPath` 根据当前局部代价地图生成控制量；
5. 如果失败，行为树会清局部/全局代价地图，再做 `Spin`、`Wait`、`BackUp` 这类恢复动作。

对应代码和配置里能直接看到：

- 行为树里有 `ComputePathToPose` 和 `FollowPath`；
- `planner_id` 是 `GridBased`；
- `controller_id` 是 `FollowPath`；
- `controller_server_ab` 用的是 DWB 局部规划器；
- `local_costmap` 和 `global_costmap` 都有障碍物层和膨胀层。

### 8.3 障碍物怎么进入系统

在 `navigate_to_pose_params.yaml` 里，局部和全局代价地图都用了 `lidar_obstacle_layer/ObstacleLayer`。

它会从这些输入里取障碍物：

- `scan` 激光；
- `camera/depth/color/points` 点云。

然后做两件事：

- `marking`：把障碍物标成代价高的区域；
- `clearing`：沿着激光射线把空白区域清出来。

再加上 `inflation_layer`，障碍物周围会被扩成更大的不可通行区域，给局部控制留安全边界。

### 8.4 路径为什么会绕障

绕障不是“路径点之间遇到障碍后手工改线”，而是分两层：

- 全局层：`ComputePathToPose` 每秒重新规划一条尽量可行的路径；
- 局部层：`FollowPath` 结合局部代价地图，实时避开当前附近的障碍。

如果局部路被挡住，DWB 不是硬冲过去，而是根据 `BaseObstacle`、`PathAlign`、`GoalAlign`、`PathDist`、`GoalDist` 这些代价项选一个更安全的速度输出。

### 8.5 恢复动作怎么触发

如果 `FollowPath` 或全局规划失败，行为树会进入恢复分支，常见动作是：

- 清局部代价地图；
- 清全局代价地图；
- 原地转圈；
- 等待几秒；
- 后退一小段。

这说明系统的策略是“先尝试局部绕开，再尝试恢复，再失败才上报终止”，不是单纯遇到障碍就停。

## 9. 可行性和边界

### 9.1 可行的地方

这套方案在以下场景里是可行的：

- 机器人已经有可用的 2D 地图；
- `/scan` 或深度点云能稳定看到前方障碍；
- `odom_out`、`map`、`base_link` 之间的 TF 链路稳定；
- 定位误差没有大到让机器人一直跑偏；
- 障碍物主要是地面层或机器人传感器能看到的高度范围。

### 9.2 主要边界

这套方案的边界也很明确：

- 它是 2D 代价地图，不是 3D 语义避障；
- 如果障碍物没进激光或点云，系统就不知道；
- 如果地形很复杂，单靠 DWB 和局部代价地图不一定够；
- 如果定位漂移大，路径再怎么重规划也会失真；
- 如果地图本身不准，避障会跟着偏。

### 9.3 对你当前场景的判断

就现有代码看，这套系统更适合：

- 普通室外地面行走；
- 任务点之间的导航；
- 传感器能稳定看见前方障碍的场景。

它不适合被理解成“自动识别所有障碍并做智能绕行”的系统。它更像是：

- 有地图；
- 有局部传感器；
- 有局部规划；
- 有失败后的恢复动作。

### 9.4 你该怎么验证它是否真的可行

现场最直接的验证顺序是：

1. 先确认 `odom_out`、`map`、`base_link` 的 TF 是通的。
2. 再确认 `scan` 或点云真的有数据。
3. 再发一个短距离目标，看 `active_goal` 和 `goal_records` 是否正常推进。
4. 最后在路径旁边放一个小障碍，观察是绕开、恢复，还是直接失败。

如果你愿意，我下一步可以继续把这份文档补成“实机调试表”，把每一步对应的 `ros2` 命令、正常现象、异常现象都写成三列表。 
