# URAN 节点开发任务清单

**开发环境：Ubuntu 22.04 + ROS 2 Humble**

**参考文档：`URAN节点设计文档.md`、`云端与URAN节点通信字段手册.md`**

**原则：每个功能包开发完成后进行实机部署测试，由 MQTT 工具等模拟云端下发验证。**

---

## 阶段〇：基础环境与公共定义

### T0.1 ROS 2 工作空间初始化

- 创建 `uran_ws/src/` 工作空间结构
- 配置 `colcon` 构建环境，确认 ROS 2 Humble 可正常编译
- 建立统一的 `.gitignore`、`README.md`

### T0.2 uran_msgs 消息包

> 设计文档参考：第十二章「消息与服务定义汇总」§12.1

创建 `uran_msgs` 包，定义以下 `.msg` 文件：

| 消息文件 | 关键字段 |
|---------|---------|
| `StateField.msg` | field_name, value_json, persistent, source_pkg, timestamp_ns |
| `StateSnapshot.msg` | msg_version, timestamp_ns, device_id, fields_json |
| `HeartbeatStatus.msg` | timestamp_ns, protocol, last_sent_ts, success |
| `UnifiedMoveCmd.msg` | msg_version, device_id, timestamp_ns, controller, linear_vel_x/y/z, angular_vel_z, target_roll/pitch/yaw, action, extra_json |
| `ModeSwitchCmd.msg` | control_mode, controller, timestamp_ns |
| `MediaSwitchCmd.msg` | action, protocol, timestamp_ns |
| `UplinkProtocolCmd.msg` | protocol, timestamp_ns |
| `UplinkPayload.msg` | source_pkg, data_type, preferred_protocol, payload_json, urgent, timestamp_ns |
| `MediaCtrlCmd.msg` | action, protocol, channel_id, signal_json, timestamp_ns |
| `FrpcCtrlCmd.msg` | action, frps_host, frps_port, service_name, local_port, remote_port, auth_token, timestamp_ns |
| `TaskCtrlCmd.msg` | msg_version, task_id, action, task_type, task_params_json, timestamp_ns |
| `ParamUpdateCmd.msg` | params_json, timestamp_ns |

### T0.3 uran_srvs 服务包

> 设计文档参考：第十二章 §12.2

创建 `uran_srvs` 包，定义以下 `.srv` 文件：

| 服务文件 | 请求 → 响应 |
|---------|------------|
| `GetStateField.srv` | field_names[] → fields_json, success |
| `SetStateField.srv` | field_name, value_json, persistent → success, message |
| `ConnectProtocol.srv` | protocol, action → success, message |
| `GetNetworkStatus.srv` | (空) → protocol_table_json |
| `SwitchMovePlugin.srv` | plugin_id → success, message, current_plugin |
| `ReloadConfig.srv` | config_path → success, message |
| `ListSensors.srv` | (空) → sensors_json |
| `GetTaskStatus.srv` | task_id → status_json |
| `TriggerStateReport.srv` | reason → success, message |
| `ConfigureStateReport.srv` | interval_ms, protocol, report_on_change → success, message, current_interval_ms, current_protocol |

### T0.4 编译验证

- `colcon build` 确认 uran_msgs、uran_srvs 编译通过
- 用 `ros2 interface show` 验证各消息/服务定义正确

---

## 阶段一：URAN-core 核心包

> 设计文档参考：第四章「URAN-core 核心包」全部内容

### T1.1 包骨架与配置文件

- 创建 `uran_core` 包（ament_cmake / ament_python 按需选择）
- 创建 `config/network.yaml`（§4.3.2 入网配置结构）
- 创建 `config/core.yaml`（§4.6 定期上报配置）
- 节点入口 `uran_core_node`

### T1.2 状态空间管理

> 设计文档参考：§4.2

- 实现内存状态空间（非持久化字段），参考 §4.2.3 标准字段表
- 实现持久化存储（SQLite / CSV），用于 device_id、template_id 等持久化字段
- 实现 ROS Topic `/uran/core/state/write`（订阅 `StateField`，功能包写入）
- 实现 ROS Topic `/uran/core/state/broadcast`（发布 `StateSnapshot`，定期广播）
- 实现 ROS Service `/uran/core/state/get`（`GetStateField`）
- 实现 ROS Service `/uran/core/state/set`（`SetStateField`）

### T1.3 MQTT 入网与心跳

> 设计文档参考：§4.3

- 集成 MQTT 客户端库（如 paho-mqtt）
- 读取 `config/network.yaml` 中 MQTT 配置
- 实现启动注册流程（§4.2.2）：携带 device_id + template_id + token 向云端注册
- 处理注册响应（registered / auto_registered / rejected）
- 实现统一心跳包定时发送（§4.3.3 JSON 格式），默认 5s 间隔
- 发布 `/uran/core/heartbeat/status`（`HeartbeatStatus`）
- 实现 Service `/uran/core/network/connect`、`/uran/core/network/status`

**实机测试（T1.3）**：
- 用 MQTT 工具（如 MQTTX / mosquitto_pub）搭建本地 Broker
- 验证节点启动后能连接 Broker、发送心跳、接收心跳确认
- 模拟注册响应，验证三种结果处理

### T1.4 控制切换信令处理

> 设计文档参考：§4.4

- 订阅 MQTT 下行 Topic `/oivs/{tenant_id}/{device_id}/down`
- 解析 `msg_type="control_switch"` 信令（§4.4.2 JSON 格式）
- 更新状态空间（control_mode / current_controller / primary_uplink_protocol）
- 发布 `/uran/core/switch/mode`（`ModeSwitchCmd`）
- 发布 `/uran/core/switch/media`（`MediaSwitchCmd`）
- 发布 `/uran/core/switch/uplink_protocol`（`UplinkProtocolCmd`）

**实机测试（T1.4）**：
- 用 MQTT 工具向下行 Topic 发送 control_switch JSON
- 用 `ros2 topic echo` 验证各 switch Topic 正确发布
- 验证状态空间字段已更新

### T1.5 数据上下行通路

> 设计文档参考：§4.5

- 实现上行通路：订阅 `/uran/core/uplink/data`（`UplinkPayload`），根据 preferred_protocol 和通路表选择协议发送
- 实现下行路由：接收 MQTT 下行消息，按 msg_type 路由到对应 ROS Topic
  - `move_cmd` → `/uran/core/downlink/move_cmd`
  - `task_ctrl` → `/uran/core/downlink/task_ctrl`
  - `media_ctrl` → `/uran/core/downlink/media_ctrl`
  - `frpc_ctrl` → `/uran/core/downlink/frpc_ctrl`
  - `param_update` → `/uran/core/downlink/param_update`
  - `state_query` → 直接响应
- 实现协议通路表（§4.5.3），定期探测各协议可达性
- 实现通路降级策略：mqtt → websocket → tcp → udp → lora

**实机测试（T1.5）**：
- 用 MQTT 工具发送各类 msg_type 下行消息
- 用 `ros2 topic echo` 验证路由到正确的下行 Topic
- 用 `ros2 topic pub` 发布 UplinkPayload，验证 MQTT Broker 收到上行消息

### T1.6 定期状态上报

> 设计文档参考：§4.6

- 实现对内广播定时器（默认 1s，`state_broadcast_interval_ms`）
- 实现对外上报定时器（默认 10s，`state_report_interval_ms`）
- 实现即时上报触发（控制切换完成、urgent StateField、online_status/error_code 变更）
- 上报数据结构含 trigger 字段（periodic / change / switch）
- 实现 Service `/uran/core/state_report/trigger`、`/uran/core/state_report/configure`

**实机测试（T1.6）**：
- 验证 MQTT Broker 定期收到 state_snapshot 上行消息
- 通过 Service 修改上报周期，验证生效
- 触发即时上报条件，验证立即收到上报

---

## 阶段二：URAN-move 运控包

> 设计文档参考：第五章「URAN-move 运控包」全部内容

### T2.1 包骨架与插件框架

- 创建 `uran_move` 包
- 定义插件基类 `MovePluginBase`（§5.4.1 C++ 头文件）
  - `init()`, `execute()`, `device_type()`, `version()`, `internal_state_json()`
  - `on_failsafe()`, `on_failsafe_recovered()`（§5.7.4）
- 创建 `config/plugins.yaml`（§5.4.3 插件注册配置）
- 实现 pluginlib 动态加载机制

### T2.2 统一运控指令接收与预检限速

> 设计文档参考：§5.3, §5.3.1, §5.3.2

- 订阅 `/uran/core/downlink/move_cmd`（`UnifiedMoveCmd`）
- 订阅 `/uran/core/switch/mode`（`ModeSwitchCmd`）
- 实现手动/自动模式行为（§5.6）：manual 模式拒绝 auto 来源，auto 模式拒绝 cloud/field 来源
- 实现预检限速层：
  - 从状态空间读取 linear_vel_limit、angular_vel_limit
  - 合速度向量模长等比缩放（§5.3.1 公式）
  - angular_vel_z clamp
  - 截断时上报 move_clamp_event

### T2.3 ros_twist 插件（通用差速底盘）

> 设计文档参考：§5.4.2 ros_twist 行

- 实现 `ros_twist` 插件：将 UnifiedMoveCmd 映射为 `geometry_msgs/Twist` 发布到 `/cmd_vel`
- linear_vel_x → Twist.linear.x，angular_vel_z → Twist.angular.z
- 忽略 linear_vel_y/z、target_roll/pitch/yaw
- action="stop" / "emergency_stop" → 发布零速度

**实机测试（T2.3）**：
- 用 MQTT 工具发送 move_cmd 下行消息
- 用 `ros2 topic echo /cmd_vel` 验证 Twist 输出
- 验证限速截断、模式冲突拒绝、执行结果上报

### T2.4 unitree_go2 插件（宇树机器狗）

> 设计文档参考：§5.4.2 unitree_go2 行，§5.4.2.1 适配行为

- 实现 `unitree_go2` 插件，适配宇树 SDK
- 速度直接映射（SDK 同为体坐标系）
- 步态切换（extra_json 中 gait 字段）
- action 映射：stop → 零速度，emergency_stop → 锁定关节，stand/sit 等

### T2.5 px4_mavros 插件（PX4 无人机）

> 设计文档参考：§5.4.2 px4_mavros 行，§5.4.2.1 飞行状态机

- 实现 `px4_mavros` 插件，内置飞行状态机（DISARMED → ARMED → OFFBOARD → TAKINGOFF → LANDING → RTL）
- 坐标系转换说明（ENU 体坐标系，MAVROS 同为 ENU，无需取反）
- action 映射：takeoff → arm + OFFBOARD + 上升，land → AUTO.LAND，return_home → AUTO.RTL，stop → AUTO.HOLD，emergency_stop → disarm
- OFFBOARD 保活定时器（20Hz）
- 超时自动悬停（默认 3s）

### T2.6 失控保护

> 设计文档参考：§5.7

- 订阅 `/uran/core/heartbeat/status`，监听通路表
- 实现失控保护触发条件：所有协议不可用 + 持续超过 failsafe_timeout_s（默认 5s）
- 实现保护动作：stop / return_home / land / hold_position / custom
- 实现恢复逻辑：链路恢复后延迟 failsafe_recover_delay_s（默认 2s）确认
- 写入状态空间 failsafe_active、failsafe_action_executed
- 上报 failsafe_event

### T2.7 运行时插件切换

- 实现 Service `/uran/move/switch_plugin`（`SwitchMovePlugin`）

**实机测试（T2 整体）**：
- 模拟云端发送 move_cmd，验证 ros_twist 插件输出 /cmd_vel
- 切换插件，验证 SwitchMovePlugin Service
- 模拟全部链路中断，验证失控保护触发与恢复
- 验证执行结果通过 uplink 上报到 MQTT Broker

---

## 阶段三：URAN-sensor 传感器包

> 设计文档参考：第七章「URAN-sensor 传感器包」全部内容

### T3.1 包骨架与插件框架

- 创建 `uran_sensor` 包
- 定义转换插件基类 `SensorConverterBase`（§7.3.1 C++ 接口）
- 创建 `config/sensor.yaml`（§7.2 传感器数据源配置）

### T3.2 传感器数据采集与转换管线

- 根据 config/sensor.yaml 动态订阅各传感器 ROS Topic
- 调用对应 converter_plugin 进行数据转换
- 按配置决定是否写入状态空间（update_state_space + state_space_field）
- 按配置决定是否上报云端（report_to_cloud + report_interval_ms）
- 上报通过 `/uran/core/uplink/data`（data_type="sensor"）

### T3.3 官方预设转换插件

> 设计文档参考：§7.3.2

实现以下插件（按优先级排序）：

| 插件 ID | 输入 | 输出 |
|---------|------|------|
| `navsat_to_position` | sensor_msgs/NavSatFix | { lat, lon, alt, covariance } |
| `imu_to_attitude` | sensor_msgs/Imu | { roll, pitch, yaw, angular_velocity, linear_acceleration } |
| `battery_state` | sensor_msgs/BatteryState | { percentage, voltage, current, status } |
| `odometry` | nav_msgs/Odometry | { position, orientation, linear_velocity, angular_velocity } |
| `range_sensor` | sensor_msgs/Range | { range_m, min_range, max_range, field_of_view } |
| `pointcloud_summary` | sensor_msgs/PointCloud2 | { point_count, density, bounding_box } |
| `laser_scan` | sensor_msgs/LaserScan | { min_angle, max_angle, ranges_json } |

### T3.4 Service 接口

- 实现 `/uran/sensor/reload_config`（`ReloadConfig`）
- 实现 `/uran/sensor/list`（`ListSensors`）

**实机测试（T3）**：
- 用 `ros2 topic pub` 模拟发布 NavSatFix、Imu 等传感器数据
- 验证状态空间中 position、attitude 等字段更新
- 验证 MQTT Broker 收到 sensor 类型上行数据
- 验证 reload_config Service 可热加载新传感器配置

---

## 阶段四：URAN-media 流媒体包

> 设计文档参考：第六章「URAN-media 流媒体包」全部内容

### T4.1 包骨架与配置

- 创建 `uran_media` 包
- 创建 `config/media.yaml`（§6.3 视频数据输入适配配置）
- 支持多通道视频源配置（channel_id、ros_topic、encoding、fps）

### T4.2 WebRTC 信令与推流

> 设计文档参考：§6.4 WebRTC 信令流程

- 集成 WebRTC 库（如 libdatachannel / GStreamer webrtcbin）
- 订阅 `/uran/core/downlink/media_ctrl`（`MediaCtrlCmd`）
- 订阅 `/uran/core/switch/media`（`MediaSwitchCmd`）
- 实现信令流程：
  1. 收到 action="start", protocol="webrtc", channel_id → 创建 PeerConnection
  2. 生成 SDP Offer → 通过 uplink 上报
  3. 收到 SDP Answer → 设置远端描述
  4. 交换 ICE Candidate
  5. 连接建立 → 订阅对应 ROS 摄像头 Topic → 编码推流
- 每个 channel_id 独立 PeerConnection，支持并发多路
- 断线自动重连（可配置重试次数）

### T4.3 RTSP 推流（次选）

- 集成 RTSP Server 库（如 GStreamer / live555）
- 收到 action="start", protocol="rtsp" → 启动本地 RTSP Server
- 上报 RTSP Stream URL 至云端

### T4.4 本地录制与弱网回传

> 设计文档参考：§6.6

- 实现 action="record_start" / "record_stop" 控制
- 按通道分别录制，时间戳分片
- 链路恢复后切片上传（data_type="media_upload"）

**实机测试（T4）**：
- 用 USB 摄像头发布 ROS Image Topic
- 用 MQTT 工具发送 media_ctrl 启动 WebRTC
- 用浏览器或 WebRTC 客户端验证视频流
- 验证录制文件生成与回传

---

## 阶段五：URAN-frpcpoint 端口转发服务

> 设计文档参考：第八章「URAN-frpcpoint 端口转发服务」全部内容

### T5.1 包骨架

- 创建 `uran_frpcpoint` 包
- 确保系统已安装 frpc 客户端

### T5.2 端口转发控制

> 设计文档参考：§8.2, §8.3

- 订阅 `/uran/core/downlink/frpc_ctrl`（`FrpcCtrlCmd`）
- 收到 action="start" → 动态生成 frpc 配置文件 → 启动 frpc 进程
- 收到 action="stop" → 停止 frpc 进程
- 收到 action="update" → 更新配置并重启
- 上报端口映射状态至状态空间和 uplink（data_type="frpc_status"）

**实机测试（T5）**：
- 搭建本地 frps 服务端
- 用 MQTT 工具发送 frpc_ctrl 启动 SSH 端口映射
- 通过映射端口 SSH 连接设备验证

---

## 阶段六：URAN-autotask 自动化巡检服务

> 设计文档参考：第九章「URAN-autotask 自动化巡检服务」全部内容

### T6.1 包骨架

- 创建 `uran_autotask` 包

### T6.2 任务生命周期管理

> 设计文档参考：§9.2

- 订阅 `/uran/core/downlink/task_ctrl`（`TaskCtrlCmd`）
- 订阅 `/uran/core/switch/mode`（`ModeSwitchCmd`）
- 实现任务状态机：idle → executing → paused → completed / aborted
- 支持 action：start / pause / resume / stop / update_params
- 解析 task_params_json（航点任务参数结构）

### T6.3 航点任务执行

- 按航点序列依次执行：
  1. 计算当前位置到目标航点的运动指令
  2. 通过 `/uran/core/downlink/move_cmd` 发送 UnifiedMoveCmd（controller="auto"）
  3. 到达航点后执行 hover_time_s 悬停
  4. 执行航点 actions（capture_image、sensor_report 等）
  5. 推进到下一航点
- 低电量检测（abort_on_low_battery + low_battery_threshold）
- 异常处置（§9.2.3 标准异常分类表）

### T6.4 进度上报

- 持续上报 task_progress（data_type="task_progress"）
- 包含 stage、current_waypoint_seq、progress_percent、status、event、error
- 写入状态空间 task_id、task_stage

### T6.5 Service 接口

- 实现 `/uran/autotask/status`（`GetTaskStatus`）

**实机测试（T6）**：
- 用 MQTT 工具发送 task_ctrl 启动航点任务
- 验证 URAN-move 收到 auto 来源的运控指令
- 验证 MQTT Broker 收到 task_progress 上报
- 测试 pause / resume / stop 控制
- 模拟低电量触发 abort

---

## 阶段七：系统联调与集成测试

### T7.1 全链路联调

- 启动所有 URAN 节点（core + move + sensor + media + frpcpoint + autotask）
- 用 MQTT 工具模拟完整云端交互流程：
  1. 节点启动 → 注册 → 心跳上报
  2. 控制切换（manual ↔ auto，cloud ↔ field）
  3. 手动运控指令下发与执行
  4. 传感器数据上报
  5. 流媒体通道建立
  6. 自动巡检任务下发与执行
  7. 端口转发建立
- 验证状态空间数据一致性
- 验证通路降级与失控保护

### T7.2 Launch 文件与部署

- 编写统一 launch 文件，一键启动所有节点
- 参数化配置（device_id、broker 地址等通过 launch 参数传入）
- 编写部署说明文档

---

*文档结束*
