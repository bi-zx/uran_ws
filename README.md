# URAN — 统一机器人接入节点

URAN（Unified Robotics Access Node）是 OIVS 系统在无人装备侧的统一接入与能力承载层。

当前仓库已完成 **ROS1 Noetic** 适配，现阶段主要包含：
- `uran_core`：统一接入、状态管理、MQTT 上下行
- `uran_media`：基于 **aiortc** 的标准 WebRTC / RTSP 媒体通道
- `uran_msgs` / `uran_srvs`：统一消息与服务定义

同时，项目方向已做如下调整：
- ROS1 版本 `uran_move` **不再适配 CyberDog2**，后续面向 **px4_mavros**
- `uran_media` **抛弃桥接方案**，采用 **aiortc 标准实现**
- **移除 `camera_service` 方案**，仅适配 **RealSense 原生 ROS 话题**

---

## 目录

- [环境要求](#环境要求)
- [快速开始](#快速开始)
- [软件包结构](#软件包结构)
- [uran_core 使用说明](#uran_core-使用说明)
  - [MQTT 控制说明](#mqtt-控制说明)
  - [配置文件](#配置文件)
  - [ROS 接口](#ros-接口)
- [uran_move 使用说明](#uran_move-使用说明)
- [uran_media 使用说明](#uran_media-使用说明)
  - [依赖安装](#依赖安装)
  - [摄像头配置](#摄像头配置)
  - [WebRTC](#webrtc)
  - [RTSP 推流](#rtsp-推流)
  - [本地录制](#本地录制)
  - [ROS 接口（media）](#ros-接口media)
- [消息与服务定义](#消息与服务定义)
- [项目说明与迁移备注](#项目说明与迁移备注)
- [相关文档](#相关文档)

---

## 环境要求

| 项目 | 版本 |
|------|------|
| 操作系统 | Ubuntu 20.04 |
| ROS | ROS1 Noetic |
| Python | 3 |

推荐先安装：

```bash
sudo apt install python3-catkin-tools python3-rospkg python3-yaml python3-pip
```

`uran_media` 额外依赖见下文的[依赖安装](#依赖安装)。

---

## 快速开始

### 1. 编译工作空间

```bash
cd ~/uran_ws
catkin_make -DROS_EDITION=ROS1
```

### 2. 加载环境

```bash
source /opt/ros/noetic/setup.bash
source ~/uran_ws/devel/setup.bash
```

### 3. 启动 uran_core

```bash
rosrun uran_core uran_core_node
# 或
roslaunch uran_core uran_core.launch
```

### 4. 启动 uran_media

```bash
rosrun uran_media uran_media_node
```

---

## 软件包结构

```text
uran_ws/
├── src/
│   ├── uran_core/     # ROS1 核心接入节点
│   ├── uran_move/     # ROS1 运动控制节点（px4ctrl 桥接）
│   ├── uran_media/    # ROS1 媒体节点（aiortc + RTSP）
│   ├── uran_msgs/     # 自定义消息定义
│   └── uran_srvs/     # 自定义服务定义
├── URAN节点设计文档.md
├── 云端与URAN节点通信字段手册.md
└── URAN开发任务清单.md
```

说明：
- 当前仓库中 **未包含 ROS1 版 `uran_move` 实现**
- `uran_move` 的 ROS1 适配方向已调整为 **px4_mavros**，不再维护 CyberDog2 适配路线

---

## uran_core 使用说明

`uran_core` 负责：
- 设备状态空间管理
- MQTT 上下行通信
- 云端控制指令下发
- 统一 ROS Topic / Service 对外接口

### MQTT 控制说明

#### 网络拓扑

```text
[主机 / 云端]                    [设备侧]
MQTT Broker  ◄──── 网络 ────►  uran_core_node
                                    │
                                    ├── /uran/core/downlink/move_cmd
                                    ├── /uran/core/downlink/media_ctrl
                                    └── /uran/core/uplink/data
```

节点使用以下 MQTT Topic 与云端通信：
- 上行：`/oivs/{tenant_id}/{device_id}/up`
- 下行：`/oivs/{tenant_id}/{device_id}/down`

#### 支持的下行 `msg_type`

| msg_type | 处理方式 |
|----------|---------|
| `control_switch` | 更新模式/控制者/媒体状态 |
| `move_cmd` | 路由到 `/uran/core/downlink/move_cmd` |
| `task_ctrl` | 路由到 `/uran/core/downlink/task_ctrl` |
| `media_ctrl` | 路由到 `/uran/core/downlink/media_ctrl` |
| `frpc_ctrl` | 路由到 `/uran/core/downlink/frpc_ctrl` |
| `param_update` | 路由到 `/uran/core/downlink/param_update` |
| `state_query` | 直接响应状态查询 |

#### 常用调试命令

启动本地 broker：

```bash
sudo apt install mosquitto mosquitto-clients
mosquitto -p 1883 -v
```

监听上行：

```bash
mosquitto_sub -h localhost -t '/oivs/default/device_002/up' -v
```

发送模式切换：

```bash
mosquitto_pub -h localhost -p 1884 -t '/oivs/default/device_002/down' -m '{
  "msg_type": "control_switch",
  "msg_version": "1.0",
  "device_id": "device_002",
  "timestamp_ms": 0,
  "switch": {
    "control_mode": "manual",
    "controller": "cloud",
    "primary_uplink_protocol": "mqtt",
    "media": {"action": "stop", "protocol": ""}
  }
}'
```

发送媒体控制：

```bash
mosquitto_pub -h localhost -p 1884 -t '/oivs/default/device_002/down' -m '{
  "msg_type": "media_ctrl",
  "msg_version": "1.0",
  "device_id": "device_002",
  "timestamp_ms": 0,
  "action": "start",
  "protocol": "webrtc",
  "channel_id": "color",
  "signal_json": ""
}'
```

### 配置文件

#### `src/uran_core/config/network.yaml`

当前默认配置示例：

```yaml
network:
  device_id: "device_002"
  template_id: "template_002"
  tenant_id: "default"
  auth:
    username: "cynlux_device"
    token: "cynlux_device"
    cert_path: ""
  mqtt:
    enabled: true
    broker_host: "43.138.18.163"
    broker_port: 1884
    keepalive: 60
    topic_prefix: "/oivs/{tenant_id}/{device_id}"
  websocket:
    enabled: false
    url: ""
  tcp:
    enabled: false
    host: ""
    port: 0
  udp:
    enabled: false
    host: ""
    port: 0
  heartbeat_interval_ms: 5000
```

#### `src/uran_core/config/core.yaml`

```yaml
uran_core:
  state_broadcast_interval_ms: 1000
  state_report_interval_ms: 10000
  state_report_protocol: "mqtt"
  state_report_on_change: true
  db_path: "/tmp/uran_core_state.db"
```

### ROS 接口

#### 发布的 Topic

| Topic | 消息类型 | 说明 |
|-------|---------|------|
| `/uran/core/state/broadcast` | `StateSnapshot` | 状态空间全量快照 |
| `/uran/core/heartbeat/status` | `HeartbeatStatus` | 心跳发送状态 |
| `/uran/core/switch/mode` | `ModeSwitchCmd` | 模式切换通知 |
| `/uran/core/switch/media` | `MediaSwitchCmd` | 媒体切换通知 |
| `/uran/core/switch/uplink_protocol` | `UplinkProtocolCmd` | 上行协议切换 |
| `/uran/core/downlink/move_cmd` | `UnifiedMoveCmd` | 运控指令下发 |
| `/uran/core/downlink/task_ctrl` | `TaskCtrlCmd` | 任务控制 |
| `/uran/core/downlink/media_ctrl` | `MediaCtrlCmd` | 媒体控制 |
| `/uran/core/downlink/frpc_ctrl` | `FrpcCtrlCmd` | FRP 控制 |
| `/uran/core/downlink/param_update` | `ParamUpdateCmd` | 参数更新 |

#### 订阅的 Topic

| Topic | 消息类型 | 说明 |
|-------|---------|------|
| `/uran/core/state/write` | `StateField` | 功能包写入状态 |
| `/uran/core/uplink/data` | `UplinkPayload` | 功能包统一上报入口 |

#### 提供的 Service

| Service | 类型 | 说明 |
|---------|------|------|
| `/uran/core/state/get` | `GetStateField` | 查询状态字段 |
| `/uran/core/state/set` | `SetStateField` | 写入状态字段 |
| `/uran/core/network/connect` | `ConnectProtocol` | 连接/断开协议 |
| `/uran/core/network/status` | `GetNetworkStatus` | 查询网络状态 |
| `/uran/core/state_report/trigger` | `TriggerStateReport` | 手动触发上报 |
| `/uran/core/state_report/configure` | `ConfigureStateReport` | 修改上报参数 |

---

## uran_media 使用说明

`uran_media` 是 ROS1 版本的媒体主节点，负责：
- 订阅 RealSense 原生 ROS 图像话题
- 基于 **aiortc** 建立标准 WebRTC PeerConnection
- 提供 RTSP 推流
- 支持本地录制

### 依赖安装

```bash
pip3 install aiortc av aioice opencv-python-headless numpy
sudo apt install python3-gi gir1.2-gst-rtsp-server-1.0 \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-x264 gstreamer1.0-rtsp
```

### 摄像头配置

配置文件：`src/uran_media/config/media.yaml`

当前仅使用 `ros_topic` 方式接入视频源，不再支持 `camera_service` 或 `realsense_lifecycle`。

当前默认视频源：

| channel_id | ROS 话题 | 说明 |
|-----------|---------|------|
| `color` | `/camera/color/image_raw` | RealSense 彩色图像 |
| `depth` | `/camera/depth/image_rect_raw` | RealSense 深度图 |
| `infra1` | `/camera/infra1/image_rect_raw` | RealSense 左红外 |
| `infra2` | `/camera/infra2/image_rect_raw` | RealSense 右红外 |
| `aligned_depth` | `/camera/aligned_depth_to_color/image_raw` | 对齐深度图 |

配置示例：

```yaml
uran_media:
  video_sources:
    - channel_id: "color"
      source_type: "ros_topic"
      ros_topic: "/camera/color/image_raw"
      msg_type: "sensor_msgs/Image"
      width: 640
      height: 480
      fps: 30
```

### WebRTC

ROS1 版本 `uran_media` 直接使用 `aiortc` 建立标准 WebRTC 通道，不再依赖外部桥接节点。

支持两种协商方式：
1. 设备生成 SDP Offer，云端/前端回传 SDP Answer
2. 云端/前端在 `start` 中携带 SDP Offer，设备返回 SDP Answer

WebRTC 启动流程：
1. 向 `/uran/core/downlink/media_ctrl` 发送 `start + protocol=webrtc`
2. `uran_media` 为对应 `channel_id` 创建 `RTCPeerConnection`
3. 若 `signal_json` 为空：设备生成 SDP Offer，并通过 `/uran/core/uplink/data` 上报 `data_type=media_signal`
4. 若 `signal_json.type == "offer"`：设备直接生成 SDP Answer，并通过 `/uran/core/uplink/data` 上报 `data_type=media_signal`
5. 双端继续交换 ICE Candidate
6. `uran_media` 完成协商并发送视频流

启动 WebRTC（设备先发 Offer）：

```bash
rostopic pub -1 /uran/core/downlink/media_ctrl uran_msgs/MediaCtrlCmd \
  '{action: "start", protocol: "webrtc", channel_id: "color", signal_json: ""}'
```

启动 WebRTC（前端先发 Offer）：

```bash
rostopic pub -1 /uran/core/downlink/media_ctrl uran_msgs/MediaCtrlCmd \
  '{action: "start", protocol: "webrtc", channel_id: "color", signal_json: "{\"type\":\"offer\",\"sdp\":\"...\"}"}'
```

监听信令上报：

```bash
rostopic echo /uran/core/uplink/data
```

### RTSP 推流

默认 RTSP 端口为 `8554`。

启动单路 RTSP：

```bash
rostopic pub -1 /uran/core/downlink/media_ctrl uran_msgs/MediaCtrlCmd \
  '{action: "start", protocol: "rtsp", channel_id: "depth", signal_json: ""}'
```

停止单路 RTSP：

```bash
rostopic pub -1 /uran/core/downlink/media_ctrl uran_msgs/MediaCtrlCmd \
  '{action: "stop", protocol: "rtsp", channel_id: "depth", signal_json: ""}'
```

监听 RTSP URL：

```bash
rostopic echo /uran/core/uplink/data
```

拉流示例：

```bash
ffplay rtsp://设备IP:8554/depth
```

### 本地录制

开始录制：

```bash
rostopic pub -1 /uran/core/downlink/media_ctrl uran_msgs/MediaCtrlCmd \
  '{action: "record_start", protocol: "", channel_id: "color", signal_json: ""}'
```

停止录制：

```bash
rostopic pub -1 /uran/core/downlink/media_ctrl uran_msgs/MediaCtrlCmd \
  '{action: "record_stop", protocol: "", channel_id: "color", signal_json: ""}'
```

默认录制目录：

```bash
ls /tmp/uran_media_record
```

### ROS 接口（media）

#### 订阅的 Topic

| Topic | 消息类型 | 说明 |
|-------|---------|------|
| `/uran/core/downlink/media_ctrl` | `MediaCtrlCmd` | 通道控制 |
| `/uran/core/switch/media` | `MediaSwitchCmd` | 全局媒体切换 |
| 各视频源话题 | `sensor_msgs/Image` / `sensor_msgs/CompressedImage` | 图像输入 |

#### 发布的 Topic

| Topic | 消息类型 | 说明 |
|-------|---------|------|
| `/uran/core/uplink/data` | `UplinkPayload` | Offer / ICE / RTSP URL / 录制完成通知 |
| `/uran/core/state/write` | `StateField` | 写入媒体状态 |

#### 上行 `data_type`

| data_type | 说明 |
|-----------|------|
| `media_signal` | WebRTC SDP / ICE 信令 |
| `media_rtsp_url` | RTSP 通道地址 |
| `media_upload` | 录制完成通知 |

---

## uran_move 使用说明

`uran_move` 是 URAN 的运动控制桥接节点，将 URAN-core 下发的统一运控指令（`UnifiedMoveCmd`）转换为 px4ctrl 所需的接口调用。

### 启动

```bash
roslaunch uran_move uran_move.launch
# 或
rosrun uran_move uran_move_node
```

### ROS 接口

| 方向 | Topic | 消息类型 | 说明 |
|------|-------|----------|------|
| 订阅 | `/uran/core/downlink/move_cmd` | `uran_msgs/UnifiedMoveCmd` | 来自 URAN-core 的统一运控指令 |
| 订阅 | `/mavros/rc/in` | `mavros_msgs/RCIn` | RC 遥控输入，起飞前安全检查 |
| 发布 | `position_cmd` | `quadrotor_msgs/PositionCommand` | 速度/悬停指令（20 Hz 保活） |
| 发布 | `takeoff_land` | `quadrotor_msgs/TakeoffLand` | 起降指令 |

> `position_cmd` 和 `takeoff_land` 为相对话题名，实际全路径取决于 px4ctrl 节点命名空间。

### action 字段映射

| `UnifiedMoveCmd.action` | px4ctrl 行为 |
|---|---|
| `takeoff` | 发布 `TakeoffLand(TAKEOFF)`（须通过起飞前 RC 检查） |
| `land` | 先停止 `PositionCommand` 保活约 `0.6s`，再发布 `TakeoffLand(LAND)` |
| `return_home` | 先停止 `PositionCommand` 保活约 `0.6s`，再发布 `TakeoffLand(LAND)`（px4ctrl 无 RTL 接口） |
| `emergency_stop` | 先停止 `PositionCommand` 保活约 `0.6s`，再发布 `TakeoffLand(LAND)`（尽力而为） |
| `stop` 或全零速度 | 发布零速 `PositionCommand`（原地悬停） |
| 空串（速度指令） | 发布带速度字段的 `PositionCommand` |

### 起飞前 RC 安全检查

发布起飞指令前，节点会检查 `/mavros/rc/in` 的以下通道（PWM 值）：

| 通道 | 索引 | 要求 | 说明 |
|------|------|------|------|
| CH3（油门） | 2 | 1400–1600 | 油门须处于中位 |
| CH5 | 4 | 1900–2100 | 须处于高位 |
| CH6 | 5 | 1900–2100 | 须处于高位 |

任一条件不满足时，指令被拒绝并输出 `WARN` 日志，不发送起飞指令。

### MQTT 运控指令格式

URAD-core 将 MQTT 下行消息路由为 `UnifiedMoveCmd` 后发布到 `/uran/core/downlink/move_cmd`。

**MQTT 下行 Topic：**
```
/oivs/{tenant_id}/{device_id}/down
```

**消息体示例（起飞）：**
```json
{
  "msg_type": "move_cmd",
  "msg_version": "1.0",
  "device_id": "<device_id>",
  "timestamp_ns": 1741564800000000000,
  "controller": "cloud",
  "action": "takeoff",
  "linear_vel_x": 0.0,
  "linear_vel_y": 0.0,
  "linear_vel_z": 0.0,
  "angular_vel_z": 0.0,
  "target_yaw": null
}
```

**消息体示例（速度控制，前进 1 m/s）：**
```json
{
  "msg_type": "move_cmd",
  "msg_version": "1.0",
  "device_id": "<device_id>",
  "timestamp_ns": 1741564800000000000,
  "controller": "cloud",
  "action": "",
  "linear_vel_x": 1.0,
  "linear_vel_y": 0.0,
  "linear_vel_z": 0.0,
  "angular_vel_z": 0.0,
  "target_yaw": null
}
```

**消息体示例（降落）：**
```json
{
  "msg_type": "move_cmd",
  "msg_version": "1.0",
  "device_id": "<device_id>",
  "timestamp_ns": 1741564800000000000,
  "controller": "cloud",
  "action": "land"
}
```

**坐标系约定（ROS REP-103 ENU 体坐标系）：**

| 字段 | 正方向 |
|------|--------|
| `linear_vel_x` | 前进 |
| `linear_vel_y` | 左移 |
| `linear_vel_z` | 上升 |
| `angular_vel_z` | 左转（逆时针，俯视） |
| `target_yaw` | ENU 世界系，0 = 正东，逆时针为正，NaN = 不指定 |

---

## 消息与服务定义

### uran_msgs

| 消息 | 用途 |
|------|------|
| `StateField` | 写入状态空间字段 |
| `StateSnapshot` | 状态空间全量快照 |
| `HeartbeatStatus` | 心跳发送状态 |
| `UnifiedMoveCmd` | 统一运控指令 |
| `ModeSwitchCmd` | 模式切换 |
| `MediaSwitchCmd` | 流媒体切换 |
| `UplinkProtocolCmd` | 上行协议切换 |
| `UplinkPayload` | 统一上行数据载体 |
| `MediaCtrlCmd` | 媒体控制 |
| `FrpcCtrlCmd` | 端口转发控制 |
| `TaskCtrlCmd` | 任务控制 |
| `ParamUpdateCmd` | 参数更新 |

### uran_srvs

| 服务 | 用途 |
|------|------|
| `GetStateField` | 查询状态字段 |
| `SetStateField` | 写入状态字段 |
| `ConnectProtocol` | 控制协议连接 |
| `GetNetworkStatus` | 查询网络状态 |
| `TriggerStateReport` | 触发即时状态上报 |
| `ConfigureStateReport` | 配置状态上报 |

查看接口定义：

```bash
rosmsg show uran_msgs/StateField
rossrv show uran_srvs/GetStateField
```

---

## 项目说明与迁移备注

### 1. ROS 版本迁移

本仓库 README 已按当前代码同步到 **ROS1 Noetic**，不再适用 ROS2 Humble 的构建与启动方式。

### 2. uran_move 方向调整

ROS1 版本 `uran_move` 已不再面向 CyberDog2。后续若恢复/补充运控实现，应以 **px4_mavros** 为统一适配方向。

### 3. uran_media 架构调整

相比早期方案，当前实现有三个明确变化：
- 使用 **aiortc** 做标准 WebRTC 实现
- 不再依赖面向特定平台的桥接流媒体方案
- 不再使用 `camera_service`，统一改为订阅 **RealSense 原生话题**

---

## 相关文档

- `URAN节点设计文档.md` — 完整架构与接口设计
- `云端与URAN节点通信字段手册.md` — 上下行通信字段参考
- `URAN开发任务清单.md` — 开发任务分解与测试方案
