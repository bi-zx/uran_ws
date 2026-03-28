# URAN 节点设计文档

**文档版本：v1.0**

**编写组：ROS开发组**

**更新时间：2026年3月10日**

---

## 目录

1. [概述](#一概述)
2. [整体架构](#二整体架构)
3. [软件包规范](#三软件包规范)
4. [URAN-core 核心包](#四uran-core-核心包)
5. [URAN-move 运控包](#五uran-move-运控包)
6. [URAN-media 流媒体包](#六uran-media-流媒体包)
7. [URAN-sensor 传感器包](#七uran-sensor-传感器包)
8. [URAN-frpcpoint 端口转发服务](#八uran-frpcpoint-端口转发服务)
9. [URAN-autotask 自动化巡检服务](#九uran-autotask-自动化巡检服务)
10. [跨组接口约定](#十跨组接口约定)
11. [二次开发扩展规范](#十一二次开发扩展规范)
12. [消息与服务定义汇总](#十二消息与服务定义汇总)

---

## 一、概述

### 1.1 定位

URAN（Unified Robotics Access Node，统一机器人接入节点）是 OIVS 系统在无人装备侧的统一接入与能力承载层。其核心职责如下：

- 将云端/网关/手持终端下发的统一信令，翻译为设备可执行的底层动作；
- 将设备的状态、传感数据与流媒体，按统一规范上报至 IPON 网络；
- 作为单个无人装备实体在 IPON 网络拓扑中的末端子节点，与 OIVS 云端控制台、IPON 网关、手持终端协同工作。

单个 URAN 节点对应一个装备实例（即一台实体设备），同一时刻最多接入一个 OIVS 单体最小系统。

### 1.2 交付形态

URAN 节点以 **ROS 2 软件包**（兼顾 ROS 1）形态交付，采用"核心包 + 可扩展包"架构：

| 软件包             | 职责摘要                                                           |
| ------------------ | ------------------------------------------------------------------ |
| **URAN-core**      | 状态空间、多协议入网、心跳、控制切换、上下行数据通路、定期状态上报 |
| **URAN-move**      | 统一运控指令接收 → 厂商 SDK/ROS 接口转换 → 执行闭环                |
| **URAN-media**     | WebRTC/RTSP 流媒体信令与通道管理                                   |
| **URAN-sensor**    | 多源传感器数据采集、转换、上报                                     |
| **URAN-frpcpoint** | 反向隧道/端口转发（远程运维调试）                                  |
| **URAN-autotask**  | 自动化巡检任务生命周期管理与接口                                   |

所有功能包均以 **URAN-core 为唯一依赖前提**，通过约定的 ROS topic/service 接口与核心包解耦对接。

### 1.3 设计原则

1. **解耦性**：各功能包仅通过标准 ROS 接口与 URAN-core 交互，不直接互相依赖；
2. **可扩展性**：通过插件接口支持第三方指令转换逻辑、传感器适配逻辑，以及自定义协议接入；
3. **冗余容错**：维护多协议通路表，关键信令在主通路不可达时自动切换备用通路；
4. **统一规范**：心跳包、控制切换信令、运控指令、传感器数据均采用统一字段格式，版本化管理；
5. **工业级可靠性**：软件包应具备相对独立性，支持克隆后直接编译，适配 ROS 2 各发行版。

---

## 二、整体架构

### 2.1 模块关系图

```
┌─────────────────────────────────────────────────────────┐
│                   IPON 网络（上行方向）                   │
│   MQTT / WebSocket / TCP / UDP / LoRA / Zigbee / BLE    │
└───────────────────────┬─────────────────────────────────┘
                        │ 上行数据 / 下行信令
┌───────────────────────▼─────────────────────────────────┐
│                     URAN-core                           │
│  ┌──────────────┐  ┌─────────────┐  ┌────────────────┐  │
│  │  状态空间     │  │ 多协议通路表 │  │ 控制切换信令处理│  │
│  └──────────────┘  └─────────────┘  └────────────────┘  │
│  ┌──────────────────────────────────────────────────┐    │
│  │          数据上下行通路（转发路由）               │    │
│  └──────────────────────────────────────────────────┘    │
└────┬──────────┬──────────┬──────────┬──────────┬────────┘
     │          │          │          │          │
     ▼          ▼          ▼          ▼          ▼
URAN-move  URAN-media URAN-sensor URAN-frpc  URAN-autotask
  运控包    流媒体包    传感器包   point端口   自动巡检服务
                               转发服务
     │
     ▼
厂商 SDK / ROS 控制接口（设备底层）
```

### 2.2 数据流方向

**下行（云端 → 设备）：**

```
云端控制台/网关/手持终端
    → URAN-core（协议接收 + 路由分发）
        → URAN-move（运控指令执行）
        → URAN-autotask（任务控制信令）
        → URAN-media（媒体通道切换信令）
        → URAN-frpcpoint（端口映射控制）
```

**上行（设备 → 云端）：**

```
传感器/底层驱动/摄像头
    → URAN-sensor / URAN-media / URAN-move（数据产生）
        → URAN-core（状态空间写入 + 协议通路选择）
            → 云端控制台/网关/手持终端
```

---

## 三、软件包规范

### 3.1 命名规范

| 元素         | 规范                                                                                     |
| ------------ | ---------------------------------------------------------------------------------------- |
| 包名         | `uran_core`、`uran_move`、`uran_media`、`uran_sensor`、`uran_frpcpoint`、`uran_autotask` |
| 节点名       | `uran_core_node`、`uran_move_node` 等，保持与包名一致                                    |
| Topic 前缀   | `/uran/<package_short>/...`                                                              |
| Service 前缀 | `/uran/<package_short>/...`                                                              |
| Action 前缀  | `/uran/<package_short>/...`                                                              |
| 自定义消息包 | `uran_msgs`                                                                              |
| 自定义服务包 | `uran_srvs`                                                                              |

### 3.2 消息/服务版本管理

- 所有自定义消息与服务定义集中在 `uran_msgs` 包中统一维护；
- 字段变更遵循向后兼容原则，重大变更需升级消息版本号（在字段 `msg_version` 中标注）；
- 云端与设备侧字段命名保持一致，变更须同步与软件组协定。

### 3.3 配置文件规范

- 每个软件包通过 `config/<package>.yaml` 管理可配参数；
- URAN-core 启动时从云端拉取状态空间元数据，覆盖本地默认配置；
- 插件转换逻辑通过 `config/plugins.yaml` 进行注册与切换。

---

## 四、URAN-core 核心包

### 4.1 功能概览

URAN-core 是整个 URAN 节点的核心总线，承担以下职责：

1. **状态空间管理**：集中存储设备运行态与业务字段，提供内部共享数据源；
2. **多协议入网与心跳**：管理 MQTT、WebSocket、TCP、UDP 及私有协议连接，以统一格式周期上报心跳；
3. **控制切换处理**：接收并分发控制切换信令，协调各功能包进入对应模式；
4. **数据上下行通路**：统一路由上行数据与下行信令，维护多协议通路表。

### 4.2 状态空间

#### 4.2.1 数据分类

| 类型         | 说明                                                        | 存储方式                            |
| ------------ | ----------------------------------------------------------- | ----------------------------------- |
| **非持久化** | 运行态数据（在线状态、当前控制者、通路可用性、姿态、速度…） | 内存（ROS 参数服务器 / 内存结构体） |
| **持久化**   | 关键配置与审计数据（设备 ID、历史报警、任务记录…）          | SQLite / CSV 文件                   |

#### 4.2.2 元数据拉取与注册流程

URAN-core 本地配置文件中预置 `device_id`（装备实例主键）与 `template_id`（装备模板主键），启动时以此为凭据向云端完成注册校验并拉取两层元数据，完成状态空间初始化：

```
URAN-core 启动
    ├─ 1. 读取本地 config/network.yaml，取得 device_id（实例主键）与 template_id（模板主键）
    ├─ 2. 连接云端控制台，携带 device_id + template_id + token 发起注册请求
    ├─ 3. 云端校验 device_id 占用状态：
    │       ├─ device_id 已被另一连接占用
    │       │       → 拒绝连接（冲突），本地记录错误日志，节点退出
    │       ├─ device_id 已存在且无活跃连接
    │       │       → 云端标记该实例为"在线"，返回注册确认（registered）
    │       └─ device_id 不存在于数据库
    │               → 云端自动创建装备实例记录，绑定 template_id
    │               → 标记"在线"，返回注册确认（auto_registered）
    ├─ 4. 以 template_id 拉取"装备模板"公共字段元数据
    │       （物模型字段定义、线速度/角速度等公共参数默认值）
    ├─ 5. 以 device_id 拉取"装备实例"特有字段元数据
    │       （自动注册时返回空，使用模板默认值；合并方式：追加 / 覆写）
    └─ 6. 初始化本地状态空间结构，将 device_id 与 template_id 写入持久化存储
```

云端注册响应结构：

```json
{
  "result": "registered" | "auto_registered" | "rejected",
  "reason": "",        // 仅 rejected 时填写，如 "device_id_conflict"
  "device_id": "...",
  "template_id": "..."
}
```

> **设计说明**：云端对 `device_id` 的唯一约束是**不允许并发占用**——同一 `device_id` 已有活跃连接时拒绝新连接，以防止状态混乱。`device_id` 不存在不构成拒绝条件，云端将自动创建对应装备实例记录并绑定请求中携带的 `template_id`。`template_id` 须在云端预先存在（描述该类设备的通用物模型），自动注册的实例以模板默认值初始化状态空间。

#### 4.2.3 状态空间字段模板（标准字段）

| 字段名                    | 类型   | 持久化 | 说明                                                                              |
| ------------------------- | ------ | ------ | --------------------------------------------------------------------------------- |
| `device_id`               | string | 是     | **装备实例**主键（后端数据库唯一标识，对应一台实体设备）                          |
| `template_id`             | string | 是     | **装备模板**主键（标识该设备所属模板/型号类别）                                   |
| `device_type`             | string | 是     | 设备类型（quadruped/uav/usv/…）                                                   |
| `online_status`           | bool   | 否     | 当前在线状态                                                                      |
| `current_controller`      | string | 否     | 当前控制者（cloud/field/auto）                                                    |
| `control_mode`            | string | 否     | 当前巡检模式（manual/auto）                                                       |
| `primary_uplink_protocol` | string | 否     | 当前主上行协议（mqtt/ws/tcp/…）                                                   |
| `protocol_table`          | object | 否     | 各协议通路可用性表（见 4.4.2）                                                    |
| `battery_level`           | float  | 否     | 电量百分比                                                                        |
| `position`                | object | 否     | GPS 定位（lat/lon/alt）                                                           |
| `velocity`                | object | 否     | 当前速度（vx/vy/vz）                                                              |
| `attitude`                | object | 否     | 姿态（roll/pitch/yaw）                                                            |
| `linear_vel_limit`        | float  | 是     | 合速度上限（m/s，作用于 `linear_vel_x/y/z` 的向量模长），由装备模板下发，默认 1.0 |
| `angular_vel_limit`       | float  | 是     | 偏航角速度上限（rad/s，作用于 `angular_vel_z` 绝对值），由装备模板下发，默认 1.0  |
| `error_code`              | int    | 否     | 当前错误码（0=正常）                                                              |
| `task_id`                 | string | 否     | 当前执行任务 ID（无任务时为空）                                                   |
| `task_stage`              | string | 否     | 当前任务阶段                                                                      |
| `uptime_seconds`          | int    | 否     | 节点运行时长（秒）                                                                |

> 云端可通过装备模板/实例配置追加或覆写上述字段。

#### 4.2.4 状态空间接口

**ROS Topic（写入状态空间）**

| Topic                        | 消息类型                  | 方向          | 说明                       |
| ---------------------------- | ------------------------- | ------------- | -------------------------- |
| `/uran/core/state/write`     | `uran_msgs/StateField`    | 功能包 → core | 任意功能包写入状态空间字段 |
| `/uran/core/state/broadcast` | `uran_msgs/StateSnapshot` | core → 功能包 | 定期广播当前状态空间快照   |

**ROS Service（查询状态空间）**

| Service                | 请求/响应类型             | 说明                         |
| ---------------------- | ------------------------- | ---------------------------- |
| `/uran/core/state/get` | `uran_srvs/GetStateField` | 查询单个或多个字段当前值     |
| `/uran/core/state/set` | `uran_srvs/SetStateField` | 同步写入字段（含持久化标志） |

**`uran_msgs/StateField` 消息结构**

```
string  field_name       # 字段名
string  value_json       # JSON 序列化值
bool    persistent       # 是否需要持久化写入
string  source_pkg       # 来源软件包名
uint64  timestamp_ns     # 写入时间戳（纳秒）
```

**`uran_msgs/StateSnapshot` 消息结构**

```
string  msg_version      # 消息版本
uint64  timestamp_ns
string  device_id
string  fields_json      # 全量字段的 JSON 序列化
```

### 4.3 多协议入网与心跳

#### 4.3.1 支持的协议

| 协议                   | 类别     | 优先级              | 备注                   |
| ---------------------- | -------- | ------------------- | ---------------------- |
| MQTT                   | 互联网   | 主通路（默认）      | 心跳主通路             |
| WebSocket              | 互联网   | 次通路              | 双向低延迟             |
| TCP                    | 互联网   | 备用通路            | 稳定传输               |
| UDP                    | 互联网   | 备用通路            | 低延迟，可用于实时控制 |
| LoRA                   | 私有网络 | 网关中转            | 低带宽、远距离         |
| Zigbee                 | 私有网络 | 网关中转            | 低功耗短距离           |
| Bluetooth              | 私有网络 | 网关中转 / 手持直连 | 近场直连               |
| 40GHz / 60GHz 私有协议 | 私有网络 | 扩展预留            | 接口预留               |

WebRTC 仅用于流媒体通道，不参与心跳与控制信令传输。

#### 4.3.2 入网配置接口

**配置文件 `config/network.yaml` 结构**

```yaml
network:
  device_id: "device_001"       # 装备实例主键（与云端后端数据库对应）
  template_id: "template_001"   # 装备模板主键（用于拉取公共物模型字段元数据）
  auth:
    token: "xxxxxxxx"          # 云端下发的鉴权 Token
    cert_path: ""              # 证书路径（可选）
  mqtt:
    enabled: true
    broker_host: "mqtt.example.com"
    broker_port: 1883
    keepalive: 60
    topic_prefix: "/oivs/{tenant}/{device_id}"
  websocket:
    enabled: true
    url: "wss://ws.example.com/uran"
  tcp:
    enabled: false
    host: ""
    port: 0
  udp:
    enabled: false
    host: ""
    port: 0
  lora:
    enabled: false
    serial_port: "/dev/ttyUSB0"
    baud_rate: 9600
  zigbee:
    enabled: false
    serial_port: "/dev/ttyUSB1"
    baud_rate: 115200
  bluetooth:
    enabled: false
  heartbeat_interval_ms: 5000  # 心跳间隔（毫秒）
  ipv6_enabled: false           # 是否启用 IPv6 直连
```

#### 4.3.3 统一心跳包格式

所有协议的心跳包采用统一 JSON 结构，以保证任一可达通路均可被云端判断在线状态：

```json
{
  "msg_type": "heartbeat",
  "msg_version": "1.0",
  "device_id": "device_001",
  "timestamp_ms": 1741564800000,
  "online": true,
  "uptime_seconds": 3600,
  "battery_level": 85.5,
  "control_mode": "manual",
  "current_controller": "cloud",
  "primary_uplink_protocol": "mqtt",
  "protocol_table": {
    "mqtt": true,
    "websocket": true,
    "tcp": false,
    "udp": false,
    "lora": false
  },
  "position": {
    "lat": 39.9042,
    "lon": 116.4074,
    "alt": 50.0
  },
  "error_code": 0
}
```

> **注意**：WebRTC 协议不参与心跳上报。

#### 4.3.4 心跳相关 ROS 接口

**ROS Topic**

| Topic                         | 消息类型                    | 方向          | 说明                 |
| ----------------------------- | --------------------------- | ------------- | -------------------- |
| `/uran/core/heartbeat/status` | `uran_msgs/HeartbeatStatus` | core → 功能包 | 当前心跳发送状态广播 |

**ROS Service**

| Service                      | 请求/响应类型                | 说明                       |
| ---------------------------- | ---------------------------- | -------------------------- |
| `/uran/core/network/connect` | `uran_srvs/ConnectProtocol`  | 请求连接或断开某一协议通路 |
| `/uran/core/network/status`  | `uran_srvs/GetNetworkStatus` | 查询当前各协议连接状态     |

### 4.4 控制切换信令处理

#### 4.4.1 控制切换维度

URAN-core 处理三类控制切换：

| 维度           | 说明                       | 影响范围                        |
| -------------- | -------------------------- | ------------------------------- |
| 巡检模式切换   | 自动 ↔ 手动                | URAN-move、URAN-autotask        |
| 控制者切换     | 云端 / 现场手持 / 自动任务 | URAN-move、URAN-autotask        |
| 上行协议切换   | 主上行协议变更             | URAN-sensor、URAN-core 上行路由 |
| 流媒体通道切换 | WebRTC / RTSP 切换或开关   | URAN-media                      |

#### 4.4.2 控制切换信令格式（下行）

```json
{
  "msg_type": "control_switch",
  "msg_version": "1.0",
  "device_id": "device_001",
  "timestamp_ms": 1741564800000,
  "switch": {
    "control_mode": "auto",            // "manual" | "auto" | null（不变）
    "controller": "cloud",             // "cloud" | "field" | "auto" | null
    "primary_uplink_protocol": "mqtt", // "mqtt" | "websocket" | "tcp" | null
    "media": {
      "action": "start",               // "start" | "stop" | "switch"
      "protocol": "webrtc"             // "webrtc" | "rtsp" | null
    }
  }
}
```

#### 4.4.3 控制切换处理流程

```
接收控制切换信令（来自 MQTT / WebSocket / TCP / LoRA…）
    │
    ├─ 更新状态空间（control_mode / current_controller / primary_uplink_protocol）
    │
    ├─ 若 control_mode / controller 变更
    │       → 发布 /uran/core/switch/mode（→ URAN-move, URAN-autotask）
    │
    ├─ 若 media 变更
    │       → 发布 /uran/core/switch/media（→ URAN-media）
    │
    └─ 若 primary_uplink_protocol 变更
            → 更新通路表，通知各功能包按新协议偏好上报
```

#### 4.4.4 控制切换 ROS 接口

**ROS Topic（URAN-core 发布，功能包订阅）**

| Topic                               | 消息类型                      | 说明                |
| ----------------------------------- | ----------------------------- | ------------------- |
| `/uran/core/switch/mode`            | `uran_msgs/ModeSwitchCmd`     | 模式/控制者切换通知 |
| `/uran/core/switch/media`           | `uran_msgs/MediaSwitchCmd`    | 流媒体通道切换通知  |
| `/uran/core/switch/uplink_protocol` | `uran_msgs/UplinkProtocolCmd` | 上行协议切换通知    |

**`uran_msgs/ModeSwitchCmd` 消息结构**

```
string  control_mode       # "manual" | "auto"
string  controller         # "cloud" | "field" | "auto"
uint64  timestamp_ns
```

**`uran_msgs/MediaSwitchCmd` 消息结构**

```
string  action             # "start" | "stop" | "switch"
string  protocol           # "webrtc" | "rtsp"
uint64  timestamp_ns
```

**`uran_msgs/UplinkProtocolCmd` 消息结构**

```
string  protocol           # "mqtt" | "websocket" | "tcp" | "udp" | "lora"
uint64  timestamp_ns
```

### 4.5 数据上下行通路与通路表

#### 4.5.1 上行通路（设备 → 云端）

URAN-core 作为统一上行出口：

- 各功能包将需要上报的数据发送至 URAN-core 的上行接收 Topic；
- URAN-core 根据数据指定的协议偏好与当前通路表，选择可用协议进行发送；
- 若指定协议不可用，自动降级至备用协议。

**ROS Topic（功能包 → URAN-core）**

| Topic                    | 消息类型                  | 说明                     |
| ------------------------ | ------------------------- | ------------------------ |
| `/uran/core/uplink/data` | `uran_msgs/UplinkPayload` | 功能包上报数据的统一入口 |

**`uran_msgs/UplinkPayload` 消息结构**

```
string  source_pkg          # 来源软件包
string  data_type           # 数据类型标识（"sensor" / "state" / "event" / "media_signal"）
string  preferred_protocol  # 偏好上行协议，空串表示跟随当前主协议
string  payload_json        # JSON 序列化的数据体
bool    urgent              # 是否紧急（紧急数据优先发送，失败时枚举所有通路重试）
uint64  timestamp_ns
```

#### 4.5.2 下行通路（云端 → 设备）

URAN-core 接收下行信令并按消息类型路由：

| 下行消息类型 (`msg_type`) | 路由目标                             |
| ------------------------- | ------------------------------------ |
| `control_switch`          | URAN-core 内部处理，发布到对应 Topic |
| `move_cmd`                | `/uran/move/cmd_recv`                |
| `task_ctrl`               | `/uran/autotask/ctrl_recv`           |
| `media_ctrl`              | `/uran/media/ctrl_recv`              |
| `frpc_ctrl`               | `/uran/frpcpoint/ctrl_recv`          |
| `state_query`             | URAN-core 直接响应，返回状态空间字段 |
| `param_update`            | URAN-core 更新配置并广播至各功能包   |

**ROS Topic（URAN-core 下发，功能包订阅）**

| Topic                              | 消息类型                   | 说明                 |
| ---------------------------------- | -------------------------- | -------------------- |
| `/uran/core/downlink/move_cmd`     | `uran_msgs/UnifiedMoveCmd` | 运控指令下发         |
| `/uran/core/downlink/task_ctrl`    | `uran_msgs/TaskCtrlCmd`    | 任务控制指令下发     |
| `/uran/core/downlink/media_ctrl`   | `uran_msgs/MediaCtrlCmd`   | 媒体通道控制指令下发 |
| `/uran/core/downlink/frpc_ctrl`    | `uran_msgs/FrpcCtrlCmd`    | 端口转发控制指令下发 |
| `/uran/core/downlink/param_update` | `uran_msgs/ParamUpdateCmd` | 参数更新广播         |

#### 4.5.3 协议通路表

URAN-core 在状态空间中维护实时通路表，定期对各协议进行可达性探测：

```json
{
  "protocol_table": {
    "mqtt":      { "available": true,  "latency_ms": 45,  "last_check_ts": 1741564800 },
    "websocket": { "available": true,  "latency_ms": 60,  "last_check_ts": 1741564800 },
    "tcp":       { "available": false, "latency_ms": -1,  "last_check_ts": 1741564800 },
    "udp":       { "available": false, "latency_ms": -1,  "last_check_ts": 1741564800 },
    "lora":      { "available": false, "latency_ms": -1,  "last_check_ts": 1741564800 }
  }
}
```

通路优先级降级策略（可配置）：

```
优先使用 preferred_protocol
    → 若不可用，依次尝试 mqtt → websocket → tcp → udp → lora
    → 若全部不可用，缓存数据并等待通路恢复（urgent 数据持续重试）
```

### 4.6 定期状态上报

URAN-core 除被动转发各功能包数据外，还维护两条独立的定期主动上报机制：对内的状态空间广播（供各功能包使用）和对外的状态快照上报（发往云端/网关）。

#### 4.6.1 对内广播（URAN-core → 功能包）

URAN-core 以固定周期将当前状态空间完整快照广播至 `/uran/core/state/broadcast`，各功能包可按需订阅，用于同步运行态数据（如当前控制模式、主上行协议等）。

**触发机制**：定时器驱动，周期可通过配置文件设定。

**配置项（`config/core.yaml`）**

```yaml
uran_core:
  state_broadcast_interval_ms: 1000   # 对内广播周期，默认 1s
```

#### 4.6.2 对外上报（URAN-core → 云端/网关）

URAN-core 以独立定时器周期性构建 `state_snapshot` 数据包，通过主上行协议（优先 MQTT）上报至云端控制台，供云端刷新设备物模型字段与在线展示。

**上报内容**：从当前状态空间中提取全量字段，序列化为 JSON 后封装进 `UplinkPayload`（`data_type="state_snapshot"`），经上行通路发送。

**触发机制**：

```
┌─────────────────────────────────────────┐
│         定时器（state_report_timer）     │
│   周期到达 → 读取状态空间全量字段        │
│           → 封装 UplinkPayload          │
│           → 写入上行通路队列             │
│           → 按通路表选择可用协议发送     │
└─────────────────────────────────────────┘

此外，以下事件触发即时（非周期）上报：
  - 控制切换信令处理完成
  - 功能包写入 urgent=true 的 StateField
  - online_status 或 error_code 发生变化
```

**配置项（`config/core.yaml`）**

```yaml
uran_core:
  state_report_interval_ms: 10000     # 对外定期上报周期，默认 10s
  state_report_protocol: "mqtt"       # 首选上行协议，空串表示跟随主通路
  state_report_on_change: true        # 关键字段变更时是否立即触发即时上报
  state_report_change_fields:         # 触发即时上报的关键字段列表
    - "online_status"
    - "error_code"
    - "control_mode"
    - "current_controller"
    - "battery_level"
```

#### 4.6.3 上报数据结构

上报包复用 `UplinkPayload`，`payload_json` 内容为状态空间全量字段 JSON（与 `StateSnapshot` 消息的 `fields_json` 格式一致）：

```json
{
  "msg_type": "state_snapshot",
  "msg_version": "1.0",
  "device_id": "device_001",
  "timestamp_ns": 1741564800000000000,
  "trigger": "periodic",             // "periodic" | "change" | "switch"
  "fields": {
    "online_status": true,
    "control_mode": "manual",
    "current_controller": "cloud",
    "primary_uplink_protocol": "mqtt",
    "battery_level": 85.5,
    "position": { "lat": 39.9042, "lon": 116.4074, "alt": 50.0 },
    "attitude": { "roll": 0.01, "pitch": 0.02, "yaw": 1.57 },
    "velocity": { "vx": 0.0, "vy": 0.0, "vz": 0.0 },
    "error_code": 0,
    "task_id": "",
    "protocol_table": {
      "mqtt":      { "available": true,  "latency_ms": 45 },
      "websocket": { "available": true,  "latency_ms": 60 },
      "tcp":       { "available": false, "latency_ms": -1 }
    }
  }
}
```

> **`trigger` 字段说明**：`"periodic"` 为定时触发；`"change"` 为关键字段变更触发；`"switch"` 为控制切换完成后触发。云端可据此区分主动推送与变更通知，按需更新展示。

#### 4.6.4 定期上报相关 ROS 接口

**ROS Service**

| Service                             | 请求/响应类型                    | 说明                         |
| ----------------------------------- | -------------------------------- | ---------------------------- |
| `/uran/core/state_report/trigger`   | `uran_srvs/TriggerStateReport`   | 手动触发一次即时状态快照上报 |
| `/uran/core/state_report/configure` | `uran_srvs/ConfigureStateReport` | 运行时修改上报周期与协议     |

**`uran_srvs/TriggerStateReport` 定义**

```
# Request
string  reason            # 触发原因描述（可空）
---
# Response
bool    success
string  message
```

**`uran_srvs/ConfigureStateReport` 定义**

```
# Request
uint32  interval_ms       # 新的上报周期（0 表示不修改）
string  protocol          # 新的首选协议（空串表示不修改）
bool    report_on_change  # 是否开启变更即时上报
---
# Response
bool    success
string  message
uint32  current_interval_ms
string  current_protocol
```

---

## 五、URAN-move 运控包

### 5.1 功能概览

URAN-move 负责：

1. 接收 URAN-core 转发的统一运控指令；
2. 通过可插拔的转换逻辑插件，将统一指令转换为设备厂商特有指令格式；
3. 调用厂商 SDK 或 ROS 控制接口执行运动/姿态控制；
4. 将执行结果回报 URAN-core，更新状态空间并上报云端，实现指令闭环。

### 5.2 坐标系约定

**所有 `UnifiedMoveCmd` 中的线速度、角速度与姿态角字段，统一采用 [ROS REP-103](https://www.ros.org/reps/rep-0103.html) 规定的右手体坐标系（Body Frame，ENU 惯例）：**

| 轴               | 正方向         | 说明                 |
| ---------------- | -------------- | -------------------- |
| x                | 机体**前方**   | 前进为正，后退为负   |
| y                | 机体**左方**   | 左移为正，右移为负   |
| z                | 机体**上方**   | 上升为正，下降为负   |
| yaw（绕 z 轴）   | **逆时针**为正 | 从上方俯视，左转为正 |
| roll（绕 x 轴）  | 右手定则       | 右滚为正             |
| pitch（绕 y 轴） | 右手定则       | 抬头（机头上仰）为负 |

> **安全说明**：不同平台（PX4 使用 NED 坐标系，MAVROS local frame 使用 ENU 世界系等）的坐标系惯例各不相同，直接透传会导致方向解释偏差并引发安全事故。**坐标系转换由各插件内部负责实现**，云端、手持终端、URAN-autotask 下发指令时始终使用本节定义的体坐标系，无需关心底层平台差异。

### 5.3 统一运控指令格式

URAN-move 接收来自 URAN-core 的统一运控指令，格式固定如下：

**`uran_msgs/UnifiedMoveCmd` 消息结构**

```
string  msg_version             # 消息版本
string  device_id
uint64  timestamp_ns
string  controller              # 指令来源（"cloud" / "field" / "auto"）

# 运动控制（坐标系：ROS REP-103 右手体坐标系，x前/y左/z上）
float64 linear_vel_x            # 前进速度（m/s）：正 = 前，负 = 后
float64 linear_vel_y            # 横向速度（m/s）：正 = 左，负 = 右
float64 linear_vel_z            # 垂直速度（m/s）：正 = 上，负 = 下（无人机/USV等）
float64 angular_vel_z           # 偏航角速度（rad/s）：正 = 左转（逆时针，俯视）

# 姿态控制（可选，设备不支持时忽略；角度单位 rad，右手体坐标系）
float64 target_roll             # 目标横滚角（rad）：正 = 右滚
float64 target_pitch            # 目标俯仰角（rad）：正 = 低头（机头下俯）
float64 target_yaw              # 目标偏航角（rad）：绝对值，NaN 表示不指定；
                                # 参考系为 ENU 世界系，0 = 正东，逆时针为正

# 特殊动作
string  action                  # 特殊动作标识，空串表示无特殊动作
                                # 常用值："stop" / "emergency_stop" /
                                #         "takeoff" / "land" / "return_home"

# 扩展字段（插件自定义解析）
string  extra_json              # JSON 序列化的扩展参数
```

> **约定**：云端/手持端/自动任务下发时，均使用此固定格式，坐标系始终为体坐标系（见 5.2）。URAN-move 各插件内部负责将其转换为目标平台所需的坐标系与指令格式。

### 5.3.1 字段单位与有效范围

| 字段            | 单位  | 有效范围                                                     | 超限处理               |
| --------------- | ----- | ------------------------------------------------------------ | ---------------------- |
| `linear_vel_x`  | m/s   | `[-linear_vel_limit, +linear_vel_limit]`（由状态空间读取）   | 插件内部 clamp         |
| `linear_vel_y`  | m/s   | `[-linear_vel_limit, +linear_vel_limit]`                     | 插件内部 clamp         |
| `linear_vel_z`  | m/s   | `[-linear_vel_limit, +linear_vel_limit]`                     | 插件内部 clamp         |
| `angular_vel_z` | rad/s | `[-angular_vel_limit, +angular_vel_limit]`（由状态空间读取） | 插件内部 clamp         |
| `target_roll`   | rad   | 设备平台决定，通常 `[-π/4, +π/4]`                            | 插件内部 clamp 或忽略  |
| `target_pitch`  | rad   | 设备平台决定，通常 `[-π/4, +π/4]`                            | 插件内部 clamp 或忽略  |
| `target_yaw`    | rad   | `[-π, +π]` 或 NaN（不指定）                                  | NaN 时插件保持当前偏航 |

> **线速度合速度约束**：clamp 不对单轴独立截断，而是对合速度向量模长做比例缩放，保持方向不变：
>
> $$
> v_{\text{norm}} = \sqrt{v_x^2 + v_y^2 + v_z^2}
> $$
>
> $$
> \text{若 } v_{\text{norm}} > v_{\text{limit}}, \text{ 则 } v_x, v_y, v_z \mathrel{*}= \frac{v_{\text{limit}}}{v_{\text{norm}}}
> $$

### 5.3.2 限速保护机制

URAN-move 在调用插件 `execute()` 前进行**预检限速**，插件内部在发送给平台 SDK 前再做**二次 clamp**，形成双重保护：

```
云端下发 UnifiedMoveCmd
    │
    ▼
URAN-move 接收（预检限速层）
    ├─ 从状态空间读取 linear_vel_limit、angular_vel_limit
    ├─ 计算合速度向量模长，若超限则等比缩放 linear_vel_x/y/z
    ├─ 对 angular_vel_z 取 clamp(val, -limit, +limit)
    ├─ 若发生截断，将截断事件写入状态空间并通过 uplink 上报（data_type="move_clamp_event"）
    └─ 将处理后的 cmd 传入插件 execute()
            │
            ▼
        插件内部（二次 clamp 层）
            ├─ 针对平台 SDK 的物理约束再次 clamp（如 PX4 最大速度由飞控参数决定）
            └─ 发送给底层 SDK / ROS 接口
```

**`move_clamp_event` 上报结构（`payload_json` 中）**

```json
{
  "event": "move_clamp",
  "original": { "vx": 2.5, "vy": 0.0, "vz": 0.0, "wz": 0.0 },
  "clamped":  { "vx": 1.0, "vy": 0.0, "vz": 0.0, "wz": 0.0 },
  "limit_applied": { "linear_vel_limit": 1.0, "angular_vel_limit": 1.0 },
  "timestamp_ns": 1741564800000000000
}
```

### 5.4 指令转换插件机制

#### 5.4.1 插件接口规范

所有指令转换逻辑（官方预设与用户自定义）均须实现统一的插件基类接口：

**插件基类（C++ 头文件规范）**

```cpp
/* uran_move/plugin_base.hpp */
namespace uran_move {

class MovePluginBase {
public:
  virtual ~MovePluginBase() = default;

  // 插件初始化（在此进行 ROS 节点句柄、SDK 初始化等操作）
  virtual bool init(rclcpp::Node::SharedPtr node) = 0;

  // 执行统一运控指令
  // @param cmd  统一格式的运控指令（已由调用方完成预检限速，单位均为 m/s、rad/s）
  // @param result_json  执行结果（JSON 序列化），写回此参数
  // @return true 表示执行成功，false 表示失败
  // 注意：插件实现须在调用平台 SDK 前对 cmd 中的速度值再次 clamp 到平台物理约束范围，
  //       并在注释中明确标注各字段到平台接口的单位与坐标系映射关系。
  virtual bool execute(const uran_msgs::msg::UnifiedMoveCmd& cmd,
                       std::string& result_json) = 0;

  // 获取当前插件支持的设备类型描述符
  virtual std::string device_type() const = 0;

  // 获取插件版本
  virtual std::string version() const = 0;

  // 查询插件内部当前状态（JSON 字符串）
  // 对于有状态机的平台（如 px4_mavros），返回当前飞行状态；
  // 对于无状态机的平台（如 cyberdog2），返回空对象 "{}"
  virtual std::string internal_state_json() const { return "{}"; }
};

} // namespace uran_move
```

#### 5.4.2 官方预设插件列表

| 插件标识      | 适配设备类型       | 说明                                                         |
| ------------- | ------------------ | ------------------------------------------------------------ |
| `cyberdog2`   | 小米 CyberDog2     | 适配 CyberDog2 ROS 运控接口（servo topic + result service）  |
| `dji_m300`    | 大疆 M300 无人机   | 适配大疆 Onboard SDK（OSDK）                                 |
| `dji_m30`     | 大疆 M30 无人机    | 适配大疆 MSDK/OSDK                                           |
| `px4_mavros`  | PX4 无人机（通用） | 基于 MAVROS topic/service 适配，内置飞行状态机               |

> **暂未适配**：`unitree_go2`/`unitree_b2`（宇树机器狗）、`ros_twist`（通用 ROS 差速底盘）。后续按需补充。

#### 5.4.2.1 各平台适配行为说明

不同平台对 `UnifiedMoveCmd` 各字段的支持程度不同，且各平台底层坐标系惯例不同。**插件负责屏蔽两类差异：字段支持差异与坐标系差异**，上层始终以 REP-103 体坐标系下发。

**字段适配与坐标系转换矩阵**

| 统一指令字段              | `px4_mavros`（无人机）                                                                                | `cyberdog2`（小米机器狗）                                    |
| ------------------------- | ----------------------------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| `linear_vel_x`            | ✅ 插件转换：ENU体系 x→ MAVROS body_vel x（同向）                                                      | ✅ 直接映射到 `vel_des[0]`（SDK 同为体坐标系前向）            |
| `linear_vel_y`            | ✅ 插件转换：ENU体系 y→ MAVROS body_vel y（同向）                                                      | ✅ 直接映射到 `vel_des[1]`（侧步，左正）                      |
| `linear_vel_z`            | ✅ 插件转换：ENU体系 z（上正）→ MAVROS body_vel z（上正，与NED相反，插件内部取反后发给MAVROS NED接口） | ❌ 忽略（机器狗不支持垂直速度控制）                           |
| `angular_vel_z`           | ✅ 插件转换：ENU体系逆时针正→ MAVROS yaw_rate（同向，MAVROS 本地系同为逆时针正）                       | ✅ 直接映射到 `vel_des[2]`（偏航角速度，逆时针正）            |
| `target_roll/pitch`       | ❌ 忽略（由飞控自主姿态环控制）                                                                        | ✅ 映射到 `rpy_des[0/1]`（姿态控制，仅在 WALK_STAND 模式有效）|
| `target_yaw`              | ✅ ENU世界系绝对偏航角，插件转换为 MAVROS 对应坐标系角度                                               | ❌ 忽略（servo 模式无绝对偏航控制，通过 angular_vel_z 转向）  |
| `action="takeoff"`        | ✅ 触发内部飞行状态机                                                                                  | ❌ 不支持（记录警告）                                         |
| `action="land"`           | ✅ 触发降落状态机                                                                                      | ❌ 不支持（记录警告）                                         |
| `action="return_home"`    | ✅ 切换 RTL 模式                                                                                       | ❌ 不支持（记录警告）                                         |
| `action="stop"`           | ✅ 悬停（切换 HOLD 模式）                                                                              | ✅ 发布零速度 servo 指令（保持当前步态）                      |
| `action="emergency_stop"` | ✅ 立即 disarm（危险）                                                                                 | ✅ 调用 `motion_result_cmd` 服务，`motion_id=ESTOP(0)`        |
| `action="stand"`          | ❌ 不支持                                                                                              | ✅ 调用 `motion_result_cmd` 服务，`motion_id=RECOVERYSTAND(111)` |
| `action="sit"`            | ❌ 不支持                                                                                              | ✅ 调用 `motion_result_cmd` 服务，`motion_id=GETDOWN(101)`    |
| `extra_json`              | 可传入飞行模式名等扩展参数                                                                            | 可传入 `motion_id`（覆盖默认步态）、`step_height` 等扩展参数 |

> ❌ 表示该平台忽略该字段，不报错；插件有责任记录 DEBUG 日志说明忽略原因。
>
> **坐标转换责任边界**：云端、手持端、URAN-autotask 只需按 REP-103 体坐标系下发，不承担任何坐标转换责任。所有平台相关的坐标系适配逻辑封装在对应插件的 `execute()` 实现内部，对上层完全透明。

---

**`px4_mavros` 插件——内置飞行状态机与坐标转换**

PX4 通过 MAVROS 控制无人机时，速度指令必须在**解锁（arm）且处于 OFFBOARD 模式**下才会被接受，因此 `px4_mavros` 插件内部维护一套飞行状态机，`execute()` 调用会依据当前状态决定执行路径，而非直接透传速度值。

**坐标系转换**：MAVROS 的 `setpoint_velocity/cmd_vel_unstamped` 接受 ENU 体坐标系速度（与 REP-103 一致），因此 `linear_vel_x/y` 可直接使用；但 `linear_vel_z`（统一协议：上为正）对应 MAVROS body_vel z（ENU 上为正，与 NED 相反），**无需取反，直接赋值**即可。`target_yaw` 为 ENU 世界系角度（逆时针正），MAVROS 同采用 ENU 基准，同样无需转换。插件在调用 MAVROS 接口前须将所有字段确认映射正确，映射关系须在插件实现代码的注释中明确标注。

**飞行状态定义**

```
DISARMED      → 未解锁，所有速度指令被 PX4 拒绝
ARMED         → 已解锁，仍在 MANUAL/POSITION 等模式，速度指令被 PX4 拒绝
OFFBOARD      → 已解锁 + OFFBOARD 模式，可接受速度指令
TAKINGOFF     → 起飞过渡中（高度尚未到达目标）
LANDING       → 降落过渡中
RTL           → 返航模式（Return To Launch）
```

**状态转移与 `action` 映射**

```
action="takeoff"
    DISARMED → 发布 /mavros/cmd/arming(true) → 等待 armed
             → 发布 /mavros/set_mode(OFFBOARD) → 进入 OFFBOARD
             → 持续发布上升速度指令至目标高度 → 进入 OFFBOARD（持续控制）
    已在 OFFBOARD → 直接发布上升速度（幂等）
    其他状态 → 返回错误，上报 E_FLIGHT_STATE_CONFLICT

action="land"
    OFFBOARD / TAKINGOFF → 发布 /mavros/set_mode(AUTO.LAND) → 进入 LANDING
    其他状态 → 返回错误

action="return_home"
    任意已 arm 状态 → 发布 /mavros/set_mode(AUTO.RTL) → 进入 RTL
    DISARMED → 返回错误

action="stop"  （或所有速度字段为 0）
    OFFBOARD → 发布 /mavros/set_mode(AUTO.HOLD) → 悬停
    RTL / LANDING → 忽略（等待飞控自主完成）

action="emergency_stop"
    任意状态 → 发布 /mavros/cmd/arming(false)（立即 disarm，危险操作，仅紧急使用）
             → 进入 DISARMED，上报 E_EMERGENCY_DISARM 事件

速度指令（无 action 或 action=""）
    OFFBOARD → 直接发布 /mavros/setpoint_velocity/cmd_vel_unstamped
    非 OFFBOARD → 拒绝执行，上报 E_NOT_IN_OFFBOARD，建议先发 action="takeoff"
```

**OFFBOARD 模式维持**

PX4 要求 OFFBOARD 模式下必须以 ≥2Hz 的频率持续接收 setpoint，否则自动退出 OFFBOARD（默认超时 0.5s）。`px4_mavros` 插件通过内部定时器以 20Hz 持续发布最后一次有效 setpoint（悬停值），无需上层持续下发指令：

```
内部保活定时器（20Hz）
    → 若当前状态为 OFFBOARD
        → 发布最后一次缓存的 setpoint_velocity（或零速悬停）
    → 若超过可配置超时（默认 3s）未收到新的 UnifiedMoveCmd
        → 自动切换为零速悬停 setpoint，并通过 state/write 上报"控制超时悬停"事件
```

**状态机相关配置项（`config/plugins.yaml` 中 `px4_mavros` 段）**

```yaml
px4_mavros:
  takeoff_height_m: 1.5           # 默认起飞高度（米）
  takeoff_speed_mps: 0.5          # 起飞上升速度（m/s）
  land_speed_mps: 0.3             # 降落下降速度（m/s）
  offboard_keepalive_hz: 20       # 保活发布频率
  cmd_timeout_s: 3.0              # 超时后自动悬停的等待时间
  emergency_disarm_enabled: true  # 是否允许 emergency_stop 触发 disarm
  mavros_ns: "/mavros"            # MAVROS 命名空间
```

---

**`cyberdog2` 插件——双模式运控接口**

CyberDog2 提供两类 ROS 接口，插件根据指令类型自动选择：

- **Servo 模式（连续速度控制）**：向 `motion_servo_cmd` topic 发布 `protocol::msg::MotionServoCmd`，适用于持续行走控制。
- **Result 模式（一次性动作）**：调用 `motion_result_cmd` service（`protocol::srv::MotionResultCmd`），适用于站立、趴下、急停等离散动作，调用阻塞直至动作完成或超时。

**速度控制（Servo 模式）字段映射**

| `MotionServoCmd` 字段 | 来源                                | 说明                                      |
| --------------------- | ----------------------------------- | ----------------------------------------- |
| `motion_id`           | 固定 `WALK_USERTROT = 303`          | 自适应步频行走模式                        |
| `cmd_type`            | 固定 `SERVO_START = 0`              | 每帧均以 SERVO_START 发送（SDK 要求）     |
| `vel_des[0]`          | `linear_vel_x`                      | 前进速度（m/s），前正后负                 |
| `vel_des[1]`          | `linear_vel_y`                      | 侧移速度（m/s），左正右负                 |
| `vel_des[2]`          | `angular_vel_z`                     | 偏航角速度（rad/s），逆时针正             |
| `rpy_des[0]`          | `target_roll`（仅 WALK_STAND 有效） | 横滚角（rad）                             |
| `rpy_des[1]`          | `target_pitch`（仅 WALK_STAND 有效）| 俯仰角（rad）                             |
| `step_height`         | `extra_json.step_height`（可选）    | 步高 [前腿, 后腿]（m），默认 [0.05, 0.05]|

> **坐标系说明**：CyberDog2 SDK `vel_des` 采用体坐标系（前/左/逆时针正），与 REP-103 完全一致，无需坐标转换，直接赋值。

**`action` 映射**

| `action` 值        | 控制方式     | SDK 调用                                                     | 说明                         |
| ------------------ | ------------ | ------------------------------------------------------------ | ---------------------------- |
| `"stop"`           | Servo 模式   | 发布零速度 `vel_des=[0,0,0]`，`motion_id=WALK_USERTROT`      | 原地停止，保持站立步态       |
| `"emergency_stop"` | Result 模式  | `motion_result_cmd`，`motion_id=ESTOP(0)`                    | 急停，进入阻尼模式           |
| `"stand"`          | Result 模式  | `motion_result_cmd`，`motion_id=RECOVERYSTAND(111)`          | 从任意姿态恢复站立           |
| `"sit"`            | Result 模式  | `motion_result_cmd`，`motion_id=GETDOWN(101)`                | 高阻尼趴下                   |
| `extra_json` 中 `motion_id` | Result 模式 | `motion_result_cmd`，使用指定 `motion_id`           | 执行任意预设动作（如翻滚等） |

**`extra_json` 扩展参数**

```json
{
  "motion_id": 124,          // 可选：覆盖默认步态，直接调用 motion_result_cmd（优先级高于 action）
  "step_height": [0.08, 0.08] // 可选：步高（m），仅 servo 模式有效
}
```

**状态反馈**

插件订阅 `motion_status` topic（`protocol::msg::MotionStatus`），将 `switch_status` 映射为插件内部状态并写入状态空间：

| `switch_status` 值 | 含义           | 插件处理                                   |
| ------------------ | -------------- | ------------------------------------------ |
| `NORMAL(0)`        | 正常运行       | 正常接受指令                               |
| `TRANSITIONING(1)` | 动作切换中     | 暂缓下一条 result 指令，等待切换完成       |
| `ESTOP(2)`         | 急停状态       | 拒绝速度指令，上报 `E_ESTOP` 事件          |
| `EDAMP(3)`         | 阻尼模式       | 拒绝速度指令，需先发 `action="stand"` 恢复 |
| `LOW_BAT(7)`       | 低电量         | 写入状态空间 `battery_low=true`            |
| 其他错误状态       | 硬件/过热/过流 | 上报对应错误码，拒绝运控指令               |

**配置项（`config/plugins.yaml` 中 `cyberdog2` 段）**

```yaml
cyberdog2:
  servo_topic: "motion_servo_cmd"       # Servo 控制 topic 名
  result_service: "motion_result_cmd"   # Result 控制 service 名
  status_topic: "motion_status"         # 状态反馈 topic 名
  default_step_height: [0.05, 0.05]     # 默认步高（m）
  result_cmd_timeout_s: 10.0            # motion_result_cmd 服务调用超时时间
  servo_publish_hz: 20                  # servo 指令发布频率（Hz），用于保活
  cmd_timeout_s: 0.5                    # 超过此时间未收到新指令则发布零速度
```

---

#### 5.4.3 插件注册配置

**config/plugins.yaml**

```yaml
uran_move:
  active_plugin: "cyberdog2"    # 当前激活的转换插件
  plugins:
    - id: "cyberdog2"
      lib: "liburan_move_cyberdog2.so"
    - id: "custom_myplugin"
      lib: "/path/to/libcustom_myplugin.so"  # 用户自定义插件路径
```

#### 5.4.4 运行时插件切换 Service

| Service                    | 请求/响应类型                | 说明                   |
| -------------------------- | ---------------------------- | ---------------------- |
| `/uran/move/switch_plugin` | `uran_srvs/SwitchMovePlugin` | 运行时切换指令转换插件 |

**`uran_srvs/SwitchMovePlugin` 定义**

```
# Request
string plugin_id
---
# Response
bool   success
string message
string current_plugin
```

### 5.5 ROS 接口

#### 5.5.1 订阅 Topic

| Topic                          | 消息类型                   | 说明                           |
| ------------------------------ | -------------------------- | ------------------------------ |
| `/uran/core/downlink/move_cmd` | `uran_msgs/UnifiedMoveCmd` | 接收 URAN-core 转发的运控指令  |
| `/uran/core/switch/mode`       | `uran_msgs/ModeSwitchCmd`  | 订阅控制模式切换，进入对应模式 |

#### 5.5.2 发布 Topic

| Topic                    | 消息类型                  | 说明                                    |
| ------------------------ | ------------------------- | --------------------------------------- |
| `/uran/core/uplink/data` | `uran_msgs/UplinkPayload` | 执行结果回报（data_type="move_result"） |
| `/uran/core/state/write` | `uran_msgs/StateField`    | 写入当前控制模式、执行状态等            |

#### 5.5.3 执行结果数据结构（`payload_json` 中）

```json
{
  "cmd_timestamp_ns": 1741564800000000000,
  "success": true,
  "error_code": 0,
  "error_msg": "",
  "current_control_mode": "manual",
  "plugin_id": "cyberdog2",
  "plugin_internal_state": {}
}
```

> 对于 `px4_mavros` 插件，`plugin_internal_state` 包含当前飞行状态机状态，例如：
>
> ```json
> {
>   "flight_state": "OFFBOARD",
>   "armed": true,
>   "mode": "OFFBOARD",
>   "altitude_m": 1.47,
>   "last_setpoint_age_ms": 50
> }
> ```
>
> URAN-core 将此字段写入状态空间（`plugin_state` 字段），供云端展示无人机当前飞行状态。

### 5.6 手动/自动模式行为

| 模式     | URAN-move 行为                                                                         |
| -------- | -------------------------------------------------------------------------------------- |
| `manual` | 接受并执行来自 `controller` 为 `cloud` 或 `field` 的指令；拒绝 `auto` 来源指令         |
| `auto`   | 接受来自 `controller` 为 `auto` 的指令（由 URAN-autotask 发出）；拒绝云端/手持直接运控 |

模式冲突时，URAN-move 拒绝冲突指令并通过 `/uran/core/uplink/data` 上报拒绝事件。

### 5.7 失控保护

#### 5.7.1 触发条件

URAN-move 持续监听 URAN-core 广播的网络状态与心跳状态。当以下条件**同时满足**时，触发失控保护（Failsafe）：

1. URAN-core 通路表中**所有已配置协议均不可用**（`protocol_table` 中无任何 `available=true` 的通路）；
2. 且持续时间超过可配置阈值 `failsafe_timeout_s`（默认 5 秒）。

> **区别于心跳超时**：单次心跳丢包或短暂抖动不触发失控保护；须全部通路同时中断并持续超过阈值才进入保护模式，避免误触。

#### 5.7.2 失控保护动作

进入失控保护后，URAN-move 根据配置的 `failsafe_action` 执行对应动作：

| `failsafe_action` 配置值 | 行为                                      | 适用平台             |
| ------------------------ | ----------------------------------------- | -------------------- |
| `"stop"`                 | 立即停止运动（向平台发送零速度/悬停指令） | 所有平台（默认）     |
| `"return_home"`          | 触发返航（等效于 `action="return_home"`） | 支持返航的无人机平台 |
| `"land"`                 | 触发就地降落（等效于 `action="land"`）    | 无人机平台           |
| `"hold_position"`        | 保持当前位置（向平台发送位置保持指令）    | 支持位置保持的平台   |
| `"custom"`               | 调用插件的 `on_failsafe()` 扩展方法       | 用户自定义           |

失控保护触发后：

- 拒绝一切新到的 `UnifiedMoveCmd`（`urgent` 标志也无效），直至链路恢复；
- 将 `failsafe_active=true` 写入状态空间，并上报 `E_FAILSAFE_TRIGGERED` 事件；
- 链路恢复（任一协议通路重新可达）后，延迟 `failsafe_recover_delay_s`（默认 2 秒）确认稳定，再自动退出保护模式，并上报 `E_FAILSAFE_RECOVERED` 事件，恢复接受指令。

#### 5.7.3 配置项

在 `config/plugins.yaml` 的全局段（各插件均继承）：

```yaml
uran_move:
  failsafe:
    enabled: true                  # 是否启用失控保护，默认 true
    failsafe_timeout_s: 5.0        # 全部链路中断多久后触发，默认 5s
    failsafe_action: "stop"        # 保护动作，见上表
    failsafe_recover_delay_s: 2.0  # 链路恢复后延迟退出保护的确认时间，默认 2s
```

`px4_mavros` 插件的额外默认值建议：`failsafe_action: "return_home"`（无人机优先返航而非原地悬停）。

#### 5.7.4 插件扩展接口

插件基类新增可选方法 `on_failsafe()`，供有特殊失控处置需求的平台覆写：

```cpp
// 失控保护触发时调用（failsafe_action="custom" 时由框架调用，其他动作也会在执行后调用）
// 默认实现：空操作（no-op）
virtual void on_failsafe() {}

// 失控保护解除时调用
// 默认实现：空操作（no-op）
virtual void on_failsafe_recovered() {}
```

#### 5.7.5 ROS 接口

**订阅 Topic（新增）**

| Topic                         | 消息类型                    | 说明                               |
| ----------------------------- | --------------------------- | ---------------------------------- |
| `/uran/core/heartbeat/status` | `uran_msgs/HeartbeatStatus` | 监听心跳发送状态，辅助判断链路中断 |

**发布 Topic（通过现有 uplink 通路，data_type 不同）**

| `data_type`        | 触发时机            | `payload_json` 关键字段                                                             |
| ------------------ | ------------------- | ----------------------------------------------------------------------------------- |
| `"failsafe_event"` | 失控保护触发 / 恢复 | `event`（"triggered"/"recovered"）、`failsafe_action`、`duration_s`、`timestamp_ns` |

**写入状态空间字段（新增）**

| 字段名                     | 类型   | 持久化 | 说明                     |
| -------------------------- | ------ | ------ | ------------------------ |
| `failsafe_active`          | bool   | 否     | 当前是否处于失控保护模式 |
| `failsafe_action_executed` | string | 否     | 最近一次执行的保护动作   |

---

## 六、URAN-media 流媒体包

### 6.1 功能概览

URAN-media 负责：

1. 管理视频/音频流媒体通道的建立、维护与断线重连；
2. 支持 WebRTC 和 RTSP 两种主要流媒体协议；
3. 通过 URAN-core 作为信令转发通道，与云端/手持端完成信令交互；
4. 支持本地录制与任务留痕，在弱网场景下本地缓存，待链路恢复后切片上传。

### 6.2 支持的流媒体协议

| 协议     | 优先级   | 数据格式输入 | 说明                          |
| -------- | -------- | ------------ | ----------------------------- |
| WebRTC   | 优先实现 | RGB / BGR 帧 | 低延迟，NAT 穿透（STUN/TURN） |
| RTSP     | 次选实现 | RGB / BGR 帧 | 标准协议，兼容性强            |
| 扩展预留 | -        | -            | 接口预留，支持二次开发        |

### 6.3 视频数据输入适配

URAN-media 通过订阅 ROS topic 获取摄像头数据，支持多通道并发：

**config/media.yaml**

```yaml
uran_media:
  video_sources:
    - channel_id: "front_cam"
      ros_topic: "/camera/front/image_raw"
      msg_type: "sensor_msgs/Image"     # 或 "sensor_msgs/CompressedImage"
      encoding: "bgr8"                  # 输入编码格式
      fps: 30
    - channel_id: "back_cam"
      ros_topic: "/camera/back/image_raw"
      msg_type: "sensor_msgs/Image"
      encoding: "rgb8"
      fps: 15
  audio_sources:
    - channel_id: "mic"
      ros_topic: "/audio/raw"
      msg_type: "audio_common_msgs/AudioData"
  default_protocol: "webrtc"
  local_record:
    enabled: false
    storage_path: "/tmp/uran_media_record"
    max_size_mb: 2048
```

### 6.4 信令流程

信令通过 URAN-core 转发，不直接建立独立网络连接：

```
云端控制台 → IPON → URAN-core → /uran/core/downlink/media_ctrl → URAN-media
URAN-media → /uran/core/uplink/data（data_type="media_signal"）→ URAN-core → IPON → 云端控制台
```

**WebRTC 信令流程概要**

> **设计决策**：每个 `channel_id` 对应一个独立的 PeerConnection 实例。多路画面（如前置摄像头、后置摄像头）和音频通道分别建立各自的 PeerConnection，互相独立，支持单独启停与控制。

```
1. 云端发送 media_ctrl（action="start", protocol="webrtc", channel_id="<通道ID>"）
2. URAN-media 接收，为该 channel_id 创建独立的 PeerConnection
3. 若 `signal_json` 为空，URAN-media 生成该通道的 SDP Offer，携带 channel_id，经 URAN-core 上行发送至云端
4. 若 `signal_json.type == "offer"`，URAN-media 接收该通道的远端 Offer，生成 SDP Answer，并携带 channel_id，经 URAN-core 上行发送至云端
5. 双端交换该通道的 ICE Candidate（经 URAN-core 中转，始终携带 channel_id 以区分归属）
6. 该通道 PeerConnection 建立，启动对应视频/音频流推送
7. 多路通道可并发建立，各通道独立管理生命周期与重连逻辑
8. 断线时按通道独立自动重连（最多重试 N 次，可配置）
```

**RTSP 信令流程概要**

```
1. 云端发送 media_ctrl（action="start", protocol="rtsp"）
2. URAN-media 启动本地 RTSP Server，监听指定端口
3. URAN-media 通过 URAN-core 上报 RTSP Stream URL
4. 云端/手持端使用该 URL 拉流
```

### 6.5 ROS 接口

#### 6.5.1 订阅 Topic

| Topic                            | 消息类型                      | 说明                         |
| -------------------------------- | ----------------------------- | ---------------------------- |
| `/uran/core/downlink/media_ctrl` | `uran_msgs/MediaCtrlCmd`      | 接收媒体通道控制指令         |
| `/uran/core/switch/media`        | `uran_msgs/MediaSwitchCmd`    | 接收模式切换中的媒体切换通知 |
| `/camera/*/image_raw`            | `sensor_msgs/Image`           | 各摄像头视频帧（可配置多路） |
| `/audio/raw`                     | `audio_common_msgs/AudioData` | 音频数据（可选）             |

#### 6.5.2 发布 Topic

| Topic                    | 消息类型                  | 说明                                                  |
| ------------------------ | ------------------------- | ----------------------------------------------------- |
| `/uran/core/uplink/data` | `uran_msgs/UplinkPayload` | 上报信令、通道状态（data_type="media_signal"）        |
| `/uran/core/state/write` | `uran_msgs/StateField`    | 写入媒体通道状态（active_protocol、channel_count 等） |

#### 6.5.3 `uran_msgs/MediaCtrlCmd` 消息结构

```
string  action            # "start" | "stop" | "switch" | "record_start" | "record_stop"
string  protocol          # "webrtc" | "rtsp"
string  channel_id        # 指定通道，空串表示全部
string  signal_json       # 信令内容（SDP/ICE 等），JSON 序列化
uint64  timestamp_ns
```

### 6.6 本地录制与弱网回传

| 功能         | 说明                                                                                        |
| ------------ | ------------------------------------------------------------------------------------------- |
| 本地录制触发 | 接收 `action="record_start"` 控制指令，或自动化任务启动时自动开启                           |
| 存储格式     | 按通道分别录制，以时间戳分片（可配置分片时长）                                              |
| 回传触发     | 链路恢复后，URAN-core 通知 URAN-media 按云端要求进行切片上传                                |
| 上传接口     | 通过 `/uran/core/uplink/data`（data_type="media_upload"）将文件路径或数据块传递给 URAN-core |

---

## 七、URAN-sensor 传感器包

### 7.1 功能概览

URAN-sensor 面向非媒体类传感数据（雷达、超声波、测距、点云、GPS 等），提供：

1. 订阅设备侧 ROS topic，获取原始传感数据；
2. 通过可插拔的数据转换逻辑，将原始数据规约为云端可解析的标准格式；
3. 将转换结果写入状态空间，或通过指定协议上报至云端/网关；
4. 支持多源传感器并行接入，每类数据可独立指定上行协议偏好。

### 7.2 传感器数据源配置

**config/sensor.yaml**

```yaml
uran_sensor:
  sensors:
    - sensor_id: "gps_main"
      ros_topic: "/fix"
      msg_type: "sensor_msgs/NavSatFix"
      converter_plugin: "navsat_to_position"
      uplink_protocol: "mqtt"            # 偏好上行协议
      update_state_space: true           # 是否写入状态空间
      state_space_field: "position"      # 写入的状态空间字段名
      report_to_cloud: true             # 是否额外上报至云端
      report_interval_ms: 1000

    - sensor_id: "imu_main"
      ros_topic: "/imu/data"
      msg_type: "sensor_msgs/Imu"
      converter_plugin: "imu_to_attitude"
      uplink_protocol: "mqtt"
      update_state_space: true
      state_space_field: "attitude"
      report_to_cloud: false

    - sensor_id: "lidar_front"
      ros_topic: "/lidar/points"
      msg_type: "sensor_msgs/PointCloud2"
      converter_plugin: "pointcloud_summary"
      uplink_protocol: "websocket"
      update_state_space: false
      report_to_cloud: true
      report_interval_ms: 500

    - sensor_id: "custom_sensor_1"
      ros_topic: "/my_sensor/data"
      msg_type: "my_pkg/MySensorMsg"
      converter_plugin: "custom_converter"  # 用户自定义插件
      uplink_protocol: "mqtt"
      update_state_space: false
      report_to_cloud: true
      report_interval_ms: 2000
```

### 7.3 传感器数据转换插件机制

#### 7.3.1 插件接口规范

```cpp
/* uran_sensor/converter_base.hpp */
namespace uran_sensor {

class SensorConverterBase {
public:
  virtual ~SensorConverterBase() = default;

  // 初始化
  virtual bool init(rclcpp::Node::SharedPtr node) = 0;

  // 转换：将原始 ROS 消息转换为 JSON 字符串（云端可解析格式）
  // @param raw_msg_serialized  序列化后的原始 ROS 消息（字节流）
  // @param result_json         转换结果（JSON 字符串），写回此参数
  // @return true 表示转换成功
  virtual bool convert(const std::vector<uint8_t>& raw_msg_serialized,
                       std::string& result_json) = 0;

  // 返回插件 ID
  virtual std::string plugin_id() const = 0;
};

} // namespace uran_sensor
```

#### 7.3.2 官方预设转换插件

| 插件 ID              | 输入消息类型               | 输出字段说明                                                   |
| -------------------- | -------------------------- | -------------------------------------------------------------- |
| `navsat_to_position` | `sensor_msgs/NavSatFix`    | `{ lat, lon, alt, covariance }`                                |
| `imu_to_attitude`    | `sensor_msgs/Imu`          | `{ roll, pitch, yaw, angular_velocity, linear_acceleration }`  |
| `battery_state`      | `sensor_msgs/BatteryState` | `{ percentage, voltage, current, status }`                     |
| `range_sensor`       | `sensor_msgs/Range`        | `{ range_m, min_range, max_range, field_of_view }`             |
| `pointcloud_summary` | `sensor_msgs/PointCloud2`  | `{ point_count, density, bounding_box }`                       |
| `laser_scan`         | `sensor_msgs/LaserScan`    | `{ min_angle, max_angle, ranges_json }`                        |
| `odometry`           | `nav_msgs/Odometry`        | `{ position, orientation, linear_velocity, angular_velocity }` |

### 7.4 ROS 接口

#### 7.4.1 发布 Topic

| Topic                    | 消息类型                  | 说明                                      |
| ------------------------ | ------------------------- | ----------------------------------------- |
| `/uran/core/uplink/data` | `uran_msgs/UplinkPayload` | 传感数据上报（data_type="sensor"）        |
| `/uran/core/state/write` | `uran_msgs/StateField`    | 写入状态空间（如 position / attitude 等） |

#### 7.4.2 Service

| Service                      | 请求/响应类型            | 说明                       |
| ---------------------------- | ------------------------ | -------------------------- |
| `/uran/sensor/reload_config` | `uran_srvs/ReloadConfig` | 运行时重载传感器配置       |
| `/uran/sensor/list`          | `uran_srvs/ListSensors`  | 查询当前已注册的传感器列表 |

### 7.5 上报数据结构（`payload_json` 中）

```json
{
  "sensor_id": "gps_main",
  "sensor_type": "NavSatFix",
  "converter_plugin": "navsat_to_position",
  "timestamp_ns": 1741564800000000000,
  "data": {
    "lat": 39.9042,
    "lon": 116.4074,
    "alt": 50.0,
    "covariance": [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
  }
}
```

---

## 八、URAN-frpcpoint 端口转发服务

### 8.1 功能概览

URAN-frpcpoint 通过反向隧道/端口转发，打通对设备侧服务（如 SSH、Web 调试界面等）的远程访问能力，适用于无人值守场景下的远程运维与故障排查。

其工作原理：设备侧运行 frp 客户端（frpc），云端运行 frp 服务端（frps），通过 URAN-frpcpoint 动态控制映射配置，实现在不暴露设备公网入口的前提下进行安全访问。

### 8.2 工作流程

```
1. 云端控制台为设备分配可用的服务端端口（Port Pool 管理）
2. 云端通过 IPON 下发 frpc_ctrl 指令，携带服务端地址与分配端口
3. URAN-frpcpoint 接收指令，动态生成 frpc 配置文件
4. URAN-frpcpoint 启动或更新 frpc 进程，建立反向隧道
5. URAN-frpcpoint 上报当前端口映射状态至 URAN-core（写入状态空间）
6. 运维人员通过云端控制台获取 SSH 连接信息（frps地址:分配端口）进行远程访问
```

### 8.3 ROS 接口

#### 8.3.1 订阅 Topic

| Topic                           | 消息类型                | 说明                 |
| ------------------------------- | ----------------------- | -------------------- |
| `/uran/core/downlink/frpc_ctrl` | `uran_msgs/FrpcCtrlCmd` | 接收端口转发控制指令 |

#### 8.3.2 发布 Topic

| Topic                    | 消息类型                  | 说明                                        |
| ------------------------ | ------------------------- | ------------------------------------------- |
| `/uran/core/state/write` | `uran_msgs/StateField`    | 写入当前端口映射状态                        |
| `/uran/core/uplink/data` | `uran_msgs/UplinkPayload` | 上报端口映射结果（data_type="frpc_status"） |

#### 8.3.3 `uran_msgs/FrpcCtrlCmd` 消息结构

```
string  action              # "start" | "stop" | "update"
string  frps_host           # frp 服务端地址
uint32  frps_port           # frp 服务端端口
string  service_name        # 服务标识（如 "ssh"）
uint32  local_port          # 本机服务端口（如 SSH 的 22）
uint32  remote_port         # 服务端分配的远程端口
string  auth_token          # frp 鉴权 token
uint64  timestamp_ns
```

### 8.4 配置与安全

- frpc 配置由 URAN-frpcpoint 动态生成，不直接暴露于外部；
- 鉴权 Token 由云端控制台统一管理，通过 IPON 加密信道下发；
- 端口映射仅在云端控制台主动分配并下发指令后才建立，设备侧不主动发起；

---

## 九、URAN-autotask 自动化巡检服务

### 9.1 功能概览

URAN-autotask 承接云端任务引擎下发的巡检任务，负责：

1. **任务生命周期管理**：开始、暂停、继续、终止、进度推进；
2. **协同调度**：通过标准接口驱动 URAN-move（路径/动作执行）、URAN-sensor（数据采集）、URAN-media（证据录制）；
3. **progress 上报**：持续上报任务阶段、关键事件、异常情况至 URAN-core，供云端实时展示；
4. **异常处置**：检测任务异常并归一化语义，支持暂停/绕行/返航等处置动作。

### 9.2 任务数据结构

#### 9.2.1 任务定义（下行）

**`uran_msgs/TaskCtrlCmd` 消息结构**

```
string  msg_version
string  task_id              # 任务唯一 ID
string  action               # "start" | "pause" | "resume" | "stop" | "update_params"
string  task_type            # "waypoint" | "patrol" | "custom"
string  task_params_json     # 任务参数（JSON 序列化，见下）
uint64  timestamp_ns
```

**任务参数 JSON（`task_type="waypoint"` 示例）**

```json
{
  "task_id": "task_20260310_001",
  "task_type": "waypoint",
  "waypoints": [
    {
      "seq": 0,
      "lat": 39.9042,
      "lon": 116.4074,
      "alt": 10.0,
      "heading_deg": 90.0,
      "speed_mps": 1.5,
      "hover_time_s": 5,
      "actions": ["capture_image", "sensor_report"]
    },
    {
      "seq": 1,
      "lat": 39.9050,
      "lon": 116.4080,
      "alt": 10.0,
      "heading_deg": 180.0,
      "speed_mps": 1.5,
      "hover_time_s": 0,
      "actions": []
    }
  ],
  "abort_on_low_battery": true,
  "low_battery_threshold": 20,
  "abort_action": "return_home",
  "media_record": true,
  "sensor_report_interval_ms": 2000
}
```

#### 9.2.2 任务进度上报（上行）

上报通过 `/uran/core/uplink/data`（data_type="task_progress"）发送，`payload_json` 结构：

```json
{
  "task_id": "task_20260310_001",
  "timestamp_ns": 1741564800000000000,
  "stage": "executing",
  "current_waypoint_seq": 1,
  "total_waypoints": 5,
  "progress_percent": 20.0,
  "status": "normal",
  "event": null,
  "error": null,
  "position": { "lat": 39.9050, "lon": 116.4080, "alt": 10.0 },
  "battery_level": 75.0
}
```

#### 9.2.3 异常事件上报

```json
{
  "task_id": "task_20260310_001",
  "timestamp_ns": 1741564800000000000,
  "stage": "paused",
  "status": "exception",
  "event": "obstacle_detected",
  "error": {
    "code": "E_OBSTACLE",
    "severity": "warning",
    "description": "前方检测到障碍物，任务暂停",
    "suggested_action": "reroute"
  }
}
```

**标准异常分类表**

| 异常码                    | 语义                                     | 建议处置                     |
| ------------------------- | ---------------------------------------- | ---------------------------- |
| `E_LOW_BATTERY`           | 电量不足                                 | 返航 / 暂停                  |
| `E_GPS_LOSS`              | GPS 信号丢失                             | 悬停 / 返航                  |
| `E_LINK_TIMEOUT`          | 通信链路超时                             | 本地继续 / 返航              |
| `E_OBSTACLE`              | 检测到障碍物                             | 绕行 / 暂停                  |
| `E_EXEC_TIMEOUT`          | 执行超时                                 | 跳过当前航点 / 终止          |
| `E_MOVE_FAIL`             | 运控执行失败                             | 重试 / 终止                  |
| `E_SENSOR_FAIL`           | 传感器故障                               | 记录并继续 / 终止            |
| `E_TASK_CONFLICT`         | 控制冲突                                 | 任务暂停，等待控制切换       |
| `E_NOT_IN_OFFBOARD`       | PX4 未进入 OFFBOARD 模式，速度指令被拒绝 | 重新执行 takeoff 流程 / 终止 |
| `E_FLIGHT_STATE_CONFLICT` | 当前飞行状态不允许执行该 action          | 等待状态就绪后重试 / 终止    |
| `E_EMERGENCY_DISARM`      | 触发了紧急 disarm（飞机已断电降落）      | 任务终止，人工介入           |
| `E_FAILSAFE_TRIGGERED`    | URAN-move 失控保护已触发（全部链路中断） | 任务暂停，等待链路恢复后继续 |

### 9.3 ROS 接口

#### 9.3.1 订阅 Topic

| Topic                           | 消息类型                  | 说明             |
| ------------------------------- | ------------------------- | ---------------- |
| `/uran/core/downlink/task_ctrl` | `uran_msgs/TaskCtrlCmd`   | 接收任务控制指令 |
| `/uran/core/switch/mode`        | `uran_msgs/ModeSwitchCmd` | 订阅控制模式切换 |

#### 9.3.2 发布 Topic

| Topic                          | 消息类型                   | 说明                                  |
| ------------------------------ | -------------------------- | ------------------------------------- |
| `/uran/core/downlink/move_cmd` | `uran_msgs/UnifiedMoveCmd` | 向 URAN-move 发送运控指令（自动模式） |
| `/uran/core/uplink/data`       | `uran_msgs/UplinkPayload`  | 上报任务进度/异常                     |
| `/uran/core/state/write`       | `uran_msgs/StateField`     | 写入当前任务状态至状态空间            |

#### 9.3.3 Service

| Service                 | 请求/响应类型             | 说明             |
| ----------------------- | ------------------------- | ---------------- |
| `/uran/autotask/status` | `uran_srvs/GetTaskStatus` | 查询当前任务状态 |

> **注意**：URAN-autotask 与 URAN-move 的交互通过公共 Topic `/uran/core/downlink/move_cmd` 进行，URAN-autotask 在 `controller` 字段中填写 `"auto"`，URAN-move 在自动模式下只接受来源为 `"auto"` 的指令。

---

## 十、跨组接口约定

### 10.1 与【软件组】（云端）接口约定

#### 10.1.1 云端 → 设备（下行信令）

云端通过 MQTT（主要）或 WebSocket/TCP 向设备下发，所有下行消息均包含顶层字段 `msg_type` 进行路由：

| `msg_type` 值    | 含义                   | URAN 侧处理            |
| ---------------- | ---------------------- | ---------------------- |
| `control_switch` | 控制切换信令           | URAN-core 内部处理     |
| `move_cmd`       | 运控指令               | 路由至 URAN-move       |
| `task_ctrl`      | 任务控制               | 路由至 URAN-autotask   |
| `media_ctrl`     | 媒体通道控制           | 路由至 URAN-media      |
| `frpc_ctrl`      | 端口转发控制           | 路由至 URAN-frpcpoint  |
| `state_query`    | 状态查询               | URAN-core 直接响应     |
| `param_update`   | 参数下发（速度限制等） | URAN-core 更新并广播   |
| `heartbeat_ack`  | 心跳确认               | URAN-core 记录通路可达 |

**MQTT Topic 规范（下行）**

```
/oivs/{tenant_id}/{device_id}/down
```

**MQTT Topic 规范（上行）**

```
/oivs/{tenant_id}/{device_id}/up
/oivs/{tenant_id}/{device_id}/heartbeat
/oivs/{tenant_id}/{device_id}/state
```

#### 10.1.2 设备 → 云端（上行数据）

| 数据类型 (`data_type`) | 上行频率         | 首选协议         | 说明                 |
| ---------------------- | ---------------- | ---------------- | -------------------- |
| `heartbeat`            | 5s/次（可配置）  | MQTT（主）       | 统一心跳包           |
| `state_snapshot`       | 10s/次（可配置） | MQTT             | 完整状态空间快照     |
| `sensor`               | 按传感器配置     | MQTT / WebSocket | 传感数据上报         |
| `move_result`          | 每次指令执行后   | MQTT             | 运控执行结果         |
| `task_progress`        | 任务执行中实时   | MQTT             | 任务进度与事件       |
| `media_signal`         | 信令交互时       | WebSocket / MQTT | 流媒体信令           |
| `media_upload`         | 弱网恢复后       | HTTP / WebSocket | 本地录制文件切片上传 |
| `frpc_status`          | 端口映射变更时   | MQTT             | 端口映射状态         |

#### 10.1.3 软件组需提供给 ROS 组的服务

1. **状态空间元数据接口**：提供装备模板与实例的字段元数据拉取 REST API，支持覆写/追加两种合并模式；
2. **统一鉴权接入点**：提供 MQTT/TCP/WebSocket Broker 地址、端口及 Token/证书鉴权机制；
3. **端口分配接口**：为 URAN-frpcpoint 提供 frps 地址与可用端口分配接口；
4. **信令与字段命名对齐**：在联调前确认心跳包、控制切换信令、运控指令等字段命名与版本。

### 10.2 与【设备组】（网关/手持终端）接口约定

#### 10.2.1 网关侧协议约定

- 网关上行封包格式：将 LoRA/Zigbee 等私有协议报文封装为 MQTT/TCP，顶层字段格式与云端下行格式保持一致；
- 手持端接管信令：手持端接管时，必须先发出 `control_switch` 信令（`controller="field"`），URAN-core 收到后切换为手动模式，屏蔽云端直接运控。

#### 10.2.2 设备组需提供给 ROS 组的服务

1. **私有协议封包规则对齐**：明确 LoRA/Zigbee 在网关侧封装/解封为标准协议的字段映射规则；
2. **手持接管信令规范**：约定手持终端接管/释放控制权时的信令格式；
3. **网关设备 MAC/ID 上报**：网关上线时上报自身 ID，URAN-core 在通路表中记录网关路径。

### 10.3 与【算法组】接口约定

#### 10.3.1 算法组需提供给 ROS 组的服务

1. **航点数据结构规范**：明确 waypoint 中 GPS/高度/朝向/速度约束等字段名与单位；
2. **异常分类语义对齐**：对齐 `E_OBSTACLE` 等异常码的语义与建议处置逻辑；
3. **再规划指令格式**：明确算法组在线再规划时下发新航点序列的格式（复用 `task_ctrl` 中 `update_params` 动作）。

#### 10.3.2 ROS 组为算法组提供的服务

1. **任务闭环执行**：URAN-autotask + URAN-move 负责落地执行算法输出的航点序列；
2. **实时状态反馈**：通过状态空间持续上报位置、姿态、电量、任务阶段等，供算法组在线评估；
3. **多源传感器数据**：URAN-sensor 按约定频率上报 GPS/IMU/激光雷达等观测数据；
4. **任务证据数据**：URAN-media 录制任务视频，URAN-sensor 采集关键传感快照，形成可追溯证据链；
5. **异常语义上报**：设备侧异常统一归类为标准异常码后上报，算法组以统一方式处理不同设备异常。

---

## 十一、二次开发扩展规范

### 11.1 包开发规范

用户自定义扩展包须满足以下要求：

- 在 `package.xml` 中声明 `uran_core` 为依赖；
- 通过 `uran_msgs` 和 `uran_srvs` 包中的标准消息与 URAN-core 交互；
- 不得直接调用其他 URAN-* 功能包的内部接口，只能通过标准 Topic/Service；
- 提供 `config/<your_pkg>.yaml` 进行参数配置。

### 11.2 自定义运控转换插件（URAN-move 扩展）

步骤：

1. 新建 ROS 2 包，依赖 `uran_move`；
2. 继承 `uran_move::MovePluginBase`，实现 `init()`、`execute()`、`device_type()`、`version()`；
3. 注册插件（使用 `pluginlib`）：
   ```xml
   <!-- plugin_description.xml -->
   <library path="libmy_move_plugin">
     <class type="my_pkg::MyMovePlugin"
            base_class_type="uran_move::MovePluginBase">
       <description>My custom device move plugin</description>
     </class>
   </library>
   ```
4. 在 `config/plugins.yaml` 中注册插件 ID 并设为激活。

### 11.3 自定义传感器转换插件（URAN-sensor 扩展）

步骤：

1. 新建 ROS 2 包，依赖 `uran_sensor`；
2. 继承 `uran_sensor::SensorConverterBase`，实现 `init()`、`convert()`、`plugin_id()`；
3. 注册在 `plugin_description.xml` 中；
4. 在 `config/sensor.yaml` 中配置对应 `converter_plugin` 字段，指向自定义插件 ID。

### 11.4 自定义数据上报包

用户可开发自定义 ROS 包，通过以下接口直接接入 URAN 数据上下行通路：

- 写入状态空间：发布 `uran_msgs/StateField` 到 `/uran/core/state/write`；
- 上报数据到云端：发布 `uran_msgs/UplinkPayload` 到 `/uran/core/uplink/data`，并指定 `preferred_protocol`；
- 接收控制指令：订阅 URAN-core 对应的下行 Topic（需在 `param_update` 中注册自定义消息路由）。

---

## 十二、消息与服务定义汇总

### 12.1 自定义消息（`uran_msgs` 包）

| 消息名              | 主要字段                                                                                                                                   | 用途             |
| ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ | ---------------- |
| `StateField`        | `field_name`, `value_json`, `persistent`, `source_pkg`, `timestamp_ns`                                                                     | 状态空间字段写入 |
| `StateSnapshot`     | `msg_version`, `timestamp_ns`, `device_id`, `fields_json`                                                                                  | 状态空间全量广播 |
| `HeartbeatStatus`   | `timestamp_ns`, `protocol`, `last_sent_ts`, `success`                                                                                      | 心跳发送状态     |
| `UnifiedMoveCmd`    | `msg_version`, `device_id`, `timestamp_ns`, `controller`, `linear_vel_*`, `angular_vel_z`, `target_roll/pitch/yaw`, `action`, `extra_json` | 统一运控指令     |
| `ModeSwitchCmd`     | `control_mode`, `controller`, `timestamp_ns`                                                                                               | 模式/控制者切换  |
| `MediaSwitchCmd`    | `action`, `protocol`, `timestamp_ns`                                                                                                       | 媒体通道切换     |
| `UplinkProtocolCmd` | `protocol`, `timestamp_ns`                                                                                                                 | 上行协议切换     |
| `UplinkPayload`     | `source_pkg`, `data_type`, `preferred_protocol`, `payload_json`, `urgent`, `timestamp_ns`                                                  | 统一上行数据载体 |
| `MediaCtrlCmd`      | `action`, `protocol`, `channel_id`, `signal_json`, `timestamp_ns`                                                                          | 媒体通道控制     |
| `FrpcCtrlCmd`       | `action`, `frps_host`, `frps_port`, `service_name`, `local_port`, `remote_port`, `auth_token`, `timestamp_ns`                              | 端口转发控制     |
| `TaskCtrlCmd`       | `msg_version`, `task_id`, `action`, `task_type`, `task_params_json`, `timestamp_ns`                                                        | 任务控制         |
| `ParamUpdateCmd`    | `params_json`, `timestamp_ns`                                                                                                              | 参数更新广播     |

### 12.2 自定义服务（`uran_srvs` 包）

| 服务名                 | 请求字段                                      | 响应字段                                             | 用途                     |
| ---------------------- | --------------------------------------------- | ---------------------------------------------------- | ------------------------ |
| `GetStateField`        | `field_names[]`                               | `fields_json`, `success`                             | 查询状态空间字段         |
| `SetStateField`        | `field_name`, `value_json`, `persistent`      | `success`, `message`                                 | 同步写入状态空间         |
| `ConnectProtocol`      | `protocol`, `action` ("connect"/"disconnect") | `success`, `message`                                 | 控制协议连接             |
| `GetNetworkStatus`     | （无）                                        | `protocol_table_json`                                | 查询网络连接状态         |
| `SwitchMovePlugin`     | `plugin_id`                                   | `success`, `message`, `current_plugin`               | 切换运控插件             |
| `ReloadConfig`         | `config_path`                                 | `success`, `message`                                 | 重载传感器/媒体配置      |
| `ListSensors`          | （无）                                        | `sensors_json`                                       | 列出已注册传感器         |
| `GetTaskStatus`        | `task_id`                                     | `status_json`                                        | 查询任务状态             |
| `TriggerStateReport`   | `reason`                                      | `success`, `message`                                 | 手动触发即时状态快照上报 |
| `ConfigureStateReport` | `interval_ms`, `protocol`, `report_on_change` | `success`, `current_interval_ms`, `current_protocol` | 运行时修改上报周期与协议 |

### 12.3 Topic 汇总

| Topic 路径                          | 消息类型            | 发布者                   | 订阅者                   |
| ----------------------------------- | ------------------- | ------------------------ | ------------------------ |
| `/uran/core/state/write`            | `StateField`        | 所有功能包               | URAN-core                |
| `/uran/core/state/broadcast`        | `StateSnapshot`     | URAN-core                | 所有功能包（可选）       |
| `/uran/core/uplink/data`            | `UplinkPayload`     | 所有功能包               | URAN-core                |
| `/uran/core/heartbeat/status`       | `HeartbeatStatus`   | URAN-core                | 所有功能包（可选）       |
| `/uran/core/switch/mode`            | `ModeSwitchCmd`     | URAN-core                | URAN-move, URAN-autotask |
| `/uran/core/switch/media`           | `MediaSwitchCmd`    | URAN-core                | URAN-media               |
| `/uran/core/switch/uplink_protocol` | `UplinkProtocolCmd` | URAN-core                | URAN-sensor, 扩展包      |
| `/uran/core/downlink/move_cmd`      | `UnifiedMoveCmd`    | URAN-core, URAN-autotask | URAN-move                |
| `/uran/core/downlink/task_ctrl`     | `TaskCtrlCmd`       | URAN-core                | URAN-autotask            |
| `/uran/core/downlink/media_ctrl`    | `MediaCtrlCmd`      | URAN-core                | URAN-media               |
| `/uran/core/downlink/frpc_ctrl`     | `FrpcCtrlCmd`       | URAN-core                | URAN-frpcpoint           |
| `/uran/core/downlink/param_update`  | `ParamUpdateCmd`    | URAN-core                | 所有功能包               |

---

*文档结束*

*本文档依据功能分析文档（v1.0，2026年3月1日）与芯烛天巡-OIVS系统技术实现要点编写，各接口字段定义以本文档为准，与云端/手持终端/设备组的联调前须进行接口对齐确认。*
