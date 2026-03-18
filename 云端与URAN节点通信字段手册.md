# URAN 节点通信字段


---

## 一、通信总览

### 1.1 通信协议

| 协议 | 类别 | 优先级 | 用途 |
|------|------|--------|------|
| MQTT | 互联网 | 主通路（默认） | 心跳、信令、数据上报 |
| WebSocket | 互联网 | 次通路 | 双向低延迟信令 |
| TCP | 互联网 | 备用通路 | 稳定传输 |
| UDP | 互联网 | 备用通路 | 低延迟实时控制 |
| LoRA | 私有网络 | 网关中转 | 低带宽远距离 |
| Zigbee | 私有网络 | 网关中转 | 低功耗短距离 |
| Bluetooth | 私有网络 | 网关中转/手持直连 | 近场直连 |
| WebRTC | 互联网 | 仅流媒体 | 视频/音频流（不参与心跳与控制信令） |

### 1.2 MQTT Topic 规范

| 方向 | Topic |
|------|-------|
| 下行（云端→设备） | `/oivs/{tenant_id}/{device_id}/down` |
| 上行（通用数据） | `/oivs/{tenant_id}/{device_id}/up` |
| 上行（心跳） | `/oivs/{tenant_id}/{device_id}/heartbeat` |
| 上行（状态） | `/oivs/{tenant_id}/{device_id}/state` |

### 1.3 通路降级策略

```
优先使用 preferred_protocol
  → 若不可用，依次尝试 mqtt → websocket → tcp → udp → lora
  → 若全部不可用，缓存数据并等待通路恢复（urgent 数据持续重试）
```

---

## 二、下行信令（云端 → URAN 节点）

所有下行消息均包含顶层字段 `msg_type` 用于路由分发。

### 2.1 msg_type 路由表

| `msg_type` | 含义 | URAN 侧处理目标 |
|------------|------|-----------------|
| `control_switch` | 控制切换信令 | URAN-core 内部处理 |
| `move_cmd` | 运控指令 | 路由至 URAN-move |
| `task_ctrl` | 任务控制 | 路由至 URAN-autotask |
| `media_ctrl` | 媒体通道控制 | 路由至 URAN-media |
| `frpc_ctrl` | 端口转发控制 | 路由至 URAN-frpcpoint |
| `state_query` | 状态查询 | URAN-core 直接响应 |
| `param_update` | 参数下发（速度限制等） | URAN-core 更新并广播 |
| `heartbeat_ack` | 心跳确认 | URAN-core 记录通路可达 |

---

### 2.2 注册请求与响应

**请求方向**：URAN → 云端（启动时发起）

**请求携带字段**：`device_id` + `template_id` + `token`

**云端注册响应结构**：

```json
{
  "result": "registered | auto_registered | rejected",
  "reason": "",
  "device_id": "...",
  "template_id": "..."
}
```

| `result` 值 | 含义 |
|-------------|------|
| `registered` | device_id 已存在且无活跃连接，标记在线 |
| `auto_registered` | device_id 不存在，云端自动创建实例并绑定 template_id |
| `rejected` | device_id 已被另一连接占用，拒绝（reason 填写如 `device_id_conflict`） |

---

### 2.3 控制切换信令（control_switch）

```json
{
  "msg_type": "control_switch",
  "msg_version": "1.0",
  "device_id": "device_001",
  "timestamp_ms": 1741564800000,
  "switch": {
    "control_mode": "auto",
    "controller": "cloud",
    "primary_uplink_protocol": "mqtt",
    "media": {
      "action": "start",
      "protocol": "webrtc"
    }
  }
}
```

| 字段 | 类型 | 可选值 | 说明 |
|------|------|--------|------|
| `msg_type` | string | `"control_switch"` | 固定值 |
| `msg_version` | string | `"1.0"` | 消息版本 |
| `device_id` | string | - | 目标设备 ID |
| `timestamp_ms` | int | - | 毫秒时间戳 |
| `switch.control_mode` | string/null | `"manual"` / `"auto"` / null | 巡检模式，null 表示不变 |
| `switch.controller` | string/null | `"cloud"` / `"field"` / `"auto"` / null | 控制者，null 表示不变 |
| `switch.primary_uplink_protocol` | string/null | `"mqtt"` / `"websocket"` / `"tcp"` / null | 主上行协议，null 表示不变 |
| `switch.media.action` | string | `"start"` / `"stop"` / `"switch"` | 媒体通道动作 |
| `switch.media.protocol` | string/null | `"webrtc"` / `"rtsp"` / null | 媒体协议 |

**切换维度影响范围**：

| 维度 | 影响包 |
|------|--------|
| 巡检模式切换（auto ↔ manual） | URAN-move、URAN-autotask |
| 控制者切换 | URAN-move、URAN-autotask |
| 上行协议切换 | URAN-sensor、URAN-core 上行路由 |
| 流媒体通道切换 | URAN-media |

---

### 2.4 运控指令（move_cmd）

```json
{
  "msg_type": "move_cmd",
  "msg_version": "1.0",
  "device_id": "device_001",
  "timestamp_ns": 1741564800000000000,
  "controller": "cloud",
  "linear_vel_x": 0.5,
  "linear_vel_y": 0.0,
  "linear_vel_z": 0.0,
  "angular_vel_z": 0.0,
  "target_roll": 0.0,
  "target_pitch": 0.0,
  "target_yaw": "NaN",
  "action": "",
  "extra_json": "{}"
}
```

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `msg_version` | string | - | 消息版本 |
| `device_id` | string | - | 设备 ID |
| `timestamp_ns` | uint64 | 纳秒 | 时间戳 |
| `controller` | string | - | 指令来源：`"cloud"` / `"field"` / `"auto"` |
| `linear_vel_x` | float64 | m/s | 前进速度（正=前，负=后） |
| `linear_vel_y` | float64 | m/s | 横向速度（正=左，负=右） |
| `linear_vel_z` | float64 | m/s | 垂直速度（正=上，负=下） |
| `angular_vel_z` | float64 | rad/s | 偏航角速度（正=左转/逆时针） |
| `target_roll` | float64 | rad | 目标横滚角（可选） |
| `target_pitch` | float64 | rad | 目标俯仰角（可选） |
| `target_yaw` | float64 | rad | 目标偏航角（NaN=不指定，ENU 世界系） |
| `action` | string | - | 特殊动作（见下表），空串=无 |
| `extra_json` | string | - | JSON 扩展参数（步态等） |

**action 可选值**：

| action | 适用平台 | 说明 |
|--------|---------|------|
| `""` | 全部 | 无特殊动作，执行速度控制（由 linear_vel_* / angular_vel_z 驱动） |
| `"stop"` | 全部 | 停止运动（机器狗：发零速 servo；无人机：切换 HOLD 悬停） |
| `"emergency_stop"` | 全部 | 紧急停止（机器狗：ESTOP 锁关节；无人机：立即 disarm） |
| `"stand"` | cyberdog2 | 从任意姿态恢复站立（RECOVERYSTAND，motion_id=111） |
| `"sit"` | cyberdog2 | 高阻尼趴下（GETDOWN，motion_id=101） |
| `"takeoff"` | px4_mavros | 解锁 + 切 OFFBOARD + 上升至目标高度 |
| `"land"` | px4_mavros | 切 AUTO.LAND 降落 |
| `"return_home"` | px4_mavros | 切 AUTO.RTL 返航 |

> `action` 与速度字段互斥：填写非空 `action` 时，速度字段被忽略（`"stop"` 除外，其本质是零速）。

**extra_json 扩展参数（cyberdog2）**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `motion_id` | int | 直接调用指定 motion_id（优先级高于 action，走 Result 模式） |
| `step_height` | float[2] | 步高 [前腿, 后腿]（m），默认 [0.05, 0.05]，仅 servo 模式有效 |

```json
// 示例：直接调用 motion_id=303（WALK_USERTROT）
{ "extra_json": "{\"motion_id\": 303}" }

// 示例：调高步高行走
{ "linear_vel_x": 0.3, "action": "", "extra_json": "{\"step_height\": [0.08, 0.08]}" }
```

**坐标系**：ROS REP-103 右手体坐标系（x前/y左/z上，yaw 逆时针为正）

**限速约束**：
- 线速度合速度 `√(vx²+vy²+vz²)` 不超过 `linear_vel_limit`，超限等比缩放
- 角速度 `|angular_vel_z|` 不超过 `angular_vel_limit`，超限 clamp

---

### 2.5 任务控制指令（task_ctrl）

```json
{
  "msg_type": "task_ctrl",
  "msg_version": "1.0",
  "task_id": "task_20260310_001",
  "action": "start",
  "task_type": "waypoint",
  "task_params_json": "...",
  "timestamp_ns": 1741564800000000000
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `msg_version` | string | 消息版本 |
| `task_id` | string | 任务唯一 ID |
| `action` | string | `"start"` / `"pause"` / `"resume"` / `"stop"` / `"update_params"` |
| `task_type` | string | `"waypoint"` / `"patrol"` / `"custom"` |
| `task_params_json` | string | 任务参数 JSON |
| `timestamp_ns` | uint64 | 纳秒时间戳 |

**航点任务参数示例（task_type="waypoint"）**：

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
    }
  ],
  "abort_on_low_battery": true,
  "low_battery_threshold": 20,
  "abort_action": "return_home",
  "media_record": true,
  "sensor_report_interval_ms": 2000
}
```

| 航点字段 | 类型 | 说明 |
|----------|------|------|
| `seq` | int | 航点序号 |
| `lat` | float | 纬度 |
| `lon` | float | 经度 |
| `alt` | float | 高度（米） |
| `heading_deg` | float | 朝向（度） |
| `speed_mps` | float | 速度（m/s） |
| `hover_time_s` | int | 悬停时间（秒） |
| `actions` | string[] | 到达后执行的动作列表 |

---

### 2.6 媒体通道控制指令（media_ctrl）

```json
{
  "msg_type": "media_ctrl",
  "action": "start",
  "protocol": "webrtc",
  "channel_id": "front_cam",
  "signal_json": "...",
  "timestamp_ns": 1741564800000000000
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `action` | string | `"start"` / `"stop"` / `"switch"` / `"record_start"` / `"record_stop"` |
| `protocol` | string | `"webrtc"` / `"rtsp"` |
| `channel_id` | string | 指定通道 ID，空串=全部 |
| `signal_json` | string | 信令内容（SDP/ICE 等 JSON） |
| `timestamp_ns` | uint64 | 纳秒时间戳 |

**WebRTC 信令流程**：
1. 云端发送 `action="start", protocol="webrtc", channel_id="<通道ID>"`
2. URAN-media 为该 channel_id 创建独立 PeerConnection
3. URAN-media 生成 SDP Offer，携带 channel_id 经 URAN-core 上行发送
4. 云端返回 SDP Answer，经 URAN-core 下行发送
5. 双端交换 ICE Candidate（始终携带 channel_id）
6. PeerConnection 建立，启动流推送

---

### 2.7 端口转发控制指令（frpc_ctrl）

```json
{
  "msg_type": "frpc_ctrl",
  "action": "start",
  "frps_host": "frps.example.com",
  "frps_port": 7000,
  "service_name": "ssh",
  "local_port": 22,
  "remote_port": 50001,
  "auth_token": "xxxxxxxx",
  "timestamp_ns": 1741564800000000000
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `action` | string | `"start"` / `"stop"` / `"update"` |
| `frps_host` | string | frp 服务端地址 |
| `frps_port` | uint32 | frp 服务端端口 |
| `service_name` | string | 服务标识（如 `"ssh"`） |
| `local_port` | uint32 | 本机服务端口（如 22） |
| `remote_port` | uint32 | 服务端分配的远程端口 |
| `auth_token` | string | frp 鉴权 token |
| `timestamp_ns` | uint64 | 纳秒时间戳 |

---

### 2.8 参数更新（param_update）

云端下发参数更新（如速度限制等），URAN-core 接收后更新本地配置并广播至各功能包。

| 字段 | 类型 | 说明 |
|------|------|------|
| `msg_type` | string | `"param_update"` |
| `params_json` | string | JSON 序列化的参数键值对 |
| `timestamp_ns` | uint64 | 纳秒时间戳 |

---

## 三、上行数据（URAN 节点 → 云端）

所有上行数据通过 `UplinkPayload` 统一封装，由 `data_type` 区分类型。

### 3.1 上行数据类型总览

| `data_type` | 上行频率 | 首选协议 | 说明 |
|-------------|----------|----------|------|
| `heartbeat` | 5s/次（可配置） | MQTT | 统一心跳包 |
| `state_snapshot` | 10s/次（可配置） | MQTT | 完整状态空间快照 |
| `sensor` | 按传感器配置 | MQTT / WebSocket | 传感数据上报 |
| `move_result` | 每次指令执行后 | MQTT | 运控执行结果 |
| `move_clamp_event` | 限速截断时 | MQTT | 限速截断事件 |
| `move_reject_event` | 指令被模式过滤或失控保护拒绝时 | MQTT | 指令拒绝事件 |
| `cyberdog2_motion_abnormal` | motion_status 异常时 | MQTT | CyberDog2 运动状态异常 |
| `plugin_switch_event` | 运行时切换插件后 | MQTT | 插件切换事件 |
| `failsafe_event` | 失控保护触发/恢复 | MQTT | 失控保护事件 |
| `task_progress` | 任务执行中实时 | MQTT | 任务进度与事件 |
| `media_signal` | 信令交互时 | WebSocket / MQTT | 流媒体信令 |
| `media_upload` | 弱网恢复后 | HTTP / WebSocket | 本地录制文件切片上传 |
| `frpc_status` | 端口映射变更时 | MQTT | 端口映射状态 |

### 3.2 UplinkPayload 统一封装结构

```
source_pkg          # 来源软件包（string）
data_type           # 数据类型标识（string）
preferred_protocol  # 偏好上行协议，空串=跟随主协议（string）
payload_json        # JSON 序列化的数据体（string）
urgent              # 是否紧急（bool）
timestamp_ns        # 纳秒时间戳（uint64）
```

---

### 3.3 心跳包（heartbeat）

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

| 字段 | 类型 | 说明 |
|------|------|------|
| `msg_type` | string | 固定 `"heartbeat"` |
| `msg_version` | string | `"1.0"` |
| `device_id` | string | 设备 ID |
| `timestamp_ms` | int | 毫秒时间戳 |
| `online` | bool | 在线状态 |
| `uptime_seconds` | int | 节点运行时长（秒） |
| `battery_level` | float | 电量百分比 |
| `control_mode` | string | `"manual"` / `"auto"` |
| `current_controller` | string | `"cloud"` / `"field"` / `"auto"` |
| `primary_uplink_protocol` | string | 当前主上行协议 |
| `protocol_table` | object | 各协议可用性（bool） |
| `position` | object | GPS 定位 `{lat, lon, alt}` |
| `error_code` | int | 错误码（0=正常） |

默认间隔：5000ms（`heartbeat_interval_ms` 配置）

---

### 3.4 状态快照上报（state_snapshot）

```json
{
  "msg_type": "state_snapshot",
  "msg_version": "1.0",
  "device_id": "device_001",
  "timestamp_ns": 1741564800000000000,
  "trigger": "periodic",
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

| 字段 | 类型 | 说明 |
|------|------|------|
| `msg_type` | string | 固定 `"state_snapshot"` |
| `msg_version` | string | `"1.0"` |
| `device_id` | string | 设备 ID |
| `timestamp_ns` | uint64 | 纳秒时间戳 |
| `trigger` | string | 触发类型：`"periodic"`（定时）/ `"change"`（字段变更）/ `"switch"`（控制切换后） |
| `fields` | object | 状态空间全量字段 |

**触发条件**：
- 定时器周期触发（默认 10s，`state_report_interval_ms` 配置）
- 关键字段变更即时触发（`online_status`、`error_code`、`control_mode`、`current_controller`、`battery_level`）
- 控制切换信令处理完成后触发
- 功能包写入 `urgent=true` 的 StateField 时触发

---

### 3.5 运控执行结果（move_result）

```json
{
  "cmd_timestamp_ns": 1741564800000000000,
  "success": true,
  "error_code": 0,
  "error_msg": "",
  "current_control_mode": "manual",
  "plugin_id": "cyberdog2",
  "plugin_internal_state": {
    "switch_status": 0
  }
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `cmd_timestamp_ns` | uint64 | 对应指令的时间戳 |
| `success` | bool | 执行是否成功 |
| `error_code` | int | 错误码（0=成功，1=失败） |
| `error_msg` | string | 错误描述（失败时填充） |
| `current_control_mode` | string | 当前控制模式 |
| `plugin_id` | string | 当前激活的插件 ID |
| `plugin_internal_state` | object | 插件内部状态（见各平台说明） |

**cyberdog2 插件内部状态**：

```json
{ "switch_status": 0 }
```

| `switch_status` | 含义 | 对运控的影响 |
|-----------------|------|-------------|
| `0` NORMAL | 正常运行 | 接受所有指令 |
| `1` TRANSITIONING | 动作切换中 | 暂缓 result 指令 |
| `2` ESTOP | 急停状态 | 拒绝速度指令，需先 `action="stand"` 恢复 |
| `3` EDAMP | 阻尼模式 | 拒绝速度指令，需先 `action="stand"` 恢复 |
| `4` LIFTED | 被抬起 | 拒绝运动指令 |
| `5` BAN_TRANS | 禁止切换 | 拒绝步态切换 |
| `6` OVER_HEAT | 过热 | 拒绝运动指令 |
| `7` LOW_BAT | 低电量 | 写入 `battery_low=true`，仍可运动 |
| `14` CHARGING | 充电中 | 拒绝所有运动指令 |

**px4_mavros 插件内部状态**：

```json
{
  "flight_state": "OFFBOARD",
  "armed": true,
  "mode": "OFFBOARD",
  "altitude_m": 1.47,
  "last_setpoint_age_ms": 50
}
```

---

### 3.5a 指令拒绝事件（move_reject_event）

```json
{
  "reason": "manual mode rejects auto controller",
  "controller": "auto",
  "control_mode": "manual"
}
```

| `reason` 值 | 触发条件 |
|-------------|---------|
| `"manual mode rejects auto controller"` | manual 模式下收到 `controller="auto"` 的指令 |
| `"auto mode rejects manual controller"` | auto 模式下收到 `controller="cloud"` 或 `"field"` 的指令 |
| `"failsafe active"` | 失控保护期间收到任何运控指令 |

---

### 3.5b CyberDog2 运动状态异常（cyberdog2_motion_abnormal）

当 `motion_status.switch_status` 从 NORMAL 变为异常时上报：

```json
{
  "switch_status": 14
}
```

`switch_status` 含义见 §3.5 cyberdog2 插件内部状态表。

---

### 3.5c 插件切换事件（plugin_switch_event）

```json
{
  "prev_plugin": "cyberdog2",
  "new_plugin": "cyberdog2",
  "timestamp_ns": 1741564800000000000
}
```

---

### 3.6 限速截断事件（move_clamp_event）

```json
{
  "event": "move_clamp",
  "original": { "vx": 2.5, "vy": 0.0, "vz": 0.0, "wz": 0.0 },
  "clamped":  { "vx": 1.0, "vy": 0.0, "vz": 0.0, "wz": 0.0 },
  "limit_applied": { "linear_vel_limit": 1.0, "angular_vel_limit": 1.0 },
  "timestamp_ns": 1741564800000000000
}
```

---

### 3.7 传感器数据上报（sensor）

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

| 字段 | 类型 | 说明 |
|------|------|------|
| `sensor_id` | string | 传感器标识 |
| `sensor_type` | string | 原始消息类型 |
| `converter_plugin` | string | 使用的转换插件 ID |
| `timestamp_ns` | uint64 | 纳秒时间戳 |
| `data` | object | 转换后的标准化数据 |

**官方预设转换插件输出格式**：

| 插件 ID | 输入类型 | 输出字段 |
|---------|----------|----------|
| `navsat_to_position` | NavSatFix | `{ lat, lon, alt, covariance }` |
| `imu_to_attitude` | Imu | `{ roll, pitch, yaw, angular_velocity, linear_acceleration }` |
| `battery_state` | BatteryState | `{ percentage, voltage, current, status }` |
| `range_sensor` | Range | `{ range_m, min_range, max_range, field_of_view }` |
| `pointcloud_summary` | PointCloud2 | `{ point_count, density, bounding_box }` |
| `laser_scan` | LaserScan | `{ min_angle, max_angle, ranges_json }` |
| `odometry` | Odometry | `{ position, orientation, linear_velocity, angular_velocity }` |

---

### 3.8 任务进度上报（task_progress）

**正常进度**：

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

**异常事件**：

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

| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | string | 任务 ID |
| `timestamp_ns` | uint64 | 纳秒时间戳 |
| `stage` | string | 任务阶段（`"executing"` / `"paused"` / `"completed"` 等） |
| `current_waypoint_seq` | int | 当前航点序号 |
| `total_waypoints` | int | 总航点数 |
| `progress_percent` | float | 进度百分比 |
| `status` | string | `"normal"` / `"exception"` |
| `event` | string/null | 事件标识 |
| `error` | object/null | 异常详情 |
| `error.code` | string | 异常码 |
| `error.severity` | string | 严重程度 |
| `error.description` | string | 描述 |
| `error.suggested_action` | string | 建议处置 |
| `position` | object | 当前位置 |
| `battery_level` | float | 当前电量 |

---

### 3.9 失控保护事件（failsafe_event）

```json
{
  "event": "triggered",
  "failsafe_action": "stop",
  "duration_s": 5.0,
  "timestamp_ns": 1741564800000000000
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `event` | string | `"triggered"` / `"recovered"` |
| `failsafe_action` | string | 执行的保护动作 |
| `duration_s` | float | 持续时间（秒） |
| `timestamp_ns` | uint64 | 纳秒时间戳 |

---

## 四、状态空间标准字段

| 字段名 | 类型 | 持久化 | 说明 |
|--------|------|--------|------|
| `device_id` | string | 是 | 装备实例主键 |
| `template_id` | string | 是 | 装备模板主键 |
| `device_type` | string | 是 | 设备类型（quadruped/uav/usv/…） |
| `online_status` | bool | 否 | 当前在线状态 |
| `current_controller` | string | 否 | 当前控制者（cloud/field/auto） |
| `control_mode` | string | 否 | 巡检模式（manual/auto） |
| `primary_uplink_protocol` | string | 否 | 当前主上行协议 |
| `protocol_table` | object | 否 | 各协议通路可用性表 |
| `battery_level` | float | 否 | 电量百分比 |
| `position` | object | 否 | GPS 定位 `{lat, lon, alt}` |
| `velocity` | object | 否 | 当前速度 `{vx, vy, vz}` |
| `attitude` | object | 否 | 姿态 `{roll, pitch, yaw}` |
| `linear_vel_limit` | float | 是 | 合速度上限（m/s），默认 1.0 |
| `angular_vel_limit` | float | 是 | 偏航角速度上限（rad/s），默认 1.0 |
| `error_code` | int | 否 | 当前错误码（0=正常） |
| `task_id` | string | 否 | 当前执行任务 ID |
| `task_stage` | string | 否 | 当前任务阶段 |
| `uptime_seconds` | int | 否 | 节点运行时长（秒） |
| `failsafe_active` | bool | 否 | 是否处于失控保护模式 |
| `failsafe_action_executed` | string | 否 | 最近一次执行的保护动作 |

---

## 五、协议通路表结构

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

| 字段 | 类型 | 说明 |
|------|------|------|
| `available` | bool | 通路是否可用 |
| `latency_ms` | int | 延迟（ms），-1 表示不可用 |
| `last_check_ts` | int | 上次探测时间戳（秒） |

---

## 六、标准异常码表

| 异常码 | 语义 | 建议处置 |
|--------|------|----------|
| `E_LOW_BATTERY` | 电量不足 | 返航 / 暂停 |
| `E_GPS_LOSS` | GPS 信号丢失 | 悬停 / 返航 |
| `E_LINK_TIMEOUT` | 通信链路超时 | 本地继续 / 返航 |
| `E_OBSTACLE` | 检测到障碍物 | 绕行 / 暂停 |
| `E_EXEC_TIMEOUT` | 执行超时 | 跳过当前航点 / 终止 |
| `E_MOVE_FAIL` | 运控执行失败 | 重试 / 终止 |
| `E_SENSOR_FAIL` | 传感器故障 | 记录并继续 / 终止 |
| `E_TASK_CONFLICT` | 控制冲突 | 任务暂停，等待控制切换 |
| `E_NOT_IN_OFFBOARD` | PX4 未进入 OFFBOARD 模式 | 重新执行 takeoff / 终止 |
| `E_FLIGHT_STATE_CONFLICT` | 飞行状态不允许该 action | 等待状态就绪后重试 / 终止 |
| `E_EMERGENCY_DISARM` | 紧急 disarm（飞机已断电） | 任务终止，人工介入 |
| `E_FAILSAFE_TRIGGERED` | 失控保护已触发（全部链路中断） | 任务暂停，等待链路恢复 |

---

## 七、各平台对运控指令字段的支持矩阵

| 统一指令字段 | cyberdog2（小米机器狗） | px4_mavros（无人机） | ros_twist（差速底盘） |
|-------------|----------------------|---------------------|---------------------|
| `linear_vel_x` | ✅ → `vel_des[0]`（前进，m/s） | ✅ ENU body_vel x | ✅ → Twist.linear.x |
| `linear_vel_y` | ✅ → `vel_des[1]`（侧步，左正） | ✅ ENU body_vel y | ❌ 忽略 |
| `linear_vel_z` | ❌ 忽略 | ✅ ENU body_vel z（上正） | ❌ 忽略 |
| `angular_vel_z` | ✅ → `vel_des[2]`（偏航，逆时针正） | ✅ yaw_rate | ✅ → Twist.angular.z |
| `target_roll` | ✅ → `rpy_des[0]`（仅 WALK_STAND 有效） | ❌ 忽略 | ❌ 忽略 |
| `target_pitch` | ✅ → `rpy_des[1]`（仅 WALK_STAND 有效） | ❌ 忽略 | ❌ 忽略 |
| `target_yaw` | ❌ 忽略（用 angular_vel_z 转向） | ✅ ENU 世界系绝对偏航角 | ❌ 忽略 |
| `action="stand"` | ✅ RECOVERYSTAND（motion_id=111） | ❌ | ❌ |
| `action="sit"` | ✅ GETDOWN（motion_id=101） | ❌ | ❌ |
| `action="stop"` | ✅ 零速 servo（保持站立步态） | ✅ 切 AUTO.HOLD 悬停 | ✅ 零速度 |
| `action="emergency_stop"` | ✅ ESTOP（motion_id=0，锁关节） | ✅ 立即 disarm（危险） | ✅ 零速度+报警 |
| `action="takeoff"` | ❌ | ✅ 飞行状态机 | ❌ |
| `action="land"` | ❌ | ✅ AUTO.LAND | ❌ |
| `action="return_home"` | ❌ | ✅ AUTO.RTL | ❌ |
| `extra_json.motion_id` | ✅ 直接调用 motion_result_cmd | ❌ | ❌ |
| `extra_json.step_height` | ✅ 覆盖步高 [前腿,后腿]（m） | ❌ | ❌ |

---

*文档结束*
