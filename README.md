# URAN — 统一机器人接入节点

URAN（Unified Robotics Access Node）是 OIVS 系统在无人装备侧的统一接入与能力承载层，以 ROS 2 软件包形态交付。

---

## 目录

- [环境要求](#环境要求)
- [快速开始](#快速开始)
- [软件包结构](#软件包结构)
- [uran_core 使用说明](#uran_core-使用说明)
  - [MQTT 控制说明](#mqtt-控制说明)
  - [配置文件](#配置文件)
  - [ROS 接口](#ros-接口)
- [消息与服务定义](#消息与服务定义)
- [实机测试（CyberDog2）](#实机测试cyberdog2)
- [相关文档](#相关文档)

---

## 环境要求

| 项目 | 版本 |
|------|------|
| 操作系统 | Ubuntu 22.04 |
| ROS 2 | Humble |
| Python | 3.10（conda 环境） |
| Conda | Miniconda / Anaconda |

---

## 快速开始

### 1. 创建 Conda 环境

```bash
conda create -n uran python=3.10 -y
conda activate uran
pip install empy==3.3.4 lark catkin_pkg rospkg numpy colcon-common-extensions paho-mqtt
```

> Python 必须为 3.10，与 ROS 2 Humble 绑定的系统 Python 版本一致。

### 2. 编译工作空间

```bash
conda activate uran
source /opt/ros/humble/setup.bash
cd ~/uran_ws
python -m colcon build
```

使用 `python -m colcon` 而非直接 `colcon build`，确保编译使用 conda 环境中的 Python，避免与系统 Python 冲突。

编译单个包：

```bash
python -m colcon build --packages-select uran_core
```

### 3. 加载环境

每次新开终端都需要执行：

```bash
conda activate uran
source /opt/ros/humble/setup.bash
source ~/uran_ws/install/setup.bash
```

### 4. 启动 uran_core 节点

```bash
ros2 run uran_core uran_core_node
# 或使用 launch 文件
ros2 launch uran_core uran_core.launch.py
```

---

## 软件包结构

```
uran_ws/
├── src/
│   ├── uran_msgs/          # 自定义消息定义（12 条）
│   ├── uran_srvs/          # 自定义服务定义（10 条）
│   ├── uran_core/          # 核心包 ← 已实现
│   ├── uran_move/          # 运控包 ← 已实现（T2.1–T2.3）
│   ├── uran_media/         # 流媒体包（待开发）
│   ├── uran_sensor/        # 传感器包（待开发）
│   ├── uran_frpcpoint/     # 端口转发服务（待开发）
│   └── uran_autotask/      # 自动化巡检服务（待开发）
├── URAN节点设计文档.md
├── 云端与URAN节点通信字段手册.md
└── URAN开发任务清单.md
```

---

## uran_core 使用说明

### MQTT 控制说明

#### 网络拓扑

```
[主机 / 云端]                        [机器狗]
mosquitto broker ◄──── WiFi ────► uran_core_node
mosquitto_pub/sub                       │
                                   uran_move_node
                                        │
                                  motion_manager
```

机器狗上的 `uran_core_node` 作为 MQTT 客户端连接到主机（或云端）的 Broker，所有控制指令通过 MQTT 下行，执行结果通过 MQTT 上行。

#### 前置准备

**主机上安装并启动 Broker：**

```bash
sudo apt install mosquitto mosquitto-clients
mosquitto -p 1883 -v   # -v 显示所有收发消息，便于调试
```

**机器狗上修改 Broker 地址（改为主机 IP）：**

```bash
# 查看主机 IP
ip addr show | grep "inet " | grep -v 127.0.0.1

# 编辑配置（改完需重新编译或直接改 install 目录下的副本）
nano /SDCARD/uran_ws/src/uran_core/config/network.yaml
# 将 broker_host: "localhost" 改为 broker_host: "192.168.x.x"

# 若直接改 install 目录下的副本则无需重新编译：
nano /SDCARD/uran_ws/install/share/uran_core/config/network.yaml
```

**主机上开启上行监听（另开终端，全程保持）：**

```bash
mosquitto_sub -h localhost -t '/oivs/default/device_001/up' -v
```

节点启动后 5 秒内应收到第一条 `msg_type=register`，随后每 5 秒收到 `msg_type=heartbeat`。

---

#### 消息格式

所有下行消息均为 JSON，必须包含以下公共字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| `msg_type` | string | 消息类型，见下表 |
| `msg_version` | string | 固定 `"1.0"` |
| `device_id` | string | 与 `network.yaml` 中一致，默认 `"device_001"` |
| `timestamp_ms` | int | Unix 毫秒时间戳，调试时填 `0` |

下行 Topic：`/oivs/default/device_001/down`
上行 Topic：`/oivs/default/device_001/up`

---

#### 下行指令参考

**1. 控制模式切换（control_switch）**

切换 manual / auto 模式，影响 uran_move 的指令来源过滤。

```bash
# 切换为 manual 模式（接受 cloud/field 来源，拒绝 auto）
mosquitto_pub -h localhost -t '/oivs/default/device_001/down' -m '{
  "msg_type": "control_switch",
  "msg_version": "1.0",
  "device_id": "device_001",
  "timestamp_ms": 0,
  "switch": {
    "control_mode": "manual",
    "controller": "cloud",
    "primary_uplink_protocol": "mqtt",
    "media": {"action": "stop", "protocol": ""}
  }
}'

# 切换为 auto 模式（接受 auto 来源，拒绝 cloud/field）
mosquitto_pub -h localhost -t '/oivs/default/device_001/down' -m '{
  "msg_type": "control_switch",
  "msg_version": "1.0",
  "device_id": "device_001",
  "timestamp_ms": 0,
  "switch": {
    "control_mode": "auto",
    "controller": "cloud",
    "primary_uplink_protocol": "mqtt",
    "media": {"action": "stop", "protocol": ""}
  }
}'
```

切换后上行会收到 `msg_type=state_snapshot`，其中 `control_mode` 字段反映新状态。

---

**2. 运控指令（move_cmd）**

`controller` 字段须与当前 `control_mode` 匹配：
- `manual` 模式 → `controller` 填 `"cloud"` 或 `"field"`
- `auto` 模式 → `controller` 填 `"auto"`

```bash
# 站立
mosquitto_pub -h localhost -t '/oivs/default/device_001/down' -m '{
  "msg_type": "move_cmd",
  "msg_version": "1.0",
  "device_id": "device_001",
  "timestamp_ms": 0,
  "controller": "cloud",
  "linear_vel_x": 0.0, "linear_vel_y": 0.0, "linear_vel_z": 0.0,
  "angular_vel_z": 0.0, "target_roll": 0.0, "target_pitch": 0.0, "target_yaw": 0.0,
  "action": "stand",
  "extra_json": ""
}'

# 坐下
mosquitto_pub -h localhost -t '/oivs/default/device_001/down' -m '{
  "msg_type": "move_cmd",
  "msg_version": "1.0",
  "device_id": "device_001",
  "timestamp_ms": 0,
  "controller": "cloud",
  "linear_vel_x": 0.0, "linear_vel_y": 0.0, "linear_vel_z": 0.0,
  "angular_vel_z": 0.0, "target_roll": 0.0, "target_pitch": 0.0, "target_yaw": 0.0,
  "action": "sit",
  "extra_json": ""
}'

# 向前走 0.3 m/s（单次发送，保活定时器 0.5s 后自动停止）
mosquitto_pub -h localhost -t '/oivs/default/device_001/down' -m '{
  "msg_type": "move_cmd",
  "msg_version": "1.0",
  "device_id": "device_001",
  "timestamp_ms": 0,
  "controller": "cloud",
  "linear_vel_x": 0.3, "linear_vel_y": 0.0, "linear_vel_z": 0.0,
  "angular_vel_z": 0.0, "target_roll": 0.0, "target_pitch": 0.0, "target_yaw": 0.0,
  "action": "",
  "extra_json": ""
}'

# 原地左转（angular_vel_z 正值为左转）
mosquitto_pub -h localhost -t '/oivs/default/device_001/down' -m '{
  "msg_type": "move_cmd",
  "msg_version": "1.0",
  "device_id": "device_001",
  "timestamp_ms": 0,
  "controller": "cloud",
  "linear_vel_x": 0.0, "linear_vel_y": 0.0, "linear_vel_z": 0.0,
  "angular_vel_z": 0.5, "target_roll": 0.0, "target_pitch": 0.0, "target_yaw": 0.0,
  "action": "",
  "extra_json": ""
}'

# 急停
mosquitto_pub -h localhost -t '/oivs/default/device_001/down' -m '{
  "msg_type": "move_cmd",
  "msg_version": "1.0",
  "device_id": "device_001",
  "timestamp_ms": 0,
  "controller": "cloud",
  "linear_vel_x": 0.0, "linear_vel_y": 0.0, "linear_vel_z": 0.0,
  "angular_vel_z": 0.0, "target_roll": 0.0, "target_pitch": 0.0, "target_yaw": 0.0,
  "action": "emergency_stop",
  "extra_json": ""
}'
```

> 速度指令需持续发送才能持续行走。可用 shell 循环模拟：
> ```bash
> while true; do
>   mosquitto_pub -h localhost -t '/oivs/default/device_001/down' -m \
>     '{"msg_type":"move_cmd","msg_version":"1.0","device_id":"device_001","timestamp_ms":0,"controller":"cloud","linear_vel_x":0.3,"linear_vel_y":0.0,"linear_vel_z":0.0,"angular_vel_z":0.0,"target_roll":0.0,"target_pitch":0.0,"target_yaw":0.0,"action":"","extra_json":""}'
>   sleep 0.2
> done
> # Ctrl+C 停止后，保活定时器 0.5s 内自动发零速
> ```

---

**3. 状态查询（state_query）**

```bash
mosquitto_pub -h localhost -t '/oivs/default/device_001/down' -m '{
  "msg_type": "state_query",
  "msg_version": "1.0",
  "device_id": "device_001",
  "timestamp_ms": 0,
  "field_names": ["control_mode", "battery_level", "cyberdog2_switch_status"]
}'
# 上行收到 msg_type=state_query_response，fields_json 包含查询结果
```

---

**4. 参数更新（param_update）**

动态修改速度限制，无需重启节点：

```bash
mosquitto_pub -h localhost -t '/oivs/default/device_001/down' -m '{
  "msg_type": "param_update",
  "msg_version": "1.0",
  "device_id": "device_001",
  "timestamp_ms": 0,
  "params": {
    "linear_vel_limit": 0.5,
    "angular_vel_limit": 0.8
  }
}'
```

---

#### 上行消息说明

| msg_type | 触发时机 | 关键字段 |
|----------|---------|---------|
| `register` | 节点启动时 | `device_id`, `template_id` |
| `heartbeat` | 每 5 秒 | `online`, `battery_level`, `control_mode`, `current_controller` |
| `state_snapshot` | 周期上报 / 关键字段变更 / 模式切换后 | `fields_json`（全量状态） |
| `uplink_data` | 每次运控指令执行后 | `data_type=move_result`, `success`, `error_code`, `error_msg` |
| `uplink_data` | 指令被限速截断时 | `data_type=move_clamp_event`, `original_v_norm`, `linear_limit` |
| `uplink_data` | 指令被模式过滤拒绝时 | `data_type=move_reject_event`, `reason`, `controller` |
| `uplink_data` | motion_status 异常时 | `data_type=cyberdog2_motion_abnormal`, `switch_status` |
| `state_query_response` | 收到 state_query 后 | `fields_json` |

---

#### 常见问题

**Q: 机器狗收不到 MQTT 指令**

检查 Broker 地址配置和网络连通性：
```bash
# 在机器狗上 ping 主机
ping 192.168.x.x

# 确认 uran_core 已连接（日志应有 MQTT connected）
# 或查询网络状态
ros2 service call /uran/core/network/status uran_srvs/srv/GetNetworkStatus '{}'
```

**Q: 发送 move_cmd 后上行收到 move_reject_event**

`controller` 与当前 `control_mode` 不匹配。先发 `control_switch` 切换到正确模式，或将 `controller` 改为 `"cloud"` / `"field"`（manual 模式下有效）。

**Q: 上行收到 move_result 但 success=false，error_msg 含 CHARGING**

机器狗正在充电，motion_manager 拒绝运动指令。拔掉充电线后重试。

---

### 配置文件

启动前修改 `install/share/uran_core/config/` 下的配置文件（或直接修改源码 `src/uran_core/config/`，重新编译后生效）。

**`config/network.yaml`** — 入网与 MQTT 配置：

```yaml
network:
  device_id: "device_001"      # 装备实例主键，须与云端一致
  template_id: "template_001"  # 装备模板主键
  tenant_id: "default"         # 租户 ID
  auth:
    token: "changeme"          # 云端鉴权 Token
  mqtt:
    enabled: true
    broker_host: "localhost"   # MQTT Broker 地址
    broker_port: 1883
    keepalive: 60
    topic_prefix: "/oivs/{tenant_id}/{device_id}"
  heartbeat_interval_ms: 5000  # 心跳间隔（ms）
```

**`config/core.yaml`** — 状态上报配置：

```yaml
uran_core:
  state_broadcast_interval_ms: 1000   # 对内广播周期（ms）
  state_report_interval_ms: 10000     # 对外上报周期（ms）
  state_report_on_change: true        # 关键字段变更时立即上报
  state_report_change_fields:
    - "online_status"
    - "error_code"
    - "control_mode"
    - "current_controller"
    - "battery_level"
  db_path: "/tmp/uran_core_state.db"  # 持久化 SQLite 路径
```

### ROS 接口

**发布的 Topic：**

| Topic | 消息类型 | 说明 |
|-------|---------|------|
| `/uran/core/state/broadcast` | `StateSnapshot` | 状态空间全量快照（1s 周期） |
| `/uran/core/heartbeat/status` | `HeartbeatStatus` | 心跳发送状态 |
| `/uran/core/switch/mode` | `ModeSwitchCmd` | 模式/控制者切换通知 |
| `/uran/core/switch/media` | `MediaSwitchCmd` | 流媒体通道切换通知 |
| `/uran/core/switch/uplink_protocol` | `UplinkProtocolCmd` | 上行协议切换通知 |
| `/uran/core/downlink/move_cmd` | `UnifiedMoveCmd` | 运控指令下发 |
| `/uran/core/downlink/task_ctrl` | `TaskCtrlCmd` | 任务控制指令下发 |
| `/uran/core/downlink/media_ctrl` | `MediaCtrlCmd` | 媒体通道控制指令下发 |
| `/uran/core/downlink/frpc_ctrl` | `FrpcCtrlCmd` | 端口转发控制指令下发 |
| `/uran/core/downlink/param_update` | `ParamUpdateCmd` | 参数更新广播 |

**订阅的 Topic：**

| Topic | 消息类型 | 说明 |
|-------|---------|------|
| `/uran/core/state/write` | `StateField` | 功能包写入状态空间字段 |
| `/uran/core/uplink/data` | `UplinkPayload` | 功能包上报数据统一入口 |

**提供的 Service：**

| Service | 类型 | 说明 |
|---------|------|------|
| `/uran/core/state/get` | `GetStateField` | 查询状态空间字段 |
| `/uran/core/state/set` | `SetStateField` | 写入状态空间字段 |
| `/uran/core/network/connect` | `ConnectProtocol` | 连接/断开协议通路 |
| `/uran/core/network/status` | `GetNetworkStatus` | 查询各协议连接状态 |
| `/uran/core/state_report/trigger` | `TriggerStateReport` | 手动触发即时状态上报 |
| `/uran/core/state_report/configure` | `ConfigureStateReport` | 修改上报周期与协议 |

### MQTT 通信格式

节点使用以下 MQTT Topic 与云端通信：

- **上行**：`/oivs/{tenant_id}/{device_id}/up`
- **下行**：`/oivs/{tenant_id}/{device_id}/down`

**支持的下行 `msg_type`：**

| msg_type | 处理方式 |
|----------|---------|
| `control_switch` | 更新状态空间，发布 switch 系列 Topic |
| `move_cmd` | 路由到 `/uran/core/downlink/move_cmd` |
| `task_ctrl` | 路由到 `/uran/core/downlink/task_ctrl` |
| `media_ctrl` | 路由到 `/uran/core/downlink/media_ctrl` |
| `frpc_ctrl` | 路由到 `/uran/core/downlink/frpc_ctrl` |
| `param_update` | 路由到 `/uran/core/downlink/param_update`，同时更新本地状态 |
| `state_query` | 直接响应，返回状态空间字段 |

---

## 消息与服务定义

### uran_msgs

| 消息 | 关键字段 | 用途 |
|------|---------|------|
| `StateField` | field_name, value_json, persistent, urgent | 写入状态空间字段 |
| `StateSnapshot` | msg_version, device_id, fields_json | 状态空间全量快照 |
| `HeartbeatStatus` | protocol, success, timestamp_ns | 心跳发送状态 |
| `UnifiedMoveCmd` | controller, linear_vel_x/y/z, angular_vel_z, action | 统一运控指令 |
| `ModeSwitchCmd` | control_mode, controller | 模式/控制者切换 |
| `MediaSwitchCmd` | action, protocol | 流媒体通道切换 |
| `UplinkProtocolCmd` | protocol | 上行协议切换 |
| `UplinkPayload` | source_pkg, data_type, payload_json, urgent | 统一上行数据载体 |
| `MediaCtrlCmd` | action, protocol, channel_id, signal_json | 媒体通道控制 |
| `FrpcCtrlCmd` | action, frps_host, frps_port, local_port, remote_port | 端口转发控制 |
| `TaskCtrlCmd` | task_id, action, task_type, task_params_json | 任务控制 |
| `ParamUpdateCmd` | params_json | 参数更新广播 |

### uran_srvs

| 服务 | 请求 | 响应 | 用途 |
|------|------|------|------|
| `GetStateField` | field_names[] | fields_json, success | 查询状态字段 |
| `SetStateField` | field_name, value_json, persistent | success, message | 写入状态字段 |
| `ConnectProtocol` | protocol, action | success, message | 控制协议连接 |
| `GetNetworkStatus` | — | protocol_table_json | 查询网络状态 |
| `SwitchMovePlugin` | plugin_id | success, current_plugin | 切换运控插件 |
| `ReloadConfig` | config_path | success, message | 重载配置 |
| `ListSensors` | — | sensors_json | 列出传感器 |
| `GetTaskStatus` | task_id | status_json | 查询任务状态 |
| `TriggerStateReport` | reason | success, message | 触发即时上报 |
| `ConfigureStateReport` | interval_ms, protocol, report_on_change | success, current_interval_ms | 配置上报参数 |

查看任意接口定义：

```bash
ros2 interface show uran_msgs/msg/StateField
ros2 interface show uran_srvs/srv/GetStateField
```

---

## 实机测试（CyberDog2）

本节说明如何在小米 CyberDog2 上联合启动 cyberdog_ws 原生节点与 URAN 节点，并进行端到端验证。

### 前提条件

- 已通过 SSH 连接到机器狗板载计算机（NX 板），或在机器狗上直接操作
- cyberdog_ws 已在机器狗上编译完成（原厂固件已包含，通常无需重新编译）
- uran_ws 已在机器狗上编译完成（见[快速开始](#快速开始)）
- 机器狗处于趴下/待机状态，电量充足（建议 > 30%）

> **安全提示**：首次测试时将机器狗放置在开阔平地，周围留出至少 1m 净空。确认急停方式：直接断电或通过 App 切换为手动模式。

---

### 架构说明

```
[云端 / MQTT] ──► uran_core_node ──► /uran/core/downlink/move_cmd
                                              │
                                     uran_move_node（cyberdog2 插件）
                                              │
                              ┌───────────────┴───────────────┐
                              ▼                               ▼
                    motion_servo_cmd               motion_result_cmd
                    （速度 Servo 指令）              （站立/坐下等动作）
                              │                               │
                         motion_manager（cyberdog_ws 原生节点）
```

uran_move_node 通过 `protocol` 包的 ROS 接口与 cyberdog_ws 的 `motion_manager` 通信，**不需要修改任何 cyberdog_ws 代码**。

---

### 步骤一：确认 cyberdog_ws 节点已运行

机器狗开机后，cyberdog_ws 的核心节点（包括 `motion_manager`）通常由系统服务自动启动。验证：

```bash
# 检查 motion_manager 是否在线
ros2 node list | grep motion

# 检查关键 topic/service 是否存在
ros2 topic list | grep motion_servo_cmd
ros2 service list | grep motion_result_cmd
```

预期输出中应包含：
- `/motion_servo_cmd`（topic）
- `/motion_result_cmd`（service）
- `/motion_status`（topic）

如果节点未启动，手动启动 cyberdog_ws（在机器狗上执行）：

```bash
source /opt/ros/humble/setup.bash
source ~/cyberdog_ws/install/setup.bash
ros2 launch cyberdog_bringup main.launch.py
```

---

### 步骤二：编译并安装 uran_ws

在机器狗上（或通过 SSH）：

```bash
# 激活 conda 环境（若机器狗上已安装 conda）
eval "$(conda shell.bash hook)" && conda activate uran

# 若机器狗上没有 conda，直接使用系统 Python 3.10
source /opt/ros/humble/setup.bash
cd ~/uran_ws
python -m colcon build
```

---

### 步骤三：启动 URAN 节点

需要开两个终端（或使用 tmux）。

**终端 1 — uran_core：**

```bash
eval "$(conda shell.bash hook)" && conda activate uran
source /opt/ros/humble/setup.bash
source ~/uran_ws/install/setup.bash
ros2 run uran_core uran_core_node
```

**终端 2 — uran_move：**

```bash
eval "$(conda shell.bash hook)" && conda activate uran
source /opt/ros/humble/setup.bash
source ~/cyberdog_ws/install/setup.bash   # 必须先 source，使 protocol 包可见
source ~/uran_ws/install/setup.bash
ros2 run uran_move uran_move_node
```

> `cyberdog_ws/install/setup.bash` 必须在 `uran_ws/install/setup.bash` **之前** source，否则 `protocol` 包找不到。

启动成功后，uran_move 日志应显示：

```
[INFO] [uran_move_node]: CyberDog2Plugin initialized
[INFO] [uran_move_node]: Plugin loaded: cyberdog2 (cyberdog2 v1.0.0)
[INFO] [uran_move_node]: uran_move_node ready, active_plugin=cyberdog2
```

也可以用 launch 文件一次启动两个节点：

```bash
ros2 launch uran_core uran_core.launch.py &
ros2 launch uran_move uran_move.launch.py
```

---

### 步骤四：验证 ROS 接口注册

```bash
# 确认 uran_move 订阅/发布正常
ros2 topic list | grep uran
ros2 service list | grep uran/move

# 预期包含：
# /uran/core/downlink/move_cmd
# /uran/core/uplink/data
# /uran/core/state/write
# /uran/move/switch_plugin
```

---

### 步骤五：功能验证

#### 5.1 站立指令

```bash
# 通过 ROS topic 直接发送（绕过 MQTT，用于调试）
ros2 topic pub --once /uran/core/downlink/move_cmd uran_msgs/msg/UnifiedMoveCmd \
  '{controller: "field", action: "stand", timestamp_ns: 0}'
```

机器狗应执行站立动作（RECOVERYSTAND，motion_id=111）。

#### 5.2 坐下指令

```bash
ros2 topic pub --once /uran/core/downlink/move_cmd uran_msgs/msg/UnifiedMoveCmd \
  '{controller: "field", action: "sit", timestamp_ns: 0}'
```

#### 5.3 速度行走

```bash
# 以 0.2 m/s 向前走（持续发送，停止后自动零速保活）
ros2 topic pub -r 10 /uran/core/downlink/move_cmd uran_msgs/msg/UnifiedMoveCmd \
  '{controller: "field", linear_vel_x: 0.2, angular_vel_z: 0.0, action: "", timestamp_ns: 0}'
```

停止发送后，uran_move 的保活定时器（20Hz）会在 0.5s 内自动发布零速指令，机器狗停止行走。

#### 5.4 急停

```bash
ros2 topic pub --once /uran/core/downlink/move_cmd uran_msgs/msg/UnifiedMoveCmd \
  '{controller: "field", action: "emergency_stop", timestamp_ns: 0}'
```

#### 5.5 通过 MQTT 下发运控指令

参见 [MQTT 控制说明](#mqtt-控制说明) 中的完整指令参考。主机 IP 需填入机器狗的 `network.yaml`，Broker 在主机上启动：

```bash
mosquitto -p 1883 -v
```

#### 5.6 切换运控插件（运行时）

```bash
ros2 service call /uran/move/switch_plugin uran_srvs/srv/SwitchMovePlugin \
  "{plugin_id: 'cyberdog2'}"
```

---

### 步骤六：模式切换验证

uran_move 会过滤与当前控制模式不匹配的指令来源：

```bash
# 切换为 manual 模式（只接受 cloud/field 指令，拒绝 auto）
ros2 topic pub --once /uran/core/switch/mode uran_msgs/msg/ModeSwitchCmd \
  '{control_mode: "manual", controller: "cloud", timestamp_ns: 0}'

# 此时发送 auto 来源的指令会被拒绝，并上报 move_reject_event
ros2 topic pub --once /uran/core/downlink/move_cmd uran_msgs/msg/UnifiedMoveCmd \
  '{controller: "auto", linear_vel_x: 0.2, action: "", timestamp_ns: 0}'

# 监听拒绝事件
ros2 topic echo /uran/core/uplink/data
```

---

### 常见问题

**Q: uran_move 启动时报 `protocol package not found`**

`protocol` 包来自 cyberdog_ws，需要先 source cyberdog_ws 的 install：

```bash
source ~/cyberdog_ws/install/setup.bash
source ~/uran_ws/install/setup.bash
ros2 run uran_move uran_move_node
```

**Q: `motion_result_cmd` service not ready**

motion_manager 未启动或未就绪。检查：

```bash
ros2 service list | grep motion_result_cmd
```

若不存在，等待 cyberdog_ws 完全启动（通常需要 10–30s）。

**Q: 发送速度指令后机器狗没有反应**

1. 确认机器狗已站立（先发 `action: "stand"`）
2. 检查 `motion_status` topic 的 `switch_status` 是否为 0（NORMAL）：
   ```bash
   ros2 topic echo /motion_status
   ```
3. 检查 uran_move 日志是否有 `motion not normal` 错误

**Q: 机器狗行走后无法停止**

uran_move 保活定时器会在停止发布指令后 0.5s 内自动发零速。若异常，手动急停：

```bash
ros2 topic pub --once /uran/core/downlink/move_cmd uran_msgs/msg/UnifiedMoveCmd \
  '{controller: "field", action: "emergency_stop", timestamp_ns: 0}'
```

---

### uran_core 本地调试（无 CyberDog2）

以下测试只需本地 MQTT Broker，不需要机器狗。

```bash
# 安装 mosquitto
sudo apt install mosquitto mosquitto-clients

# 终端 1：启动 Broker
mosquitto -p 1883

# 终端 2：启动节点
conda activate uran && source /opt/ros/humble/setup.bash && source ~/uran_ws/install/setup.bash
ros2 run uran_core uran_core_node
```

验证心跳上报：

```bash
mosquitto_sub -h localhost -t '/oivs/default/device_001/up'
```

模拟控制切换下行：

```bash
mosquitto_pub -h localhost -t '/oivs/default/device_001/down' -m '{
  "msg_type": "control_switch",
  "msg_version": "1.0",
  "device_id": "device_001",
  "timestamp_ms": 0,
  "switch": {
    "control_mode": "auto",
    "controller": "cloud",
    "primary_uplink_protocol": "mqtt",
    "media": {"action": "start", "protocol": "webrtc"}
  }
}'
```

查询/写入状态空间：

```bash
ros2 service call /uran/core/state/get uran_srvs/srv/GetStateField \
  "{field_names: ['control_mode', 'battery_level']}"

ros2 service call /uran/core/state/set uran_srvs/srv/SetStateField \
  "{field_name: 'battery_level', value_json: '85.0', persistent: false}"
```

触发即时状态上报：

```bash
ros2 service call /uran/core/state_report/trigger uran_srvs/srv/TriggerStateReport \
  "{reason: 'manual'}"
```

---

## 相关文档

- `URAN节点设计文档.md` — 完整架构与接口设计
- `云端与URAN节点通信字段手册.md` — 上下行通信字段参考
- `URAN开发任务清单.md` — 开发任务分解与测试方案
