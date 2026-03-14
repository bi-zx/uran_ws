# URAN — 统一机器人接入节点

OIVS 系统在无人装备侧的统一接入与能力承载层，以 ROS 2 软件包形态交付。

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Miniconda / Anaconda

## 快速开始

### 1. 创建 Conda 环境

```bash
conda create -n uran python=3.10 -y
conda activate uran
pip install empy==3.3.4 lark catkin_pkg rospkg numpy colcon-common-extensions
```

> Python 版本必须为 3.10，与 ROS 2 Humble 绑定的系统 Python 一致。

### 2. 编译

```bash
conda activate uran
source /opt/ros/humble/setup.bash
cd ~/uran_ws
python -m colcon build
```

使用 `python -m colcon` 而非直接 `colcon build`，确保编译使用 conda 环境中的 Python 和依赖，避免与系统 Python 冲突。

编译单个包：

```bash
python -m colcon build --packages-select uran_msgs
```

### 3. 运行

```bash
conda activate uran
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run <package_name> <node_name>
```

### 4. 验证消息/服务定义

```bash
source install/setup.bash
ros2 interface show uran_msgs/msg/StateField
ros2 interface show uran_srvs/srv/GetStateField
```

## 软件包结构

```
src/
├── uran_msgs/          自定义消息定义
├── uran_srvs/          自定义服务定义
├── uran_core/          核心包（状态空间、入网、心跳、控制切换、上下行通路）
├── uran_move/          运控包（统一运控指令 → 厂商 SDK 转换）
├── uran_media/         流媒体包（WebRTC/RTSP）
├── uran_sensor/        传感器包（多源数据采集与上报）
├── uran_frpcpoint/     端口转发服务（frpc 反向隧道）
└── uran_autotask/      自动化巡检服务
```

## 消息定义（uran_msgs）

| 消息 | 用途 |
|------|------|
| `StateField` | 功能包向 URAN-core 写入状态空间字段 |
| `StateSnapshot` | URAN-core 广播状态空间全量快照 |
| `HeartbeatStatus` | 心跳发送状态 |
| `UnifiedMoveCmd` | 统一运控指令 |
| `ModeSwitchCmd` | 模式/控制者切换通知 |
| `MediaSwitchCmd` | 流媒体通道切换通知 |
| `UplinkProtocolCmd` | 上行协议切换通知 |
| `UplinkPayload` | 统一上行数据载体 |
| `MediaCtrlCmd` | 媒体通道控制指令 |
| `FrpcCtrlCmd` | 端口转发控制指令 |
| `TaskCtrlCmd` | 任务控制指令 |
| `ParamUpdateCmd` | 参数更新广播 |

## 服务定义（uran_srvs）

| 服务 | 用途 |
|------|------|
| `GetStateField` | 查询状态空间字段 |
| `SetStateField` | 同步写入状态空间 |
| `ConnectProtocol` | 控制协议连接/断开 |
| `GetNetworkStatus` | 查询网络连接状态 |
| `SwitchMovePlugin` | 切换运控插件 |
| `ReloadConfig` | 重载配置 |
| `ListSensors` | 列出已注册传感器 |
| `GetTaskStatus` | 查询任务状态 |
| `TriggerStateReport` | 手动触发状态上报 |
| `ConfigureStateReport` | 修改上报周期与协议 |

## 相关文档

- `URAN节点设计文档.md` — 完整架构与接口设计
- `云端与URAN节点通信字段手册.md` — 上下行通信字段参考
- `URAN开发任务清单.md` — 开发任务分解与测试方案
