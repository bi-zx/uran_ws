# URAN Base

`uran_base` 是 URAN 的基础架构层，用来承接后续对 `uran_core` 的结构化重构。
它的目标不是再堆一个“大而全的单节点文件”，而是把核心能力拆成清晰、可扩展、
可二次开发的基础模块，方便后续统一规范 `uran_core`、`uran_move`、`uran_media`
和 `uran_autotask` 的接入方式。

## 设计目标

- 将状态管理、协议接入、ROS 路由、下行分发、上行转发、包注册等职责解耦。
- 提供统一的包注册表，便于后续做能力声明、依赖治理和扩展管理。
- 提供统一的 ROS 路由目录，避免 topic 和 service 名称散落在各处硬编码。
- 提供协议注册层，让 MQTT 成为“一个实现”，而不是“唯一写死的实现”。
- 提供可复用的结构化核心节点，作为后续 `uran_core` 迁移的基础运行时。

## 当前目录结构

```text
uran_base/
├── config/
│   ├── core.yaml
│   ├── network.yaml
│   └── routes.yaml
├── resource/
│   └── uran_base
├── uran_base/
│   ├── protocols/
│   │   ├── __init__.py
│   │   ├── base.py
│   │   └── mqtt_adapter.py
│   ├── __init__.py
│   ├── config_loader.py
│   ├── downlink_router.py
│   ├── models.py
│   ├── node_base.py
│   ├── registries.py
│   ├── route_catalog.py
│   ├── state_schema.py
│   ├── state_store.py
│   ├── structured_core_node.py
│   └── uplink_gateway.py
├── package.xml
├── setup.cfg
└── setup.py
```

## 当前已实现的能力

### 1. 状态空间基础层

- 在 `state_schema.py` 中定义了 URAN 基础状态字段。
- 在 `state_store.py` 中实现了线程安全的状态读写。
- 支持默认值初始化。
- 支持 SQLite 持久化字段保存。
- 支持字段变化回调，可用于状态变化触发即时上报。

当前已内置的典型状态字段包括：

- `device_id`
- `template_id`
- `control_mode`
- `current_controller`
- `primary_uplink_protocol`
- `protocol_table`
- `package_registry`
- `route_registry`
- `battery_level`
- `position`
- `task_id`
- `task_stage`
- `motion_control_lock`

### 2. 包注册表

在 `registries.py` 中实现了 `PackageRegistry`，用于统一管理 URAN 各功能包的元信息。

当前支持：

- 注册包 ID、分类、版本、是否启用。
- 声明包提供的能力列表。
- 导出为字典，写入状态空间。

默认在 `config/core.yaml` 中预注册了：

- `uran_core`
- `uran_move`
- `uran_media`
- `uran_autotask`

后续可以继续扩展为：

- 插件注册表
- 任务执行器注册表
- 设备适配器注册表
- 算法模块注册表

### 3. 协议注册层与解耦方向

在 `registries.py` 中实现了 `ProtocolRegistry`，在 `protocols/` 中实现了协议适配接口。

当前已实现：

- `ProtocolAdapter` 抽象基类
- `MqttProtocolAdapter` MQTT 协议适配器
- `NullProtocolAdapter` 空协议适配器
- `ProtocolFactory` 协议工厂
- 协议健康状态汇总
- 协议注册与统一发布入口

这样后续如果你要扩展：

- WebSocket
- TCP
- UDP
- LoRa

只需要继续补一个新的协议适配器，并注册到协议工厂里，而不需要把逻辑继续写死到核心节点里。

### 4. ROS 路由目录

在 `route_catalog.py` 中实现了 `RouteCatalog`，统一从 `config/routes.yaml` 读取主题和服务路由。

当前已经统一收敛了这些核心路由：

- 状态写入
- 状态广播
- 上行数据
- 心跳广播
- 模式切换
- 媒体切换
- 上行协议切换
- 运动控制下行
- 任务控制下行
- 媒体控制下行
- FRPC 控制下行
- 参数更新下行
- 核心状态查询与配置服务

后续其他包可以直接依赖这套路由目录，而不是继续自己拼 topic 名。

### 5. 下行消息路由器

在 `downlink_router.py` 中实现了 `DownlinkRouter`，用于把协议层收到的下行消息按 `msg_type`
分发到具体处理器。

当前结构化核心节点已经接入了这些下行类型：

- `control_switch`
- `move_cmd`
- `task_ctrl`
- `media_ctrl`
- `frpc_ctrl`
- `param_update`
- `state_query`

这意味着后续新增一种云端下行指令时，不需要把所有逻辑继续堆在一个巨大的 `if/elif`
里，而是可以按消息类型继续扩展。

### 6. 上行网关

在 `uplink_gateway.py` 中实现了 `UplinkGateway`，用于统一处理 ROS 包发往核心层的上行数据。

当前支持：

- 将 `UplinkPayload` 转换为统一上行包结构。
- 根据首选协议或当前主协议选择上行通道。
- 当前主协议失败时尝试其他已注册协议。

这部分是后续统一传感器数据、任务进度、告警事件、媒体信令上报的重要基础。

### 7. 结构化核心节点

在 `structured_core_node.py` 中实现了一个结构化的参考核心节点，作为后续 `uran_core`
迁移的目标基础实现。

当前节点已具备这些能力：

- 加载基础配置、网络配置、路由配置。
- 初始化状态空间、包注册表、协议注册表。
- 创建统一的 ROS publisher、subscriber、service。
- 接入 MQTT 协议并处理注册流程。
- 接收协议下行数据并按类型路由。
- 将下行命令转发到现有 URAN ROS 主题。
- 接收来自功能包的上行数据并转发到协议层。
- 定时广播状态快照。
- 定时发送心跳。
- 支持周期性状态上报。
- 支持状态变更触发上报。
- 支持状态查询与网络状态查询服务。
- 支持运行时修改状态上报周期和协议。

当前这部分已经进一步向“装配器”方向演进：

- `structured_core_node.py` 负责装配状态、协议、路由、服务和定时器
- 具体下行处理逻辑拆到了 `handlers/` 目录
- 新增下行处理上下文 `DownlinkContext`
- core 不再亲自实现每一种消息处理细节，而是为 handler 提供依赖

### 8. 节点基类

在 `node_base.py` 中提供了一个可直接继承的基础节点类 `UranBaseNode`。

当前它已经具备：

- 自动接入 core 的状态写入 topic
- 自动接入 core 的上行转发 topic
- 统一发布状态字段的方法
- 统一发布上行消息的方法
- 订阅状态广播的方法
- 订阅心跳的方法
- 订阅模式切换的方法
- 订阅媒体切换的方法
- 订阅上行协议切换的方法
- 订阅 move/task/media/param 等核心下行路由的方法

这会让后续 `uran_media`、`uran_move`、`uran_autotask` 继承后更容易接入 `uran_core` 的统一转发体系。

### 9. 标准化下行钩子与注册机制

为了让业务节点接入方式更像“插件”，`UranBaseNode` 继续补充了两类机制：

- 标准化下行钩子注册
- 标准注册上报机制

#### 标准化下行钩子

业务节点可以通过统一方法注册自己关心的 core 下行路由，而不是自己手写 topic 名：

- `register_downlink_hook(...)`
- `subscribe_move_cmd(...)`
- `subscribe_task_ctrl(...)`
- `subscribe_media_ctrl(...)`
- `subscribe_param_update(...)`
- `subscribe_frpc_ctrl(...)`
- 以及状态广播、心跳、模式切换等辅助订阅方法

每次注册下行订阅时，基类会自动记录：

- 路由名
- 消息类型
- 描述信息

这样 core 层后续就能知道某个业务节点“声明自己接收什么”。

#### 标准注册上报

业务节点可以通过：

- `declare_capability(...)`
- `declare_uplink_type(...)`
- `register_with_core(...)`
- `on_core_ready()`

向 core 上报自己的运行时注册信息。

当前注册信息包括：

- `package_name`
- `node_name`
- `capabilities`
- `downlink_subscriptions`
- `uplink_data_types`
- `metadata`

这些信息会被写入 core 状态中的 `node_registry`，同时也会通过上行消息发给 core。

这使得后续你可以继续往“节点插件化目录”“节点能力发现”“节点治理面板”“节点动态接入”
这些方向演进。

## 当前这版的定位

这次实现的 `uran_base` 主要是“把基础架构搭起来”，它已经具备规范化核心层的基本骨架，
但还没有直接替换现有 `src/uran_core` 的运行逻辑。

也就是说，现在的状态是：

- `uran_base` 已经完成基础架构层实现。
- 它可以作为后续 `uran_core` 重构迁移的目标底座。
- 现有 `uran_core`、`uran_move`、`uran_media`、`uran_autotask` 还没有正式改为直接依赖它。

## 后续建议的迁移路径

建议按下面顺序继续推进：

1. 让现有 `uran_core` 变成一个薄封装，只负责启动 `uran_base.structured_core_node`。
2. 把现有 `uran_core` 中残留的状态、路由、协议逻辑逐步下沉到 `uran_base`。
3. 让 `uran_move`、`uran_media`、`uran_autotask` 逐步改为依赖 `RouteCatalog` 和基础发布接口。
4. 继续在 `uran_base` 中补充插件注册表、任务注册表、设备适配器注册表等扩展机制。

## 当前验证情况

当前已完成的基础验证：

- `python -m compileall src\uran_base\uran_base`

说明：

- 已通过 Python 语法级检查。
- 尚未执行完整 ROS 运行级联调。
