from abc import ABC, abstractmethod


class MovePluginBase(ABC):
    """运控插件基类，所有设备适配插件必须继承此类。"""

    @abstractmethod
    def init(self, node, params: dict) -> bool:
        """初始化插件，创建 ROS publisher/subscriber/client。

        Args:
            node: rclpy.node.Node 实例（主节点）
            params: plugins.yaml 中该插件的 params 字典
        Returns:
            True 表示初始化成功
        """
        ...

    @abstractmethod
    def execute(self, cmd) -> tuple:
        """执行运控指令。

        Args:
            cmd: UnifiedMoveCmd 消息
        Returns:
            (success: bool, result_json: str)
        """
        ...

    @abstractmethod
    def device_type(self) -> str:
        """返回设备类型标识，如 'cyberdog2'。"""
        ...

    @abstractmethod
    def version(self) -> str:
        """返回插件版本字符串。"""
        ...

    def internal_state_json(self) -> str:
        """返回插件内部状态的 JSON 字符串，用于上报。"""
        return "{}"

    def on_failsafe(self):
        """失控保护触发时调用，插件应执行安全停止动作。"""
        pass

    def on_failsafe_recovered(self):
        """失控保护恢复时调用。"""
        pass

    def destroy(self):
        """节点关闭时调用，释放资源。"""
        pass
