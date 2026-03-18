"""lifecycle_camera.py — ROS 2 Lifecycle 节点适配器

用于激活 stereo_camera 等 lifecycle 节点（configure → activate）。
多个 channel 可共享同一个 lifecycle 节点，通过引用计数管理生命周期。
"""

import time

try:
    from lifecycle_msgs.srv import ChangeState, GetState
    _LIFECYCLE_AVAILABLE = True
except ImportError:
    _LIFECYCLE_AVAILABLE = False

# Lifecycle transition IDs
_CONFIGURE = 1
_ACTIVATE = 3
_DEACTIVATE = 4

# Lifecycle state IDs
_STATE_UNCONFIGURED = 1
_STATE_INACTIVE = 2
_STATE_ACTIVE = 3


class LifecycleNodeAdapter:
    """管理单个 ROS 2 Lifecycle 节点的状态转换，支持引用计数。"""

    def __init__(self, node, lifecycle_node_name: str):
        self._node = node
        self._node_name = lifecycle_node_name
        self._ref_count = 0
        self._change_client = None
        self._get_client = None
        self._available = False
        self._init_clients()

    def _init_clients(self):
        if not _LIFECYCLE_AVAILABLE:
            self._node.get_logger().warning(
                '[LifecycleAdapter] lifecycle_msgs not available, will subscribe topic directly'
            )
            return
        try:
            self._change_client = self._node.create_client(
                ChangeState, f'{self._node_name}/change_state'
            )
            self._get_client = self._node.create_client(
                GetState, f'{self._node_name}/get_state'
            )
            self._available = True
            self._node.get_logger().info(
                f'[LifecycleAdapter] clients created for {self._node_name}'
            )
        except Exception as e:
            self._node.get_logger().warning(f'[LifecycleAdapter] init failed: {e}')

    def acquire(self) -> bool:
        """增加引用计数，首次调用时激活节点。"""
        self._ref_count += 1
        if self._ref_count == 1:
            return self._activate()
        return True

    def release(self):
        """减少引用计数，归零时停用节点。"""
        self._ref_count = max(0, self._ref_count - 1)
        if self._ref_count == 0:
            self._deactivate()

    def _activate(self) -> bool:
        if not self._available:
            return True

        state = self._get_state()
        if state is None:
            self._node.get_logger().warning(
                f'[LifecycleAdapter] Cannot get state of {self._node_name}, proceeding anyway'
            )
            return True

        if state == _STATE_ACTIVE:
            self._node.get_logger().info(
                f'[LifecycleAdapter] {self._node_name} already active'
            )
            return True

        if state == _STATE_UNCONFIGURED:
            if not self._transition(_CONFIGURE, 'configure'):
                return False

        ok = self._transition(_ACTIVATE, 'activate')
        if ok:
            self._node.get_logger().info(
                f'[LifecycleAdapter] {self._node_name} activated'
            )
        return ok

    def _deactivate(self):
        if not self._available:
            return
        state = self._get_state()
        if state == _STATE_ACTIVE:
            self._transition(_DEACTIVATE, 'deactivate')
            self._node.get_logger().info(
                f'[LifecycleAdapter] {self._node_name} deactivated'
            )

    def _get_state(self):
        if not self._get_client.wait_for_service(timeout_sec=2.0):
            return None
        try:
            future = self._get_client.call_async(GetState.Request())
            deadline = time.time() + 3.0
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)
            if future.done():
                return future.result().current_state.id
        except Exception as e:
            self._node.get_logger().warning(f'[LifecycleAdapter] get_state error: {e}')
        return None

    def _transition(self, transition_id: int, name: str) -> bool:
        if not self._change_client.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().warning(
                f'[LifecycleAdapter] change_state not available for {self._node_name}'
            )
            return False
        try:
            req = ChangeState.Request()
            req.transition.id = transition_id
            future = self._change_client.call_async(req)
            deadline = time.time() + 5.0
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)
            if not future.done():
                self._node.get_logger().warning(
                    f'[LifecycleAdapter] {name} timeout for {self._node_name}'
                )
                return True  # 超时时乐观继续，节点可能已处于目标状态
            return future.result().success
        except Exception as e:
            self._node.get_logger().error(f'[LifecycleAdapter] {name} error: {e}')
            return False
