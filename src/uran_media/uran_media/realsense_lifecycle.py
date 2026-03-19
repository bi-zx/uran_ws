"""realsense_lifecycle.py — RealSense 生命周期管理

通过 lifecycle_msgs/srv/ChangeState 控制 /camera/camera 或 /camera/camera_align 节点。
多个 channel 共享同一 lifecycle 节点时使用引用计数，避免重复激活/停止。
"""

import time


class RealSenseLifecycleAdapter:
    TRANSITION_CONFIGURE = 1
    TRANSITION_CLEANUP = 2
    TRANSITION_ACTIVATE = 3
    TRANSITION_DEACTIVATE = 4

    def __init__(self, node, lifecycle_node_name: str):
        self._node = node
        self._name = lifecycle_node_name
        self._ref_count = 0
        self._client = None
        self._ChangeState = None
        self._available = False
        self._init_client()

    def _init_client(self):
        try:
            from lifecycle_msgs.srv import ChangeState
            svc = f'{self._name}/change_state'
            self._client = self._node.create_client(ChangeState, svc)
            self._ChangeState = ChangeState
            self._available = True
            self._node.get_logger().info(f'[RealSense] lifecycle client: {svc}')
        except Exception as e:
            self._node.get_logger().warning(
                f'[RealSense] lifecycle_msgs unavailable ({e}), will subscribe topic directly'
            )

    def activate(self) -> bool:
        self._ref_count += 1
        if self._ref_count > 1:
            return True  # 已由其他 channel 激活

        if not self._available:
            return True

        if not self._client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().warning(
                f'[RealSense] {self._name}/change_state not available, proceeding anyway'
            )
            return True

        # configure（若已 configured/active 会返回失败，忽略）
        self._transition(self.TRANSITION_CONFIGURE)
        # activate
        ok = self._transition(self.TRANSITION_ACTIVATE)
        if ok:
            self._node.get_logger().info(f'[RealSense] {self._name} activated')
        else:
            self._node.get_logger().warning(
                f'[RealSense] activate failed (may already be active), proceeding'
            )
        return True

    def deactivate(self):
        self._ref_count = max(0, self._ref_count - 1)
        if self._ref_count > 0:
            return
        if not self._available or self._client is None:
            return
        self._transition(self.TRANSITION_DEACTIVATE)
        self._node.get_logger().info(f'[RealSense] {self._name} deactivated')

    def _transition(self, transition_id: int) -> bool:
        try:
            req = self._ChangeState.Request()
            req.transition.id = transition_id
            future = self._client.call_async(req)
            deadline = time.time() + 10.0
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)
            if future.done():
                return future.result().success
            self._node.get_logger().warning(
                f'[RealSense] transition {transition_id} timeout'
            )
            return False
        except Exception as e:
            self._node.get_logger().error(f'[RealSense] transition error: {e}')
            return False
