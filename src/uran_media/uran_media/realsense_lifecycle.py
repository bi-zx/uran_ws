"""realsense_lifecycle.py — RealSense 生命周期管理

完整流程：
  1. 检查节点是否已运行（ros2 lifecycle get）
  2. 若不存在，自动 ros2 launch 启动节点
  3. ros2 lifecycle set configure → activate

多个 channel 共享同一 lifecycle 节点时使用引用计数，避免重复激活/停止。
使用 subprocess 调用 ros2 lifecycle 命令，避免 executor 回调内 service client 死锁。
"""

import subprocess
import time


def _run_cmd(cmd: list, timeout: float = 15.0) -> tuple:
    """运行命令，返回 (success, stdout)。"""
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode == 0, r.stdout.strip()
    except subprocess.TimeoutExpired:
        return False, 'timeout'
    except Exception as e:
        return False, str(e)


class RealSenseLifecycleAdapter:

    def __init__(self, node, lifecycle_node_name: str,
                 launch_pkg: str = '', launch_file: str = ''):
        self._node = node
        self._name = lifecycle_node_name
        self._launch_pkg = launch_pkg
        self._launch_file = launch_file
        self._ref_count = 0
        self._launch_proc = None

    def _is_node_running(self) -> bool:
        """检查 lifecycle 节点是否已运行。"""
        ok, out = _run_cmd(['ros2', 'lifecycle', 'get', self._name], timeout=5.0)
        return ok

    def _launch_node(self) -> bool:
        """自动启动 RealSense 节点。"""
        if not self._launch_pkg or not self._launch_file:
            self._node.get_logger().warning(
                '[RealSense] launch_pkg/launch_file not configured, cannot auto-launch'
            )
            return False

        if self._launch_proc and self._launch_proc.poll() is None:
            return True

        cmd = ['ros2', 'launch', self._launch_pkg, self._launch_file]
        self._node.get_logger().info(f'[RealSense] Launching: {" ".join(cmd)}')
        try:
            self._launch_proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            return True
        except Exception as e:
            self._node.get_logger().error(f'[RealSense] Launch failed: {e}')
            return False

    def _wait_for_node(self, timeout: float = 20.0) -> bool:
        """等待 lifecycle 节点就绪。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._is_node_running():
                return True
            time.sleep(1.0)
        return False

    def _lifecycle_set(self, transition: str) -> bool:
        """调用 ros2 lifecycle set。"""
        cmd = ['ros2', 'lifecycle', 'set', self._name, transition]
        ok, out = _run_cmd(cmd, timeout=15.0)
        self._node.get_logger().info(
            f'[RealSense] lifecycle set {transition}: {"OK" if ok else out}'
        )
        return ok

    def activate(self) -> bool:
        self._ref_count += 1
        if self._ref_count > 1:
            return True

        # 检查节点是否已运行
        if not self._is_node_running():
            self._node.get_logger().info(
                f'[RealSense] {self._name} not running, attempting auto-launch...'
            )
            if not self._launch_node():
                return True  # launch 失败，跳过

            if not self._wait_for_node():
                self._node.get_logger().warning(
                    f'[RealSense] {self._name} not ready after launch, proceeding anyway'
                )
                return True

        # configure（已 configured/active 时会失败，忽略）
        self._lifecycle_set('configure')
        # activate
        ok = self._lifecycle_set('activate')
        if not ok:
            self._node.get_logger().warning(
                f'[RealSense] activate may have failed (node might already be active)'
            )
        return True

    def deactivate(self):
        self._ref_count = max(0, self._ref_count - 1)
        if self._ref_count > 0:
            return
        self._lifecycle_set('deactivate')

    def shutdown(self):
        """终止 launch 进程。"""
        if self._launch_proc and self._launch_proc.poll() is None:
            self._launch_proc.terminate()
            self._node.get_logger().info('[RealSense] Launch process terminated')
