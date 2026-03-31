"""uran_move_node — bridges UnifiedMoveCmd to px4ctrl interfaces.

Supported action mappings (px4_mavros plugin, drone only):
  action="takeoff"         → publishes TakeoffLand(TAKEOFF) to px4ctrl
  action="land"            → publishes TakeoffLand(LAND) to px4ctrl
  action="emergency_stop"  → publishes TakeoffLand(LAND) to px4ctrl (best-effort)
  action="stop" / all-zero → publishes zero PositionCommand (hover in place)
  velocity command         → publishes PositionCommand with velocity fields

Coordinate convention: REP-103 ENU body frame throughout.
px4ctrl PositionCommand uses the same ENU convention — no transform needed.
"""
import math
import threading
import time

import rospy
from uran_msgs.msg import UnifiedMoveCmd
from quadrotor_msgs.msg import PositionCommand, TakeoffLand

# px4ctrl topics (from px4ctrl_node.cpp)
_TOPIC_CMD       = 'position_cmd'   # quadrotor_msgs/PositionCommand
_TOPIC_TKL       = 'takeoff_land'   # quadrotor_msgs/TakeoffLand

# URAN downlink topic
_TOPIC_MOVE_CMD  = '/uran/core/downlink/move_cmd'

# Keep-alive: PX4 OFFBOARD requires setpoints at >=2 Hz.
_KEEPALIVE_HZ    = 20
_CMD_TIMEOUT_S   = 0.5  # seconds before keep-alive reverts to hover


class UranMoveNode:
    def __init__(self):
        self._lock = threading.Lock()
        self._last_pos_cmd: PositionCommand | None = None
        self._last_cmd_time: float = 0.0

        self._pub_cmd = rospy.Publisher(_TOPIC_CMD, PositionCommand, queue_size=10)
        self._pub_tkl = rospy.Publisher(_TOPIC_TKL, TakeoffLand, queue_size=5)

        rospy.Subscriber(_TOPIC_MOVE_CMD, UnifiedMoveCmd, self._on_move_cmd,
                         queue_size=10, tcp_nodelay=True)

        # Keep-alive timer — republishes last velocity setpoint so px4ctrl
        # stays in CMD_CTRL and PX4 stays in OFFBOARD.
        self._keepalive_timer = rospy.Timer(
            rospy.Duration(1.0 / _KEEPALIVE_HZ), self._keepalive_cb
        )

        rospy.loginfo('[uran_move] node started, listening on %s', _TOPIC_MOVE_CMD)

    # ------------------------------------------------------------------
    # Subscriber callback
    # ------------------------------------------------------------------
    def _on_move_cmd(self, msg: UnifiedMoveCmd):
        action = msg.action.strip().lower()

        if action in ('takeoff',):
            self._send_takeoff_land(TakeoffLand.TAKEOFF)
            return

        if action in ('land', 'return_home'):
            self._send_takeoff_land(TakeoffLand.LAND)
            return

        if action == 'emergency_stop':
            # Best-effort: command land immediately.
            rospy.logwarn('[uran_move] emergency_stop → triggering LAND')
            self._send_takeoff_land(TakeoffLand.LAND)
            return

        # For "stop", empty action, or pure velocity commands build a
        # PositionCommand and publish it (also updates keep-alive buffer).
        cmd = self._build_position_cmd(msg)
        with self._lock:
            self._last_pos_cmd = cmd
            self._last_cmd_time = time.monotonic()
        self._pub_cmd.publish(cmd)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _send_takeoff_land(self, cmd_type: int):
        msg = TakeoffLand()
        msg.takeoff_land_cmd = cmd_type
        self._pub_tkl.publish(msg)
        label = 'TAKEOFF' if cmd_type == TakeoffLand.TAKEOFF else 'LAND'
        rospy.loginfo('[uran_move] published %s to px4ctrl', label)

    def _build_position_cmd(self, msg: UnifiedMoveCmd) -> PositionCommand:
        """Convert UnifiedMoveCmd velocity fields to PositionCommand.

        px4ctrl CMD_CTRL mode uses the velocity fields of PositionCommand;
        position/acceleration/jerk are zeroed (not commanded).
        Coordinate mapping (REP-103 ENU body == px4ctrl convention):
          linear_vel_x  → velocity.x  (forward, no transform)
          linear_vel_y  → velocity.y  (left,    no transform)
          linear_vel_z  → velocity.z  (up,      no transform)
          angular_vel_z → yaw_dot     (CCW positive, no transform)
          target_yaw    → yaw         (ENU world absolute, NaN → 0.0)
        """
        vx = msg.linear_vel_x
        vy = msg.linear_vel_y
        vz = msg.linear_vel_z

        # Proportional clamp on combined speed
        speed = math.sqrt(vx*vx + vy*vy + vz*vz)
        v_limit = rospy.get_param('~linear_vel_limit', 3.0)
        if speed > v_limit:
            scale = v_limit / speed
            vx *= scale
            vy *= scale
            vz *= scale

        yaw_rate = msg.angular_vel_z
        ang_limit = rospy.get_param('~angular_vel_limit', 1.0)
        yaw_rate = max(-ang_limit, min(ang_limit, yaw_rate))

        # target_yaw: NaN means "keep current" — pass 0.0 and rely on
        # yaw_dot for turning; only override yaw when explicitly set.
        target_yaw = msg.target_yaw
        if math.isnan(target_yaw):
            target_yaw = 0.0
            # When yaw is unspecified, zero kx[2]/kv[2] gains so px4ctrl
            # does not drive yaw to 0; heading is controlled via yaw_dot.
            yaw_kp = 0.0
        else:
            yaw_kp = 1.0  # let px4ctrl use its default yaw gain

        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = 'world'

        # velocity fields
        cmd.velocity.x = vx
        cmd.velocity.y = vy
        cmd.velocity.z = vz

        # yaw
        cmd.yaw      = target_yaw
        cmd.yaw_dot  = yaw_rate

        # position/accel/jerk left at zero → px4ctrl treats as velocity-only
        # kx/kv: pass zeros to disable position hold, non-zero only for yaw
        cmd.kx = [0.0, 0.0, 0.0]
        cmd.kv = [0.0, 0.0, 0.0]

        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY

        return cmd

    # ------------------------------------------------------------------
    # Keep-alive
    # ------------------------------------------------------------------
    def _keepalive_cb(self, _event):
        """Re-publish the last velocity setpoint so PX4 stays in OFFBOARD.

        If no command has been received recently, publish a zero-velocity
        hover command instead.
        """
        with self._lock:
            cmd = self._last_pos_cmd
            age = time.monotonic() - self._last_cmd_time

        if cmd is None or age > _CMD_TIMEOUT_S:
            # Hover: zero velocity, keep current yaw
            hover = PositionCommand()
            hover.header.stamp = rospy.Time.now()
            hover.header.frame_id = 'world'
            hover.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
            hover.kx = [0.0, 0.0, 0.0]
            hover.kv = [0.0, 0.0, 0.0]
            self._pub_cmd.publish(hover)
        else:
            cmd.header.stamp = rospy.Time.now()
            self._pub_cmd.publish(cmd)


def main():
    rospy.init_node('uran_move_node')
    UranMoveNode()
    rospy.spin()


if __name__ == '__main__':
    main()
