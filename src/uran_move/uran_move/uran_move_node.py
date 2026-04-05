"""uran_move_node — bridges UnifiedMoveCmd to px4ctrl interfaces.

Supported action mappings (px4ctrl, drone only):
  action="takeoff"         -> publishes TakeoffLand(TAKEOFF) to px4ctrl
  action="land"            -> pauses PositionCommand, then publishes TakeoffLand(LAND)
  action="emergency_stop"  -> pauses PositionCommand, then publishes TakeoffLand(LAND)
  action="stop" / all-zero -> holds current setpoint in CMD_CTRL
  velocity command         -> integrates a world-frame PositionCommand setpoint

UnifiedMoveCmd linear velocities are body-frame commands (x forward / y left /
z up). px4ctrl, however, consumes a full world-frame position + velocity target.
This node therefore rotates body-frame commands into the world frame using the
latest odom yaw and integrates them into a continuous setpoint stream.
"""
import math
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from uran_msgs.msg import UnifiedMoveCmd
from quadrotor_msgs.msg import PositionCommand, TakeoffLand
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import RCIn

# px4ctrl topics (from px4ctrl_node.cpp)
_TOPIC_CMD       = '/position_cmd'   # quadrotor_msgs/PositionCommand
_TOPIC_TKL       = '/px4ctrl/takeoff_land'   # quadrotor_msgs/TakeoffLand
_DEFAULT_ODOM_TOPIC = '/robot/dlio/odom_node/odom'

# URAN downlink topic
_TOPIC_MOVE_CMD  = '/uran/core/downlink/move_cmd'

# RC pre-takeoff checks
_CH_THROTTLE     = 2   # CH3 (0-indexed)
_CH5             = 4
_CH6             = 5
_THROTTLE_LO     = 1400
_THROTTLE_HI     = 1600
_HIGH_LO         = 1900
_HIGH_HI         = 2100

# Keep-alive: PX4 OFFBOARD requires setpoints at >=2 Hz.
# Keep-alive: PX4 OFFBOARD requires setpoints at >=2 Hz.
_KEEPALIVE_HZ    = 20
_CMD_TIMEOUT_S   = 0.5  # seconds before keep-alive reverts to hover
_LAND_RETRY_DELAY_S = 0.6


class UranMoveNode:
    def __init__(self):
        self._lock = threading.Lock()
        self._sp_pos: list[float] | None = None
        self._sp_yaw: float = 0.0
        self._sp_vel = (0.0, 0.0, 0.0)
        self._sp_yaw_rate: float = 0.0
        self._setpoint_time: float | None = None
        self._last_motion_req = self._zero_motion_req()
        self._last_cmd_time: float = 0.0
        self._odom_pos: list[float] | None = None
        self._odom_yaw: float | None = None

        # px4ctrl readiness tracking
        self._px4ctrl_ready = False  # True after receiving traj_start_trigger
        self._takeoff_pending = False
        self._land_retry_delay = rospy.get_param('~land_retry_delay_s', _LAND_RETRY_DELAY_S)
        self._land_retry_seq = 0
        self._land_retry_timer: rospy.Timer | None = None

        self._pub_cmd = rospy.Publisher(_TOPIC_CMD, PositionCommand, queue_size=10)
        self._pub_tkl = rospy.Publisher(_TOPIC_TKL, TakeoffLand, queue_size=5)

        self._rc_lock = threading.Lock()
        self._last_rc: RCIn | None = None
        rospy.Subscriber('/mavros/rc/in', RCIn, self._on_rc_in, queue_size=5)
        odom_topic = rospy.get_param('~odom_topic', _DEFAULT_ODOM_TOPIC)
        rospy.Subscriber(odom_topic, Odometry, self._on_odom,
                         queue_size=20, tcp_nodelay=True)

        # Subscribe to traj_start_trigger — px4ctrl publishes when entering CMD_CTRL
        rospy.Subscriber('/traj_start_trigger', PoseStamped, self._on_traj_trigger, queue_size=5)

        rospy.Subscriber(_TOPIC_MOVE_CMD, UnifiedMoveCmd, self._on_move_cmd,
                         queue_size=10, tcp_nodelay=True)

        # Keep-alive timer — republishes last velocity setpoint so px4ctrl
        # stays in CMD_CTRL and PX4 stays in OFFBOARD.
        self._keepalive_timer = rospy.Timer(
            rospy.Duration(1.0 / _KEEPALIVE_HZ), self._keepalive_cb
        )

        rospy.loginfo('[uran_move] node started, listening on %s, odom=%s',
                      _TOPIC_MOVE_CMD, odom_topic)

    # ------------------------------------------------------------------
    # Subscriber callback
    # ------------------------------------------------------------------
    def _on_move_cmd(self, msg: UnifiedMoveCmd):
        action = msg.action.strip().lower()

        if action in ('takeoff',):
            self._cancel_pending_land()
            if not self._rc_preflight_ok():
                return
            self._px4ctrl_ready = False  # reset until traj_start_trigger received
            self._takeoff_pending = True
            with self._lock:
                self._reset_motion_state_locked()
            self._send_takeoff_land(TakeoffLand.TAKEOFF)
            rospy.loginfo('[uran_move] takeoff command sent, waiting for traj_start_trigger...')
            return

        if action in ('land', 'return_home'):
            self._begin_delayed_land(action)
            return

        if action == 'emergency_stop':
            rospy.logwarn('[uran_move] emergency_stop → triggering LAND')
            self._begin_delayed_land(action)
            return

        # Block velocity commands until px4ctrl is ready (after takeoff)
        if not self._px4ctrl_ready:
            rospy.logwarn('[uran_move] velocity cmd blocked: px4ctrl not ready (send takeoff first)')
            return

        with self._lock:
            now = time.monotonic()
            if not self._ensure_setpoint_locked(now):
                rospy.logwarn_throttle(1.0, '[uran_move] velocity cmd blocked: setpoint not initialized yet')
                return
            self._advance_setpoint_locked(now)
            self._last_motion_req = self._motion_req_from_msg(msg, action)
            self._last_cmd_time = now
            self._advance_setpoint_locked(now)
            cmd = self._build_position_cmd_locked()
        self._pub_cmd.publish(cmd)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _on_rc_in(self, msg: RCIn):
        with self._rc_lock:
            self._last_rc = msg

    def _on_odom(self, msg: Odometry):
        yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        with self._lock:
            self._odom_pos = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
            self._odom_yaw = yaw

    def _on_traj_trigger(self, msg: PoseStamped):
        """px4ctrl publishes this when entering CMD_CTRL (ready for commands)."""
        yaw = self._yaw_from_quaternion(msg.pose.orientation)
        with self._lock:
            self._sp_pos = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ]
            self._sp_yaw = yaw
            self._sp_vel = (0.0, 0.0, 0.0)
            self._sp_yaw_rate = 0.0
            self._setpoint_time = time.monotonic()
            self._last_motion_req = self._zero_motion_req()
            self._last_cmd_time = 0.0
            cmd = self._build_position_cmd_locked()
        if self._takeoff_pending:
            rospy.loginfo('[uran_move] traj_start_trigger received, px4ctrl ready for velocity commands')
            self._takeoff_pending = False
        self._px4ctrl_ready = True
        self._pub_cmd.publish(cmd)

    def _rc_preflight_ok(self) -> bool:
        with self._rc_lock:
            rc = self._last_rc
        if rc is None:
            rospy.logwarn('[uran_move] takeoff blocked: no RC data received')
            return False
        ch = rc.channels
        if len(ch) <= max(_CH_THROTTLE, _CH5, _CH6):
            rospy.logwarn('[uran_move] takeoff blocked: RC channel count %d too low', len(ch))
            return False
        thr = ch[_CH_THROTTLE]
        ch5 = ch[_CH5]
        ch6 = ch[_CH6]
        if not (_THROTTLE_LO <= thr <= _THROTTLE_HI):
            rospy.logwarn('[uran_move] takeoff blocked: CH3 throttle=%d not in [%d,%d]',
                          thr, _THROTTLE_LO, _THROTTLE_HI)
            return False
        if not (_HIGH_LO <= ch5 <= _HIGH_HI):
            rospy.logwarn('[uran_move] takeoff blocked: CH5=%d not in [%d,%d]',
                          ch5, _HIGH_LO, _HIGH_HI)
            return False
        if not (_HIGH_LO <= ch6 <= _HIGH_HI):
            rospy.logwarn('[uran_move] takeoff blocked: CH6=%d not in [%d,%d]',
                          ch6, _HIGH_LO, _HIGH_HI)
            return False
        return True

    def _send_takeoff_land(self, cmd_type: int):
        msg = TakeoffLand()
        msg.takeoff_land_cmd = cmd_type
        self._pub_tkl.publish(msg)
        label = 'TAKEOFF' if cmd_type == TakeoffLand.TAKEOFF else 'LAND'
        rospy.loginfo('[uran_move] published %s to px4ctrl', label)

    def _begin_delayed_land(self, action: str):
        self._px4ctrl_ready = False
        self._takeoff_pending = False
        timer_to_cancel = None
        with self._lock:
            self._reset_motion_state_locked()
            self._land_retry_seq += 1
            seq = self._land_retry_seq
            timer_to_cancel = self._land_retry_timer
            self._land_retry_timer = rospy.Timer(
                rospy.Duration(self._land_retry_delay),
                lambda event, seq=seq: self._delayed_land_cb(event, seq),
                oneshot=True,
            )
        if timer_to_cancel is not None:
            timer_to_cancel.shutdown()
        rospy.loginfo(
            '[uran_move] %s requested, pausing PositionCommand for %.2fs before LAND',
            action,
            self._land_retry_delay,
        )

    def _delayed_land_cb(self, _event, seq: int):
        with self._lock:
            if seq != self._land_retry_seq:
                return
            self._land_retry_timer = None
        self._send_takeoff_land(TakeoffLand.LAND)
        rospy.loginfo(
            '[uran_move] delayed LAND sent after %.2fs PositionCommand quiet period',
            self._land_retry_delay,
        )

    def _cancel_pending_land(self):
        timer_to_cancel = None
        with self._lock:
            if self._land_retry_timer is None:
                return
            self._land_retry_seq += 1
            timer_to_cancel = self._land_retry_timer
            self._land_retry_timer = None
        timer_to_cancel.shutdown()
        rospy.loginfo('[uran_move] canceled pending delayed LAND')

    def _motion_req_from_msg(self, msg: UnifiedMoveCmd, action: str) -> dict:
        """Normalize a UnifiedMoveCmd into the internal motion request format."""
        vx = msg.linear_vel_x
        vy = msg.linear_vel_y
        vz = msg.linear_vel_z

        # Proportional clamp on combined speed
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        v_limit = rospy.get_param('~linear_vel_limit', 3.0)
        if speed > v_limit:
            scale = v_limit / speed
            vx *= scale
            vy *= scale
            vz *= scale

        yaw_rate = msg.angular_vel_z
        ang_limit = rospy.get_param('~angular_vel_limit', 1.0)
        yaw_rate = max(-ang_limit, min(ang_limit, yaw_rate))

        target_yaw = msg.target_yaw
        if not math.isnan(target_yaw):
            target_yaw = self._normalize_angle(target_yaw)

        if action == 'stop':
            return self._zero_motion_req()

        return {
            'body_vx': vx,
            'body_vy': vy,
            'vz': vz,
            'yaw_rate': yaw_rate,
            'target_yaw': target_yaw,
        }

    def _zero_motion_req(self) -> dict:
        return {
            'body_vx': 0.0,
            'body_vy': 0.0,
            'vz': 0.0,
            'yaw_rate': 0.0,
            'target_yaw': math.nan,
        }

    def _reset_motion_state_locked(self):
        self._sp_pos = None
        self._sp_yaw = 0.0
        self._sp_vel = (0.0, 0.0, 0.0)
        self._sp_yaw_rate = 0.0
        self._setpoint_time = None
        self._last_motion_req = self._zero_motion_req()
        self._last_cmd_time = 0.0

    def _ensure_setpoint_locked(self, now: float) -> bool:
        if self._sp_pos is not None:
            return True
        if self._odom_pos is None:
            return False
        self._sp_pos = list(self._odom_pos)
        self._sp_yaw = self._odom_yaw if self._odom_yaw is not None else 0.0
        self._sp_vel = (0.0, 0.0, 0.0)
        self._sp_yaw_rate = 0.0
        self._setpoint_time = now
        return True

    def _advance_setpoint_locked(self, now: float):
        if self._sp_pos is None:
            return

        if self._setpoint_time is None:
            self._setpoint_time = now

        req = self._last_motion_req
        dt = max(0.0, now - self._setpoint_time)

        heading = self._odom_yaw if self._odom_yaw is not None else self._sp_yaw
        cos_yaw = math.cos(heading)
        sin_yaw = math.sin(heading)
        vx_world = cos_yaw * req['body_vx'] - sin_yaw * req['body_vy']
        vy_world = sin_yaw * req['body_vx'] + cos_yaw * req['body_vy']
        vz_world = req['vz']

        self._sp_pos[0] += vx_world * dt
        self._sp_pos[1] += vy_world * dt
        self._sp_pos[2] += vz_world * dt

        if math.isfinite(req['target_yaw']):
            self._sp_yaw = req['target_yaw']
            self._sp_yaw_rate = 0.0
        else:
            self._sp_yaw = self._normalize_angle(self._sp_yaw + req['yaw_rate'] * dt)
            self._sp_yaw_rate = req['yaw_rate']

        self._sp_vel = (vx_world, vy_world, vz_world)
        self._setpoint_time = now

    def _build_position_cmd_locked(self) -> PositionCommand:
        if self._sp_pos is None:
            raise RuntimeError('setpoint not initialized')

        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = 'world'

        cmd.position.x = self._sp_pos[0]
        cmd.position.y = self._sp_pos[1]
        cmd.position.z = self._sp_pos[2]

        cmd.velocity.x = self._sp_vel[0]
        cmd.velocity.y = self._sp_vel[1]
        cmd.velocity.z = self._sp_vel[2]

        cmd.acceleration.x = 0.0
        cmd.acceleration.y = 0.0
        cmd.acceleration.z = 0.0
        cmd.jerk.x = 0.0
        cmd.jerk.y = 0.0
        cmd.jerk.z = 0.0

        cmd.yaw = self._sp_yaw
        cmd.yaw_dot = self._sp_yaw_rate

        cmd.kx = [0.0, 0.0, 0.0]
        cmd.kv = [0.0, 0.0, 0.0]
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY

        return cmd

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    # ------------------------------------------------------------------
    # Keep-alive
    # ------------------------------------------------------------------
    def _keepalive_cb(self, _event):
        """Re-publish the current setpoint so PX4 stays in OFFBOARD."""
        # Only publish keepalive when px4ctrl is ready for commands
        if not self._px4ctrl_ready:
            return

        with self._lock:
            now = time.monotonic()
            if not self._ensure_setpoint_locked(now):
                rospy.logwarn_throttle(1.0, '[uran_move] keepalive skipped: no odom/setpoint available')
                return

            if (self._last_cmd_time > 0.0 and
                    now - self._last_cmd_time > _CMD_TIMEOUT_S and
                    not self._is_zero_motion_locked()):
                cutoff = self._last_cmd_time + _CMD_TIMEOUT_S
                if self._setpoint_time is None or cutoff > self._setpoint_time:
                    self._advance_setpoint_locked(cutoff)
                self._last_motion_req = self._zero_motion_req()

            self._advance_setpoint_locked(now)
            cmd = self._build_position_cmd_locked()

        self._pub_cmd.publish(cmd)

    def _is_zero_motion_locked(self) -> bool:
        req = self._last_motion_req
        return (
            abs(req['body_vx']) < 1e-6 and
            abs(req['body_vy']) < 1e-6 and
            abs(req['vz']) < 1e-6 and
            abs(req['yaw_rate']) < 1e-6 and
            not math.isfinite(req['target_yaw'])
        )


def main():
    rospy.init_node('uran_move_node')
    UranMoveNode()
    rospy.spin()


if __name__ == '__main__':
    main()
