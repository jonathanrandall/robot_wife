#!/usr/bin/env python3
"""
finger_follower — head tracks a raised index fingertip.

The robot PC runs MediaPipe and publishes `person_state_msgs/HandState` on
`/jessica/hand_state`, with the index fingertip position in the camera optical
frame (Z forward, X right, Y down, metres).  The camera is mounted on the
pan-tilt head, so this is a closed visual-servo loop: we back-project the
fingertip to a pixel, measure how far it is from the image centre, and nudge
the pan/tilt servos to drive that error to zero.  Only the head moves.

Enable/disable via `/jessica/finger_follow/enable` (std_msgs/Bool) so the
chatbot (LLM) can turn the mode on/off by voice.  The LLM never runs inside
the control loop — it only flips the flag.

Run:
    ros2 run jessica_robot finger_follower
Needs the hardware stack up (pan_tilt_controller consuming the trajectory) and
the PC publishing /jessica/hand_state.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg
from person_state_msgs.msg import HandState

# ── Camera intrinsics (rectified) — from the PC calibration, see documentation.md
F      = 185.05    # focal length, px
CX     = 170.0     # principal point x, px
CY     = 132.6     # principal point y, px
IMG_CX = 160.0     # EYE_WIDTH  / 2 — keep the fingertip here (visual centre)
IMG_CY = 120.0     # EYE_HEIGHT / 2

# ── Head joint limits (rad) — must match the URDF ros2_control limits.
PAN_MIN,  PAN_MAX  = -1.57, 1.57
TILT_MIN, TILT_MAX = -1.5,  0.87

# ── Rest position (matches the chatbot's HEAD_HOME_*).
HOME_PAN, HOME_TILT = -0.5, 0.1


def _clamp(v, lo, hi):
    return max(lo, min(v, hi))


class FingerFollower(Node):
    def __init__(self):
        super().__init__("finger_follower")

        # ── Tunables (ros2 param set /finger_follower <name> <val>) ──────────
        self.declare_parameter("pan_gain", 0.0015)   # rad of head move per px error
        self.declare_parameter("tilt_gain", 0.0015)
        # Direction signs — flip on the robot if the head chases the wrong way.
        # Head convention: left = +pan, right = -pan; up = +tilt, down = -tilt.
        # Finger to the camera's right  -> pan_error > 0 -> pan right (-pan).
        # Finger below centre           -> tilt_error > 0 -> tilt down (-tilt).
        self.declare_parameter("pan_sign", -1.0)
        self.declare_parameter("tilt_sign", -1.0)
        self.declare_parameter("deadband_px", 8.0)    # ignore tiny errors (no jitter)
        self.declare_parameter("max_step_rad", 0.12)  # per-cycle clamp (~2.4 rad/s @20Hz)
        self.declare_parameter("control_rate", 20.0)  # Hz command output
        self.declare_parameter("lost_timeout", 0.7)   # s w/o fingertip -> hold
        self.declare_parameter("start_enabled", True)

        self.pan_gain     = self.get_parameter("pan_gain").value
        self.tilt_gain    = self.get_parameter("tilt_gain").value
        self.pan_sign     = self.get_parameter("pan_sign").value
        self.tilt_sign    = self.get_parameter("tilt_sign").value
        self.deadband     = self.get_parameter("deadband_px").value
        self.max_step     = self.get_parameter("max_step_rad").value
        rate              = self.get_parameter("control_rate").value
        self.lost_timeout = self.get_parameter("lost_timeout").value
        self.enabled      = self.get_parameter("start_enabled").value

        # ── Internal target (open-loop servos, so we track our own commanded goal).
        self.tgt_pan  = HOME_PAN
        self.tgt_tilt = HOME_TILT

        # ── Latest fingertip pixel error, refreshed by the hand callback.
        self._pan_err  = None
        self._tilt_err = None
        self._last_seen = None   # ros time (float sec) of last valid fingertip

        # ── Publishers / subscribers ─────────────────────────────────────────
        self.head_pub = self.create_publisher(
            JointTrajectory, "/pan_tilt_controller/joint_trajectory", 10)

        # Camera-derived stream: best-effort keep-last-1 so we always act on the
        # freshest frame and stay compatible with a best-effort publisher.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            HandState, "/jessica/hand_state", self.on_hand, sensor_qos)
        self.create_subscription(
            Bool, "/jessica/finger_follow/enable", self.on_enable, 10)

        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self.control_step)
        self._log_ctr = 0

        self.get_logger().info(
            f"finger_follower ready (enabled={self.enabled}). "
            f"Subscribing /jessica/hand_state -> /pan_tilt_controller/joint_trajectory.")

    # ── Callbacks ────────────────────────────────────────────────────────────
    def on_enable(self, msg: Bool):
        if msg.data == self.enabled:
            return
        self.enabled = msg.data
        self.get_logger().info(f"finger following {'ENABLED' if self.enabled else 'DISABLED'}")
        if not self.enabled:
            # Drop stale error so we don't lurch when re-enabled.
            self._pan_err = self._tilt_err = None

    def on_hand(self, msg: HandState):
        """Pick the raised fingertip and cache its pixel error from centre."""
        tip = self._pick_fingertip(msg)
        if tip is None:
            return
        z = tip.position.z
        if z <= 0.05:            # implausible / behind camera — ignore
            return
        x_px = (tip.position.x / z) * F + CX
        y_px = (tip.position.y / z) * F + CY
        self._pan_err  = x_px - IMG_CX   # +ve: fingertip is to the right
        self._tilt_err = y_px - IMG_CY   # +ve: fingertip is below centre
        self._last_seen = self._now()

    def _pick_fingertip(self, msg: HandState):
        """Whichever index tip is detected + depth-valid; if both, the higher one."""
        cands = []
        if msg.right_hand_detected and msg.right_index_tip.depth_valid:
            cands.append(msg.right_index_tip)
        if msg.left_hand_detected and msg.left_index_tip.depth_valid:
            cands.append(msg.left_index_tip)
        if not cands:
            return None
        # "Raised" == higher in the frame == smaller y/z (Y is down-positive).
        return min(cands, key=lambda t: t.position.y / max(t.position.z, 1e-3))

    # ── Control loop ─────────────────────────────────────────────────────────
    def control_step(self):
        if not self.enabled or self._pan_err is None:
            return
        # Lost the finger? Hold position (stop nudging) until it reappears.
        if self._last_seen is None or (self._now() - self._last_seen) > self.lost_timeout:
            return

        pan_err, tilt_err = self._pan_err, self._tilt_err

        d_pan  = 0.0 if abs(pan_err)  < self.deadband else self.pan_sign  * self.pan_gain  * pan_err
        d_tilt = 0.0 if abs(tilt_err) < self.deadband else self.tilt_sign * self.tilt_gain * tilt_err

        d_pan  = _clamp(d_pan,  -self.max_step, self.max_step)
        d_tilt = _clamp(d_tilt, -self.max_step, self.max_step)
        if d_pan == 0.0 and d_tilt == 0.0:
            return   # centred within deadband — nothing to send

        self.tgt_pan  = _clamp(self.tgt_pan  + d_pan,  PAN_MIN,  PAN_MAX)
        self.tgt_tilt = _clamp(self.tgt_tilt + d_tilt, TILT_MIN, TILT_MAX)
        self._publish_head(self.tgt_pan, self.tgt_tilt)

        self._log_ctr += 1
        if self._log_ctr % 20 == 0:   # ~1 Hz
            self.get_logger().info(
                f"err=({pan_err:+.0f},{tilt_err:+.0f})px  "
                f"head=({self.tgt_pan:+.2f},{self.tgt_tilt:+.2f})rad")

    def _publish_head(self, pan, tilt):
        traj = JointTrajectory()
        traj.joint_names = ["pan_joint", "tilt_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [float(pan), float(tilt)]
        # Aim slightly past one control period so the controller always has a
        # fresh future goal to interpolate toward — smooth, continuous motion.
        goal_t = self.dt * 2.5
        pt.time_from_start = DurationMsg(
            sec=int(goal_t), nanosec=int((goal_t % 1.0) * 1e9))
        traj.points.append(pt)
        self.head_pub.publish(traj)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = FingerFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
