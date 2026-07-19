#!/usr/bin/env python3
"""
person_follower — the base and head follow Jonny, using his shoulders.

The robot PC runs MediaPipe on the stereo camera and publishes
`person_state_msgs/PersonState` on `/jessica/person_state`, whose
`shoulder_midpoint` (camera optical frame: Z forward, X right, Y down, metres)
is the follow reference.  The camera rides on the pan-tilt head, so:

- the HEAD runs the same visual-servo P-loop as finger_follower, keeping the
  shoulder midpoint centred in the image (fast tracking), and
- the BASE steers toward the person's bearing (= current pan angle plus the
  small in-image offset) and P-controls distance to hold `target_dist`
  behind them.  It never reverses toward the person and never advances
  inside `min_dist`.

Auto-stop (disables following + halts the base):
- person turns around  — shoulders are anatomically labelled, so from behind
  the left shoulder appears camera-left; when the x-order flips (debounced),
  they are facing the robot;
- raised open palm     — from `/jessica/hand_state` landmarks: fingers
  extended and pointing up (tips above wrist; optical Y is DOWN), debounced.
  Only evaluated while following is enabled, so waving in normal conversation
  can never trigger anything;
- (person merely lost just stops the base and waits — following stays armed.)

Enable/disable via `/jessica/person_follow/enable` (std_msgs/Bool) so the
chatbot can voice-toggle it ("Jessica darling, follow me").  The LLM is never
in the control loop.  Everything here is timer/pub-sub driven — non-blocking.

Run:
    ros2 run jessica_robot person_follower
Needs the hardware stack up and the PC publishing person/hand state.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg
from rcl_interfaces.msg import SetParametersResult
from person_state_msgs.msg import PersonState, HandState

# ── Camera intrinsics (rectified) — from the PC calibration, see robot_docs/documentation.md
F      = 185.05    # focal length, px
CX     = 170.0     # principal point x, px
CY     = 132.6     # principal point y, px

# ── Head joint limits (rad) — must match the URDF ros2_control limits.
PAN_MIN,  PAN_MAX  = -1.57, 1.57
TILT_MIN, TILT_MAX = -1.5,  0.87

# Hand landmark indices (see HandState.msg).
WRIST = 0
FINGERS = ((8, 6), (12, 10), (16, 14))   # (tip, pip) for index/middle/ring


def _clamp(v, lo, hi):
    return max(lo, min(v, hi))


class PersonFollower(Node):
    def __init__(self):
        super().__init__("person_follower")

        # ── Head servo loop — same tuning as finger_follower (2026-07-05).
        self.declare_parameter("pan_gain", 0.00035)
        self.declare_parameter("tilt_gain", 0.00035)
        self.declare_parameter("pan_sign", -1.0)
        self.declare_parameter("tilt_sign", -1.0)
        self.declare_parameter("deadband_px", 8.0)
        self.declare_parameter("max_step_rad", 0.06)
        self.declare_parameter("err_smooth", 0.4)
        self.declare_parameter("max_err_px", 165.0)
        # Shoulders sit mid-frame (unlike the raised-finger target).
        self.declare_parameter("target_x_px", 160.0)
        self.declare_parameter("target_y_px", 120.0)

        # ── Base following.
        self.declare_parameter("target_dist", 0.70)   # m to hold behind Jonny
        self.declare_parameter("dist_deadband", 0.10) # m — no hunting inside this
        self.declare_parameter("min_dist", 0.50)      # m — never advance inside
        self.declare_parameter("k_lin", 0.8)          # m/s per m of distance error
        self.declare_parameter("max_lin", 0.35)       # m/s cap
        self.declare_parameter("k_ang", 0.7)          # rad/s per rad of bearing
        self.declare_parameter("max_ang", 0.9)        # rad/s cap
        self.declare_parameter("bearing_deadband", 0.05)  # rad

        # ── Timings / auto-stop.
        self.declare_parameter("control_rate", 15.0)
        self.declare_parameter("lost_timeout", 1.0)    # s w/o person -> base stops
        self.declare_parameter("turnaround_s", 0.6)    # facing-robot debounce
        self.declare_parameter("turnaround_margin", 0.05)  # m of shoulder x-swap
        self.declare_parameter("palm_hold_s", 0.4)     # raised-palm debounce
        self.declare_parameter("palm_up_margin", 0.05)  # m fingertip above wrist
        self.declare_parameter("palm_ext_margin", 0.02) # m fingertip above pip
        # Default OFF — only the chatbot (voice) or an explicit Bool enables it.
        self.declare_parameter("start_enabled", False)

        g = lambda n: self.get_parameter(n).value
        self.pan_gain, self.tilt_gain = g("pan_gain"), g("tilt_gain")
        self.pan_sign, self.tilt_sign = g("pan_sign"), g("tilt_sign")
        self.deadband, self.max_step  = g("deadband_px"), g("max_step_rad")
        self.err_smooth, self.max_err_px = g("err_smooth"), g("max_err_px")
        self.target_x, self.target_y = g("target_x_px"), g("target_y_px")
        self.target_dist, self.dist_deadband = g("target_dist"), g("dist_deadband")
        self.min_dist = g("min_dist")
        self.k_lin, self.max_lin = g("k_lin"), g("max_lin")
        self.k_ang, self.max_ang = g("k_ang"), g("max_ang")
        self.bearing_deadband = g("bearing_deadband")
        self.lost_timeout = g("lost_timeout")
        self.turnaround_s, self.turnaround_margin = g("turnaround_s"), g("turnaround_margin")
        self.palm_hold_s = g("palm_hold_s")
        self.palm_up_margin, self.palm_ext_margin = g("palm_up_margin"), g("palm_ext_margin")
        self.enabled = g("start_enabled")

        # ── State.
        self.tgt_pan = 0.0
        self.tgt_tilt = 0.0
        self._joint_pan = None    # actual head pose from /joint_states
        self._joint_tilt = None
        self._pan_err = None      # smoothed pixel errors (head loop)
        self._tilt_err = None
        self._dist = None         # latest slant range to shoulder midpoint (m)
        self._bearing = None      # latest person bearing in base frame (rad, +left)
        self._last_seen = None    # time of last valid shoulder midpoint
        self._facing_since = None # time the shoulder x-order first looked flipped
        self._palm_since = None   # time the raised palm was first seen
        self._was_moving = False  # so we send a single zero Twist on loss/stop

        # ── Publishers / subscribers.
        self.head_pub = self.create_publisher(
            JointTrajectory, "/pan_tilt_controller/joint_trajectory", 10)
        # Autonomous channel: twist_mux gives the joystick priority over us.
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PersonState, "/jessica/person_state", self.on_person, sensor_qos)
        self.create_subscription(
            HandState, "/jessica/hand_state", self.on_hand, sensor_qos)
        self.create_subscription(
            Bool, "/jessica/person_follow/enable", self.on_enable, 10)
        # General stop (both-arms-raised gesture / stop_gesture node).
        self.create_subscription(Empty, "/jessica/stop", self.on_stop, 10)
        self.create_subscription(
            JointState, "/joint_states", self.on_joint_states, 10)

        self.dt = 1.0 / g("control_rate")
        self.timer = self.create_timer(self.dt, self.control_step)
        self._log_ctr = 0

        self.add_on_set_parameters_callback(self._on_set_params)
        self.get_logger().info(
            f"person_follower ready (enabled={self.enabled}). "
            "Subscribing /jessica/person_state -> head + /cmd_vel.")

    def _on_set_params(self, params):
        attr = {
            "pan_gain": "pan_gain", "tilt_gain": "tilt_gain",
            "pan_sign": "pan_sign", "tilt_sign": "tilt_sign",
            "deadband_px": "deadband", "max_step_rad": "max_step",
            "err_smooth": "err_smooth", "max_err_px": "max_err_px",
            "target_x_px": "target_x", "target_y_px": "target_y",
            "target_dist": "target_dist", "dist_deadband": "dist_deadband",
            "min_dist": "min_dist", "k_lin": "k_lin", "max_lin": "max_lin",
            "k_ang": "k_ang", "max_ang": "max_ang",
            "bearing_deadband": "bearing_deadband",
            "lost_timeout": "lost_timeout", "turnaround_s": "turnaround_s",
            "turnaround_margin": "turnaround_margin",
            "palm_hold_s": "palm_hold_s", "palm_up_margin": "palm_up_margin",
            "palm_ext_margin": "palm_ext_margin",
        }
        for p in params:
            if p.name in attr:
                setattr(self, attr[p.name], p.value)
        return SetParametersResult(successful=True)

    # ── Callbacks ────────────────────────────────────────────────────────────
    def on_joint_states(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name == "pan_joint":
                self._joint_pan = pos
            elif name == "tilt_joint":
                self._joint_tilt = pos

    def on_enable(self, msg: Bool):
        if msg.data == self.enabled:
            return
        self.enabled = msg.data
        self.get_logger().info(
            f"person following {'ENABLED' if self.enabled else 'DISABLED'}")
        self._pan_err = self._tilt_err = None
        self._facing_since = self._palm_since = None
        if self.enabled:
            # Start from the head's ACTUAL pose (single source of truth:
            # /joint_states) so enabling never jerks the head.
            if self._joint_pan is not None and self._joint_tilt is not None:
                self.tgt_pan, self.tgt_tilt = self._joint_pan, self._joint_tilt
            else:
                self.tgt_pan = self.tgt_tilt = 0.0
                self.get_logger().warn("no /joint_states yet — assuming head at 0,0")
        else:
            self._stop_base()

    def on_stop(self, _msg):
        if self.enabled:
            self._auto_stop("general stop gesture")

    def on_person(self, msg: PersonState):
        if not msg.person_visible:
            return

        # ── Turn-around detection (works even on frames w/o a valid midpoint).
        ls, rs = msg.left_shoulder, msg.right_shoulder
        if self.enabled and ls.depth_valid and rs.depth_valid:
            facing = (ls.position.x - rs.position.x) > self.turnaround_margin
            if facing:
                if self._facing_since is None:
                    self._facing_since = self._now()
                elif (self._now() - self._facing_since) > self.turnaround_s:
                    self._auto_stop("you turned around")
                    return
            else:
                self._facing_since = None

        if not msg.shoulder_midpoint_valid:
            return
        p = msg.shoulder_midpoint
        if p.z <= 0.05:
            return

        # ── Head visual-servo error (pixels from target point).
        x_px = (p.x / p.z) * F + CX
        y_px = (p.y / p.z) * F + CY
        pan_err  = x_px - self.target_x
        tilt_err = y_px - self.target_y
        if abs(pan_err) > self.max_err_px or abs(tilt_err) > self.max_err_px:
            return
        if self._pan_err is None:
            self._pan_err, self._tilt_err = pan_err, tilt_err
        else:
            a = self.err_smooth
            self._pan_err  = a * pan_err  + (1.0 - a) * self._pan_err
            self._tilt_err = a * tilt_err + (1.0 - a) * self._tilt_err

        # ── Base references: slant range + bearing in the base frame.
        # Camera X is right; pan + is left — so the person's bearing is the
        # current pan angle minus the in-image angular offset.
        self._dist = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
        pan_now = self._joint_pan if self._joint_pan is not None else self.tgt_pan
        self._bearing = pan_now - math.atan2(p.x, p.z)
        self._last_seen = self._now()

    def on_hand(self, msg: HandState):
        """Raised-open-palm auto-stop. Only evaluated while following — a wave
        during normal conversation can never reach this state."""
        if not self.enabled:
            self._palm_since = None
            return
        palm = (self._is_open_palm_up(msg.left_hand_detected, msg.left_hand_landmarks)
                or self._is_open_palm_up(msg.right_hand_detected, msg.right_hand_landmarks))
        if palm:
            if self._palm_since is None:
                self._palm_since = self._now()
            elif (self._now() - self._palm_since) > self.palm_hold_s:
                self._auto_stop("raised palm")
        else:
            self._palm_since = None

    def _is_open_palm_up(self, detected: bool, lm) -> bool:
        """Fingers extended and pointing up: tip above pip and well above the
        wrist (optical Y is DOWN, so 'above' = smaller y). Needs 2 of the
        index/middle/ring fingers — stereo depth on fingers can be spotty."""
        if not detected:
            return False
        wrist = lm[WRIST]
        if not wrist.depth_valid:
            return False
        n_up = 0
        for tip_i, pip_i in FINGERS:
            tip, pip = lm[tip_i], lm[pip_i]
            if not (tip.depth_valid and pip.depth_valid):
                continue
            if (tip.position.y < pip.position.y - self.palm_ext_margin
                    and tip.position.y < wrist.position.y - self.palm_up_margin):
                n_up += 1
        return n_up >= 2

    # ── Control loop ─────────────────────────────────────────────────────────
    def control_step(self):
        if not self.enabled:
            return
        # Person lost: stop the base, hold the head, stay armed.
        if self._last_seen is None or (self._now() - self._last_seen) > self.lost_timeout:
            if self._was_moving:
                self._stop_base()
                self.get_logger().info("person lost — base stopped, waiting")
            return

        # ── Head: nudge toward the shoulder midpoint.
        if self._pan_err is not None:
            d_pan  = 0.0 if abs(self._pan_err)  < self.deadband else \
                self.pan_sign  * self.pan_gain  * self._pan_err
            d_tilt = 0.0 if abs(self._tilt_err) < self.deadband else \
                self.tilt_sign * self.tilt_gain * self._tilt_err
            d_pan  = _clamp(d_pan,  -self.max_step, self.max_step)
            d_tilt = _clamp(d_tilt, -self.max_step, self.max_step)
            if d_pan or d_tilt:
                self.tgt_pan  = _clamp(self.tgt_pan  + d_pan,  PAN_MIN,  PAN_MAX)
                self.tgt_tilt = _clamp(self.tgt_tilt + d_tilt, TILT_MIN, TILT_MAX)
                self._publish_head(self.tgt_pan, self.tgt_tilt)

        # ── Base: steer to the person's bearing, hold target distance.
        cmd = Twist()
        if self._dist is not None and self._dist >= self.min_dist:
            dist_err = self._dist - self.target_dist
            if dist_err > self.dist_deadband:
                # Forward only — never reverse toward the person.
                cmd.linear.x = _clamp(self.k_lin * dist_err, 0.0, self.max_lin)
            if abs(self._bearing) > self.bearing_deadband:
                cmd.angular.z = _clamp(self.k_ang * self._bearing,
                                       -self.max_ang, self.max_ang)
        self.cmd_pub.publish(cmd)
        self._was_moving = bool(cmd.linear.x or cmd.angular.z)

        self._log_ctr += 1
        if self._log_ctr % 15 == 0:   # ~1 Hz
            self.get_logger().info(
                f"dist={self._dist:.2f}m bearing={math.degrees(self._bearing):+.0f}° "
                f"cmd=({cmd.linear.x:.2f},{cmd.angular.z:+.2f}) "
                f"head=({self.tgt_pan:+.2f},{self.tgt_tilt:+.2f})")

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _auto_stop(self, reason: str):
        self.get_logger().info(f"auto-stop: {reason} — following DISABLED")
        self.enabled = False
        self._facing_since = self._palm_since = None
        self._stop_base()

    def _stop_base(self):
        self.cmd_pub.publish(Twist())
        self._was_moving = False

    def _publish_head(self, pan, tilt):
        traj = JointTrajectory()
        traj.joint_names = ["pan_joint", "tilt_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [float(pan), float(tilt)]
        goal_t = self.dt * 2.5
        pt.time_from_start = DurationMsg(
            sec=int(goal_t), nanosec=int((goal_t % 1.0) * 1e9))
        traj.points.append(pt)
        self.head_pub.publish(traj)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_base()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
