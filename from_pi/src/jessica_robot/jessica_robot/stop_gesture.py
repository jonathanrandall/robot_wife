#!/usr/bin/env python3
"""
stop_gesture — general "STOP EVERYTHING" gesture: both arms raised.

Watches `/jessica/person_state` for both wrists held a margin above their own
shoulders (camera optical frame, Y is DOWN so "above" = smaller y). When the
pose is held for `hold_s`, publishes one `std_msgs/Empty` on `/jessica/stop`.

Consumers (each stops itself — no single point of failure):
- jessica_chatbot   → cancels any timed move/twirl/dance, zeroes the base,
                      switches finger + person following off
- finger_follower   → disables itself
- person_follower   → disables itself + zeroes the base

Edge-triggered: fires once per raise (re-arms when the arms come down, or
after `refire_s` if they stay up, in case a consumer missed the first one).

NOTE: landmarks in PersonState are only valid when BOTH stereo eyes saw them
(the PC zeroes single-eye landmarks), so this needs wrists + shoulders visible
to both cameras. Detection from a single eye would need a small PC-side
addition (2-D per-eye check published as a bool).

Nothing here blocks: one best-effort subscription, pure arithmetic in the
callback, one publish. The chatbot handles /jessica/stop on its ROS spin
thread, separate from the audio loop — no waiting on record/LLM/TTS.

Run:
    ros2 run jessica_robot stop_gesture
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Empty
from person_state_msgs.msg import PersonState


class StopGesture(Node):
    def __init__(self):
        super().__init__("stop_gesture")

        # Wrist must be this many metres above its shoulder (optical frame).
        self.declare_parameter("wrist_margin", 0.08)
        # Gesture must be held this long before firing (debounce).
        self.declare_parameter("hold_s", 0.4)
        # If the arms stay up, allow another fire after this long.
        self.declare_parameter("refire_s", 2.0)

        self.wrist_margin = self.get_parameter("wrist_margin").value
        self.hold_s       = self.get_parameter("hold_s").value
        self.refire_s     = self.get_parameter("refire_s").value

        self.stop_pub = self.create_publisher(Empty, "/jessica/stop", 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PersonState, "/jessica/person_state", self.on_person, sensor_qos)

        self._raised_since = None   # when both-arms-up was first seen
        self._fired_at = None       # when we last published a stop

        self.get_logger().info(
            "stop_gesture ready — both wrists above shoulders => /jessica/stop")

    def on_person(self, msg: PersonState):
        if not msg.person_visible:
            self._raised_since = None
            return

        if not self._both_arms_raised(msg):
            self._raised_since = None
            self._fired_at = None   # arms down — re-arm for the next raise
            return

        now = self._now()
        if self._raised_since is None:
            self._raised_since = now
            return
        if (now - self._raised_since) < self.hold_s:
            return
        # Held long enough. Fire once, then hold off until re-armed/refire_s.
        if self._fired_at is not None and (now - self._fired_at) < self.refire_s:
            return
        self._fired_at = now
        self.get_logger().info("BOTH ARMS RAISED — publishing /jessica/stop")
        self.stop_pub.publish(Empty())

    def _both_arms_raised(self, msg: PersonState) -> bool:
        """Each wrist above its OWN shoulder by wrist_margin. All four
        landmarks must be stereo-valid (zeroed otherwise — see module note)."""
        pairs = ((msg.left_wrist, msg.left_shoulder),
                 (msg.right_wrist, msg.right_shoulder))
        for wrist, shoulder in pairs:
            if not (wrist.depth_valid and shoulder.depth_valid):
                return False
            if not (wrist.position.y < shoulder.position.y - self.wrist_margin):
                return False
        return True

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = StopGesture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
