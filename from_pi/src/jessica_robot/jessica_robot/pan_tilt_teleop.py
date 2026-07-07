#!/usr/bin/env python3
"""
Joystick teleop for Jessica's pan-tilt head.

Reads /joy and publishes trajectory_msgs/JointTrajectory to the
JointTrajectoryController. Each joystick update nudges the target pan/tilt
angle and sends a single short-duration trajectory point, so the controller
interpolates smoothly between updates.

This shares the head controller with the chatbot's gesture commands — whichever
publishes last wins, so manual control naturally overrides idle gestures.
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PanTiltTeleop(Node):
    def __init__(self):
        super().__init__('pan_tilt_teleop')

        self.declare_parameters(
            namespace='',
            parameters=[
                # Axes for the pad's X-input mode (kernel "Generic X-Box pad"):
                # 0=LX 1=LY 2=LT 3=RX 4=RY 5=RT. Never use 2/5 here — triggers
                # rest at +1.0, which reads as a permanent full-deflection input.
                ('pan_axis', 3),
                ('tilt_axis', 4),
                ('enable_pan', True),
                ('enable_tilt', True),
                ('invert_pan', False),
                ('invert_tilt', False),
                ('deadzone', 0.1),
                ('pan_min', -1.57),
                ('pan_max', 1.57),
                ('tilt_min', -1.5),
                ('tilt_max', 0.87),
                ('pan_speed', 0.02),
                ('tilt_speed', 0.02),
                ('publish_rate', 30.0),
                ('point_time', 0.1),  # time_from_start for each trajectory point (s)
                ('controller_topic', '/pan_tilt_controller/joint_trajectory'),
            ],
        )

        g = lambda n: self.get_parameter(n).value
        self.pan_axis = g('pan_axis')
        self.tilt_axis = g('tilt_axis')
        self.enable_pan = g('enable_pan')
        self.enable_tilt = g('enable_tilt')
        self.invert_pan = g('invert_pan')
        self.invert_tilt = g('invert_tilt')
        self.deadzone = g('deadzone')
        self.pan_min = g('pan_min')
        self.pan_max = g('pan_max')
        self.tilt_min = g('tilt_min')
        self.tilt_max = g('tilt_max')
        self.pan_speed = g('pan_speed')
        self.tilt_speed = g('tilt_speed')
        self.point_time = g('point_time')
        self.publish_period = 1.0 / g('publish_rate')

        self.pan = 0.0
        self.tilt = 0.0
        self.last_pub_time = 0.0

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(JointTrajectory, g('controller_topic'), 10)

        self.get_logger().info('Pan/Tilt joystick teleop started (JointTrajectory).')

    def joy_callback(self, msg: Joy):
        now = time.time()
        if now - self.last_pub_time < self.publish_period:
            return

        moved = False

        if self.enable_pan and self.pan_axis < len(msg.axes):
            v = msg.axes[self.pan_axis]
            if abs(v) > self.deadzone:
                v = -v if self.invert_pan else v
                self.pan = max(self.pan_min, min(self.pan + v * self.pan_speed, self.pan_max))
                moved = True

        if self.enable_tilt and self.tilt_axis < len(msg.axes):
            v = msg.axes[self.tilt_axis]
            if abs(v) > self.deadzone:
                v = -v if self.invert_tilt else v
                self.tilt = max(self.tilt_min, min(self.tilt + v * self.tilt_speed, self.tilt_max))
                moved = True

        if not moved:
            return

        traj = JointTrajectory()
        traj.joint_names = ['pan_joint', 'tilt_joint']
        point = JointTrajectoryPoint()
        point.positions = [self.pan, self.tilt]
        sec = int(self.point_time)
        point.time_from_start = Duration(sec=sec, nanosec=int((self.point_time - sec) * 1e9))
        traj.points = [point]
        self.cmd_pub.publish(traj)
        self.last_pub_time = now


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
