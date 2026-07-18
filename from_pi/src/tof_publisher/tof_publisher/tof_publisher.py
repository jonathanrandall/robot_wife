#!/usr/bin/env python3

import sys
sys.path.insert(0, '/home/jonny/venvs/jazzy/lib/python3.12/site-packages')

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import ArducamDepthCamera as ac


TOPIC      = "/jessica/tof/image/compressed"
PUBLISH_HZ = 10

# Grayscale mapping: 255 = touching the lens, 0 = at/beyond max range.
#   gray = (MAX_DISTANCE_MM - depth) * 255 / MAX_DISTANCE_MM
# Invalid pixels (no return / low confidence) are forced to 0 (far), not 255 —
# otherwise a dropout would look like an obstacle on the lens.
MAX_DISTANCE_MM = 2000  # supported hardware ranges are 2000 or 4000

# 0 disables confidence filtering (same default as the working preview example;
# try 10-30 later if the image is noisy).
CONFIDENCE_THRESHOLD = 0

FRAME_TIMEOUT_MS = 200


class TofPublisher(Node):
    def __init__(self):
        super().__init__("tof_publisher")

        self.publisher_ = self.create_publisher(CompressedImage, TOPIC, 10)

        self.camera = ac.ArducamCamera()
        ret = self.camera.open(ac.Connection.CSI, 0)
        if ret != 0:
            self.get_logger().error(f"Failed to open ToF camera (CSI): error {ret}")
            raise RuntimeError("ToF camera open failed")

        ret = self.camera.start(ac.FrameType.DEPTH)
        if ret != 0:
            self.camera.close()
            self.get_logger().error(f"Failed to start ToF camera stream: error {ret}")
            raise RuntimeError("ToF camera start failed")

        self.camera.setControl(ac.Control.RANGE, MAX_DISTANCE_MM)
        actual_range = self.camera.getControl(ac.Control.RANGE)
        info = self.camera.getCameraInfo()
        self.get_logger().info(
            f"ToF camera up: {info.width}x{info.height}, range {actual_range} mm"
        )
        if actual_range != MAX_DISTANCE_MM:
            self.get_logger().warn(
                f"Camera range is {actual_range} mm, not the requested "
                f"{MAX_DISTANCE_MM} mm; grayscale still maps 0 to {MAX_DISTANCE_MM} mm"
            )

        self.timeouts = 0
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.timer_callback)
        self.get_logger().info(f"Publishing on {TOPIC} at {PUBLISH_HZ} Hz")

    def timer_callback(self):
        frame = self.camera.requestFrame(FRAME_TIMEOUT_MS)
        if frame is None:
            self.timeouts += 1
            if self.timeouts % 50 == 1:
                self.get_logger().warn(f"ToF frame timeout (x{self.timeouts})")
            return
        self.timeouts = 0

        try:
            if not isinstance(frame, ac.DepthData):
                self.get_logger().warn(f"Unexpected frame type: {type(frame)}")
                return

            depth = np.nan_to_num(
                np.asarray(frame.depth_data), nan=0.0, posinf=0.0, neginf=0.0
            )
            invalid = depth <= 0
            if CONFIDENCE_THRESHOLD > 0:
                confidence = np.asarray(frame.confidence_data)
                invalid |= confidence < CONFIDENCE_THRESHOLD

            clipped = np.clip(depth, 0.0, float(MAX_DISTANCE_MM))
            gray = (
                (MAX_DISTANCE_MM - clipped) * (255.0 / MAX_DISTANCE_MM)
            ).astype(np.uint8)
            gray[invalid] = 0
        finally:
            self.camera.releaseFrame(frame)

        # PNG (lossless) so the PC-side processing sees the exact depth levels.
        ok, buffer = cv2.imencode(".png", gray)
        if not ok:
            self.get_logger().warn("PNG encode failed")
            return

        msg = CompressedImage()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "tof_camera"
        msg.format          = "png"
        msg.data            = buffer.tobytes()

        self.publisher_.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down ToF publisher")
        self.camera.stop()
        self.camera.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TofPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
