#!/usr/bin/env python3

import sys
sys.path.insert(0, '/home/jonny/venvs/jazzy/lib/python3.12/site-packages')

import subprocess

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


WEBCAM_NAME  = "3D USB Camera"
TOPIC        = "/jessica/camera/image/compressed"
PUBLISH_HZ   = 22    # dropped from 30 to cut JPEG-encode CPU (was starving the mic)
JPEG_QUALITY = 80

# The stereo camera outputs a side-by-side frame (e.g. 1280×480).
# Publishing at half resolution keeps bandwidth reasonable.
FRAME_WIDTH  = 640
FRAME_HEIGHT = 240


def find_webcam_index(device_name: str) -> int | None:
    """Return the /dev/videoN index for the first device matching device_name."""
    try:
        output = subprocess.check_output(["v4l2-ctl", "--list-devices"], text=True)
    except subprocess.CalledProcessError:
        return None

    for block in output.split("\n\n"):
        if device_name in block:
            for line in block.splitlines():
                line = line.strip()
                if line.startswith("/dev/video"):
                    try:
                        return int(line[len("/dev/video"):])
                    except ValueError:
                        continue
    return None


class WebcamPublisher(Node):
    def __init__(self):
        super().__init__("webcam_publisher")

        self.publisher_ = self.create_publisher(CompressedImage, TOPIC, 10)

        webcam_index = find_webcam_index(WEBCAM_NAME)
        if webcam_index is None:
            self.get_logger().error(f"Webcam '{WEBCAM_NAME}' not found — check USB connection")
            raise RuntimeError("Webcam device not found")

        self.get_logger().info(f"Opening webcam '{WEBCAM_NAME}' on /dev/video{webcam_index}")
        self.cap = cv2.VideoCapture(webcam_index)

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open /dev/video{webcam_index}")
            raise RuntimeError("Webcam open failed")

        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.timer_callback)
        self.get_logger().info(f"Publishing on {TOPIC} at {PUBLISH_HZ} Hz")

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Failed to read frame from webcam")
            return

        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT), interpolation=cv2.INTER_LINEAR)

        _, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])

        msg = CompressedImage()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "webcam"
        msg.format          = "jpeg"
        msg.data            = buffer.tobytes()

        self.publisher_.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down webcam publisher")
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WebcamPublisher()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
