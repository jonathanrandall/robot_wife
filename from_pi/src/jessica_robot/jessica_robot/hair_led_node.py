#!/usr/bin/env python3
"""
ROS 2 node: subscribes to /jessica/hair_hue (Int32) and drives the 5×WS2811
LED strip that forms Jessica's hair.

Hue encoding (degrees, 0–359):
  0   red        25  orange     60  yellow
  120 green      180 cyan       240 blue
  270 purple     300 magenta    340 pink
  -1  white (S=0)
  -2  rainbow

Hardware (RPi 5 SPI mode):
  Data wire → GPIO 10 / physical pin 19 (SPI0 MOSI)
  Enable SPI: add  dtparam=spi=on  to /boot/firmware/config.txt and reboot.
  Add user to spi group to avoid sudo:  sudo usermod -aG spi $USER
"""

import sys
sys.path.insert(0, '/home/jonny/venvs/jazzy/lib/python3.12/site-packages')

import colorsys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from pi5neo import Pi5Neo, EPixelType

# ---------------------------------------------------------------------
# LED strip config
# ---------------------------------------------------------------------

LED_COUNT  = 5
SPI_DEV    = '/dev/spidev0.0'
SPI_SPEED  = 800            # kHz
BRIGHTNESS = 0.8            # 0.0–1.0

HAIR_HUE_TOPIC = "/jessica/hair_hue"


class HairLedNode(Node):
    def __init__(self):
        super().__init__("hair_led_node")

        self.strip = Pi5Neo(SPI_DEV, LED_COUNT, SPI_SPEED, pixel_type=EPixelType.RGB)
        self._set_all(0, 0, 0)  # start dark

        self.sub = self.create_subscription(Int32, HAIR_HUE_TOPIC, self._on_hue, 10)
        self.get_logger().info("Hair LED node ready — listening on " + HAIR_HUE_TOPIC)

    def _scale(self, value: int) -> int:
        return int(value * BRIGHTNESS)

    def _set_all(self, r: int, g: int, b: int):
        self.strip.fill_strip(self._scale(r), self._scale(g), self._scale(b))
        self.strip.update_strip()

    def _set_pixel(self, index: int, r: int, g: int, b: int):
        self.strip.set_led_color(index, self._scale(r), self._scale(g), self._scale(b))

    def _on_hue(self, msg: Int32):
        hue_val = msg.data

        if hue_val == -1:
            self._set_all(255, 255, 255)
            self.get_logger().info("Hair → white")
            return

        if hue_val == -2:
            rainbow = [(255, 0, 0), (255, 105, 0), (255, 255, 0), (0, 255, 0), (0, 0, 255)]
            for i, (r, g, b) in enumerate(rainbow):
                self._set_pixel(i, r, g, b)
            self.strip.update_strip()
            self.get_logger().info("Hair → rainbow")
            return

        h = (hue_val % 360) / 360.0
        r, g, b = colorsys.hsv_to_rgb(h, 1.0, 1.0)
        ri, gi, bi = int(r * 255), int(g * 255), int(b * 255)
        self._set_all(ri, gi, bi)
        self.get_logger().info(f"Hair → hue={hue_val}°  rgb=({ri}, {gi}, {bi})")


def main():
    rclpy.init()
    node = HairLedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
