#!/usr/bin/env python3
"""Jessica's face: the Waveshare 7" touchscreen.

Renders straight to the display via KMS/DRM (pygame) — no desktop needed,
works alongside SSH. pygame comes from the SYSTEM python (apt python3-pygame);
the pip wheel lacks the KMS/DRM driver.

Subscribes
  /jessica/ui_state     std_msgs/String
                        "listening" | "thinking" | "talking" | "idle"
  /jessica/speech_env   std_msgs/Float32MultiArray
                        data[0]  = seconds per envelope frame
                        data[1:] = RMS levels 0..1, one per frame, starting when
                                   the message arrives (the chatbot publishes it
                                   immediately before starting playback)
Publishes
  /jessica/touch        geometry_msgs/Point    x,y = pixel coords of a touch

Modes
  listening — big "Listening..." in warm hues (pink→red→orange→yellow)
  thinking  — big "Thinking..." in cool hues (green→cyan→blue→indigo)
  talking   — colourful overlapping soundwaves, amplitude = speech envelope
  idle      — dim breathing dot (low frame rate, near-zero CPU)
"""

import sys
sys.path.insert(0, '/home/jonny/venvs/jazzy/lib/python3.12/site-packages')
sys.path.append('/usr/lib/python3/dist-packages')   # system pygame lives here

import glob
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Point

# Pick the DRM card with a connected display (card0 on the Pi 5 is the render
# node with no connectors; SDL sometimes tries it first and fails).
os.environ.setdefault("SDL_VIDEODRIVER", "kmsdrm")
for _status in sorted(glob.glob("/sys/class/drm/card*-*/status")):
    with open(_status) as _f:
        if _f.read().strip() == "connected":
            _card = os.path.basename(os.path.dirname(_status)).split("-")[0]
            os.environ.setdefault("SDL_VIDEO_KMSDRM_DEVICE_INDEX", _card[-1])
            break

import pygame

ACTIVE_FPS = 25   # listening / talking
IDLE_FPS   = 10

N_WAVES = 4
WAVE_PARAMS = [
    # (frequency in cycles across screen, speed rad/s, base hue)
    (2.0, 2.5, 0),
    (3.0, -3.2, 90),
    (4.5, 4.1, 180),
    (6.0, -5.0, 270),
][:N_WAVES]

BG = (5, 5, 15)

# Hue windows for the text modes (degrees, 0-360 wheel; end > 360 wraps).
# Listening: pink→red→orange→yellow. Thinking: green→cyan→blue→indigo.
LISTENING_HUES = (290, 424)
THINKING_HUES  = (76, 280)
HUE_SPEED = 60   # deg/s sweep within the window


def hsv_color(h, s=1.0, v=1.0):
    c = pygame.Color(0)
    c.hsva = (h % 360, s * 100, v * 100, 100)
    return c


def hue_pingpong(t, lo, hi, speed=HUE_SPEED):
    """Sweep back and forth inside [lo, hi] — no snap-back at the ends."""
    span = hi - lo
    ph = (t * speed) % (2 * span)
    return (lo + (ph if ph <= span else 2 * span - ph)) % 360


class DisplayNode(Node):
    def __init__(self):
        super().__init__("jessica_display")
        self.mode = "idle"
        self.env_levels: list[float] = []
        self.env_frame_s = 0.05
        self.env_t0 = 0.0

        self.create_subscription(String, "/jessica/ui_state", self.on_state, 10)
        self.create_subscription(Float32MultiArray, "/jessica/speech_env",
                                 self.on_envelope, 10)
        self.touch_pub = self.create_publisher(Point, "/jessica/touch", 10)
        self.get_logger().info("Display node up")

    def on_state(self, msg: String):
        state = msg.data.strip().lower()
        if state in ("listening", "thinking", "talking", "idle"):
            if state != self.mode:
                self.get_logger().info(f"ui_state -> {state}")
            self.mode = state
        else:
            self.get_logger().warn(f"Unknown ui_state '{msg.data}' — ignoring")

    def on_envelope(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) < 2:
            return
        self.env_frame_s = max(0.01, float(data[0]))
        self.env_levels = data[1:]
        self.env_t0 = time.monotonic()

    def current_level(self) -> float:
        """Speech level right now, from the envelope clock."""
        if not self.env_levels:
            return 0.0
        idx = (time.monotonic() - self.env_t0) / self.env_frame_s
        i = int(idx)
        if i >= len(self.env_levels) - 1:
            return float(self.env_levels[-1]) if idx < len(self.env_levels) else 0.0
        # linear blend between adjacent frames — smooth at any fps
        frac = idx - i
        return float(self.env_levels[i] * (1 - frac)
                     + self.env_levels[i + 1] * frac)

    def publish_touch(self, x: int, y: int):
        p = Point()
        p.x, p.y, p.z = float(x), float(y), 0.0
        self.touch_pub.publish(p)


def run_display(node: DisplayNode):
    pygame.display.init()
    pygame.font.init()
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    w, h = screen.get_size()
    node.get_logger().info(
        f"Display {w}x{h}, driver={pygame.display.get_driver()}")
    pygame.mouse.set_visible(False)

    clock = pygame.time.Clock()
    big_font = pygame.font.Font(None, int(h * 0.28))
    # Render the text once in white; per-frame we tint a copy (much cheaper
    # than re-rendering the font every frame).
    listen_white = big_font.render("Listening...", True, (255, 255, 255))
    listen_rect = listen_white.get_rect(center=(w // 2, h // 2))
    think_white = big_font.render("Thinking...", True, (255, 255, 255))
    think_rect = think_white.get_rect(center=(w // 2, h // 2))

    xs = list(range(0, w + 8, 8))   # wave sample columns
    smoothed = 0.0                  # displayed level (attack/decay smoothing)

    t0 = time.monotonic()
    while rclpy.ok():
        t = time.monotonic() - t0

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                return
            elif ev.type == pygame.KEYDOWN and ev.key in (pygame.K_ESCAPE,
                                                          pygame.K_q):
                return
            elif ev.type == pygame.FINGERDOWN:
                node.publish_touch(int(ev.x * w), int(ev.y * h))
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                node.publish_touch(*ev.pos)

        mode = node.mode
        screen.fill(BG)

        if mode == "listening":
            hue = hue_pingpong(t, *LISTENING_HUES)
            tinted = listen_white.copy()
            tinted.fill(hsv_color(hue), special_flags=pygame.BLEND_RGB_MULT)
            screen.blit(tinted, listen_rect)

        elif mode == "thinking":
            hue = hue_pingpong(t, *THINKING_HUES)
            tinted = think_white.copy()
            tinted.fill(hsv_color(hue), special_flags=pygame.BLEND_RGB_MULT)
            screen.blit(tinted, think_rect)

        elif mode == "talking":
            level = node.current_level()
            rate = 0.6 if level > smoothed else 0.25   # fast attack, slow decay
            smoothed += (level - smoothed) * rate
            for i, (freq, speed, hue0) in enumerate(WAVE_PARAMS):
                amp = smoothed * h * 0.45 * (1.0 - 0.15 * i)
                phase = t * speed
                k = freq * 2 * math.pi / w
                hue = (hue0 + t * 40) % 360
                pts = [(x, h / 2 + amp * math.sin(k * x + phase)
                        * math.sin(0.5 * k * x + 0.3 * phase))
                       for x in xs]
                pygame.draw.aalines(screen, hsv_color(hue), False, pts)
                pygame.draw.aalines(screen, hsv_color(hue, v=0.6), False,
                                    [(x, y + 2) for x, y in pts])

        else:  # idle — dim breathing dot
            r = 10 + 6 * math.sin(t * 1.5)
            pygame.draw.circle(screen, (40, 40, 70), (w // 2, h // 2), int(r))

        pygame.display.flip()
        clock.tick(ACTIVE_FPS if mode != "idle" else IDLE_FPS)


def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        run_display(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
