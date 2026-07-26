# Configuration for the serial pan/tilt servo controller.
from math import pi

# --- Servo bus UART pins (xArm ESP32 board) ---
SERVO_TX = 26
SERVO_RX = 35
SERVO_TX_EN = 25
SERVO_RX_EN = 12

# --- Command UART (link to the Pi) ---
CMD_UART_ID = 1
CMD_UART_TX = 17
CMD_UART_RX = 16
CMD_UART_BAUD = 115200

# --- OLED I2C pins ---
OLED_SCL = 27
OLED_SDA = 14

# --- Servo IDs ---
PAN_ID = 6   # servo 6 = pan
TILT_ID = 5  # servo 5 = tilt

# Joint names reported in the JointState-style reply, in report order.
JOINT_NAMES = {
    PAN_ID: 'pan_joint',
    TILT_ID: 'tilt_joint',
}
JOINT_ORDER = (PAN_ID, TILT_ID)

# --- Motion ---
DEFAULT_MS = 500       # default move duration in ms
CENTER_UNITS = 500     # boot / home position

# Per-servo position limits in servo units (full servo range is 0-1000).
LIMITS = {
    PAN_ID: (0, 1000),
    TILT_ID: (220, 780),
}

# --- Unit <-> radian mapping ---
# 500 units = 0 rad, 1000 units = +135 deg, 0 units = -135 deg.
HALF_RANGE_UNITS = 500
HALF_RANGE_RAD = 135 * pi / 180

# Per-servo direction sign: ROS conventions are +pan = left, +tilt = down.
# Tilt was mirrored here (verified on the head 2026-07-14) because increasing
# units used to tip the head BACK (up) on that servo. After the 2026-07-22
# servo swap the tilt axis reads the other way round — following now runs
# away to the tilt-back limit instead of centering, the signature of an
# inverted direction — so this is flipped to +1. Flip a sign here — never
# in ROS — if a direction test shows a mirrored axis.
DIRECTIONS = {
    PAN_ID: 1,
    TILT_ID: 1,
}


def units_to_rad(servo_id, p):
    d = DIRECTIONS.get(servo_id, 1)
    return d * (p - CENTER_UNITS) * HALF_RANGE_RAD / HALF_RANGE_UNITS


def rad_to_units(servo_id, r):
    d = DIRECTIONS.get(servo_id, 1)
    return int(round(CENTER_UNITS + d * r * HALF_RANGE_UNITS / HALF_RANGE_RAD))


def clamp(servo_id, p):
    lo, hi = LIMITS.get(servo_id, (0, 1000))
    return max(lo, min(hi, p))
