# Serial pan/tilt servo controller (link to the Pi on UART1, plus USB).
#
# Commands (newline-terminated text):
#   p <units> [ms]                 move pan (servo units, 0-1000)
#   t <units> [ms]                 move tilt
#   pt <pan> <tilt> [ms]           move both
#   pr <rad> [ms]                  move pan (radians)
#   tr <rad> [ms]                  move tilt
#   ptr <pan_rad> <tilt_rad> [ms]  move both
#   pos                            just report positions
#
# Every command gets one JSON line back, shaped like sensor_msgs/JointState
# so a ROS2 node can republish it directly:
#   {"name": ["pan_joint", "tilt_joint"], "position": [0.0, 0.4712]}
# Positions are radians (500 units = 0 rad, 0/1000 = -/+135 deg, see
# config.py); a position that failed to read comes back as null.
# Parse errors reply {"error": "..."}.
#
# Note: positions are read live from the servos, so a reply right after a
# move reflects the in-flight position. Poll 'pos' for the settled value.
#
# Commands are accepted on two links, and the reply goes back on whichever
# one the command arrived on:
#   - UART1 on pins 16 (RX) / 17 (TX) — the Pi link
#   - USB (UART0/stdin, the same port as the REPL) — for bench testing
# On USB, debug print() output shares the line with the JSON replies, so
# readers must skip lines that don't start with '{'.
#
# --- Testing from a Jupyter notebook over USB (pip install pyserial) ---
#
#   import serial, json, time
#   ser = serial.Serial()
#   ser.port = '/dev/ttyUSB0'      # the board's USB port
#   ser.baudrate = 115200
#   ser.timeout = 1
#   ser.dtr = False                # don't reset the board on open
#   ser.rts = False
#   ser.open()
#
#   def cmd(s, timeout=2.0):
#       ser.reset_input_buffer()
#       ser.write((s + '\n').encode())
#       end = time.time() + timeout
#       while time.time() < end:
#           line = ser.readline().strip()
#           if line.startswith(b'{'):
#               return json.loads(line)
#       raise TimeoutError('no JSON reply to %r' % s)
#
#   cmd('pos')            # {'name': ['pan_joint','tilt_joint'], 'position': [...]}
#   cmd('ptr 0.5 -0.3')   # pan 0.5 rad, tilt -0.3 rad
#   cmd('t 750')          # tilt to its max in servo units
#   cmd('ptr 0 0 2000')   # both to center over 2 s
#
# A TimeoutError means no reply — check power/port, and that main.py is
# actually running (open the port in a terminal: Ctrl+C should show a
# KeyboardInterrupt from main.py, Ctrl+D restarts it).

import json
import sys
import time

try:
    import uselect as select
except ImportError:
    import select

from machine import Pin, I2C, UART

import config
from BusServo import BusServo
from Oled import OLED_I2C

# --- OLED (optional) ---
oled = None
i2c = I2C(0, scl=Pin(config.OLED_SCL), sda=Pin(config.OLED_SDA), freq=100000)
try:
    oled = OLED_I2C(128, 64, i2c)
except OSError:
    print('No OLED module detected')

if oled:
    oled.fill(0)
    oled.text('pan/tilt serial', 0, 0)
    oled.text('uart%d tx%d rx%d' % (config.CMD_UART_ID, config.CMD_UART_TX,
                                    config.CMD_UART_RX), 0, 20)
    oled.show()

# --- Servos ---
bus_servo = BusServo(tx=config.SERVO_TX, rx=config.SERVO_RX,
                     tx_en=config.SERVO_TX_EN, rx_en=config.SERVO_RX_EN)
bus_servo.run(config.PAN_ID, config.clamp(config.PAN_ID, config.CENTER_UNITS), config.DEFAULT_MS)
bus_servo.run(config.TILT_ID, config.clamp(config.TILT_ID, config.CENTER_UNITS), config.DEFAULT_MS)

# --- Command UART to the Pi ---
uart = UART(config.CMD_UART_ID, baudrate=config.CMD_UART_BAUD,
            tx=config.CMD_UART_TX, rx=config.CMD_UART_RX)


def joint_state():
    names = []
    positions = []
    for servo_id in config.JOINT_ORDER:
        p = bus_servo.get_position(servo_id)
        names.append(config.JOINT_NAMES[servo_id])
        positions.append(None if p is False else config.units_to_rad(servo_id, p))
    return {'name': names, 'position': positions}


def move_units(servo_id, units, ms):
    bus_servo.run(servo_id, config.clamp(servo_id, units), ms)


def move_rad(servo_id, rad, ms):
    move_units(servo_id, config.rad_to_units(servo_id, rad), ms)


def handle_command(line):
    parts = line.split()
    cmd = parts[0]
    if cmd == 'p':
        ms = int(parts[2]) if len(parts) > 2 else config.DEFAULT_MS
        move_units(config.PAN_ID, int(parts[1]), ms)
    elif cmd == 't':
        ms = int(parts[2]) if len(parts) > 2 else config.DEFAULT_MS
        move_units(config.TILT_ID, int(parts[1]), ms)
    elif cmd == 'pt':
        ms = int(parts[3]) if len(parts) > 3 else config.DEFAULT_MS
        move_units(config.PAN_ID, int(parts[1]), ms)
        move_units(config.TILT_ID, int(parts[2]), ms)
    elif cmd == 'pr':
        ms = int(parts[2]) if len(parts) > 2 else config.DEFAULT_MS
        move_rad(config.PAN_ID, float(parts[1]), ms)
    elif cmd == 'tr':
        ms = int(parts[2]) if len(parts) > 2 else config.DEFAULT_MS
        move_rad(config.TILT_ID, float(parts[1]), ms)
    elif cmd == 'ptr':
        ms = int(parts[3]) if len(parts) > 3 else config.DEFAULT_MS
        move_rad(config.PAN_ID, float(parts[1]), ms)
        move_rad(config.TILT_ID, float(parts[2]), ms)
    elif cmd != 'pos':
        return {'error': 'unknown command: %s' % cmd}
    return joint_state()


def process(line, write):
    line = line.strip()
    if not line:
        return
    try:
        reply = handle_command(line)
    except Exception as ex:
        reply = {'error': str(ex)}
    write(json.dumps(reply) + '\n')


stdin_poll = select.poll()
stdin_poll.register(sys.stdin, select.POLLIN)

uart_buf = b''
stdin_buf = ''
while True:
    try:
        idle = True

        data = uart.read()
        if data:
            idle = False
            uart_buf += data
            while b'\n' in uart_buf:
                raw, uart_buf = uart_buf.split(b'\n', 1)
                process(raw.decode(), uart.write)

        while stdin_poll.poll(0):
            idle = False
            ch = sys.stdin.read(1)
            if ch in ('\n', '\r'):
                line, stdin_buf = stdin_buf, ''
                process(line, sys.stdout.write)
            else:
                stdin_buf += ch

        if idle:
            time.sleep_ms(5)
    except Exception as ex:
        sys.print_exception(ex)
        uart_buf = b''
        stdin_buf = ''
        time.sleep_ms(100)
