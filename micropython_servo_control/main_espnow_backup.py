# ESP-NOW pan/tilt servo controller.
#
# Commands (plain text messages over ESP-NOW):
#   p <units> [ms]            move pan
#   t <units> [ms]            move tilt
#   pt <pan> <tilt> [ms]      move both
#   pos                       just report positions
#
# Every command gets a JSON reply with the servo positions in radians,
# e.g. {"pan": 0.0, "tilt": 0.4712}. 500 units = 0 rad, 0/1000 units
# = -/+ 135 deg (see config.py). Positions that fail to read are null.

import json
import sys
import time

import network
import espnow
from machine import Pin, I2C
import ubinascii

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

# --- WiFi + ESP-NOW setup ---
# ESP-NOW requires both boards on the same channel. Preferred: both join
# the same AP and inherit its channel. Fallback (AP down or USE_WIFI
# False): run unassociated on the fixed config.ESPNOW_CHANNEL.
wlan = network.WLAN(network.STA_IF)
wlan.active(True)

connected = False
if config.USE_WIFI:
    from wifi_stuff import ssid, password
    wlan.connect(ssid, password)
    deadline = time.ticks_add(time.ticks_ms(), config.WIFI_TIMEOUT_S * 1000)
    while not wlan.isconnected():
        if time.ticks_diff(deadline, time.ticks_ms()) < 0:
            print('WiFi timeout, falling back to channel', config.ESPNOW_CHANNEL)
            break
        time.sleep_ms(100)
    connected = wlan.isconnected()

if not connected:
    wlan.disconnect()  # stop reconnect attempts; they would hop channels
    try:
        wlan.config(channel=config.ESPNOW_CHANNEL)
    except (OSError, ValueError):
        pass

try:
    wlan.config(pm=wlan.PM_NONE)  # power-save drops ESP-NOW packets
except (AttributeError, OSError, ValueError):
    pass

mac = ubinascii.hexlify(wlan.config('mac'), ':').decode()
if connected:
    print('ESP-NOW MAC:', mac, 'IP:', wlan.ifconfig()[0])
else:
    print('ESP-NOW MAC:', mac, 'channel:', config.ESPNOW_CHANNEL, '(no WiFi)')

e = espnow.ESPNow()
e.active(True)

if oled:
    oled.fill(0)
    oled.text('ESP-NOW ready', 0, 0)
    oled.text(mac[:8], 0, 20)
    oled.text(mac[9:], 0, 32)
    if connected:
        oled.text(wlan.ifconfig()[0], 0, 48)
    else:
        oled.text('no wifi ch %d' % config.ESPNOW_CHANNEL, 0, 48)
    oled.show()

# --- Servos ---
bus_servo = BusServo(tx=config.SERVO_TX, rx=config.SERVO_RX,
                     tx_en=config.SERVO_TX_EN, rx_en=config.SERVO_RX_EN)
bus_servo.run(config.PAN_ID, config.clamp(config.PAN_ID, config.CENTER_UNITS), config.DEFAULT_MS)
bus_servo.run(config.TILT_ID, config.clamp(config.TILT_ID, config.CENTER_UNITS), config.DEFAULT_MS)


def read_positions_rad():
    reply = {}
    for name, servo_id in (('pan', config.PAN_ID), ('tilt', config.TILT_ID)):
        p = bus_servo.get_position(servo_id)
        reply[name] = None if p is False else config.units_to_rad(p)
    return reply


def move(servo_id, units, ms):
    bus_servo.run(servo_id, config.clamp(servo_id, units), ms)


def handle_command(line):
    parts = line.split()
    cmd = parts[0]
    if cmd == 'p':
        ms = int(parts[2]) if len(parts) > 2 else config.DEFAULT_MS
        move(config.PAN_ID, int(parts[1]), ms)
    elif cmd == 't':
        ms = int(parts[2]) if len(parts) > 2 else config.DEFAULT_MS
        move(config.TILT_ID, int(parts[1]), ms)
    elif cmd == 'pt':
        ms = int(parts[3]) if len(parts) > 3 else config.DEFAULT_MS
        move(config.PAN_ID, int(parts[1]), ms)
        move(config.TILT_ID, int(parts[2]), ms)
    elif cmd != 'pos':
        return {'err': 'unknown command: %s' % cmd}
    return read_positions_rad()


while True:
    try:
        peer, msg = e.recv()
        if msg is None:
            continue
        line = msg.decode().strip()
        print('recv from', ubinascii.hexlify(peer, ':').decode(), ':', line)
        if not line:
            continue
        try:
            reply = handle_command(line)
        except Exception as ex:
            reply = {'err': str(ex)}
        try:
            e.add_peer(peer)
        except OSError:
            pass  # already registered
        e.send(peer, json.dumps(reply))
    except Exception as ex:
        sys.print_exception(ex)
        time.sleep_ms(100)
