# Example ESP-NOW sender for another ESP32 (MicroPython).
# Set RECEIVER_MAC to the MAC printed/shown on the servo controller's OLED.

import network
import espnow
import time

RECEIVER_MAC = b'\xaa\xbb\xcc\xdd\xee\xff'  # <-- replace with real MAC bytes

# Both boards must be on the same channel. If this board is connected to
# WiFi, it uses the AP's channel and the receiver (joining the same AP)
# matches automatically. If not connected, pin the same fallback channel
# as config.ESPNOW_CHANNEL on the receiver.
FALLBACK_CHANNEL = 1

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
if not wlan.isconnected():
    wlan.disconnect()
    try:
        wlan.config(channel=FALLBACK_CHANNEL)
    except (OSError, ValueError):
        pass
try:
    # power-save drops ESP-NOW packets while associated to an AP
    wlan.config(pm=wlan.PM_NONE)
except (AttributeError, OSError, ValueError):
    pass

e = espnow.ESPNow()
e.active(True)
e.add_peer(RECEIVER_MAC)


def command(cmd, timeout_ms=1000):
    """Send a command and return the JSON reply string (or None on timeout)."""
    e.send(RECEIVER_MAC, cmd)
    peer, msg = e.recv(timeout_ms)
    return msg.decode() if msg else None


if __name__ == '__main__':
    print(command('pos'))            # read positions (radians)
    print(command('p 600'))          # pan to 600 units
    time.sleep(1)
    print(command('t 700 800'))      # tilt to 700 units over 800 ms
    time.sleep(1)
    print(command('pt 500 500'))     # both back to center
