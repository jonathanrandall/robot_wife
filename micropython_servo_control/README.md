# MicroPython Servo Control

MicroPython firmware for the ESP32 that controls Jessica's pan-tilt head servos (xArm-compatible bus servos).

This board sits between the Raspberry Pi and the servo bus. The Pi sends text commands over UART; this firmware translates them to the half-duplex bus servo protocol.

## Files

| File | Description |
|------|-------------|
| `main.py` | Main loop — listens on UART + USB, parses commands, drives servos |
| `config.py` | Servo IDs, pin assignments, limits, unit↔radian mapping |
| `BusServo.py` | Half-duplex xArm bus servo driver |
| `Oled.py` | Optional SSD1306 OLED status display driver |
| `wifi_stuff.py` | Legacy WiFi web UI (unused — kept as reference) |
| `main_wifi_backup.py` / `main_espnow_backup.py` | Earlier prototype versions |

## Command protocol

Commands are newline-terminated text, accepted on UART1 (Pi link) and USB simultaneously. Every command gets a JSON reply:

```json
{"name": ["pan_joint", "tilt_joint"], "position": [0.0, -0.3]}
```

| Command | Example | Description |
|---------|---------|-------------|
| `ptr <pan_rad> <tilt_rad> [ms]` | `ptr 0.5 -0.3 1000` | Move both axes in radians |
| `pr <rad> [ms]` | `pr 0.5` | Pan only |
| `tr <rad> [ms]` | `tr -0.3` | Tilt only |
| `pt <pan> <tilt> [ms]` | `pt 600 400` | Move both in servo units (0–1000) |
| `pos` | `pos` | Report current positions |

Servo units: 500 = 0 rad, 0 = −135°, 1000 = +135°. Per-servo limits and direction signs are in `config.py`.

## Wiring (xArm ESP32 board)

| Signal | Pin |
|--------|-----|
| Servo bus TX | GPIO 26 |
| Servo bus RX | GPIO 35 |
| Servo TX enable | GPIO 25 |
| Servo RX enable | GPIO 12 |
| UART1 TX (Pi link) | GPIO 17 |
| UART1 RX (Pi link) | GPIO 16 |
| OLED SCL | GPIO 27 |
| OLED SDA | GPIO 14 |

Pan servo = ID 6, Tilt servo = ID 5.
