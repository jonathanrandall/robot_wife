# ESP32 Bridge — Motor & Drive Firmware

PlatformIO/Arduino firmware for the custom ESP32-S3 PCB that handles all motor control and drive servo functions.

Datasheets for the key ICs are included in this directory:
- `vnh7040_datasheet.pdf` — VNH7040 H-bridge motor driver
- `MCP23017-Data-Sheet-DS20001952.pdf` — MCP23017 I2C GPIO expander
- `AP63200-AP63201-AP63203-AP63205.pdf` — AP632xx buck converter

## Quick start

```bash
pio run --target upload
pio device monitor
```

## What it does

- Closed-loop PID speed control for 4 drive motors (VNH7040 H-bridges via MCP23017)
- Quadrature encoder reading via ESP32-S3 hardware PCNT (64-bit, 1425 CPR)
- ROS 2 compatible serial interface over USB-CDC at 115200 baud
- Serial watchdog — motors stop if no command received within 1 second
- WiFi web dashboard at `http://robot.local` for manual control and telemetry

## Configuration

All user-facing settings are at the top of `src/main.cpp`: WiFi credentials, wheel dimensions, encoder CPR, PID gains, and pin assignments.
