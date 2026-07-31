# esp32_combined_hardware

A `ros2_control` hardware interface plugin that bridges the ROS 2 controller manager to the ESP32-S3 motor firmware over USB serial.

Handles **6 joints**: 4 drive wheels (velocity interface) + 2 head servos (position interface, pan and tilt).

## How it works

- Opens the serial port (configured in the URDF `<hardware>` block, typically `/dev/esp32_motor`)
- Each control loop cycle: sends `m_v0_v1_v2_v3_\r` wheel speed commands and reads back encoder velocities
- Head servo positions are sent as `ptr <pan_rad> <tilt_rad>\r` to the MicroPython servo ESP32
- Baud rate: 115200; timeout must be less than the control loop period (25 ms at 30 Hz)

## Configuration (in jessica.urdf.xacro)

```xml
<param name="device">/dev/esp32_motor</param>
<param name="baud_rate">115200</param>
<param name="timeout_ms">25</param>
<param name="wheel_radius">7.2</param>       <!-- cm -->
<param name="enc_counts_per_rev">1425</param>
```

Also contains `joy_button_bridge` — maps joystick buttons to ROS topics (enable/disable following modes, etc.). Configured via `config/joy_button_mappings.yaml`.
