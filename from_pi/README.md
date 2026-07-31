# from_pi — Raspberry Pi ROS 2 Workspace

All ROS 2 packages that run on the Raspberry Pi 5. Built with colcon (ROS 2 Jazzy).

## Build

```bash
cd from_pi
colcon build
source install/setup.bash
```

## Launch

```bash
# Full robot — hardware + cameras + chatbot + display
ros2 launch jessica_robot jessica.launch.py

# Chatbot + LEDs only (no ESP32 / joystick)
ros2 launch jessica_robot jessica.launch.py hardware:=false
```

## Packages

| Package | Description |
|---------|-------------|
| `jessica_robot` | Main package — chatbot, LEDs, head teleop, following nodes, launch file |
| `jessica_display` | Waveshare 7" touchscreen animated UI |
| `jessica_description` | URDF/xacro robot model |
| `esp32_combined_hardware` | ros2_control plugin — serial bridge to ESP32-S3 (4 wheels + head servos) |
| `esp32_servo_hardware` | ros2_control plugin — serial bridge to MicroPython head servo ESP32 |
| `camera_publisher` | USB stereo camera → `/jessica/camera/image/compressed` |
| `tof_publisher` | Arducam ToF depth camera → `/jessica/tof/image/compressed` |
| `person_state_msgs` | Custom message definitions (PersonState, HandState, Landmark) |

The `launcher/` directory contains `jessica_launcher.py` — a standalone (non-ROS) touch-screen power-on menu.
