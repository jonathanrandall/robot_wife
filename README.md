# Robot Wife — Jessica

A personal robotics project: a conversational, physically expressive companion robot called **Jessica**. She drives around on four wheels, moves her head with a pan-tilt servo, lights up her LED hair, sees through a stereo USB camera, listens via microphone, and speaks through a USB speaker — all coordinated by a Raspberry Pi 5 running ROS 2 and an ESP32-S3 handling the low-level motor and servo control.

---

## Hardware Overview

| Component | Details |
|-----------|---------|
| **Brain** | Raspberry Pi 5 |
| **Motor / Servo Controller** | Custom ESP32-S3 PCB (PlatformIO firmware) |
| **Drive system** | 4-wheel differential drive, 4× motors with 1425 CPR quadrature encoders |
| **Motor drivers** | 4× VNH7040 H-bridge ICs controlled via MCP23017 I2C GPIO expander |
| **Head** | Pan-tilt servo mount (2× servos, ±90° pan, adjustable tilt) |
| **Hair LEDs** | 5× WS2811 addressable LEDs via Raspberry Pi SPI (GPIO 10 / MOSI) |
| **Camera** | USB stereo 3D camera — published as compressed JPEG at 30 Hz |
| **Speaker** | USB audio device — text-to-speech via Piper TTS |
| **Microphone** | USB audio device — speech recognition via Whisper (offloaded to laptop) |
| **Power** | LiPo battery with buck converter; 3D-printed battery holder |

---

## Repository Structure

```
robot_wife/
├── esp32_bridge/           # ESP32-S3 PlatformIO firmware
│   ├── src/
│   │   ├── main.cpp                  — entry point, configuration, FreeRTOS tasks
│   │   ├── Motor.cpp                 — VNH7040 H-bridge driver (PWM + encoder)
│   │   ├── Mcp23017Bus.cpp           — I2C GPIO expander driver
│   │   ├── QuadratureEncoder.cpp     — hardware PCNT quadrature decoder
│   │   ├── PIDController.cpp         — general-purpose PID
│   │   ├── RobotController.cpp       — differential drive kinematics + 4× PID
│   │   ├── PanTiltController.cpp     — servo pan-tilt head control
│   │   └── WebDashboard.cpp          — WiFi web UI + REST telemetry API
│   ├── include/                      — corresponding header files
│   ├── esp32_combined_hardware.cpp   — (copy used during development)
│   └── DOCUMENTATION.md             — detailed ESP32 firmware reference
│
├── from_pi/                # ROS 2 packages that run on the Raspberry Pi
│   └── src/
│       ├── jessica_robot/            — main robot package
│       │   ├── jessica_robot/
│       │   │   ├── jessica_chatbot.py      — AI conversation loop (Ollama LLM)
│       │   │   ├── hair_led_node.py        — WS2811 LED strip ROS node
│       │   │   └── pan_tilt_teleop.py      — joystick head control node
│       │   └── launch/
│       │       └── jessica.launch.py       — full system launch file
│       ├── esp32_combined_hardware/  — ros2_control hardware interface for ESP32
│       │   ├── src/
│       │   │   ├── esp32_combined_hardware.cpp  — serial comms to ESP32 (4 wheels + 2 servos)
│       │   │   └── joy_button_bridge.cpp        — joystick button → ROS topic bridge
│       │   └── include/
│       └── camera_publisher/         — USB stereo camera → ROS topic
│           └── camera_publisher/
│               └── webcam_publisher.py
│
├── text_to_speech/         # Laptop-side services
│   └── whisper_server.py   — FastAPI server; transcribes audio via faster-whisper (CUDA)
│
└── 3d_prints/              # FreeCAD macros and STL files for printed parts
    ├── macros/             — FreeCAD parametric macros (.FCMacro)
    └── *.stl               — ready-to-print STL files
```

---

## Software Architecture

```
                         Raspberry Pi 5
┌──────────────────────────────────────────────────────────────┐
│                                                              │
│  jessica_chatbot.py                                          │
│    ├─ records microphone (sounddevice)                       │
│    ├─ transcribes via Whisper server (HTTP → laptop)         │
│    ├─ sends conversation to Ollama LLM (HTTP → laptop)       │
│    ├─ synthesises speech reply (Piper TTS, local)            │
│    ├─ plays audio (aplay → USB speaker)                      │
│    ├─ publishes /jessica/hair_hue  (Int32)                   │
│    ├─ publishes /cmd_vel           (Twist, autonomous)       │
│    └─ publishes /pan_tilt_controller/joint_trajectory        │
│                                                              │
│  hair_led_node.py                                            │
│    └─ subscribes /jessica/hair_hue → drives WS2811 LEDs      │
│                                                              │
│  webcam_publisher.py                                         │
│    └─ USB stereo camera → /jessica/camera/image/compressed   │
│                                                              │
│  pan_tilt_teleop.py                                          │
│    └─ /joy → /pan_tilt_controller/joint_trajectory           │
│                                                              │
│  twist_mux  (joystick priority > chatbot autonomous)         │
│    └─ /cmd_vel_joy + /cmd_vel → /diff_cont/cmd_vel           │
│                                                              │
│  esp32_combined_hardware  (ros2_control plugin)              │
│    └─ /diff_cont + /pan_tilt_controller → USB serial → ESP32 │
│                                                              │
└──────────────────────────────────────────────────────────────┘
                              │ USB serial (115200 baud)
                              ▼
                          ESP32-S3 Firmware
┌──────────────────────────────────────────────────────────────┐
│  FreeRTOS tasks                                              │
│    ├─ motorControlTask  (Core 1, 10 ms)  — 4× PID loops      │
│    ├─ serialCommandTask (Core 0,  1 ms)  — ROS serial bridge  │
│    └─ telemetryTask     (Core 0, 50 ms)  — encoder/current    │
│                                                              │
│  WebDashboard  — WiFi web UI at http://robot.local           │
│    └─ REST API: /api/enable /api/cmd /api/speed /api/telemetry│
│                                                              │
│  Hardware: 4× VNH7040 → motors + encoders                   │
│            MCP23017 I2C expander → direction pins + LEDs     │
│            2× servo PWM → pan-tilt head                      │
└──────────────────────────────────────────────────────────────┘

                              Laptop (any machine with GPU)
┌──────────────────────────────────────────────────────────────┐
│  whisper_server.py  — faster-whisper (CUDA, float16)         │
│    └─ POST /transcribe  ← WAV bytes from Pi chatbot          │
│                                                              │
│  Ollama  — local LLM (llama3.2:3b or similar)               │
│    └─ POST /api/chat  ← conversation from Pi chatbot         │
└──────────────────────────────────────────────────────────────┘
```

---

## ESP32-S3 Firmware

The firmware lives in `esp32_bridge/` and is built with PlatformIO (Arduino framework). See [`esp32_bridge/DOCUMENTATION.md`](esp32_bridge/DOCUMENTATION.md) for the full software reference including pin assignments, PID tuning guide, serial protocol, and web API.

### Quick summary

- **4-wheel differential drive** with independent closed-loop PID speed control per wheel
- **Quadrature encoder** reading via ESP32-S3 hardware PCNT peripheral (64-bit position)
- **VNH7040** H-bridge motor drivers; direction and MultiSense current sensing via MCP23017
- **Pan-tilt head** with two servos controlled by `PanTiltController`
- **ROS 2 serial interface** at 115200 baud — accepts wheel speed commands, returns encoder velocities
- **Serial watchdog** — motors stop automatically if no command for 1 second
- **WiFi web dashboard** at `http://robot.local` — telemetry, manual drive, PWM readout

### Build and flash

```bash
cd esp32_bridge
pio run --target upload
pio device monitor
```

### WiFi configuration

Edit the top of `esp32_bridge/src/main.cpp`:

```cpp
const char* WIFI_SSID     = "your-network";
const char* WIFI_PASSWORD = "your-password";
```

---

## ROS 2 Setup (Raspberry Pi)

The Pi runs **ROS 2 Jazzy**. All packages live in `from_pi/src/` and are built with colcon.

```bash
# On the Pi, from the workspace root above from_pi/
cd from_pi
colcon build
source install/setup.bash

# Launch everything (hardware + chatbot + LEDs)
ros2 launch jessica_robot jessica.launch.py

# Chatbot only (no ESP32 / joystick needed):
ros2 launch jessica_robot jessica.launch.py hardware:=false
```

### Key ROS topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/jessica/hair_hue` | `std_msgs/Int32` | chatbot → LED node |
| `/jessica/camera/image/compressed` | `sensor_msgs/CompressedImage` | camera publisher |
| `/cmd_vel` | `geometry_msgs/Twist` | chatbot autonomous drive |
| `/cmd_vel_joy` | `geometry_msgs/Twist` | joystick manual drive |
| `/diff_cont/cmd_vel` | `geometry_msgs/Twist` | twist_mux output → diff drive controller |
| `/pan_tilt_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | head control |
| `/joy` | `sensor_msgs/Joy` | gamepad input |

---

## AI Chatbot — Jessica

`jessica_chatbot.py` implements the full conversation loop:

1. **Listen** — records microphone until silence detected
2. **Transcribe** — sends WAV to the laptop Whisper server (`POST /transcribe`)
3. **Think** — sends conversation history to Ollama (`llama3.2:3b` by default)
4. **Execute** — parses JSON response for a robot command (`drive`, `turn`, `look`, `wave`, `nod`, `shake_head`, `change_hair_color`, `stop`, or `none`)
5. **Speak** — synthesises the reply text with Piper TTS and plays it through the USB speaker

Jessica only acts on robot commands when the user begins with **"Jessica darling"**. All other speech is treated as conversation only.

### Whisper server (laptop)

Run on any machine with a CUDA GPU:

```bash
cd text_to_speech
pip install fastapi uvicorn faster-whisper
python whisper_server.py
```

The server defaults to `whisper-small.en` on port 8765. Override with environment variables:

```bash
WHISPER_MODEL=medium.en WHISPER_PORT=8765 python whisper_server.py
```

Update `WHISPER_URL` and `OLLAMA_URL` at the top of `jessica_chatbot.py` to point to the laptop's IP.

---

## Hair LED Node

`hair_led_node.py` controls a strip of **5× WS2811** LEDs via Raspberry Pi SPI. It subscribes to `/jessica/hair_hue` (Int32):

| Value | Colour |
|-------|--------|
| 0–359 | HSV hue in degrees (S=100%, V=80%) |
| -1 | White |
| -2 | Rainbow (each LED a different colour) |

**Hardware wiring:** data wire → GPIO 10 (SPI0 MOSI, physical pin 19). Enable SPI in `/boot/firmware/config.txt`:

```
dtparam=spi=on
```

---

## 3D Printed Parts

All STL files and FreeCAD macros are in `3d_prints/`.

| File | Description |
|------|-------------|
| `battery_holder.stl` | LiPo battery mount |
| `buck_converter_mount.stl` | Voltage regulator bracket |
| `camera_stand_final_v6-Body.stl` | USB camera stand |
| `led_boobs.stl` / `wife_boobs_v2_13.stl` | Decorative LED diffuser enclosures |
| `light_diffuser_Sphere.stl` | Hemisphere light diffuser |
| `Hemisphere-Editable_13mm_Inner_Radius_Hemisphere001.stl` | Parametric hemisphere |
| `mount_holders.stl` | General component mounts |
| `pi_holder.stl` | Raspberry Pi mount |
| `servo_head_connection-Body.stl` | Pan-tilt servo bracket |
| `Speaker_Box.stl` | USB speaker enclosure |
| `Switch_holderv2.stl` | Power switch holder |
| `macros/` | FreeCAD `.FCMacro` parametric source files |

---

## Security Notes

This project assumes a **trusted private LAN**. Nothing here should be exposed to the internet.

- **WiFi credentials** belong only in `esp32_bridge/src/main.cpp` — that file is committed with an **empty password field**. Fill it in locally and take care not to commit the change (e.g. `git update-index --skip-worktree esp32_bridge/src/main.cpp`). The git history has been audited and contains no real credentials.
- **The ESP32 web dashboard has no authentication.** Anyone on the same WiFi network can drive the robot, move the head, trigger the aux output (`/api/fire`), or toggle the e-stop via plain HTTP GET requests. Keep the robot on a private/isolated network (a guest or IoT VLAN is ideal) and never port-forward to it.
- **The Whisper and Ollama servers are unauthenticated** and `whisper_server.py` binds to `0.0.0.0` (all interfaces). Run them only on machines behind your router's firewall; do not expose ports 8765 or 11434 to the internet.
- **Server IPs are hardcoded** in `jessica_chatbot.py` (`OLLAMA_URL`, `WHISPER_URL`). These are private LAN addresses, not secrets, but update them to match your own network.
- No API keys, tokens, or private keys are used anywhere in this project — all AI services (Whisper, Ollama, Piper) run locally.

---

## License

See [LICENSE](LICENSE).
