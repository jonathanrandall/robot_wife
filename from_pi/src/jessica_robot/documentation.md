# Jessica Robot — Project Documentation

## Overview

Jessica is an AI companion robot running on a Raspberry Pi 5. She has a voice
conversation interface powered by a local LLM (Ollama), speech recognition,
text-to-speech, and hardware control via ROS 2. The project will grow to include
wheels, servos, cameras, lidar, and a pan-tilt head, all managed through
ros2_control.

---

## Hardware

| Component | Details |
|-----------|---------|
| Computer | Raspberry Pi 5 |
| OS | Ubuntu (not Raspbian) |
| LED strip | 5× WS2811, RGB order, SPI via GPIO 10 (physical pin 19) |
| Speaker | USB Audio device (ALSA card: `Audio`) |
| Microphone | USB PnP Sound Device (ALSA card: `Device`) |
| LLM server | Separate laptop running Ollama at `192.168.1.106:11434` |

---

## Package Structure

```
jessica_ws/src/
├── jessica_robot/              # Current package — chatbot + LEDs
│   ├── jessica_robot/
│   │   ├── jessica_chatbot.py  # Voice conversation node
│   │   └── hair_led_node.py    # WS2811 LED strip node
│   ├── launch/
│   │   └── jessica.launch.py
│   └── setup.py
```

### Planned packages (future)

```
jessica_ws/src/
├── jessica_description/        # URDF, meshes, robot_state_publisher
├── jessica_hardware/           # ros2_control hardware interface → ESP32
│   ├── hardware/               # C++ hardware interface plugin
│   └── config/
│       ├── controllers.yaml
│       └── joystick.yaml
├── jessica_pan_tilt/           # Pan-tilt hardware interface + config
├── jessica_robot/              # Chatbot + LEDs (current)
└── jessica_bringup/            # Top-level launch files only, no code
    └── launch/
        ├── jessica_full.launch.py
        └── jessica_sim.launch.py
```

---

## Python Environment

ROS 2 Jazzy uses the system Python (`/usr/bin/python3`). Extra packages
(e.g. `rpi_ws281x`, `speech_recognition`, `gtts`, `pi5neo`) are installed
in a virtual environment at `/home/jonny/venvs/jazzy/`.

Because `ros2 run` bypasses the venv, each node that needs venv packages
adds this at the top of the file **before any other imports**:

```python
import sys
sys.path.insert(0, '/home/jonny/venvs/jazzy/lib/python3.12/site-packages')
```

---

## Build and Run

```bash
# Build
cd ~/jessica_ws && colcon build && source install/setup.bash

# FULL stack — chatbot + LEDs + ESP32 motion/head + joystick (needs hardware)
ros2 launch jessica_robot jessica.launch.py

# CHATBOT + LEDS ONLY — no ESP32 / joystick / ros2_control.
# Use this when the robot hardware is NOT connected (the full launch above will
# spam errors without the ESP32, a joystick, and ros-jazzy-teleop-twist-joy).
ros2 launch jessica_robot jessica.launch.py hardware:=false
```

> The `hardware` launch argument defaults to `true`. Set `hardware:=false` to run
> only `jessica_chatbot` + `hair_led_node`, exactly like before the motion stack
> was added.

Alternatively, run the two nodes directly (no launch file, no rebuild needed):

```bash
ros2 run jessica_robot hair_led_node      # terminal 1
ros2 run jessica_robot jessica_chatbot    # terminal 2
```

---

## Enabling SPI (Ubuntu — no raspi-config)

Add to `/boot/firmware/config.txt`:
```
dtparam=spi=on
```
Reboot, then verify with:
```bash
ls /dev/spi*
# should show /dev/spidev0.0  /dev/spidev0.1
```

Add user to spi group to avoid sudo:
```bash
sudo usermod -aG spi $USER
```

---

## LED Node (`hair_led_node`)

Subscribes to `/jessica/hair_hue` (Int32).

| Value | Colour |
|-------|--------|
| 0 | red |
| 25 | orange |
| 60 | yellow |
| 120 | green |
| 180 | cyan |
| 240 | blue |
| 270 | purple |
| 300 | magenta |
| 340 | pink |
| -1 | white |
| -2 | rainbow (static, one colour per LED) |

Uses `pi5neo` (not `rpi_ws281x`, which does not support RPi 5) with
`EPixelType.RGB` colour order.

Test command:
```bash
ros2 topic pub --once /jessica/hair_hue std_msgs/msg/Int32 "{data: 240}"
```

---

## Chatbot Node (`jessica_chatbot`)

### Conversation flow

```
IDLE
  │  (listening continuously for any speech)
  │  speech detected
  ▼
CONVERSATION
  │  process turn → Ollama → TTS → speak
  │
  ├─ "bye jessica"/"goodbye jessica" → farewell → DORMANT
  ├─ 30s silence                     → "I'll be here" → IDLE
  └─ (loop back for next turn)

DORMANT  (muted — listening but silent)
  │  ignores all speech…
  │  …until a wake phrase is heard
  ▼
CONVERSATION  (the wake utterance becomes the first turn)
```

### DORMANT state (mute on demand)

Saying **"bye jessica"** (or "goodbye jessica") puts Jessica into `DORMANT`:
she keeps listening but **stays completely silent** and ignores everything
until she is directly addressed again. This lets you mute her mid-session —
e.g. when a real-world conversation starts — without shutting her down.

She wakes from `DORMANT` only on a **wake phrase** (`WAKE_PHRASES`):

| Wake phrase | Result |
|-------------|--------|
| `jessica darling …` | Wakes **and** runs the rest of the sentence (so `"Jessica darling, change your hair to blue"` wakes her and changes the hair in one go). |
| `hello jessica` | Wakes and responds normally. |
| `hi jessica` | Wakes and responds normally. |

The wake utterance is passed straight into the conversation, so a command
spoken while waking executes immediately.

> Note: this differs from `IDLE` (startup / after a 30 s timeout), which still
> wakes on **any** speech. Only an explicit "bye jessica" triggers the muted
> `DORMANT` state. A future Porcupine wake word will replace this manual scheme.

### Key settings

| Constant | Default | Purpose |
|----------|---------|---------|
| `MAX_PHRASE_SECONDS` | 20 | Max recording length per turn |
| `CONVERSATION_TIMEOUT` | 30 | Silence (seconds) before ending conversation |
| `PAUSE_THRESHOLD` | 1.2 | Silence (seconds) = end of phrase |

Increase `PAUSE_THRESHOLD` if Jessica cuts you off mid-sentence.

### Robot command trigger

Jessica only executes hardware commands when Jonny says **"Jessica darling"**
at the start of the request, e.g.:
- *"Jessica darling, change your hair to purple."*
- *"Jessica darling, turn left."*

Without "Jessica darling", she treats the message as conversation only.

---

## Planned: Wake Word with Porcupine

Currently the chatbot wakes on any speech. To add a proper wake word
("hello Jessica darling" or similar) using Porcupine:

### 1 — Create the wake word file

1. Go to [https://console.picovoice.ai](https://console.picovoice.ai) (free account).
2. Navigate to **Porcupine** → **Wake Word**.
3. Type your phrase, e.g. `hello jessica darling`.
4. Select platform: **Raspberry Pi**.
5. Download the `.ppn` file, save to e.g. `/home/jonny/jessica_ws/wake_word/hello_jessica_darling_raspberry-pi.ppn`.

### 2 — Install the library

```bash
pip install pvporcupine
```

### 3 — Get your API key

Free-tier API key from the Picovoice console. Add to the chatbot config:

```python
PORCUPINE_API_KEY  = "your-key-here"
PORCUPINE_KEYWORD  = "/home/jonny/jessica_ws/wake_word/hello_jessica_darling_raspberry-pi.ppn"
PORCUPINE_SENSITIVITY = 0.6  # 0.0 (fewer false triggers) – 1.0 (fewer misses)
```

### 4 — Replace IDLE state in the conversation loop

The current IDLE state uses `speech_recognition`'s `listen()`. Replace it
with a Porcupine listen loop:

```python
import pvporcupine
import pyaudio
import struct

def wait_for_wake_word() -> None:
    """Block until the wake word is detected."""
    porcupine = pvporcupine.create(
        access_key=PORCUPINE_API_KEY,
        keyword_paths=[PORCUPINE_KEYWORD],
        sensitivities=[PORCUPINE_SENSITIVITY],
    )
    pa     = pyaudio.PyAudio()
    stream = pa.open(
        rate=porcupine.sample_rate,
        channels=1,
        format=pyaudio.paInt16,
        input=True,
        input_device_index=find_mic_index(),
        frames_per_buffer=porcupine.frame_length,
    )

    print("[IDLE] Listening for wake word...")
    try:
        while True:
            pcm    = stream.read(porcupine.frame_length, exception_on_overflow=False)
            pcm    = struct.unpack_from("h" * porcupine.frame_length, pcm)
            result = porcupine.process(pcm)
            if result >= 0:
                print("Wake word detected.")
                return
    finally:
        stream.stop_stream()
        stream.close()
        pa.terminate()
        porcupine.delete()
```

Then in `main()`, replace the IDLE block:

```python
if state == IDLE:
    wait_for_wake_word()
    speak("Yes love?", mp3_path)
    state        = CONVERSATION
    conversation = []
```

### 5 — Move chatbot into the launch file

Once the wake word is in place, `jessica_chatbot` no longer needs `input()`
and can be added back to `jessica.launch.py`:

```python
make_node("jessica_chatbot"),
```

### Notes

- The free Porcupine licence requires re-downloading the `.ppn` file every
  3 months (same key, same file — just refresh from the console).
- Sensitivity of 0.5–0.7 is a good starting point. Lower it if it triggers
  on background speech; raise it if it misses you.
- Porcupine runs locally on-device with negligible CPU — no API calls during
  idle listening.

---

## Motion & Head Control (ros2_control)  — IMPLEMENTED

Jessica's drive base and pan-tilt head are driven by `ros2_control` on an ESP32
(package `esp32_combined_hardware`). The chatbot's `drive`/`turn`/`look`/`nod`/
`shake_head`/`wave` commands and the joystick (manual override) both feed this
stack.

**See [`motion_and_head_control.md`](motion_and_head_control.md)** for full
detail: the ESP32 serial protocol, hardware interface, controllers, joystick and
chatbot command paths, the URDF, the launch file, build/run, tuning, and
troubleshooting.

Quick facts:
- 6 joints: 4 wheels (velocity) + pan/tilt servos (position), one serial link.
- Servos are open-loop — state mirrors the last command so JTC tolerances pass.
- `diff_cont` (DiffDriveController) + `pan_tilt_controller` (JointTrajectoryController)
  + `joint_broad` (JointStateBroadcaster).
- Joystick beats chatbot for driving via `twist_mux` priority.
- First-run TODO: `sudo apt install ros-jazzy-teleop-twist-joy` and a udev rule
  for `/dev/esp32_motor`.

## Still future

- **Cameras** — likely `image_transport` + a camera driver node
- **Lidar** — standard ROS 2 lidar driver publishing `sensor_msgs/LaserScan`

---

*Last updated: 2026-06-28*
