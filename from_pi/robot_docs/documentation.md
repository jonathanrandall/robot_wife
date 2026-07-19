# Jessica Robot — Project Documentation

Jessica is an AI companion robot on a Raspberry Pi 5: voice conversation via a
local LLM (Ollama on the PC), speech recognition, TTS, and hardware control
through ROS 2 / ros2_control — 4 mecanum-style wheels + a pan-tilt head on two
ESP32s, a stereo USB camera, a front ToF depth camera, WS2811 hair LEDs, and a
gamepad for manual override.

Contents:
1. [Launching](#1-launching)
2. [Topics — full list + commanding by topic](#2-topics)
3. [Packages](#3-packages)
4. [Appendices](#4-appendices) (Python env, hardware, audio, logging/"dreaming",
   wake-word plan, troubleshooting)

---

## 1. Launching

### Build first

```bash
cd ~/jessica_ws && colcon build && source install/setup.bash
```

(ROS Jazzy is already sourced in every shell; only the workspace overlay needs
sourcing after a build.)

### Touchscreen launcher (no SSH needed)

`~/jessica_ws/launcher/jessica_launcher.py` (outside the ROS packages — it
only spawns/stops `ros2 launch`) shows three big buttons on the Waveshare:
**Start Jessica** (full robot), **Chatbot Only** (`hardware:=false
camera:=false tof:=false`), **Drive Mode** (`chatbot:=false`). While a stack
runs, **hold a finger anywhere for 3 s** to stop it and return to the menu.
Small corner buttons: **Exit** (quit the launcher) and **Shutdown Pi** (safe
power-off) — both go through a confirm screen with "Back to menu". Shutdown
tries `systemctl poweroff`, then `sudo -n shutdown`; passwordless shutdown
needs a one-time sudoers rule (see `launcher/jessica_launcher.py` header).
Stack output goes to `~/jessica_ws/logs/launcher_stack.log`. For start-on-boot,
install `launcher/jessica-launcher.service` (instructions inside the file).

### The full robot

```bash
ros2 launch jessica_robot jessica.launch.py
```

Brings up **everything**: robot_state_publisher, controller_manager with both
hardware interfaces (wheels ESP32 + servo-head ESP32), `diff_cont`,
`pan_tilt_controller`, `joint_broad`, head homing, joystick stack, twist_mux,
stereo camera publisher, ToF depth publisher, finger/person followers,
stop-gesture watcher, chatbot, and hair LEDs.

Launch arguments (all optional):

| Argument | Default | Effect |
|---|---|---|
| `hardware:=false` | `true` | Skip ESP32s / controllers / joystick / followers. Chatbot + LEDs + cameras only — use when the robot hardware isn't connected, otherwise the launch spams errors. |
| `camera:=false` | `true` | Skip the USB stereo camera publisher (separate USB device, gated independently of `hardware`). |
| `tof:=false` | `true` | Skip the front ToF depth camera publisher (CSI device, independent of the others). |
| `display:=false` | `true` | Skip the touchscreen UI (Waveshare 7" on HDMI). |
| `chatbot:=false` | `true` | Skip the voice chatbot — drive-only mode (gamepad + manual topic publishing; mic and speakers untouched). |
| `gamepad_mode:=xinput\|dinput` | `auto` | Force a gamepad profile instead of auto-detecting from the driver. |

Examples:

```bash
# Bench: no ESP32s connected, but cameras + chatbot still useful
ros2 launch jessica_robot jessica.launch.py hardware:=false

# Everything except the ToF camera
ros2 launch jessica_robot jessica.launch.py tof:=false
```

### Launching packages individually

Each sensor/utility package can run on its own (useful for testing one thing
without the whole stack):

```bash
# Stereo USB camera → /jessica/camera/image/compressed (~22 Hz JPEG)
ros2 launch camera_publisher camera_publisher.launch.py
#   or: ros2 run camera_publisher webcam_publisher

# ToF depth camera → /jessica/tof/image/compressed (10 Hz PNG grayscale)
ros2 launch tof_publisher tof_publisher.launch.py
#   or: ros2 run tof_publisher tof_publisher

# Touchscreen UI (listening text / talking waves / touch out)
ros2 launch jessica_display jessica_display.launch.py
#   or: ros2 run jessica_display display_node

# Chatbot + LEDs without the launch file
ros2 run jessica_robot jessica_chatbot     # terminal 1
ros2 run jessica_robot hair_led_node       # terminal 2

# Followers / gesture watcher standalone (need the hardware stack + PC vision)
ros2 run jessica_robot finger_follower                                  # tracks immediately
ros2 run jessica_robot finger_follower --ros-args -p start_enabled:=false  # wait for enable
ros2 run jessica_robot person_follower
ros2 run jessica_robot stop_gesture
```

The motion stack (controller_manager + controllers) is not practical to start
by hand — use `jessica.launch.py` (with `camera:=false tof:=false` if you only
want motion). Head direction sanity test (stack must be running):

```bash
bash ~/jessica_ws/temp/test_head_directions.sh
```

### On the PC (vision + display)

```bash
ros2 launch stereo_pose_publisher stereo_pose.launch.py   # hand/person state from the stereo feed
rqt                                                       # Image View for the camera topics
```

---

## 2. Topics

### Command topics — controlling Jessica by topic instead of voice

| Topic | Type | What it does |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Drive the base (autonomous input to twist_mux, priority 10) |
| `/cmd_vel_joy` | `geometry_msgs/Twist` | Joystick's input to twist_mux (priority 100 — beats `/cmd_vel`) |
| `/pan_tilt_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Move the head (pan/tilt, radians) |
| `/jessica/hair_hue` | `std_msgs/Int32` | Hair LED colour (hue 0-359, -1 white, -2 rainbow) |
| `/jessica/finger_follow/enable` | `std_msgs/Bool` | Head-tracks-fingertip mode on/off |
| `/jessica/person_follow/enable` | `std_msgs/Bool` | Follow-me (base + head) mode on/off |
| `/jessica/stop` | `std_msgs/Empty` | General stop: halts base, cancels timed moves, disables both followers |
| `/esp32_aux_cmd` | `std_msgs/String` | Raw command to the motor ESP32: `estop` / `estopclear` |
| `/jessica/ui_state` | `std_msgs/String` | Touchscreen mode: `listening` / `talking` / `idle` |
| `/jessica/speech_env` | `std_msgs/Float32MultiArray` | TTS speech envelope for the talking waves: `[frame_s, level0, level1, …]`, levels 0-1 |

**Drive the base.** `diff_cont` has a 0.5 s command timeout (and twist_mux the
same), so a single `--once` publish only moves the robot for half a second —
stream with `-r` and Ctrl+C to stop:

```bash
# forward at 0.15 m/s until Ctrl+C
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}"

# turn on the spot, 0.8 rad/s left
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.8}}"
```

**Move the head.** Positions are radians, `[pan, tilt]`. Conventions (REP 103):
**+pan = left, +tilt = down**. Pan range ±2.36 rad (±135°), tilt limited to
−1.3 … 0.87 rad. `time_from_start` sets the move duration:

```bash
# look 45° left over 1.5 s
ros2 topic pub --once /pan_tilt_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['pan_joint', 'tilt_joint'],
    points: [{positions: [0.785, 0.0], time_from_start: {sec: 1, nanosec: 500000000}}]}"

# back to centre
ros2 topic pub --once /pan_tilt_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['pan_joint', 'tilt_joint'],
    points: [{positions: [0.0, 0.0], time_from_start: {sec: 1}}]}"
```

**Hair colour** (see the hue table under hair_led_node):

```bash
ros2 topic pub --once /jessica/hair_hue std_msgs/msg/Int32 "{data: 240}"   # blue
ros2 topic pub --once /jessica/hair_hue std_msgs/msg/Int32 "{data: -2}"    # rainbow
```

**Touchscreen states** (normally driven by the chatbot; handy for testing):

```bash
ros2 topic pub --once /jessica/ui_state std_msgs/msg/String "{data: listening}"
ros2 topic pub --once /jessica/ui_state std_msgs/msg/String "{data: idle}"
```

**Follower modes on/off:**

```bash
ros2 topic pub --once /jessica/finger_follow/enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /jessica/person_follow/enable std_msgs/msg/Bool "{data: false}"
```

**Stop everything** (same effect as the both-arms-raised gesture):

```bash
ros2 topic pub --once /jessica/stop std_msgs/msg/Empty "{}"
```

**Motor e-stop** (firmware-level, wheels only):

```bash
ros2 topic pub --once /esp32_aux_cmd std_msgs/msg/String "{data: estop}"
ros2 topic pub --once /esp32_aux_cmd std_msgs/msg/String "{data: estopclear}"
```

> Gotcha: `ros2 topic pub --once` blocks forever if there is **no subscriber**
> (e.g. the stack isn't running). If a command "freezes", check the stack is up.

### Sensor / state topics — published by the robot

| Topic | Type | Source | Notes |
|---|---|---|---|
| `/jessica/camera/image/compressed` | `sensor_msgs/CompressedImage` | webcam_publisher | Stereo side-by-side, 640×240 JPEG, ~22 Hz — feeds the PC vision |
| `/jessica/tof/image/compressed` | `sensor_msgs/CompressedImage` | tof_publisher | 240×180 mono8 PNG, 10 Hz. Gray = (2000 − depth_mm)·255/2000: 255 = at the lens, 0 = ≥2 m or invalid |
| `/joint_states` | `sensor_msgs/JointState` | joint_broad | All 6 joints (4 wheels + pan/tilt), 30 Hz — **the** source of truth for head pose |
| `/diff_cont/odom` | `nav_msgs/Odometry` | diff_cont | Wheel odometry |
| `/joy` | `sensor_msgs/Joy` | joy_node | Raw gamepad |
| `/jessica/touch` | `geometry_msgs/Point` | jessica_display | Touchscreen taps, x/y in pixels (1024×600) — for future on-screen controls |

Quick checks: `ros2 topic hz /jessica/tof/image/compressed`,
`ros2 topic echo /joint_states --once`. On the PC, view the camera topics in
`rqt` → Plugins → Visualization → Image View.

### Topics published by the PC (vision)

| Topic | Type | Notes |
|---|---|---|
| `/jessica/hand_state` | `person_state_msgs/HandState` | Index fingertip + hand landmarks, 3-D camera optical frame (Z fwd, X right, Y down, m) |
| `/jessica/person_state` | `person_state_msgs/PersonState` | Shoulder midpoint + landmarks, pointing ray |
| `/jessica/camera/pose/compressed` | `sensor_msgs/CompressedImage` | Annotated vision stream (optional, for rqt) |

### Internal topics (don't publish to these by hand)

`/diff_cont/cmd_vel_unstamped` (twist_mux output) → `/diff_cont/cmd_vel`
(stamped, into the controller); `/robot_description`; `/dynamic_joint_states`.

---

## 3. Packages

```
jessica_ws/src/
├── jessica_robot/            # Nodes (chatbot, LEDs, followers), config, main launch
├── jessica_description/      # URDF/xacro + meshes + ros2_control config
├── esp32_combined_hardware/  # ros2_control hardware interface: 4 wheels (motor ESP32)
├── esp32_servo_hardware/     # ros2_control hardware interface: pan/tilt head (servo ESP32)
├── camera_publisher/         # USB stereo camera → compressed image topic
├── tof_publisher/            # Arducam ToF depth camera → grayscale depth topic
├── jessica_display/          # Waveshare 7" touchscreen UI (KMS/DRM, pygame)
└── person_state_msgs/        # HandState / PersonState message definitions (shared with PC)
```

### jessica_robot

Owns the top-level launch (`jessica.launch.py`), all config yaml
(controllers, twist_mux, gamepad profiles) and Jessica's Python nodes:

- **jessica_chatbot** — the brain. Voice loop (arecord → Whisper on the PC →
  Ollama → Piper TTS), robot-command parsing and execution, conversation
  logging. Detailed behaviour in [Appendix: Chatbot](#chatbot-behaviour).
- **hair_led_node** — WS2811 hair LEDs, subscribes `/jessica/hair_hue`
  (`pi5neo` via SPI; `rpi_ws281x` doesn't support the Pi 5). Hue values:

  | Value | Colour | | Value | Colour |
  |---|---|---|---|---|
  | 0 | red | | 240 | blue |
  | 25 | orange | | 270 | purple |
  | 60 | yellow | | 300 | magenta |
  | 120 | green | | 340 | pink |
  | 180 | cyan | | -1 / -2 | white / rainbow |

- **pan_tilt_teleop** — gamepad stick → head trajectories.
- **finger_follower** — head visual-servo on the raised index fingertip from
  `/jessica/hand_state`. Gated by `/jessica/finger_follow/enable`; comes up
  disabled in the launch. 20 Hz P-loop; tune with
  `ros2 param set /finger_follower pan_gain 0.0015` etc.
  (params: `pan_gain`/`tilt_gain` 0.0015, `pan_sign`/`tilt_sign` −1.0,
  `deadband_px` 8, `max_step_rad` 0.12, `control_rate` 20, `lost_timeout` 0.7).
- **person_follower** — "follow me": head servo on the shoulder midpoint from
  `/jessica/person_state` + base P-control holding 0.70 m distance
  (never reverses toward you, stops inside 0.5 m). Steering = current pan
  angle + in-image offset. Auto-stops when you turn to face the robot
  (shoulder x-order flips) or raise an open palm (only checked while
  following). Gated by `/jessica/person_follow/enable`, starts disabled.
  Joystick always overrides via twist_mux priority. Side-offset modes are
  planned but only `behind` is implemented.
- **stop_gesture** — always-on watcher: both wrists above the shoulders for
  ~0.4 s → one `std_msgs/Empty` on `/jessica/stop`. Every mover (chatbot,
  both followers) subscribes and stops itself — no single point of failure,
  and it runs on the spin thread so it works even while Jessica is
  recording/speaking (gesture-to-stopped well under 0.5 s). Tune:
  `ros2 param set /stop_gesture wrist_margin 0.08`, `hold_s`, `refire_s`.

### jessica_description

URDF/xacro (`description/jessica.urdf.xacro`) + gobilda meshes. The
`ros2_control.xacro` declares **two** hardware systems in one controller
manager: `JessicaESP32` (wheels, `/dev/esp32_motor`) and `JessicaServoHead`
(head, `/dev/esp32_servo`). Tilt command limits −1.3 … 0.87 rad (servo
firmware clamps at ±1.32).

### esp32_combined_hardware

C++ `SystemInterface` for the **motor ESP32** (`/dev/esp32_motor`, ESP32-S3):
4 wheel joints, velocity command + position/velocity state.
Line protocol at 115200 baud:

- New firmware: `STATE,4×pos,4×vel,4×encoder-ok-flags` / `CMD,lf,lr,rf,rr` (cm/s).
- Old firmware (currently flashed — see issues.md §2): 10-field STATE with
  trailing pan/tilt; auto-detected, falls back to 6-field CMD.
- Encoder-ok flag going 0 logs an ERROR naming the wheel; policy TBD
  (issues.md §1).
- Also subscribes `/esp32_aux_cmd` (estop/estopclear pass-through).

Includes **joy_button_bridge**: gamepad buttons → estop/estopclear on
`/esp32_aux_cmd` and head-centre via `/pan_tilt_controller/joint_trajectory`.

Full protocol/tuning/troubleshooting detail:
[`motion_and_head_control.md`](motion_and_head_control.md).

### esp32_servo_hardware

C++ `SystemInterface` for the **servo ESP32** (`/dev/esp32_servo`, hiwonder
bus servos, MicroPython firmware in `temp/micropython_servo_control/`):
pan/tilt position command + state. Protocol: text commands
(`ptr <pan_rad> <tilt_rad> [ms]`, `pos`), one JSON JointState-style reply per
command; `null` position = failed servo read → interface falls back to
open-loop (reports commanded position) with a warning.

Key params (set in `ros2_control.xacro`): `min_send_period_ms` 100 (the board
UART overflows at the full 30 Hz loop rate), `boot_wait_ms` 3000 (the CH340
resets the board on port open → it reboots and homes the head — every
controller_manager start recentres the head), `move_ms` 100,
`state_refresh_ms` 1000.

**Direction conventions live in the firmware**, not ROS: a mirrored axis is
fixed in `config.py`'s `DIRECTIONS` map on the board (upload via
`~/venvs/jazzy/bin/mpremote connect /dev/esp32_servo cp config.py :config.py + reset`).

### camera_publisher

Python node `webcam_publisher`: finds the "3D USB Camera" by name
(`v4l2-ctl --list-devices`), captures the side-by-side stereo frame, halves it
to 640×240 and publishes JPEG (quality 80) on
`/jessica/camera/image/compressed` at ~22 Hz (rate lowered from 30 to stop
JPEG encoding starving the mic).

### tof_publisher

Python node `tof_publisher`: Arducam ToF depth camera on CSI
(`ArducamDepthCamera` SDK from the venv). Publishes
`/jessica/tof/image/compressed` — 240×180 mono8 PNG (lossless) at 10 Hz.
Depth→gray mapping (max distance 2000 mm, and the camera's RANGE control is
set to 2000 for best precision):

```
gray = (2000 - depth_mm) * 255 / 2000      # 255 = 0 mm, 0 = ≥2000 mm
```

Invalid pixels (no return / NaN) are forced to 0 (= far), so a dropout never
looks like an obstacle touching the lens. Intended consumers: rqt display now,
PC-side navigation / obstacle avoidance later.

⚠️ The camera is **single-client** — close `preview_jr.py` (in
`~/tof_test/Arducam_tof_camera/`) before launching the stack, or `start()`
spins forever. If the sensor vanishes (dmesg: `arducam-pivariety 10-000c:
probe failed`), reseat the CSI ribbon; see issues.md §4.

### jessica_display

Python node `display_node`: Jessica's face on the Waveshare 7" touchscreen
(1024×600 over HDMI, touch over USB). Renders **directly via KMS/DRM** with
pygame — no desktop session needed, works alongside SSH.

- `listening` → big "Listening..." text cycling the hue circle (6 s/rev).
- `talking` → 4 colourful overlapping soundwaves whose amplitude follows the
  **real TTS loudness**: the chatbot computes the RMS envelope of each Piper
  wav (50 ms frames, peak-normalised) and publishes it on
  `/jessica/speech_env` immediately before `aplay` starts, so the waves move
  in sync with her voice.
- `idle` → dim breathing dot at low frame rate (near-zero CPU).
- Touches are published on `/jessica/touch` (`geometry_msgs/Point`, pixel
  coords) for future on-screen controls.

State transitions come from the chatbot on `/jessica/ui_state`:
`record_speech()` publishes `listening` when the mic opens and `idle` when it
closes; `play_wav()` publishes `talking` before playback and `idle` after.

CPU: ~15 % of one core while animating (25 fps), ~nothing when idle.
**pygame must be the system one** (`sudo apt install python3-pygame`) — the
pip wheel's bundled SDL lacks the KMS/DRM driver ("kmsdrm not available").
The node appends `/usr/lib/python3/dist-packages` to its path for this.

### person_state_msgs

`HandState` / `PersonState` message definitions, shared with the PC's
`stereo_pose_publisher`. Build note: needs `empy==3.3.4` in the venv
(`~/venvs/jazzy/bin/pip install "empy==3.3.4"`) — empy 4.x breaks rosidl.

---

## 4. Appendices

### Hardware

| Component | Details |
|-----------|---------|
| Computer | Raspberry Pi 5, Ubuntu |
| Motor controller | ESP32-S3 → `/dev/esp32_motor` (udev: 303a:1001) — 4 wheels + encoders |
| Servo controller | ESP32 dev board → `/dev/esp32_servo` (udev: CH340 1a86:7523) — hiwonder bus servos, pan ID 6 / tilt ID 5 |
| Stereo camera | "3D USB Camera", side-by-side frames |
| ToF camera | Arducam ToF, CSI (cam0), 240×180, I2C 10-000c |
| LED strip | 5× WS2811, RGB order, SPI GPIO 10 (pin 19) |
| Speaker / Mic | USB (ALSA cards `Audio` / `Device`) |
| LLM server | Laptop running Ollama at `192.168.1.106:11434` |

### Python environment

ROS 2 Jazzy uses the system Python. **All** extra Python packages go in the
venv at `~/venvs/jazzy/` (`~/venvs/jazzy/bin/pip install …`). Because
`ros2 run` bypasses the venv, each node that needs venv packages starts with:

```python
import sys
sys.path.insert(0, '/home/jonny/venvs/jazzy/lib/python3.12/site-packages')
```

### Enabling SPI (Ubuntu, no raspi-config)

`/boot/firmware/config.txt`: `dtparam=spi=on`, reboot, verify `ls /dev/spi*`.
Avoid sudo: `sudo usermod -aG spi $USER`.

### Chatbot behaviour

#### Conversation flow

```
IDLE
  │  (listening continuously for any speech)
  │  speech detected
  ▼
CONVERSATION
  │  process turn → Ollama → TTS → speak
  ├─ "bye jessica"/"goodbye jessica" → farewell → DORMANT
  ├─ 30s silence                     → "I'll be here" → IDLE
  └─ (loop back for next turn)

DORMANT  (muted — listening but silent)
  │  ignores all speech until a wake phrase
  ▼
CONVERSATION  (the wake utterance becomes the first turn)
```

**DORMANT (mute on demand):** "bye jessica" mutes her without shutting down.
Wake phrases: `jessica darling …` (runs the rest of the sentence too),
`hello jessica`, `hi jessica`. Matching is punctuation-proof (`_normalise()`).

**Command gating:** hardware commands only execute when the transcript
contains **"Jessica darling"** or **"Hey Jessica"** — enforced in code
(`_gate_robot_command`), because the LLM sometimes hallucinates commands from
casual chat. Exception: any sentence containing "jessica" + "stop" always
stops.

#### Robot command list (voice)

| Action | Parameters | Notes |
|---|---|---|
| `stop` | — | Halts base + cancels timed moves + followers off |
| `drive` | fwd/back, `duration_s` 0.1–8 | Timed, non-blocking (background thread) |
| `turn` | left/right, duration | Timed |
| `twirl` | 1–3 rotations, direction | Open-loop, `TWIRL_SPEED` 1.2 rad/s |
| `dance` | — | ~14 s canned routine |
| `look`/`nod`/`shake_head`/`wave` | direction for look | Relative to current pose from `/joint_states` |
| `change_hair_color` | colour | |
| `follow_finger` / `follow_me` | on/off | Publishes the enable flags — LLM is never in the control loop |

Timed moves never block the mic; a new move cancels the previous; voice
"Jessica stop", the joystick, and the both-arms gesture all interrupt.

#### Key settings

| Constant | Default | Purpose |
|---|---|---|
| `MAX_PHRASE_SECONDS` | 20 | Max recording per turn |
| `CONVERSATION_TIMEOUT` | 30 | Silence before conversation ends |
| `PAUSE_THRESHOLD` | 1.2 | Silence = end of phrase (raise if she cuts you off) |

### Audio capture (mic)

Captured with an **`arecord` subprocess** (not sounddevice/PortAudio):
`arecord -D plughw:CARD=Device,DEV=0 -f S16_LE -c1 -t raw`, 50 ms blocks,
energy-based end-of-phrase detection. PortAudio's callback got starved under
load (camera + vision + TTS) and stalled the stream; a separate process just
buffers in the pipe instead. Tuning: `ENERGY_MARGIN` / calibrated threshold
for speech-vs-silence, `PAUSE_THRESHOLD` for phrase end.

### Conversation logging & feedback ("dreaming")

Every turn → `~/jessica_ws/logs/jessica_YYYY-MM-DD.jsonl` (best-effort,
can never break a conversation). Spoken grading right after an action:
"good girl" / "well done" / … → `label: good`; "that was wrong" / … →
`label: bad` with the words kept as `note`. Feedback entries are
self-contained (carry the original input/output) so they can become training
pairs directly.

`tools/build_examples.py` pairs feedback with turns:

```bash
python3 src/jessica_robot/tools/build_examples.py            # logs/examples.jsonl
python3 src/jessica_robot/tools/build_examples.py --prompt   # few-shot text of the good ones
python3 src/jessica_robot/tools/build_examples.py --label bad --since 2026-07-01
```

Workflow: fold `good` examples into `SYSTEM_PROMPT` as few-shot (biggest win),
review `bad` ones and add corrections; the accumulated JSONL seeds a future
QLoRA fine-tune.

Quick look: `grep '"label":"bad"' ~/jessica_ws/logs/jessica_$(date +%F).jsonl`

### Planned: Porcupine wake word

Replace the IDLE any-speech wake with a proper wake word: create a `.ppn` at
console.picovoice.ai (platform Raspberry Pi), `pip install pvporcupine` (into
the venv), add key + keyword path to the chatbot config, and swap the IDLE
listen for a `porcupine.process()` loop. Free tier: re-download the `.ppn`
every 3 months. Sensitivity 0.5–0.7 to start. Porcupine runs locally with
negligible CPU. Code sketch:

```python
import pvporcupine, pyaudio, struct

def wait_for_wake_word() -> None:
    porcupine = pvporcupine.create(
        access_key=PORCUPINE_API_KEY,
        keyword_paths=[PORCUPINE_KEYWORD],      # path to the .ppn
        sensitivities=[PORCUPINE_SENSITIVITY],  # 0.6 to start
    )
    pa = pyaudio.PyAudio()
    stream = pa.open(rate=porcupine.sample_rate, channels=1,
                     format=pyaudio.paInt16, input=True,
                     input_device_index=find_mic_index(),
                     frames_per_buffer=porcupine.frame_length)
    try:
        while True:
            pcm = stream.read(porcupine.frame_length, exception_on_overflow=False)
            pcm = struct.unpack_from("h" * porcupine.frame_length, pcm)
            if porcupine.process(pcm) >= 0:
                return
    finally:
        stream.stop_stream(); stream.close(); pa.terminate(); porcupine.delete()
```

### Troubleshooting

- **A chatbot feature "regressed" (e.g. goodbye ignored)** → almost always a
  stale duplicate process running old code and competing for the mic:
  `pgrep -af jessica_chatbot` (should be exactly one), `pkill -f jessica_chatbot`.
- **`ros2 topic pub --once` freezes** → no subscriber; the stack isn't up.
- **Launch dies with serial errors** → check `/dev/esp32_motor` and
  `/dev/esp32_servo` exist (udev rules in `/etc/udev/rules.d/`); see
  `read_this.md` and issues.md.
- **ToF topic missing** → camera single-client (close preview_jr.py) or CSI
  ribbon unseated (issues.md §4).
- **Head moves the wrong way** → fix the sign in the servo firmware's
  `config.py` `DIRECTIONS` map, **never** in ROS
  (`temp/test_head_directions.sh` to verify).
- **Killing nodes by hand** → `ros2 run` is a wrapper; killing its PID orphans
  the real node. Kill the exact child PID, and beware `pkill -f` matching your
  own shell.

---

*Last updated: 2026-07-15*
