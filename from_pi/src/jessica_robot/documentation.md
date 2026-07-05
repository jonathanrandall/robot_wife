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

# Skip the USB stereo camera publisher (e.g. camera unplugged):
ros2 launch jessica_robot jessica.launch.py camera:=false
```

The `camera` launch argument (default `true`) starts the `webcam_publisher`,
which feeds `/jessica/camera/image/compressed` to the PC vision node. It's a
separate USB device from the ESP32, so it's gated independently of `hardware` —
you can run the camera with `hardware:=false`, or skip it with `camera:=false`.

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

Phrase matching is punctuation-proof: `is_farewell()` / `is_wake_phrase()` both
run the transcript through `_normalise()` (lowercase + collapse punctuation to
spaces), so Whisper output like `"Goodbye, Jessica."` still matches.

#### Troubleshooting: "the DORMANT / goodbye feature stopped working"

Almost always a **stale duplicate chatbot process**, not a code bug. `ros2 launch`
Ctrl-C doesn't always reap its children, so a previous `jessica_chatbot` can keep
running — competing for the microphone and, if it started before the latest
build, executing **old code**. That old instance won't have the current matching
logic, so goodbyes appear to be ignored.

Check and clear before relaunching:

```bash
pgrep -af jessica_chatbot   # should be empty before launch, exactly ONE while running
pkill -f jessica_chatbot    # kill any leftover instance
```

Rule of thumb: if any chatbot feature seems to "regress," check for duplicate
processes **first**.

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

## Conversation Logging & Feedback ("Dreaming")

Every conversation turn is logged to a per-day JSONL file so the prompts and
command-handling can be reviewed and refined offline, and so good/bad examples
can be collected to teach Jessica.

**Location:** `~/jessica_ws/logs/jessica_YYYY-MM-DD.jsonl` (auto-created).
**Best-effort:** all logging is wrapped in try/except — a logging failure can
never break a conversation.

### `turn` entries

One per conversation turn, written at the end of `process_turn()`:

```json
{"id":"20260703T104725591368","type":"turn","ts":"2026-07-03T10:47:25",
 "trigger":"conversation","heard":"Jessica darling, turn left",
 "llm_raw":"{\"say\":\"Of course love\",\"robot_command\":{...}}",
 "reply_spoken":"Of course love","action":"turn",
 "params":{"direction":"left"},"executed":true}
```

| Field | Meaning |
|-------|---------|
| `id` | Unique turn id (used by feedback to reference it) |
| `trigger` | `idle` (first contact), `conversation`, or `wake` (from DORMANT) |
| `heard` | Whisper transcript of what Jonny said |
| `llm_raw` | Raw Ollama output **before** parsing (kept out of the prompt context) |
| `reply_spoken` | What Jessica actually said |
| `action` / `params` | The parsed robot command |
| `executed` | `true` unless the action was `none` |

### `feedback` entries — spoken correction / approval

Right after Jessica acts, Jonny can grade the previous turn out loud. The
feedback is **logged, acknowledged, and not run as a normal turn**:

- **Approve (👍):** "good girl", "well done", "good job", "that was perfect",
  "perfect jessica", "that was great" → logs `label: good`.
- **Correct (👎):** "that was wrong", "that's wrong", "wrong jessica",
  "you got it wrong", "that wasn't right" → logs `label: bad`, with the spoken
  words kept verbatim as `note` (e.g. *"…you should have turned right"*).

The phrase lists live in `APPROVAL_PHRASES` / `CORRECTION_PHRASES` in
`jessica_chatbot.py` — tune them to taste. Matching is punctuation-proof via
`_normalise()`, same as the wake/farewell phrases.

Each feedback entry is **self-contained** (carries the original input + output),
so it can become a training pair without joining files:

```json
{"id":"...","type":"feedback","ts":"...","ref":"20260703T104725591368",
 "label":"bad","note":"No, that was wrong, you should have looked right.",
 "orig_heard":"Jessica darling, look left",
 "orig_action":"look","orig_params":{"direction":"left"}}
```

`ref` points at the `id` of the turn being judged.

### Using the logs to improve Jessica

1. **Prompt refinement / few-shot (start here):** read the log, find `bad`
   feedback (or turns with wrong `action`/`params`), and fold the corrected
   input→output pairs into `SYSTEM_PROMPT` as few-shot examples. Biggest win for
   a small model, no weight training needed.
2. **Fine-tuning (later, optional):** the `feedback` entries are the seed of a
   real dataset — `bad` + `note` gives a correction pair, `good` marks confirmed
   examples. Curate a few hundred, then QLoRA (unsloth/axolotl) → GGUF → Ollama.

> Note: plain `turn` logging alone records what she *did* say, not what she
> *should* have — the `feedback` layer is what makes the logs trainable.

### Turning feedback into examples — `build_examples.py`

A small helper (`src/jessica_robot/tools/build_examples.py`, plain Python, no ROS
deps) scans the logs, pairs each `feedback` entry with the `turn` it graded (via
`ref` → `id`), and produces a clean examples file:

- **`good` feedback** → a confirmed `input → command` example, ready for few-shot.
- **`bad` feedback** → a correction to review: the input, what she did, and your
  spoken note, with a blank `corrected_command` for you to fill in.

```bash
cd ~/jessica_ws

# Write ~/jessica_ws/logs/examples.jsonl (good + bad):
python3 src/jessica_robot/tools/build_examples.py

# Print the confirmed (good) ones as ready-to-paste few-shot text:
python3 src/jessica_robot/tools/build_examples.py --prompt

# Only the corrections to review:
python3 src/jessica_robot/tools/build_examples.py --label bad

# Only logs on/after a date:
python3 src/jessica_robot/tools/build_examples.py --since 2026-07-01
```

Options: `--logs-dir DIR`, `--out FILE`, `--since YYYY-MM-DD`,
`--label good|bad|all`, `--prompt`.

Output line (JSONL mode):

```json
{"label":"good","input":"Jessica darling, tilt down",
 "model_command":{"action":"look","parameters":{"direction":"down"}},
 "model_say":"Tilting down now.","note":"Good girl, Jessica!",
 "command":{"action":"look","parameters":{"direction":"down"}}}
```

Recommended workflow:
1. Run with `--prompt`, paste the confirmed **good** examples into
   `SYSTEM_PROMPT` as few-shot (the quickest way to improve her).
2. Run with `--label bad`, work through each correction, fill in the
   `corrected_command`, and add those as few-shot examples too.
3. Keep the accumulated JSONL — it's the seed of a fine-tuning dataset later.

### Quick review

```bash
cat ~/jessica_ws/logs/jessica_$(date +%F).jsonl                 # today's log
# just the corrections:
grep '"label":"bad"' ~/jessica_ws/logs/jessica_$(date +%F).jsonl
```

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

## Audio capture (mic) — IMPLEMENTED

The mic is captured with an **`arecord` subprocess**, not `sounddevice`/PortAudio.
`record_speech()` spawns `arecord -D plughw:CARD=Device,DEV=0 -f S16_LE -c1 -t raw`
and reads 50 ms blocks off its stdout pipe, doing the same energy-based
silence detection (`PAUSE_THRESHOLD`, `MAX_PHRASE_SECONDS`) on the stream.
`calibrate_mic()` captures the same way.

**Why not sounddevice:** on this Pi under load (camera + vision + finger_follower
+ Piper TTS all running), the PortAudio capture callback got starved right after
playback and the stream stalled ("mic stream stalled"), plus intermittent input
overflows and transient "invalid sample rate" open failures. `arecord` is a
dedicated process, so a briefly starved main thread just leaves audio **buffered
in the pipe** instead of xrunning the capture — no stalls. Playback already uses
the same pattern (`aplay`), and `plughw` absorbs any 44100↔48000 rate mismatch.

Side benefit: because the old path dropped samples on overflow, transcription
quality effectively improves with clean, continuous audio (compounding the PC's
move to the larger `medium.en` Whisper model).

- Mic tuning: `ENERGY_MARGIN` / the calibrated `_speech_threshold` set the
  speech-vs-silence cutoff. Lower if it misses you; raise if it triggers on
  background noise. `PAUSE_THRESHOLD` is the silence gap that ends a phrase.

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

## Finger following (head tracking)

`finger_follower` (`jessica_robot/finger_follower.py`, run with
`ros2 run jessica_robot finger_follower`) makes the head track a raised index
fingertip — hold a finger up in front of the camera, move it around, the head
follows and keeps it centred. **Only the head moves.**

**How it works.** The robot PC runs MediaPipe on the stereo camera stream and
publishes `person_state_msgs/HandState` on `/jessica/hand_state`, containing the
index fingertip's 3-D position in the camera optical frame (Z forward, X right,
Y down, metres). The camera is mounted on the pan-tilt head, so this is a
closed visual-servo loop:

1. Pick the raised fingertip (whichever index tip is detected + `depth_valid`;
   if both hands, the higher one).
2. Back-project it to a pixel in the 320×240 eye image using the PC calibration
   (`f=185.05, cx=170.0, cy=132.6`) and measure the error from the image centre
   (160, 120): `pan_err = x_px-160` (+ve = finger to the right),
   `tilt_err = y_px-120` (+ve = finger below centre).
3. A 20 Hz proportional loop nudges the pan/tilt target to zero that error
   (per-cycle step clamped, target clamped to the joint limits) and publishes a
   short `JointTrajectory` to `/pan_tilt_controller/joint_trajectory`.
4. Because the camera turns with the head, the error shrinks as the head moves,
   so it converges on the fingertip. Finger lost for `lost_timeout` s → the head
   holds position until it reappears.

**Enable/disable.** The mode is gated by `std_msgs/Bool` on
`/jessica/finger_follow/enable`. `start_enabled` (param, default `true`) lets you
run the node standalone for testing.

**Voice control (via the chatbot).** The chatbot exposes a `follow_finger`
robot command, so you can turn tracking on/off by voice — the LLM only publishes
the enable flag, it is never in the control loop:

- *"Jessica darling, follow my finger."* → `follow_finger {state: on}` →
  publishes `True`.
- *"Jessica darling, stop following my finger."* → `follow_finger {state: off}`
  → publishes `False`.
- Any *"stop"* command (the base emergency-stop) **also** switches finger
  following off, so "stop" halts everything.

(As with all robot commands, it only fires when prefixed with "Jessica
darling".) You can also toggle it by hand for testing:

```bash
ros2 topic pub --once /jessica/finger_follow/enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /jessica/finger_follow/enable std_msgs/msg/Bool "{data: false}"
```

**Prerequisites**
- Hardware stack up (`jessica.launch.py`) so `pan_tilt_controller` consumes the
  trajectory.
- The PC publishing `/jessica/hand_state` (`stereo_pose_publisher` — see the
  PC's own `documentation.md`, mirrored in `temp/from_pc/`).
- `person_state_msgs` built on the Pi (it lives in `src/`; build with
  `colcon build --packages-select person_state_msgs`). Requires **`empy==3.3.4`**
  in the venv — MediaPipe/newer tooling pulls in empy 4.x which breaks the
  rosidl CMake build with `TransientParseError`. Fix:
  `~/venvs/jazzy/bin/pip install "empy==3.3.4"`.

**Running it on the robot**

1. **On the Pi** — bring up the hardware stack (motors + head controllers). This
   also starts the USB camera publisher automatically (`camera:=true` default),
   so `/jessica/camera/image/compressed` is published for the PC:
   ```bash
   cd ~/jessica_ws
   source /opt/ros/jazzy/setup.bash && source install/setup.bash
   ros2 launch jessica_robot jessica.launch.py
   ```
   Sanity-check the camera is live: `ros2 topic hz /jessica/camera/image/compressed`
   should read ~30 Hz. If it shows no publisher, the USB camera (`3D USB Camera`,
   `/dev/video0`) may be unplugged — `lsusb` and `v4l2-ctl --list-devices` confirm it.
2. **On the PC** — start the vision so `/jessica/hand_state` is published:
   ```bash
   ros2 launch stereo_pose_publisher stereo_pose.launch.py
   ```
   (Optional: watch the annotated stream in `rqt` on
   `/jessica/camera/pose/compressed`.)
3. **On the Pi** — start the follower. Two options:
   - **Standalone** (starts tracking immediately):
     ```bash
     ros2 run jessica_robot finger_follower
     ```
   - **Voice-gated** (start disabled, let the chatbot switch it on):
     ```bash
     ros2 run jessica_robot finger_follower --ros-args -p start_enabled:=false
     ```
     Then run the chatbot (`ros2 run jessica_robot jessica_chatbot`, or via the
     launch file) and say *"Jessica darling, follow my finger."*
4. Hold your index finger up in front of the camera and move it around — the
   head should follow and keep the fingertip centred. Say *"stop"* (or
   *"stop following my finger"*) to end.
5. **First run on hardware — check the direction.** If the head chases the
   finger the *wrong* way, flip the sign live (no rebuild) and re-test:
   ```bash
   ros2 param set /finger_follower pan_sign 1.0     # left/right reversed
   ros2 param set /finger_follower tilt_sign 1.0    # up/down reversed
   ```
   Once you know the correct signs, set them as the defaults in
   `finger_follower.py`.

**Tuning** (all `ros2 param set /finger_follower <name> <val>`, or edit the
`declare_parameter` defaults):

| Param | Default | Effect |
|---|---|---|
| `pan_gain` / `tilt_gain` | `0.0015` | rad of head move per px of error. Higher = snappier but can oscillate. |
| `pan_sign` / `tilt_sign` | `-1.0` | Flip if the head chases the finger the **wrong** way. |
| `deadband_px` | `8.0` | Ignore errors smaller than this (kills jitter at centre). |
| `max_step_rad` | `0.12` | Per-cycle clamp — caps head speed (~2.4 rad/s @20 Hz). |
| `control_rate` | `20.0` | Command output rate (Hz). |
| `lost_timeout` | `0.7` | Seconds without a fingertip before the head holds. |

Directions were verified offline with a synthetic `HandState`: a fingertip to
the right + below centre pans the head right and tilts it down (chasing the
finger). If the real robot moves the wrong way, flip `pan_sign`/`tilt_sign`.

## Still future

- **Follow-me / point-and-navigate** — the PC also publishes
  `/jessica/person_state` (shoulder midpoint, pointing ray). Same pattern:
  a mode node the LLM enables. See `temp/from_pc/documentation.md` Tasks 1–2.
- **Lidar** — standard ROS 2 lidar driver publishing `sensor_msgs/LaserScan`

---

*Last updated: 2026-07-04*
