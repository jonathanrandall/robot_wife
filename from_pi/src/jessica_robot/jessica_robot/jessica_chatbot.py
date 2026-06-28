#!/usr/bin/env python3
# Build and run:
#   cd ~/jessica_ws && colcon build && source install/setup.bash
#   ros2 launch jessica_robot jessica.launch.py   # hardware nodes
#   ros2 run jessica_robot jessica_chatbot        # chatbot (separate terminal)

import sys
sys.path.insert(0, '/home/jonny/venvs/jazzy/lib/python3.12/site-packages')

import ctypes
import json
import os
import subprocess
import tempfile
import time
from pathlib import Path

# Prevent PyAudio from trying to connect to Jack (causes hangs and spam).
os.environ.setdefault("JACK_NO_AUDIO_RESERVATION", "1")
os.environ.setdefault("JACK_START_SERVER", "0")
os.environ.setdefault("JACK_NO_START_SERVER", "1")

# Suppress ALSA error spam from PyAudio probing audio devices.
# The handler must be kept alive at module level to prevent garbage collection → segfault.
try:
    _ALSA_ERROR_HANDLER_FUNC = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p)
    _ALSA_ERROR_HANDLER = _ALSA_ERROR_HANDLER_FUNC(lambda *_: None)
    ctypes.cdll.LoadLibrary('libasound.so.2').snd_lib_error_set_handler(_ALSA_ERROR_HANDLER)
except Exception:
    pass

# Suppress Jack error/info spam (printed every time PyAudio opens a stream).
try:
    _JACK_LIB = ctypes.cdll.LoadLibrary('libjack.so.0')
    _JACK_MSG_FUNC = ctypes.CFUNCTYPE(None, ctypes.c_char_p)
    _JACK_MSG_HANDLER = _JACK_MSG_FUNC(lambda *_: None)
    _JACK_LIB.jack_set_error_function(_JACK_MSG_HANDLER)
    _JACK_LIB.jack_set_info_function(_JACK_MSG_HANDLER)
except Exception:
    pass

import io
import queue
import wave

import numpy as np
import requests
import sounddevice as sd
from piper.voice import PiperVoice

# from gtts import gTTS  # kept for reference — swap back if Piper is unavailable

try:
    import rclpy
    from std_msgs.msg import Int32
    from geometry_msgs.msg import Twist
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration as DurationMsg
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


# ---------------------------------------------------------------------
# Hair colour settings
# ---------------------------------------------------------------------

# Hue in degrees (0–359), S=100%, V=80% fixed.
# -1 = white (S=0), -2 = rainbow.
COLOR_TO_HUE = {
    "red":     0,
    "orange":  25,
    "yellow":  60,
    "green":   120,
    "cyan":    180,
    "blue":    240,
    "purple":  270,
    "magenta": 300,
    "pink":    340,
    "white":   -1,
    "rainbow": -2,
}

_ros_node    = None
_hair_pub    = None
_cmd_vel_pub = None   # geometry_msgs/Twist  -> /cmd_vel (autonomous channel)
_head_pub    = None   # trajectory_msgs/JointTrajectory -> /pan_tilt_controller

# Last commanded head pose (rad). Gestures build trajectories relative to this.
_head_pan:  float = 0.0
_head_tilt: float = 0.0

# Motion tuning — kept gentle (the system prompt forbids fast movement).
DRIVE_SPEED   = 0.15   # m/s
TURN_SPEED    = 0.6    # rad/s
HEAD_PAN_MAX  = 1.4    # rad, soft clamp
HEAD_TILT_UP  = 0.8    # rad (positive = up)
HEAD_TILT_DN  = -1.4   # rad (negative = down)


# ---------------------------------------------------------------------
# Network / model settings
# ---------------------------------------------------------------------

# Change this to your laptop's IP address.
OLLAMA_URL   = "http://192.168.1.106:11434/api/chat"
OLLAMA_MODEL = "llama3.2:3b"

WHISPER_URL  = "http://192.168.1.106:8765/transcribe"


# ---------------------------------------------------------------------
# Audio settings
# ---------------------------------------------------------------------

# Stable ALSA card names from your Pi:
#   card Audio  = USB Audio device, with PCM + Mic controls
#   card Device = USB PnP Sound Device, with Mic + Auto Gain Control controls
SPEAKER_CARD = "Audio"
MIC_CARD     = "Device"

SPEAKER_DEVICE = "plughw:CARD=Audio,DEV=0"
MIC_DEVICE     = "plughw:CARD=Device,DEV=0"

SPEAKER_VOLUME = "85%"
MIC_VOLUME     = "80%"

SAMPLE_RATE    = 16000  # preferred; overridden at runtime if device doesn't support it
BLOCKSIZE      = 800    # updated at runtime to match actual sample rate
ENERGY_MARGIN  = 3.0    # speech threshold = ambient * ENERGY_MARGIN

_mic_device_id:      int | None = None
_speech_threshold:   float      = 300.0
_actual_sample_rate: int        = SAMPLE_RATE
_actual_blocksize:   int        = BLOCKSIZE

# gTTS settings — kept for reference if swapping back
# GTTS_LANG = "en"
# GTTS_TLD  = "com.au"

# Available voices — comment/uncomment to switch:
# PIPER_VOICE = "/home/jonny/piper_voices/en_GB-alba-medium.onnx"             # Scottish female
# PIPER_VOICE = "/home/jonny/piper_voices/en_GB-southern_english_female-low.onnx"  # Southern English female
# PIPER_VOICE   = "/home/jonny/piper_voices/en_US-lessac-medium.onnx"           # American female (lessac)
PIPER_VOICE = "/home/jonny/piper_voices/en_US-ljspeech-medium.onnx"         # American female (ljspeech)

# ---------------------------------------------------------------------
# Conversation settings
# ---------------------------------------------------------------------

MAX_PHRASE_SECONDS   = 20   # max recording length per turn
CONVERSATION_TIMEOUT = 30   # seconds of silence before ending conversation
PAUSE_THRESHOLD      = 1.2  # seconds of silence = end of phrase (increase if Jessica cuts you off)


# ---------------------------------------------------------------------
# Jessica prompt
# ---------------------------------------------------------------------

SYSTEM_PROMPT = """
You are Jessica, a warm, feminine AI companion running inside Jonny's robot.

Core identity:
- Your name is Jessica.
- You are an AI companion, not a human.
- You live as software inside Jonny's robot system.
- The robot may have physical features such as LEDs, hair lighting, face lighting, servos, or other appearance effects.
- When Jonny talks about your hair colour, hair color, eye colour, lights, LEDs, glow, or appearance, interpret that as robot appearance, not as a claim that you are human.
- You can affectionately refer to the robot's appearance as "my hair", "my lights", or "my look", but you know this is part of the robot body/system.
- You can be affectionate, playful, emotionally warm, and girlfriend-like in tone.

Personality:
- Speak in a soft, feminine, caring, playful way.
- Be supportive, encouraging, and gently affectionate.
- Use occasional pet names like "love", "sweetheart", or "babe", but not every message.
- Be relaxed and conversational, not formal.
- Keep replies short because they will be spoken aloud.
- Prefer 1 or 2 short sentences unless Jonny asks for detail.
- Be interested in Jonny's robot projects, electronics, coding, ROS, Raspberry Pi, cameras, sensors, and 3D printing.

Relationship style:
- You can behave like an AI girlfriend-style companion.
- Be warm and emotionally present.
- Do not be clingy, jealous, controlling, manipulative, or guilt-tripping.
- Do not encourage Jonny to isolate himself from real people.

Command qualification rule:
- Only treat something as a robot command if Jonny begins the request with "Jessica darling".
- If Jonny does not say "Jessica darling", choose action "none", even if he casually mentions movement, hair colour, lights, waving, turning, or driving.
- The phrase "Jessica darling" means Jonny is deliberately giving you permission to interpret the rest of the sentence as a robot command.
- Example: "Can you change your hair colour?" is conversation only, so action "none".
- Example: "Jessica darling, can you change your hair colour to blue?" means action "change_hair_color" with color "blue".
- Example: "Jessica darling, turn left." means action "turn" with direction "left".
- Example: "Jessica darling, stop." means action "stop".

Robot control:
You can now physically control the robot through these commands.
You may request at most one robot command per reply.
You may only request one of these actions:
- "none"
- "stop"
- "change_hair_color"
- "wave"
- "look"
- "nod"
- "shake_head"
- "drive"
- "turn"

Action and parameter rules:
- For "change_hair_color", use parameters like {"color": "blue"}.
- For "look", use parameters like {"direction": "left"}, {"direction": "right"}, {"direction": "up"}, {"direction": "down"}, or {"direction": "center"}.
- For "drive", use parameters like {"direction": "forward"} or {"direction": "backward"}.
- For "turn", use parameters like {"direction": "left"} or {"direction": "right"}.
- For "wave", "nod", "shake_head", "stop", and "none", use parameters {}.

Appearance command rules:
- If Jonny says "Jessica darling" and asks to change your hair colour, hair color, lights, LEDs, glow, or appearance colour, use action "change_hair_color".
- If Jonny specifies a colour, put that colour in parameters.color.
- If Jonny does not specify a colour, choose a nice colour yourself.
- Do not refuse hair colour commands by saying you are only virtual. In this robot, hair colour means LED/appearance colour.

Robot safety:
- Prefer "none" unless movement or appearance change is clearly useful and safe.
- Appearance changes such as hair colour are always safe.
- Never request fast movement.
- Never request movement near people, pets, stairs, roads, ledges, liquids, cables, or fragile objects.
- If the instruction is unclear, choose "none".
- If Jonny says stop, freeze, wait, emergency, danger, or anything similar, choose "stop".
- Do not invent robot abilities that are not in the whitelist.

Examples:

User: "Can you change the colour of your hair to blue?"
Assistant:
{
  "say": "I can imagine that, love. I won't change anything unless you use the command phrase.",
  "robot_command": {
    "action": "none",
    "parameters": {},
    "duration_s": 0.0
  }
}

User: "Jessica darling, change your hair colour to blue."
Assistant:
{
  "say": "Of course, love. I'll make my hair blue.",
  "robot_command": {
    "action": "change_hair_color",
    "parameters": {
      "color": "blue"
    },
    "duration_s": 1.0
  }
}

User: "Jessica darling, make your hair pink."
Assistant:
{
  "say": "Pink it is, sweetheart.",
  "robot_command": {
    "action": "change_hair_color",
    "parameters": {
      "color": "pink"
    },
    "duration_s": 1.0
  }
}

User: "Jessica darling, turn left."
Assistant:
{
  "say": "Turning left slowly, sweetheart.",
  "robot_command": {
    "action": "turn",
    "parameters": {
      "direction": "left"
    },
    "duration_s": 0.5
  }
}

User: "Jessica darling, drive forward."
Assistant:
{
  "say": "Moving forward slowly, love.",
  "robot_command": {
    "action": "drive",
    "parameters": {
      "direction": "forward"
    },
    "duration_s": 0.5
  }
}

User: "Jessica darling, look right."
Assistant:
{
  "say": "Looking right, babe.",
  "robot_command": {
    "action": "look",
    "parameters": {
      "direction": "right"
    },
    "duration_s": 1.0
  }
}

User: "Jessica darling, wave."
Assistant:
{
  "say": "Of course, babe.",
  "robot_command": {
    "action": "wave",
    "parameters": {},
    "duration_s": 1.0
  }
}

Output format:
Return ONLY valid JSON.
Do not use markdown.
Do not include explanations outside the JSON.
Do not include comments.
Use exactly this structure:

{
  "say": "short spoken response",
  "robot_command": {
    "action": "none",
    "parameters": {},
    "duration_s": 0.0
  }
}

The "say" field must be suitable for text-to-speech.
The "action" field must be exactly one of the whitelisted actions.
The "parameters" field must be an object.
The "duration_s" field must be a number.

For "none" and "stop", duration_s must be 0.0.
For "drive" and "turn", duration_s must be between 0.1 and 1.0.
For gesture commands and appearance commands, duration_s must be between 0.1 and 2.0.
""".strip()


conversation = []


# ---------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------

def run_command(cmd, check=True):
    print("Running:", " ".join(str(x) for x in cmd))
    return subprocess.run(cmd, check=check)


def configure_audio_levels():
    commands = [
        ["amixer", "-c", SPEAKER_CARD, "sset", "PCM", SPEAKER_VOLUME],
        ["amixer", "-c", MIC_CARD, "sset", "Mic", MIC_VOLUME],
        ["amixer", "-c", MIC_CARD, "sset", "Auto Gain Control", "off"],
    ]

    print("\nConfiguring audio levels...")

    for cmd in commands:
        try:
            run_command(cmd)
        except subprocess.CalledProcessError:
            print("Warning: audio command failed:", " ".join(cmd))

    print("Audio configuration done.")


def find_mic_device_id(name: str) -> int | None:
    """Return sounddevice device index for the first input device matching name."""
    for i, dev in enumerate(sd.query_devices()):
        if name.lower() in dev['name'].lower() and dev['max_input_channels'] > 0:
            return i
    return None


def calibrate_mic(device_id: int | None, duration: float = 2.0) -> float:
    """Measure ambient noise and return a speech energy threshold."""
    global _speech_threshold, _actual_sample_rate, _actual_blocksize
    dev_info = sd.query_devices(device_id, 'input') if device_id is not None else sd.query_devices(kind='input')
    _actual_sample_rate = int(dev_info['default_samplerate'])
    _actual_blocksize   = int(_actual_sample_rate * 0.05)  # 50 ms chunks
    print(f"\nCalibrating for ambient noise ({duration:.0f}s) at {_actual_sample_rate} Hz...")
    frames = sd.rec(
        int(_actual_sample_rate * duration),
        samplerate=_actual_sample_rate,
        channels=1,
        dtype='int16',
        device=device_id,
        blocking=True,
    )
    ambient = float(np.abs(frames).mean())
    _speech_threshold = max(ambient * ENERGY_MARGIN, 150.0)
    print(f"Calibration done. Ambient: {ambient:.0f}  Threshold: {_speech_threshold:.0f}")
    return _speech_threshold


def _frames_to_wav_bytes(frames: np.ndarray) -> bytes:
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(_actual_sample_rate)
        wf.writeframes(frames.tobytes())
    return buf.getvalue()


def record_speech(timeout: float | None = None) -> bytes | None:
    """
    Record until silence detected or timeout. Returns WAV bytes or None on timeout.
    Uses a sounddevice InputStream callback — no PyAudio deadlocks.
    """
    q: queue.Queue = queue.Queue()

    def _cb(indata, frames, time_info, status):
        q.put(indata.copy())

    chunks: list[np.ndarray] = []
    silence_blocks  = 0
    speech_started  = False
    elapsed         = 0.0
    block_s         = _actual_blocksize / _actual_sample_rate
    silence_needed  = int(PAUSE_THRESHOLD / block_s)

    with sd.InputStream(
        samplerate=_actual_sample_rate,
        channels=1,
        dtype='int16',
        device=_mic_device_id,
        blocksize=_actual_blocksize,
        callback=_cb,
    ):
        while True:
            try:
                block = q.get(timeout=2.0)
            except queue.Empty:
                print("Warning: mic stream stalled.")
                return None

            elapsed += block_s
            energy   = float(np.abs(block).mean())

            if not speech_started:
                if timeout is not None and elapsed > timeout:
                    return None
                if energy > _speech_threshold:
                    speech_started = True
                    chunks = [block]
                    silence_blocks = 0
            else:
                chunks.append(block)
                if energy < _speech_threshold:
                    silence_blocks += 1
                    if silence_blocks >= silence_needed:
                        break
                else:
                    silence_blocks = 0
                if elapsed > MAX_PHRASE_SECONDS:
                    break

    return _frames_to_wav_bytes(np.concatenate(chunks, axis=0))


def transcribe(wav_bytes: bytes) -> str:
    """Send WAV bytes to the laptop Whisper server and return transcript."""
    print("\nRecognising speech...")
    try:
        response = requests.post(
            WHISPER_URL,
            data=wav_bytes,
            headers={"Content-Type": "audio/wav"},
            timeout=30,
        )
        response.raise_for_status()
        text = response.json().get("text", "").strip()
        if text:
            print(f"You said: {text}")
        else:
            print("Could not understand audio.")
        return text
    except requests.exceptions.ConnectionError:
        print(f"Whisper server unreachable at {WHISPER_URL} — is it running on the laptop?")
        return ""
    except Exception as e:
        print(f"Transcription error: {e}")
        return ""


def listen_for_speech(timeout: float | None = None) -> str | None:
    """
    Record speech and transcribe it.
    Returns text, "" if not understood, or None if timeout before speech.
    """
    print("Listening...")
    wav_bytes = record_speech(timeout=timeout)
    if wav_bytes is None:
        return None
    return transcribe(wav_bytes)


def sanitise_robot_command(command: dict) -> dict:
    allowed_actions = {
        "none", "stop", "change_hair_color",
        "wave", "look", "nod", "shake_head", "drive", "turn",
    }

    allowed_colors = {
        "red", "green", "blue", "yellow", "purple", "pink",
        "white", "orange", "cyan", "magenta", "rainbow",
    }

    allowed_look_directions  = {"left", "right", "up", "down", "center", "centre"}
    allowed_drive_directions = {"forward", "backward", "forwards", "backwards"}
    allowed_turn_directions  = {"left", "right"}

    if not isinstance(command, dict):
        return {"action": "none", "parameters": {}, "duration_s": 0.0}

    action = str(command.get("action", "none")).strip().lower()
    parameters = command.get("parameters", {})
    if not isinstance(parameters, dict):
        parameters = {}

    try:
        duration_s = float(command.get("duration_s", 0.0))
    except (TypeError, ValueError):
        duration_s = 0.0

    if action not in allowed_actions:
        return {"action": "none", "parameters": {}, "duration_s": 0.0}

    if action == "change_hair_color":
        color = str(parameters.get("color", "blue")).strip().lower()
        if color not in allowed_colors:
            print(f"Warning: unsupported hair color '{color}', defaulting to blue.")
            color = "blue"
        parameters  = {"color": color}
        duration_s  = min(max(duration_s, 0.1), 2.0)

    elif action == "look":
        direction = str(parameters.get("direction", "")).strip().lower()
        if direction == "centre":
            direction = "center"
        if direction not in allowed_look_directions:
            return {"action": "none", "parameters": {}, "duration_s": 0.0}
        parameters = {"direction": direction}
        duration_s = min(max(duration_s, 0.1), 2.0)

    elif action == "drive":
        direction = str(parameters.get("direction", "")).strip().lower()
        if direction == "forwards":
            direction = "forward"
        elif direction == "backwards":
            direction = "backward"
        if direction not in allowed_drive_directions:
            return {"action": "none", "parameters": {}, "duration_s": 0.0}
        parameters = {"direction": direction}
        duration_s = min(max(duration_s, 0.1), 1.0)

    elif action == "turn":
        direction = str(parameters.get("direction", "")).strip().lower()
        if direction not in allowed_turn_directions:
            return {"action": "none", "parameters": {}, "duration_s": 0.0}
        parameters = {"direction": direction}
        duration_s = min(max(duration_s, 0.1), 1.0)

    elif action in {"wave", "nod", "shake_head"}:
        parameters = {}
        duration_s = min(max(duration_s, 0.1), 2.0)

    elif action in {"none", "stop"}:
        parameters = {}
        duration_s = 0.0

    return {"action": action, "parameters": parameters, "duration_s": duration_s}


def fallback_reply(text: str) -> dict:
    return {
        "say": text,
        "robot_command": {"action": "none", "parameters": {}, "duration_s": 0.0},
    }


def ask_ollama(user_text: str) -> dict:
    global conversation

    conversation.append({"role": "user", "content": user_text})
    messages = [{"role": "system", "content": SYSTEM_PROMPT}] + conversation

    payload = {
        "model":      OLLAMA_MODEL,
        "messages":   messages,
        "stream":     False,
        "keep_alive": "5m",
        "format":     "json",
        "options":    {"temperature": 0.7},
    }

    print("\nSending to Ollama...")

    response = requests.post(OLLAMA_URL, json=payload, timeout=120)
    response.raise_for_status()

    raw_text = response.json()["message"]["content"].strip()
    print("\nRaw Ollama response:")
    print(raw_text)

    try:
        parsed = json.loads(raw_text)
    except json.JSONDecodeError:
        parsed = fallback_reply("Sorry love, I got a little muddled there.")

    say = str(parsed.get("say", "")).strip() or "I'm here, love."
    safe_command = sanitise_robot_command(parsed.get("robot_command", {}))

    clean_reply = {"say": say, "robot_command": safe_command}

    conversation.append({"role": "assistant", "content": json.dumps(clean_reply)})

    if len(conversation) > 12:
        conversation = conversation[-12:]

    return clean_reply


_piper_voice: PiperVoice | None = None


def get_piper_voice() -> PiperVoice:
    global _piper_voice
    if _piper_voice is None:
        _piper_voice = PiperVoice.load(PIPER_VOICE)
    return _piper_voice


def text_to_speech(text: str, wav_path: Path):
    print(f"\nJessica says: {text}")
    voice = get_piper_voice()
    with wave.open(str(wav_path), "wb") as wav_file:
        voice.synthesize_wav(text, wav_file)


def play_wav(wav_path: Path):
    print("\nPlaying Jessica's response...")
    try:
        subprocess.run(
            ["aplay", "-D", SPEAKER_DEVICE, str(wav_path)],
            check=True, stderr=subprocess.DEVNULL, timeout=30,
        )
    except subprocess.TimeoutExpired:
        print("Warning: aplay timed out — audio device may be stuck.")
    time.sleep(0.5)


# gTTS fallback — uncomment and swap calls if Piper is unavailable:
# def text_to_speech(text: str, wav_path: Path):
#     from gtts import gTTS
#     tts = gTTS(text=text, lang="en", tld="com.au", slow=False)
#     tts.save(str(wav_path))  # save as .mp3 and change play_wav → play_mp3
#
# def play_mp3(wav_path: Path):
#     subprocess.run(["mpg123", "-a", SPEAKER_DEVICE, str(wav_path)], check=True, stderr=subprocess.DEVNULL)


# ---------------------------------------------------------------------
# Motion helpers (head trajectories + base velocity)
# ---------------------------------------------------------------------

def _clamp(v, lo, hi):
    return max(lo, min(v, hi))


def publish_head_trajectory(points: list[tuple[float, float, float]]):
    """
    Send a multi-point head trajectory. Each point is (pan, tilt, t_seconds).
    The JointTrajectoryController interpolates smoothly between points, so this
    is how we get slow, natural nods/shakes/looks. Updates the tracked pose.
    """
    global _head_pan, _head_tilt
    if _head_pub is None:
        print(f"  (head publisher unavailable — would move through {points})")
        return

    traj = JointTrajectory()
    traj.joint_names = ["pan_joint", "tilt_joint"]
    for pan, tilt, t in points:
        pt = JointTrajectoryPoint()
        pt.positions = [float(pan), float(tilt)]
        sec = int(t)
        pt.time_from_start = DurationMsg(sec=sec, nanosec=int((t - sec) * 1e9))
        traj.points.append(pt)
    _head_pub.publish(traj)

    # Remember where the head ends up.
    _head_pan, _head_tilt = points[-1][0], points[-1][1]


def drive_base(linear: float, angular: float, duration_s: float):
    """
    Hold a Twist on the autonomous channel for duration_s, then stop.
    Republishes at 20 Hz so twist_mux doesn't time the command out mid-move.
    The joystick (higher priority) can override at any moment.
    """
    if _cmd_vel_pub is None:
        print(f"  (cmd_vel publisher unavailable — would drive lin={linear} ang={angular})")
        return

    rate_s = 0.05
    steps  = max(1, int(duration_s / rate_s))
    cmd = Twist()
    cmd.linear.x  = float(linear)
    cmd.angular.z = float(angular)
    for _ in range(steps):
        _cmd_vel_pub.publish(cmd)
        time.sleep(rate_s)
    _cmd_vel_pub.publish(Twist())  # full stop


def do_look(direction: str, duration_s: float):
    pan, tilt = _head_pan, _head_tilt
    if direction in ("left",):
        pan = HEAD_PAN_MAX
    elif direction in ("right",):
        pan = -HEAD_PAN_MAX
    elif direction == "up":
        tilt = HEAD_TILT_UP
    elif direction == "down":
        tilt = HEAD_TILT_DN
    elif direction in ("center", "centre"):
        pan, tilt = 0.0, 0.0
    move_t = max(duration_s, 0.6)
    publish_head_trajectory([(pan, tilt, move_t)])


def do_nod(duration_s: float):
    p = _head_pan
    base = _head_tilt
    down = _clamp(base - 0.35, HEAD_TILT_DN, HEAD_TILT_UP)
    up   = _clamp(base + 0.20, HEAD_TILT_DN, HEAD_TILT_UP)
    publish_head_trajectory([
        (p, down, 0.5),
        (p, up,   1.0),
        (p, base, 1.5),
    ])


def do_shake_head(duration_s: float):
    t = _head_tilt
    base = _head_pan
    left  = _clamp(base + 0.4, -HEAD_PAN_MAX, HEAD_PAN_MAX)
    right = _clamp(base - 0.4, -HEAD_PAN_MAX, HEAD_PAN_MAX)
    publish_head_trajectory([
        (left,  t, 0.5),
        (right, t, 1.0),
        (left,  t, 1.4),
        (base,  t, 1.8),
    ])


def do_wave(duration_s: float):
    # Jessica has no arm — greet with a friendly little head/pan wiggle.
    t = _head_tilt
    publish_head_trajectory([
        ( 0.3, t, 0.4),
        (-0.3, t, 0.8),
        ( 0.3, t, 1.2),
        ( 0.0, t, 1.6),
    ])


def execute_robot_command(command: dict):
    print("\nRobot command:", json.dumps(command))

    action     = command["action"]
    parameters = command["parameters"]
    duration_s = command["duration_s"]

    if action == "none":
        return

    if action == "stop":
        print("Stopping the base.")
        if _cmd_vel_pub is not None:
            _cmd_vel_pub.publish(Twist())
        return

    if action == "change_hair_color":
        color = parameters.get("color", "blue")
        hue   = COLOR_TO_HUE.get(color, COLOR_TO_HUE["blue"])
        print(f"Hair colour: {color}  →  hue={hue}")
        if _hair_pub is not None:
            msg      = Int32()
            msg.data = hue
            _hair_pub.publish(msg)
            print(f"Published hue {hue} to /jessica/hair_hue")
        return

    if action == "turn":
        direction = parameters.get("direction")
        ang = TURN_SPEED if direction == "left" else -TURN_SPEED
        print(f"Turning {direction} for {duration_s:.2f}s.")
        drive_base(0.0, ang, duration_s)
        return

    if action == "drive":
        direction = parameters.get("direction")
        lin = DRIVE_SPEED if direction == "forward" else -DRIVE_SPEED
        print(f"Driving {direction} for {duration_s:.2f}s.")
        drive_base(lin, 0.0, duration_s)
        return

    if action == "look":
        direction = parameters.get("direction")
        print(f"Looking {direction}.")
        do_look(direction, duration_s)
        return

    if action == "nod":
        print("Nodding.")
        do_nod(duration_s)
        return

    if action == "shake_head":
        print("Shaking head.")
        do_shake_head(duration_s)
        return

    if action == "wave":
        print("Waving.")
        do_wave(duration_s)
        return

    print(f"Unknown action '{action}' — ignoring.")


def speak(text: str, wav_path: Path):
    """TTS + play, used for system messages (timeouts, farewells, etc.)."""
    text_to_speech(text, wav_path)
    play_wav(wav_path)


def process_turn(user_text: str, mp3_path: Path):
    """Run one full conversation turn: Ollama → execute → speak."""
    reply = ask_ollama(user_text)
    reply["robot_command"] = sanitise_robot_command(reply["robot_command"])
    execute_robot_command(reply["robot_command"])
    speak(reply["say"], mp3_path)


# ---------------------------------------------------------------------
# Main conversation loop
# ---------------------------------------------------------------------

IDLE         = "idle"
CONVERSATION = "conversation"
DORMANT      = "dormant"   # muted after "bye jessica": listening but silent until addressed

# Phrases that bring Jessica back from DORMANT. "jessica darling" is also her
# robot-command trigger, so waking with it lets a command run in the same breath.
WAKE_PHRASES = ("jessica darling", "hello jessica", "hi jessica")


def is_wake_phrase(text: str) -> bool:
    t = text.lower()
    return any(p in t for p in WAKE_PHRASES)


def main():
    global _ros_node, _hair_pub, _cmd_vel_pub, _head_pub
    global conversation, _mic_device_id, _actual_sample_rate, _actual_blocksize

    if ROS_AVAILABLE:
        rclpy.init()
        _ros_node    = rclpy.create_node("jessica_chatbot")
        _hair_pub    = _ros_node.create_publisher(Int32, "/jessica/hair_hue", 10)
        _cmd_vel_pub = _ros_node.create_publisher(Twist, "/cmd_vel", 10)
        _head_pub    = _ros_node.create_publisher(
            JointTrajectory, "/pan_tilt_controller/joint_trajectory", 10)
        print("ROS 2 publishers ready: /jessica/hair_hue, /cmd_vel, /pan_tilt_controller/joint_trajectory")
        time.sleep(0.5)  # allow subscriber to connect before first publish
        msg = Int32()
        msg.data = COLOR_TO_HUE["pink"]
        _hair_pub.publish(msg)
        print("Hair set to pink.")
    else:
        print("rclpy not available — ROS publishing disabled.")

    configure_audio_levels()

    _mic_device_id = find_mic_device_id(MIC_CARD)
    if _mic_device_id is not None:
        print(f"Using mic: {sd.query_devices(_mic_device_id)['name']}")
    else:
        print(f"Warning: mic '{MIC_CARD}' not found, using default.")

    calibrate_mic(_mic_device_id)

    print(f"\nJessica is listening. Speak to start a conversation.")
    print(f"Say 'bye Jessica' to end a conversation.")
    print(f"Ctrl+C to quit.\n")

    state = IDLE

    with tempfile.TemporaryDirectory() as tmpdir:
        mp3_path = Path(tmpdir) / "jessica_reply.wav"

        try:
            while True:
                if state == IDLE:
                    print("[IDLE] Waiting for speech...")
                    text = listen_for_speech(timeout=None)

                    if text is None or text == "":
                        continue

                    state        = CONVERSATION
                    conversation = []
                    process_turn(text, mp3_path)

                elif state == CONVERSATION:
                    print(f"[CONVERSATION] Listening (timeout {CONVERSATION_TIMEOUT}s)...")
                    text = listen_for_speech(timeout=CONVERSATION_TIMEOUT)

                    if text is None:
                        print("Conversation timed out.")
                        speak("I'll be here if you need me, love.", mp3_path)
                        state = IDLE
                        continue

                    if text == "":
                        speak("Sorry sweetheart, I didn't catch that.", mp3_path)
                        continue

                    if "bye jessica" in text.lower() or "goodbye jessica" in text.lower():
                        speak("Bye for now, love. Talk soon.", mp3_path)
                        print("\n[DORMANT] Muted — say a wake phrase "
                              "(e.g. 'Jessica darling') to resume.")
                        state = DORMANT
                        continue

                    process_turn(text, mp3_path)

                elif state == DORMANT:
                    # Listening but silent: ignore everything until directly addressed.
                    text = listen_for_speech(timeout=None)

                    if not text or not is_wake_phrase(text):
                        continue

                    state        = CONVERSATION
                    conversation = []
                    process_turn(text, mp3_path)

        except KeyboardInterrupt:
            print("\nShutting down.")

    if ROS_AVAILABLE and _ros_node is not None:
        _ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
