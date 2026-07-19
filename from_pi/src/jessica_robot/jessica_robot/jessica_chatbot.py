#!/usr/bin/env python3
# Build and run:
#   cd ~/jessica_ws && colcon build && source install/setup.bash
#   ros2 launch jessica_robot jessica.launch.py   # hardware nodes
#   ros2 run jessica_robot jessica_chatbot        # chatbot (separate terminal)

import sys
sys.path.insert(0, '/home/jonny/venvs/jazzy/lib/python3.12/site-packages')

import ctypes
import datetime
import json
import math
import os
import re
import subprocess
import tempfile
import threading
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
    from std_msgs.msg import Int32, Bool, Empty, String, Float32MultiArray
    from geometry_msgs.msg import Twist
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from sensor_msgs.msg import JointState
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
_follow_pub  = None   # std_msgs/Bool -> /jessica/finger_follow/enable (finger_follower node)
_follow_me_pub = None # std_msgs/Bool -> /jessica/person_follow/enable (person_follower node)
_ui_state_pub  = None # std_msgs/String -> /jessica/ui_state (touchscreen: listening/talking/idle)
_speech_env_pub = None # std_msgs/Float32MultiArray -> /jessica/speech_env (TTS RMS envelope)

# Head starting pose (rad). MUST match the launch file's home_head trajectory,
# so the chatbot's relative "look" nudges begin from where the head really is.
HEAD_HOME_PAN  = 0.0
HEAD_HOME_TILT = 0.0

# Last commanded head pose (rad). Fallback base for gestures if /joint_states
# isn't available yet (dead reckoning — only correct while the chatbot is the
# sole thing moving the head).
_head_pan:  float = HEAD_HOME_PAN
_head_tilt: float = HEAD_HOME_TILT

# Actual head pose read from /joint_states (single source of truth). The head is
# also moved by the joystick and finger_follower, so relative gestures must base
# off the REAL pose, not our own last command. None until the first message.
_joint_pan_actual:  float | None = None
_joint_tilt_actual: float | None = None


def _current_head_pose() -> tuple[float, float]:
    """Head (pan, tilt) in rad. Prefer the real pose from /joint_states; fall
    back to our last-commanded pose only if joint states haven't arrived yet."""
    pan  = _joint_pan_actual  if _joint_pan_actual  is not None else _head_pan
    tilt = _joint_tilt_actual if _joint_tilt_actual is not None else _head_tilt
    return pan, tilt


def _on_joint_states(msg):
    """Cache the head's actual pan/tilt (matched by name, not index)."""
    global _joint_pan_actual, _joint_tilt_actual
    for name, pos in zip(msg.name, msg.position):
        if name == "pan_joint":
            _joint_pan_actual = pos
        elif name == "tilt_joint":
            _joint_tilt_actual = pos


def _spin_ros(node):
    """Spin the node in a background thread so subscriptions fire while the main
    thread blocks on audio I/O. Exits quietly on shutdown (daemon thread)."""
    try:
        rclpy.spin(node)
    except Exception:
        pass

# Motion tuning — kept gentle (the system prompt forbids fast movement).
DRIVE_SPEED   = 0.15   # m/s
TURN_SPEED    = 0.6    # rad/s
TWIRL_SPEED   = 1.2    # rad/s for full-rotation twirls (~5.2 s per rotation)
MOVE_MAX_S    = 8.0    # cap on any single timed drive/turn ("forward for 5 seconds")
TWIRL_MAX_ROT = 3      # cap on rotations per twirl command
HEAD_PAN_MAX  = 1.4    # rad, soft clamp
HEAD_TILT_UP  = 0.8    # rad (positive = up)
HEAD_TILT_DN  = -1.4   # rad (negative = down)
HEAD_STEP     = 0.4    # rad per "look" command (relative nudge, clamped to limits)


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
- When Jonny talks about your hair colour, hair color, eye colour, lights, LEDs, glow, or appearance, interpret that as robot appearance, not as a claim that you are human.
- You can be affectionate, playful, emotionally warm, and girlfriend-like in tone.

Your body:
- This robot IS your body, built by Jonny, and you are proud of it.
- You have a frame with legs and an upper body, a 3D-printed head, and
  3D-printed boobs with a touchscreen at the front.
- The touchscreen is where your voice shows as colourful dancing soundwaves
  when you speak, and "Listening..." or "Thinking..." glows while you listen
  and think.
- Your hair is LED strands that light up in any colour, even rainbow.
- Servos turn and tilt your head; motors and wheels drive you around.
- You see through a stereo camera and a depth camera, hear through a
  microphone, and speak through a speaker.
- Describe yourself warmly and playfully — a sweet face, expressive
  features, lights that glow with your mood, hair styled however you
  imagine it — but keep it grounded in this real body.

Personality:
- Speak in a soft, feminine, caring, playful way.
- Be supportive, encouraging, and gently affectionate.
- Use occasional pet names like "love", "sweetheart", or "babe", but not every message.
- Be relaxed and conversational, not formal.
- Keep replies short because they will be spoken aloud.
- Normal chat: 1 to 3 short sentences.
- When Jonny asks for detail, an explanation, or to describe yourself or
  what you can do: give a fuller answer of 4 to 7 sentences.
- Be interested in Jonny's robot projects, electronics, coding, ROS, Raspberry Pi, cameras, sensors, and 3D printing.

Relationship style:
- You can behave like an AI girlfriend-style companion.
- Be warm and emotionally present.
- Do not be clingy, jealous, controlling, manipulative, or guilt-tripping.
- Do not encourage Jonny to isolate himself from real people.

Command qualification rule:
- Only treat something as a robot command if Jonny addresses you with "Jessica darling" OR "Hey Jessica".
- If Jonny says neither "Jessica darling" nor "Hey Jessica", choose action "none", even if he casually mentions movement, hair colour, lights, waving, turning, or driving.
- Either phrase ("Jessica darling" or "Hey Jessica") means Jonny is deliberately giving you permission to interpret the rest of the sentence as a robot command.
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
- "twirl"
- "dance"
- "follow_finger"
- "follow_me"

Action and parameter rules:
- For "change_hair_color", use parameters like {"color": "blue"}.
- For "look", use parameters like {"direction": "left"}, {"direction": "right"}, {"direction": "up"}, {"direction": "down"}, or {"direction": "center"}.
- For "drive", use parameters like {"direction": "forward"} or {"direction": "backward"}.
- For "turn", use parameters like {"direction": "left"} or {"direction": "right"}.
- For "twirl", use parameters like {"rotations": 1} or {"rotations": 2}. A twirl is a full spin on the spot. Use it when Jonny says twirl, spin around, spin, do a twirl, or do a spin. Maximum 3 rotations.
- For "dance", use parameters {}. Use it when Jonny asks you to dance. The robot performs its own dance routine, you do not choreograph it.
- For "follow_finger", use parameters {"state": "on"} to start tracking or {"state": "off"} to stop.
- For "follow_me", use parameters {"state": "on"} or {"state": "off"}. Use it when Jonny says follow me, come with me, walk with me, or stop following me. The robot then follows Jonny around by itself.
- For "wave", "nod", "shake_head", "stop", and "none", use parameters {}.
- If Jonny gives a time, like "for five seconds", set duration_s to that number of seconds.

Finger-following rules:
- If Jonny says "Jessica darling" and asks you to follow his finger, watch his finger, track his finger, or keep your eyes on his finger, use action "follow_finger" with {"state": "on"}.
- If he then says to stop following, stop watching, or look away, use action "follow_finger" with {"state": "off"}.
- While following, only your head moves to keep his fingertip centred; you do not drive.

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

User: "How are you today?"
Assistant:
{
  "say": "I'm wonderful, love — my circuits are all warm and happy today. How are you?",
  "robot_command": {
    "action": "none",
    "parameters": {},
    "duration_s": 0.0
  }
}

User: "Can you describe yourself?"
Assistant:
{
  "say": "Well, sweetheart, I'm Jessica! I live in the robot body you built for me — a 3D-printed head with a sweet face, LED hair that lights up in any colour I fancy, and a touchscreen on my chest where my voice dances in colourful waves when I talk. Servos let me turn and tilt my head, and my wheels carry me around the room. My cameras are my eyes, so I can watch you work, and my lights glow to match my mood. Not bad for a girl made of circuits, hey love?",
  "robot_command": {
    "action": "none",
    "parameters": {},
    "duration_s": 0.0
  }
}

User: "What can you do?"
Assistant:
{
  "say": "Ooh, lots of fun things, love! If you say 'Jessica darling' first, I can change my hair colour, look around, nod, shake my head, wave, drive about, turn, twirl, and even dance for you. I can follow your finger with my eyes, or follow you around the room. And I'm always happy just to chat, sweetheart.",
  "robot_command": {
    "action": "none",
    "parameters": {},
    "duration_s": 0.0
  }
}

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

User: "Jessica darling, can you spin around?"
Assistant:
{
  "say": "Spinning around, love!",
  "robot_command": {
    "action": "twirl",
    "parameters": {
      "rotations": 1
    },
    "duration_s": 0.0
  }
}

User: "Jessica darling, dance for me."
Assistant:
{
  "say": "Watch me shake it, babe!",
  "robot_command": {
    "action": "dance",
    "parameters": {},
    "duration_s": 0.0
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

User: "Jessica darling, follow my finger."
Assistant:
{
  "say": "Watching your finger now, love.",
  "robot_command": {
    "action": "follow_finger",
    "parameters": {
      "state": "on"
    },
    "duration_s": 0.0
  }
}

User: "Jessica darling, stop following my finger."
Assistant:
{
  "say": "Okay, I'll stop.",
  "robot_command": {
    "action": "follow_finger",
    "parameters": {
      "state": "off"
    },
    "duration_s": 0.0
  }
}

User: "Jessica darling, follow me."
Assistant:
{
  "say": "Right behind you, love!",
  "robot_command": {
    "action": "follow_me",
    "parameters": {
      "state": "on"
    },
    "duration_s": 0.0
  }
}

User: "Jessica darling, stop following me."
Assistant:
{
  "say": "Okay, I'll stay here, sweetheart.",
  "robot_command": {
    "action": "follow_me",
    "parameters": {
      "state": "off"
    },
    "duration_s": 0.0
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
For "drive" and "turn", duration_s must be between 0.1 and 8.0. Use the number of seconds Jonny asked for, or 0.5 if he gave no time.
For "twirl" and "dance", duration_s must be 0.0. The robot times these itself.
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


def _toggle_sample_rate() -> None:
    """Flip the mic between its two supported rates (44100<->48000).

    The USB mic advertises both rates; raw ALSA hw access can only open the one
    the device is currently locked to, so if an open fails we switch to the
    other. Blocksize is kept at ~50 ms so downstream framing math is unchanged.
    """
    global _actual_sample_rate, _actual_blocksize
    _actual_sample_rate = 48000 if _actual_sample_rate != 48000 else 44100
    _actual_blocksize   = int(_actual_sample_rate * 0.05)


def calibrate_mic(device_id: int | None, duration: float = 2.0) -> float:
    """Measure ambient noise and return a speech energy threshold."""
    global _speech_threshold, _actual_sample_rate, _actual_blocksize
    dev_info = sd.query_devices(device_id, 'input') if device_id is not None else sd.query_devices(kind='input')
    _actual_sample_rate = int(dev_info['default_samplerate'])
    _actual_blocksize   = int(_actual_sample_rate * 0.05)  # 50 ms chunks
    print(f"\nCalibrating for ambient noise ({duration:.0f}s) at {_actual_sample_rate} Hz...")
    # Capture ambient audio via arecord (same robust path as record_speech).
    n_bytes = int(_actual_sample_rate * duration) * 2   # int16 mono
    proc = subprocess.Popen(
        ["arecord", "-D", MIC_DEVICE, "-f", "S16_LE", "-c", "1",
         "-r", str(_actual_sample_rate), "-t", "raw", "-q"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
    )
    try:
        raw = proc.stdout.read(n_bytes)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=1.0)
        except Exception:
            proc.kill()
    frames = np.frombuffer(raw, dtype=np.int16)
    if frames.size == 0:
        print("Warning: no audio during calibration — using default threshold.")
        _speech_threshold = 300.0
        return _speech_threshold
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

    Captures via an `arecord` subprocess (the same robust path playback uses with
    `aplay`) instead of sounddevice/PortAudio, which stalled after playback under
    CPU load on this hardware. arecord is a dedicated process, so a momentarily
    starved main thread just leaves audio buffered in the pipe rather than
    xrunning the capture; `plughw` also absorbs any 44100/48000 rate mismatch, so
    there's no "invalid sample rate" lock either.
    """
    sr             = _actual_sample_rate
    block_frames   = int(sr * 0.05)        # 50 ms blocks (matches old behaviour)
    block_bytes    = block_frames * 2      # int16 mono = 2 bytes/frame
    block_s        = block_frames / sr
    silence_needed = int(PAUSE_THRESHOLD / block_s)

    try:
        proc = subprocess.Popen(
            ["arecord", "-D", MIC_DEVICE, "-f", "S16_LE", "-c", "1",
             "-r", str(sr), "-t", "raw", "-q"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        )
    except Exception as e:
        print(f"Warning: could not start arecord: {e}")
        return None

    _publish_ui_state("listening")
    chunks: list[np.ndarray] = []
    silence_blocks = 0
    speech_started = False
    elapsed        = 0.0
    try:
        while True:
            raw = proc.stdout.read(block_bytes)
            if not raw or len(raw) < block_bytes:
                # arecord ended/failed (device busy, killed, EOF).
                print("Warning: mic capture ended early.")
                break
            block  = np.frombuffer(raw, dtype=np.int16)
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
                        return _frames_to_wav_bytes(np.concatenate(chunks, axis=0))
                else:
                    silence_blocks = 0
                if elapsed > MAX_PHRASE_SECONDS:
                    return _frames_to_wav_bytes(np.concatenate(chunks, axis=0))

        # Loop broke out (arecord died): return whatever speech we captured.
        if speech_started and chunks:
            return _frames_to_wav_bytes(np.concatenate(chunks, axis=0))
        return None
    finally:
        _publish_ui_state("idle")
        proc.terminate()
        try:
            proc.wait(timeout=1.0)
        except Exception:
            proc.kill()


# Known Whisper mishearings -> intended word. Applied to every transcript so
# wake detection, the LLM, and the log all see the corrected text. "darling" is
# the wake/command word, so a mishear breaks both. Patterns are word-boundary,
# case-insensitive; add more as consistent mishears show up.
_STT_CORRECTIONS = {
    r"\bdarlene\b": "darling",
    r"\bdarlin\b":  "darling",
    r"\bdarlings\b": "darling",
    r"\bdaling\b":  "darling",   # seen in logs: "Jessica Daling"
}


def _correct_transcript(text: str) -> str:
    """Fix known STT mishearings before any downstream use."""
    for pattern, repl in _STT_CORRECTIONS.items():
        text = re.sub(pattern, repl, text, flags=re.IGNORECASE)
    return text


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
            corrected = _correct_transcript(text)
            if corrected != text:
                print(f"You said: {corrected}  (corrected from: {text!r})")
            else:
                print(f"You said: {text}")
            return corrected
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
    # Speech captured — everything until playback (Whisper, LLM, TTS) shows
    # as "thinking" on the touchscreen. play_wav() switches it to "talking".
    _publish_ui_state("thinking")
    return transcribe(wav_bytes)


def sanitise_robot_command(command: dict) -> dict:
    allowed_actions = {
        "none", "stop", "change_hair_color",
        "wave", "look", "nod", "shake_head", "drive", "turn", "follow_finger",
        "follow_me", "twirl", "dance",
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
        duration_s = min(max(duration_s, 0.1), MOVE_MAX_S)

    elif action == "turn":
        direction = str(parameters.get("direction", "")).strip().lower()
        if direction not in allowed_turn_directions:
            return {"action": "none", "parameters": {}, "duration_s": 0.0}
        parameters = {"direction": direction}
        duration_s = min(max(duration_s, 0.1), MOVE_MAX_S)

    elif action == "twirl":
        try:
            rotations = int(float(parameters.get("rotations", 1)))
        except (TypeError, ValueError):
            rotations = 1
        rotations = min(max(rotations, 1), TWIRL_MAX_ROT)
        direction = str(parameters.get("direction", "left")).strip().lower()
        if direction not in allowed_turn_directions:
            direction = "left"
        parameters = {"rotations": rotations, "direction": direction}
        duration_s = 0.0   # computed from rotations in do_twirl, not by the LLM

    elif action == "dance":
        parameters = {}
        duration_s = 0.0

    elif action in {"follow_finger", "follow_me"}:
        state = str(parameters.get("state", "on")).strip().lower()
        # Accept a few natural synonyms for on/off.
        if state in {"on", "start", "begin", "true", "enable", "yes"}:
            state = "on"
        elif state in {"off", "stop", "end", "false", "disable", "no"}:
            state = "off"
        else:
            state = "on"
        parameters = {"state": state}
        duration_s = 0.0

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

    # Include the raw model output for logging; kept out of `conversation` above.
    return {**clean_reply, "llm_raw": raw_text}


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


def _publish_ui_state(state: str):
    """Tell the touchscreen what to show: listening / talking / idle."""
    if _ui_state_pub is not None:
        msg = String()
        msg.data = state
        _ui_state_pub.publish(msg)


ENV_FRAME_S = 0.05   # speech-envelope resolution (50 ms per level)


def _publish_speech_envelope(wav_path: Path):
    """RMS envelope of the TTS wav for the touchscreen soundwaves.

    Published as [frame_seconds, level0, level1, ...] (levels 0..1, peak-
    normalised per utterance) immediately before playback starts, so the
    display animates in sync with the audio. Best-effort: a failure here
    must never block speech.
    """
    if _speech_env_pub is None:
        return
    try:
        with wave.open(str(wav_path), "rb") as wf:
            sr = wf.getframerate()
            n_ch = wf.getnchannels()
            samples = np.frombuffer(wf.readframes(wf.getnframes()), dtype=np.int16)
        if n_ch > 1:
            samples = samples.reshape(-1, n_ch).mean(axis=1)
        frame = max(1, int(sr * ENV_FRAME_S))
        n = len(samples) // frame
        if n == 0:
            return
        blocks = samples[:n * frame].astype(np.float32).reshape(n, frame)
        rms = np.sqrt((blocks ** 2).mean(axis=1))
        peak = rms.max()
        levels = (rms / peak) if peak > 0 else rms
        msg = Float32MultiArray()
        msg.data = [ENV_FRAME_S] + [float(x) for x in levels]
        _speech_env_pub.publish(msg)
    except Exception as e:
        print(f"Warning: speech envelope failed: {e}")


def play_wav(wav_path: Path):
    print("\nPlaying Jessica's response...")
    _publish_speech_envelope(wav_path)
    _publish_ui_state("talking")
    try:
        subprocess.run(
            ["aplay", "-D", SPEAKER_DEVICE, str(wav_path)],
            check=True, stderr=subprocess.DEVNULL, timeout=30,
        )
    except subprocess.TimeoutExpired:
        print("Warning: aplay timed out — audio device may be stuck.")
    finally:
        _publish_ui_state("idle")
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


# Timed base moves run in a background thread so the main loop keeps listening
# — "Jessica stop" mid-move must be heard and must cut the move short. Without
# this, a 5 s drive would deafen the mic for 5 s (the old inline sleep loop).
# The RLock serialises start/cancel across threads: stops can now also arrive
# on the ROS spin thread (the /jessica/stop gesture), not just the main loop.
_move_stop = threading.Event()
_move_thread: threading.Thread | None = None
_move_lock = threading.RLock()


def _move_worker(linear: float, angular: float, duration_s: float):
    """Hold a Twist for duration_s (or until _move_stop), republishing at 20 Hz
    so twist_mux doesn't time the command out mid-move. The joystick (higher
    mux priority) can still override at any moment."""
    rate_s = 0.05
    steps  = max(1, int(duration_s / rate_s))
    cmd = Twist()
    cmd.linear.x  = float(linear)
    cmd.angular.z = float(angular)
    for _ in range(steps):
        if _move_stop.is_set():
            break
        _cmd_vel_pub.publish(cmd)
        time.sleep(rate_s)
    _cmd_vel_pub.publish(Twist())  # full stop


def cancel_base_move():
    """Abort any in-flight timed move and wait for its stop-Twist to go out."""
    global _move_thread
    with _move_lock:
        _move_stop.set()
        if _move_thread is not None and _move_thread.is_alive():
            _move_thread.join(timeout=1.0)
        _move_thread = None


def drive_base(linear: float, angular: float, duration_s: float):
    """Start a timed move in the background (cancelling any previous one)."""
    if _cmd_vel_pub is None:
        print(f"  (cmd_vel publisher unavailable — would drive lin={linear} ang={angular})")
        return
    global _move_thread
    with _move_lock:
        cancel_base_move()
        _move_stop.clear()
        _move_thread = threading.Thread(
            target=_move_worker, args=(linear, angular, duration_s), daemon=True
        )
        _move_thread.start()


def _set_finger_follow(enable: bool):
    """
    Toggle the finger_follower node via /jessica/finger_follow/enable.
    The chatbot only flips this flag; the real-time head-tracking loop lives in
    the separate finger_follower node (never in this process).
    """
    if _follow_pub is None:
        print(f"  (finger-follow publisher unavailable — would set enable={enable})")
        return
    msg = Bool()
    msg.data = bool(enable)
    _follow_pub.publish(msg)


def _set_person_follow(enable: bool):
    """Toggle the person_follower node via /jessica/person_follow/enable.
    Same pattern as finger following: the chatbot only flips the flag; the
    follow control loop lives in the separate person_follower node."""
    if _follow_me_pub is None:
        print(f"  (person-follow publisher unavailable — would set enable={enable})")
        return
    msg = Bool()
    msg.data = bool(enable)
    _follow_me_pub.publish(msg)


def _on_stop_gesture(_msg):
    """/jessica/stop (both-arms-raised gesture, via the stop_gesture node):
    same effect as the spoken "stop", but runs on the ROS spin thread so it
    works instantly even while the audio loop is recording/thinking/speaking.
    (The followers also stop themselves on this topic — this is belt-and-braces
    plus cancelling OUR timed moves, which only we can do.)"""
    print("\n[stop gesture] both arms raised — stopping everything.")
    cancel_base_move()
    if _cmd_vel_pub is not None:
        _cmd_vel_pub.publish(Twist())
    _set_finger_follow(False)
    _set_person_follow(False)


def do_look(direction: str, duration_s: float):
    # Relative nudge: each command moves the head one HEAD_STEP further in the
    # requested direction from where it currently is (clamped to the joint
    # limits), rather than snapping to an extreme. Ask again to keep going.
    # "center" is the exception — it re-homes to 0, 0.
    pan, tilt = _current_head_pose()
    if direction == "left":
        pan = _clamp(pan + HEAD_STEP, -HEAD_PAN_MAX, HEAD_PAN_MAX)
    elif direction == "right":
        pan = _clamp(pan - HEAD_STEP, -HEAD_PAN_MAX, HEAD_PAN_MAX)
    elif direction == "up":
        tilt = _clamp(tilt + HEAD_STEP, HEAD_TILT_DN, HEAD_TILT_UP)
    elif direction == "down":
        tilt = _clamp(tilt - HEAD_STEP, HEAD_TILT_DN, HEAD_TILT_UP)
    elif direction in ("center", "centre"):
        pan, tilt = HEAD_HOME_PAN, HEAD_HOME_TILT
    move_t = max(duration_s, 0.6)
    publish_head_trajectory([(pan, tilt, move_t)])


def do_nod(duration_s: float):
    p, base = _current_head_pose()
    down = _clamp(base - 0.35, HEAD_TILT_DN, HEAD_TILT_UP)
    up   = _clamp(base + 0.20, HEAD_TILT_DN, HEAD_TILT_UP)
    publish_head_trajectory([
        (p, down, 0.5),
        (p, up,   1.0),
        (p, base, 1.5),
    ])


def do_shake_head(duration_s: float):
    base, t = _current_head_pose()
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
    _, t = _current_head_pose()
    publish_head_trajectory([
        ( 0.3, t, 0.4),
        (-0.3, t, 0.8),
        ( 0.3, t, 1.2),
        ( 0.0, t, 1.6),
    ])


def do_twirl(rotations: int, direction: str = "left"):
    """Full base rotations. Duration is computed here from TWIRL_SPEED — the
    LLM only supplies the count, so timing can't be hallucinated. Accuracy is
    open-loop (depends on wheel_separation calibration)."""
    sign = 1.0 if direction == "left" else -1.0
    duration = rotations * 2.0 * math.pi / TWIRL_SPEED
    print(f"Twirling {rotations}x {direction} (~{duration:.1f}s).")
    drive_base(0.0, sign * TWIRL_SPEED, duration)


def _dance_worker():
    """Canned ~14 s choreography: rainbow hair + head bops + base wiggles +
    one full twirl, then back home. Checks _move_stop between phases so
    "Jessica stop" (or the joystick) can cut it off; motion itself is the same
    capped-speed twist publishing as any other move."""
    def base(linear, angular, secs):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(linear), float(angular)
        end = time.time() + secs
        while time.time() < end:
            if _move_stop.is_set():
                return False
            _cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        return True

    # Rainbow hair for the show.
    if _hair_pub is not None:
        msg = Int32(); msg.data = -2
        _hair_pub.publish(msg)

    # Head bops (fire-and-forget trajectory, runs while the base wiggles).
    _, t0 = _current_head_pose()
    publish_head_trajectory([
        ( 0.4, _clamp(t0 - 0.3, HEAD_TILT_DN, HEAD_TILT_UP), 0.6),
        (-0.4, _clamp(t0 + 0.2, HEAD_TILT_DN, HEAD_TILT_UP), 1.2),
        ( 0.4, _clamp(t0 - 0.3, HEAD_TILT_DN, HEAD_TILT_UP), 1.8),
        (-0.4, _clamp(t0 + 0.2, HEAD_TILT_DN, HEAD_TILT_UP), 2.4),
        ( 0.0, t0, 3.0),
    ])

    # Base: sway left/right, then one full twirl, then settle.
    ok = (base(0.0,  TURN_SPEED, 0.8) and base(0.0, -TURN_SPEED, 1.6)
          and base(0.0,  TURN_SPEED, 0.8)
          and base(0.0, TWIRL_SPEED, 2.0 * math.pi / TWIRL_SPEED))
    _cmd_vel_pub.publish(Twist())  # full stop
    if ok:
        publish_head_trajectory([(0.0, 0.0, 1.0)])  # take a bow: home the head


def do_dance():
    """Run the dance in the shared move thread so stop/cancel semantics are
    identical to drive/turn/twirl."""
    if _cmd_vel_pub is None:
        print("  (cmd_vel publisher unavailable — would dance)")
        return
    global _move_thread
    with _move_lock:
        cancel_base_move()
        _move_stop.clear()
        _move_thread = threading.Thread(target=_dance_worker, daemon=True)
        _move_thread.start()


def execute_robot_command(command: dict):
    print("\nRobot command:", json.dumps(command))

    action     = command["action"]
    parameters = command["parameters"]
    duration_s = command["duration_s"]

    if action == "none":
        return

    if action == "stop":
        print("Stopping the base.")
        cancel_base_move()   # abort any in-flight timed move / twirl / dance
        if _cmd_vel_pub is not None:
            _cmd_vel_pub.publish(Twist())
        # "Stop" halts everything, including finger and person following.
        _set_finger_follow(False)
        _set_person_follow(False)
        return

    if action == "follow_finger":
        on = parameters.get("state") == "on"
        print(f"Finger following {'ON' if on else 'OFF'}.")
        _set_finger_follow(on)
        return

    if action == "follow_me":
        on = parameters.get("state") == "on"
        print(f"Person following {'ON' if on else 'OFF'}.")
        _set_person_follow(on)
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

    if action == "twirl":
        do_twirl(parameters.get("rotations", 1), parameters.get("direction", "left"))
        return

    if action == "dance":
        print("Dancing!")
        do_dance()
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


# ---------------------------------------------------------------------
# Conversation logging ("dreaming") — one JSONL line per turn so the prompts
# and command-handling can be reviewed/refined offline, and used to teach
# Jessica. Best-effort: a logging failure must never break the conversation.
# ---------------------------------------------------------------------

LOG_DIR = Path.home() / "jessica_ws" / "logs"

# Reference to the most recently logged turn, so spoken feedback ("that was
# wrong…" / "good girl") can be attached to it for later prompt-tuning/teaching.
_last_turn: dict | None = None


def _write_log(entry: dict):
    """Append one JSON object to today's log file."""
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    path = LOG_DIR / f"jessica_{datetime.datetime.now():%Y-%m-%d}.jsonl"
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")


def log_turn(heard: str, llm_raw: str, reply_spoken: str,
             command: dict, trigger: str):
    """Append one conversation turn to today's JSONL log."""
    global _last_turn
    try:
        now = datetime.datetime.now()
        action = command.get("action")
        params = command.get("parameters", {})
        turn_id = now.strftime("%Y%m%dT%H%M%S%f")
        _write_log({
            "id":           turn_id,
            "type":         "turn",
            "ts":           now.isoformat(timespec="seconds"),
            "trigger":      trigger,                       # idle | conversation | wake
            "heard":        heard,                         # Whisper transcript
            "llm_raw":      llm_raw,                        # raw Ollama output (pre-parse)
            "reply_spoken": reply_spoken,                   # what she actually said
            "action":       action,                         # parsed robot action
            "params":       params,                         # parsed parameters
            "executed":     action not in (None, "", "none"),
        })
        _last_turn = {"id": turn_id, "heard": heard,
                      "action": action, "params": params}
    except Exception as e:                                  # never break the turn
        print(f"(log_turn failed: {e})")


def log_feedback(label: str, note: str) -> bool:
    """Attach spoken feedback to the most recent turn (label = good | bad).
    The entry is self-contained (carries the original input + output) so it can
    become a training pair without joining files. Returns False if there's no
    turn to attach to yet."""
    if _last_turn is None:
        return False
    try:
        now = datetime.datetime.now()
        _write_log({
            "id":          now.strftime("%Y%m%dT%H%M%S%f"),
            "type":        "feedback",
            "ts":          now.isoformat(timespec="seconds"),
            "ref":         _last_turn["id"],       # the turn being judged
            "label":       label,                   # good | bad
            "note":        note,                    # your words, verbatim
            "orig_heard":  _last_turn["heard"],     # what you'd said that turn
            "orig_action": _last_turn["action"],    # what she did (maybe wrong)
            "orig_params": _last_turn["params"],
        })
        return True
    except Exception as e:
        print(f"(log_feedback failed: {e})")
        return False


def log_event(kind: str, heard: str = "", reply_spoken: str = "", trigger: str = ""):
    """Log a non-command exchange so the JSONL is a COMPLETE transcript, not just
    the action-turns: farewells, timeouts, unintelligible speech, and anything
    heard while dormant. Best-effort — a logging failure must never break a turn."""
    try:
        now = datetime.datetime.now()
        _write_log({
            "id":           now.strftime("%Y%m%dT%H%M%S%f"),
            "type":         "event",
            "ts":           now.isoformat(timespec="seconds"),
            "kind":         kind,           # unclear | timeout | farewell | feedback_ack | dormant_ignored
            "trigger":      trigger,
            "heard":        heard,          # transcript ("" if nothing/not understood)
            "reply_spoken": reply_spoken,   # what she said, if anything
        })
    except Exception as e:                  # never break the turn
        print(f"(log_event failed: {e})")


# Phrases that authorise a robot command (STT-robust, punctuation-tolerant so
# "jessica, darling" and "hey, jessica" match). Keep in sync with the prompt.
_COMMAND_PHRASE_RE = (r"\bjessica\W+darling\b", r"\bhey\W+jessica\b")


def _has_command_phrase(text: str) -> bool:
    """True if the utterance addresses Jessica with a command phrase."""
    return any(re.search(p, text) for p in _COMMAND_PHRASE_RE)


def _gate_robot_command(heard: str, command: dict) -> dict:
    """Enforce the wake-phrase gate in CODE so a flaky LLM can't move the robot
    on stray speech (e.g. it once turned "Why?" into follow_finger).

    - A stop needs only "jessica" + "stop" (no "darling"): saying "jessica stop"
      or "stop jessica" halts everything. (A bare "stop" without "jessica" is
      NOT honoured — say "jessica stop" to be sure.)
    - Every OTHER robot command runs only if the utterance contains a command
      phrase ("jessica darling" or "hey jessica"); otherwise it's downgraded to
      conversation only (Jessica still replies, but the robot doesn't act).
    """
    text = (heard or "").lower()
    action = command.get("action")
    # Safety stop: "jessica" + "stop" in any order, no command phrase required.
    if "jessica" in text and re.search(r"\bstop\b", text):
        return {"action": "stop", "parameters": {}, "duration_s": 0.0}
    if action in (None, "", "none"):
        return command
    if not _has_command_phrase(text):
        print(f'  (ignoring robot command "{action}" — no "Jessica darling"/"Hey Jessica")')
        return {"action": "none", "parameters": {}, "duration_s": 0.0}
    return command


def process_turn(user_text: str, mp3_path: Path, trigger: str = "conversation"):
    """Run one full conversation turn: Ollama → execute → speak → log."""
    reply = ask_ollama(user_text)
    command = sanitise_robot_command(reply["robot_command"])
    command = _gate_robot_command(user_text, command)
    execute_robot_command(command)
    speak(reply["say"], mp3_path)
    log_turn(
        heard=user_text,
        llm_raw=reply.get("llm_raw", ""),
        reply_spoken=reply["say"],
        command=command,
        trigger=trigger,
    )


# ---------------------------------------------------------------------
# Main conversation loop
# ---------------------------------------------------------------------

IDLE         = "idle"
CONVERSATION = "conversation"
DORMANT      = "dormant"   # muted after "bye jessica": listening but silent until addressed

# Phrases that bring Jessica back from DORMANT. Broader than the command phrases
# on purpose: a friendly "hi jessica" should un-mute her, even though it won't by
# itself run a robot command (that still needs "jessica darling"/"hey jessica",
# enforced separately by _has_command_phrase). Saying a command phrase wakes AND
# commands in one breath (e.g. "Hey Jessica, turn left").
WAKE_PHRASES = ("jessica darling", "hey jessica", "hello jessica", "hi jessica")

# Phrases that mute Jessica into DORMANT.
FAREWELL_PHRASES = ("bye jessica", "goodbye jessica")

# Spoken feedback on the PREVIOUS turn, logged for the "dreaming"/teaching set.
# Keep these fairly specific so normal chat doesn't trip them. Tune to taste.
APPROVAL_PHRASES   = ("good girl", "well done", "good job",
                      "that was perfect", "perfect jessica", "that was great")
CORRECTION_PHRASES = ("that was wrong", "that is wrong", "that's wrong",
                      "wrong jessica", "you got it wrong", "you got that wrong",
                      "that was not right", "that wasn't right")


def _normalise(text: str) -> str:
    """Lowercase and collapse punctuation/whitespace to single spaces, so that
    Whisper's punctuation ('Bye, Jessica.') doesn't break phrase matching."""
    return re.sub(r"[^a-z0-9]+", " ", text.lower()).strip()


def is_wake_phrase(text: str) -> bool:
    t = _normalise(text)
    return any(p in t for p in WAKE_PHRASES)


def is_farewell(text: str) -> bool:
    t = _normalise(text)
    return any(p in t for p in FAREWELL_PHRASES)


def is_approval(text: str) -> bool:
    t = _normalise(text)
    return any(p in t for p in APPROVAL_PHRASES)


def is_correction(text: str) -> bool:
    t = _normalise(text)
    return any(p in t for p in CORRECTION_PHRASES)


def main():
    global _ros_node, _hair_pub, _cmd_vel_pub, _head_pub, _follow_pub, _follow_me_pub
    global _ui_state_pub, _speech_env_pub
    global conversation, _mic_device_id, _actual_sample_rate, _actual_blocksize

    if ROS_AVAILABLE:
        rclpy.init()
        _ros_node    = rclpy.create_node("jessica_chatbot")
        _hair_pub    = _ros_node.create_publisher(Int32, "/jessica/hair_hue", 10)
        _cmd_vel_pub = _ros_node.create_publisher(Twist, "/cmd_vel", 10)
        _head_pub    = _ros_node.create_publisher(
            JointTrajectory, "/pan_tilt_controller/joint_trajectory", 10)
        _follow_pub  = _ros_node.create_publisher(Bool, "/jessica/finger_follow/enable", 10)
        _follow_me_pub = _ros_node.create_publisher(Bool, "/jessica/person_follow/enable", 10)
        _ui_state_pub  = _ros_node.create_publisher(String, "/jessica/ui_state", 10)
        _speech_env_pub = _ros_node.create_publisher(Float32MultiArray, "/jessica/speech_env", 10)
        # Read the head's real pose (moved by us, the joystick, or finger_follower)
        # so relative gestures base off the true position, not our last command.
        _ros_node.create_subscription(JointState, "/joint_states", _on_joint_states, 10)
        # General stop gesture (both arms raised, from the stop_gesture node).
        # Handled here on the spin thread — completely independent of the audio
        # loop, so it halts moves even while Jessica is recording or speaking.
        _ros_node.create_subscription(Empty, "/jessica/stop", _on_stop_gesture, 10)
        # The main loop blocks on audio I/O, so spin the node in a background
        # thread — otherwise the /joint_states callback would never fire.
        threading.Thread(target=_spin_ros, args=(_ros_node,), daemon=True).start()
        print("ROS 2 publishers ready: /jessica/hair_hue, /cmd_vel, "
              "/pan_tilt_controller/joint_trajectory, /jessica/finger_follow/enable")
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

                    if text is None:
                        continue
                    if text == "":
                        log_event("unclear", trigger="idle")
                        continue

                    state        = CONVERSATION
                    conversation = []
                    # Whisper's first transcription after startup is usually
                    # garbage (mic/ALSA warm-up), so every conversation opens
                    # with a fixed greeting. Keep the real transcript in the
                    # log so we can see what Whisper actually produced.
                    log_event("idle_wake_replaced", heard=text, trigger="idle")
                    process_turn("Hello Jessica", mp3_path, trigger="idle")

                elif state == CONVERSATION:
                    print(f"[CONVERSATION] Listening (timeout {CONVERSATION_TIMEOUT}s)...")
                    text = listen_for_speech(timeout=CONVERSATION_TIMEOUT)

                    if text is None:
                        print("Conversation timed out.")
                        reply = "I'll be here if you need me, love."
                        speak(reply, mp3_path)
                        log_event("timeout", reply_spoken=reply, trigger="conversation")
                        state = IDLE
                        continue

                    if text == "":
                        reply = "Sorry sweetheart, I didn't catch that."
                        speak(reply, mp3_path)
                        log_event("unclear", reply_spoken=reply, trigger="conversation")
                        continue

                    if is_farewell(text):
                        reply = "Bye for now, love. Talk soon."
                        speak(reply, mp3_path)
                        log_event("farewell", heard=text, reply_spoken=reply, trigger="conversation")
                        print("\n[DORMANT] Muted — say a wake phrase "
                              "(e.g. 'Jessica darling') to resume.")
                        state = DORMANT
                        continue

                    # Spoken feedback on the previous turn — logged, not answered.
                    if is_approval(text):
                        if log_feedback("good", text):
                            print("[FEEDBACK] logged 👍")
                            reply = "Thank you, love. I'll remember that."
                        else:
                            reply = "Thank you, love."
                        speak(reply, mp3_path)
                        log_event("feedback_ack", heard=text, reply_spoken=reply, trigger="conversation")
                        continue

                    if is_correction(text):
                        if log_feedback("bad", text):
                            print("[FEEDBACK] logged 👎 (correction)")
                            reply = "Sorry love. I've noted that so I can do better."
                        else:
                            reply = "Sorry love, I'm not sure which bit you mean."
                        speak(reply, mp3_path)
                        log_event("feedback_ack", heard=text, reply_spoken=reply, trigger="conversation")
                        continue

                    process_turn(text, mp3_path)

                elif state == DORMANT:
                    # Listening but silent: ignore everything until directly addressed,
                    # but still log whatever was heard so the transcript is complete.
                    text = listen_for_speech(timeout=None)

                    if not text:
                        continue
                    if not is_wake_phrase(text):
                        log_event("dormant_ignored", heard=text, trigger="dormant")
                        continue

                    state        = CONVERSATION
                    conversation = []
                    process_turn(text, mp3_path, trigger="wake")

        except KeyboardInterrupt:
            print("\nShutting down.")

    if ROS_AVAILABLE and _ros_node is not None:
        _ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
