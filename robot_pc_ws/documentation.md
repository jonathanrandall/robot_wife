# Jessica Robot — Stereo Vision and Pose System

## Overview

This workspace runs on the robot PC and processes the stereo camera stream
published by the Raspberry Pi.  It rectifies the fisheye images, runs MediaPipe
pose and hand estimation on both eyes, computes metric 3-D positions for each
landmark using stereo disparity, and publishes an annotated video stream (for
monitoring in rqt), a `PersonState` message, and a `HandState` message that
the RPi controller subscribes to for task execution.

```
RPi                              Robot PC
────────────────                 ──────────────────────────────────────
camera_publisher   ──────────►  stereo_pose_node
  /jessica/camera/               │  rectify (fisheye remap)
    image/compressed             │  MediaPipe PoseLandmarker (L + R eye)
                                 │  MediaPipe HandLandmarker (L + R eye)
                                 │  stereo depth (Q matrix)
                                 │  FPS overlay + skeleton annotation
                                 ▼
                    /jessica/camera/pose/compressed  → rqt image view
                    /jessica/person_state            → RPi controller
                    /jessica/hand_state              → RPi controller
```

---

## Packages

### `person_state_msgs`

Custom ROS 2 message definitions.  This is a CMake package so the generated
Python/C++ bindings are available to any node on the network.

---

#### `msg/Landmark.msg`

A single pose or hand landmark with 3-D position and validity flag.

| Field | Type | Description |
|---|---|---|
| `position` | `geometry_msgs/Point` | 3-D position in the camera optical frame (metres). Zero when `depth_valid` is false. |
| `visibility` | `float32` | MediaPipe visibility score [0, 1]. For pose: minimum of left-eye and right-eye scores. For hand: set to 1.0 (hand model does not output visibility). |
| `depth_valid` | `bool` | True when stereo depth was available and `position.z` is metric depth. |

---

#### `msg/PersonState.msg`

Full body pose state published at the camera frame rate.

| Field | Type | Description |
|---|---|---|
| `header` | `std_msgs/Header` | Timestamp and `frame_id` from the source image. Use with TF to transform positions into other frames. |
| `person_visible` | `bool` | True when at least one person was detected in this frame. All other fields are zero/false when this is false. |
| `nose` | `Landmark` | Nose tip. |
| `left_shoulder` | `Landmark` | Person's anatomical left shoulder. |
| `right_shoulder` | `Landmark` | Person's anatomical right shoulder. |
| `left_elbow` | `Landmark` | |
| `right_elbow` | `Landmark` | |
| `left_wrist` | `Landmark` | |
| `right_wrist` | `Landmark` | |
| `left_hip` | `Landmark` | |
| `right_hip` | `Landmark` | |
| `left_knee` | `Landmark` | |
| `right_knee` | `Landmark` | |
| `left_ankle` | `Landmark` | |
| `right_ankle` | `Landmark` | |
| `shoulder_midpoint` | `geometry_msgs/Point` | Midpoint of both shoulders in camera frame (metres). Primary reference for the following task. |
| `shoulder_midpoint_valid` | `bool` | True when both shoulders had valid stereo depth. |
| `pointing_active` | `bool` | True when a pointing gesture is detected. |
| `pointing_arm` | `uint8` | 0 = not pointing, 1 = left arm, 2 = right arm. |
| `pointing_ray` | `geometry_msgs/Vector3` | Unit direction vector (elbow → wrist) in the camera optical frame. Transform via TF to get the ray in the robot/world frame. |

> **Note:** MediaPipe labels left/right relative to the *person* (anatomical),
> not the camera.  `left_shoulder` is the person's anatomical left.

---

#### `msg/HandState.msg`

Hand landmark state for both hands, published at the camera frame rate.

| Field | Type | Description |
|---|---|---|
| `header` | `std_msgs/Header` | Timestamp and `frame_id` from the source image. |
| `left_hand_detected` | `bool` | True when a "Left" hand was detected in both eyes. |
| `right_hand_detected` | `bool` | True when a "Right" hand was detected in both eyes. |
| `left_hand_landmarks` | `Landmark[21]` | All 21 landmarks for the left hand (camera optical frame, metres). |
| `right_hand_landmarks` | `Landmark[21]` | All 21 landmarks for the right hand. |
| `left_index_tip` | `Landmark` | Convenience field — index fingertip (landmark 8) of the left hand. |
| `right_index_tip` | `Landmark` | Convenience field — index fingertip (landmark 8) of the right hand. |

**Hand landmark indices**

```
0   wrist
1-4   thumb  (cmc, mcp, ip, tip)
5-8   index  (mcp, pip, dip, tip)  ← tip = 8
9-12  middle (mcp, pip, dip, tip)
13-16 ring   (mcp, pip, dip, tip)
17-20 pinky  (mcp, pip, dip, tip)
```

> **Handedness convention:** MediaPipe reports handedness from the camera's
> point of view (mirror convention).  For a person facing the camera,
> MediaPipe's "Left" hand is typically the person's anatomical *right* hand
> and vice versa.  Account for this in the controller.

---

### `stereo_pose_publisher`

#### `stereo_pose_node`

**Source:** `src/stereo_pose_publisher/stereo_pose_publisher/stereo_pose_node.py`

Subscribes to the side-by-side compressed image published by the Jessica stereo
camera (left eye | right eye, 640×240 px total). For every incoming frame it:

1. Splits the frame into a 320×240 left half and a 320×240 right half.
2. Applies fisheye lens rectification to each half using pre-computed remap
   tables from the calibration file.
3. Runs MediaPipe **PoseLandmarker** on each rectified half independently,
   detecting 33 body landmarks.
4. Runs MediaPipe **HandLandmarker** on each rectified half independently,
   detecting up to 2 hands with 21 landmarks each.
5. Draws the pose skeleton (green joints, red bones) and hand skeleton
   (cyan joints, magenta bones) on each half.
6. Annotates metric depth (Z) for the nose and both shoulders on the left eye
   image.
7. Draws a live FPS counter in the top-left corner of the combined image.
8. Re-combines the annotated halves side-by-side and publishes the result.
9. Builds and publishes `PersonState` (body landmarks, shoulder midpoint,
   pointing gesture) and `HandState` (hand landmarks, index fingertips).

**Parameters**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `use_gpu_pose` | `bool` | `false` | Run the pose landmarker on the GPU. |
| `use_gpu_hand` | `bool` | `false` | Run the hand landmarker on the GPU. |

> **GPU note:** Both models default to CPU.  Running both on GPU simultaneously
> causes a MediaPipe tensor write-contention error (`Multiple writes to a Tensor
> instance`), and in practice both-CPU is also faster than the mixed CPU/GPU
> split.  GPU options are available for experimentation.

**Performance**

The four model inferences (pose-left, pose-right, hand-left, hand-right) run in
parallel via a `ThreadPoolExecutor` with 4 workers.  This means the per-frame
cost is roughly the time of the slowest single inference rather than the sum of
all four.  JPEG quality is set to 65 (reduced from 80) for faster encode/decode
with negligible visual difference at rqt viewing distances.

**Depth calculation**

After rectification, matching 3-D points lie on the same horizontal scan-line
in both eye images. For a landmark at pixel `x_L` (left) and `x_R` (right):

```
disparity  d = x_L - x_R                          (pixels, positive for real objects)
depth      Z = Q[2,3] / (Q[3,2] * d + Q[3,3])    (metres)
```

**Back-projection to camera frame**

Given metric depth `Z` and the rectified projection matrix `P1`:

```
X_cam = (x_px - cx) * Z / f
Y_cam = (y_px - cy) * Z / f
Z_cam = Z
```

The result is in the camera optical frame (Z forward, X right, Y down).

**Calibration values** (from `fisheye_stereo_charuco_calibration_v2.npz`)

| Parameter | Value |
|---|---|
| Focal length `f` | 185.05 px (rectified) |
| Principal point `cx` | 170.0 px |
| Principal point `cy` | 132.6 px |
| Stereo baseline | ~62 mm |
| Left RMS reprojection error | 0.336 px |
| Right RMS reprojection error | 0.346 px |
| Stereo RMS reprojection error | 0.440 px |

**Pointing detection**

An arm is classified as pointing when the elbow angle (shoulder–elbow–wrist)
exceeds 150°. The test uses 3-D metric positions so it is invariant to the
person's distance from the camera. The `pointing_ray` unit vector (elbow →
wrist direction in camera frame) is included in `PersonState` so the
controller can transform it via TF into the robot's reference frame.

**Configuration files** (in the package share directory)

| File | Description |
|---|---|
| `config/fisheye_stereo_charuco_calibration_v2.npz` | Fisheye stereo calibration — remap tables, Q matrix, P1 matrix |
| `config/pose_landmarker_full.task` | MediaPipe pose model. Swap for `pose_landmarker_lite.task` if CPU-bound. |
| `config/hand_landmarker.task` | MediaPipe hand model (21 landmarks per hand, up to 2 hands). |

---

## ROS Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/jessica/camera/image/compressed` | `sensor_msgs/CompressedImage` | Subscribe | Raw side-by-side stereo frame from RPi |
| `/jessica/camera/pose/compressed` | `sensor_msgs/CompressedImage` | Publish | Annotated frame with skeleton, depth labels, and FPS counter |
| `/jessica/person_state` | `person_state_msgs/PersonState` | Publish | 3-D body landmark positions, shoulder midpoint, pointing state |
| `/jessica/hand_state` | `person_state_msgs/HandState` | Publish | 3-D hand landmark positions, index fingertips |

---

## Running the System

### Prerequisites

- Camera publisher running on the RPi and reachable on the same ROS 2 network
- ROS 2 Jazzy installed on the robot PC
- Python venv at `~/venvs/jazzy` with `mediapipe` installed
- `empy==3.3.4` in the venv (see Notes below)

### Build

```bash
cd ~/projects/robot_wife/software/robot_pc_ws
source /opt/ros/jazzy/setup.bash

# Build the message package first (CMake — generates Python bindings)
colcon build --packages-select person_state_msgs

# Build the pose publisher (symlink-install so edits take effect immediately)
colcon build --packages-select stereo_pose_publisher --symlink-install
```

### Launch

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Default (both on CPU — fastest and avoids tensor contention error)
ros2 launch stereo_pose_publisher stereo_pose.launch.py

# Both on GPU (may trigger tensor contention error)
ros2 launch stereo_pose_publisher stereo_pose.launch.py use_gpu_pose:=true use_gpu_hand:=true
```

### View the annotated image in rqt

In a second terminal with the same environment sourced:

```bash
rqt
```

Inside rqt: **Plugins → Visualization → Image View**, then set the topic
drop-down to `/jessica/camera/pose/compressed`.

The FPS counter is displayed in the top-left corner of the image in green.

### Inspect topics

```bash
ros2 topic echo /jessica/person_state
ros2 topic echo /jessica/hand_state
ros2 topic hz   /jessica/camera/pose/compressed
```

---

## Planned Tasks

### Task 1 — Point and navigate

The person extends one arm to point in a direction and says "go over there"
(optionally with a distance command).

**Data flow:**
1. `stereo_pose_node` detects the pointing gesture and publishes `pointing_ray`
   in the camera optical frame inside `PersonState`.
2. The RPi controller applies a TF transform to convert the ray from the camera
   frame into the robot base frame (requires the camera → robot TF from the URDF).
3. The robot either rotates its head to look along the ray, or navigates in
   that direction (distance from the voice command).

### Task 2 — Follow me

The robot follows the person, maintaining a configurable relative position
(behind, left, or right — set by voice command).

**Data flow:**
1. `stereo_pose_node` publishes `shoulder_midpoint` (Z = distance, X = lateral
   offset in camera frame) inside `PersonState`.
2. The RPi controller drives the robot to maintain a target distance and lateral
   offset.
3. If following left or right: the head pan servo keeps the person centred.
4. If the person is lost (`person_visible == false`): the robot stops and pans
   the head to search until re-detection.

### Task 3 — Follow my finger (head tracking)

The person holds up their index finger and says "follow my finger".  The robot
head tracks the fingertip as it moves.

**Data flow:**
1. `stereo_pose_node` publishes `left_index_tip` or `right_index_tip` in
   `HandState` with 3-D position in camera frame.
2. The RPi controller computes pan/tilt error from the fingertip X, Y pixel
   position relative to the image centre.
3. A proportional (or PID) controller drives the pan-tilt servos to zero the
   error, keeping the fingertip centred in the image.
4. The LLM activates/deactivates this mode in response to voice commands —
   it does not participate in the real-time control loop.

---

## TF / URDF (TODO)

The camera is mounted on a pan-tilt servo. Once the URDF is written:

- The TF tree will include the camera optical frame so that `header.frame_id`
  in `PersonState` and `HandState` can be used directly with
  `tf2_ros.Buffer.transform()`.
- `pointing_ray`, landmark positions, and fingertip positions can be
  transformed into the robot base frame or world frame for navigation and
  head control.

---

## System Architecture Notes

**Processing split**

| Component | Runs on | Reason |
|---|---|---|
| Camera capture + publish | RPi | Low latency, close to hardware |
| Fisheye rectification | PC | Calibration data lives here |
| MediaPipe pose + hand | PC (CPU) | Too heavy for RPi; CPU faster than GPU for this setup |
| PersonState / HandState publish | PC | Output of PC processing |
| Servo PID control loops | RPi | Must be low latency, no network in loop |
| LLM (Llama 3.2:3b) | PC (GPU) | Shared GPU with Whisper + MediaPipe |
| Whisper STT | PC (GPU) | |

**GPU load (PC)**

Whisper small.en and Llama 3.2:3b share the GPU.  Both MediaPipe models
(pose and hand) default to CPU, which avoids a tensor write-contention error
that occurs when both run on the GPU simultaneously, and is also faster in
practice than the mixed CPU/GPU split.

**LLM role**

The LLM sets *modes* and *goals* (e.g. "start following", "stop", "go over
there") in response to voice commands.  The RPi executes those modes in tight
real-time loops.  The LLM is not in any control loop — only in the
command/intent layer.

---

## RPi Controller — Getting Started

This section is a quick-start guide for the Claude session running on the RPi
that will write the subscriber and finger-following code.

### Step 1 — Get the message package onto the Pi

The `person_state_msgs` package must be built on the Pi before any subscriber
can use it.  Copy the package from the PC:

```bash
# Run on the PC — adjust the Pi's hostname/IP as needed
scp -r ~/projects/robot_wife/software/robot_pc_ws/src/person_state_msgs \
    pi@jessica.local:~/ros2_ws/src/
```

Then on the Pi:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select person_state_msgs
source install/setup.bash
```

### Step 2 — Topics to subscribe to

| Topic | Message type | Purpose |
|---|---|---|
| `/jessica/hand_state` | `person_state_msgs/HandState` | Index fingertip position for finger following |
| `/jessica/person_state` | `person_state_msgs/PersonState` | Body pose, shoulder midpoint, pointing |

### Step 3 — Key fields for finger following

Check detection first, then read the fingertip position:

```python
from person_state_msgs.msg import HandState

def hand_callback(msg: HandState):
    # Pick whichever hand is detected — prefer the one that is raised
    if msg.right_hand_detected and msg.right_index_tip.depth_valid:
        tip = msg.right_index_tip
    elif msg.left_hand_detected and msg.left_index_tip.depth_valid:
        tip = msg.left_index_tip
    else:
        return  # no hand visible

    # tip.position.x  — metres left/right in camera frame (right = positive)
    # tip.position.y  — metres up/down in camera frame   (down  = positive)
    # tip.position.z  — metric depth from camera (metres)
```

### Step 4 — Converting fingertip position to pan/tilt error

The camera image is 320×240 per eye.  The calibration values are:

| Value | Symbol | Number |
|---|---|---|
| Focal length | f | 185.05 px |
| Principal point x | cx | 170.0 px |
| Principal point y | cy | 132.6 px |
| Image centre x | — | 160 px  (EYE_WIDTH / 2) |
| Image centre y | — | 120 px  (EYE_HEIGHT / 2) |

Back-calculate pixel position from the 3-D point:

```python
f, cx, cy = 185.05, 170.0, 132.6

x_px = (tip.position.x / tip.position.z) * f + cx
y_px = (tip.position.y / tip.position.z) * f + cy

# Error from image centre (pixels)
pan_error  = x_px - 160   # positive = fingertip is to the right
tilt_error = y_px - 120   # positive = fingertip is below centre
```

Drive the pan servo to zero `pan_error` and the tilt servo to zero
`tilt_error`.  A simple proportional controller is a good starting point:

```python
PAN_GAIN  = 0.001   # tune these
TILT_GAIN = 0.001

pan_command  = -PAN_GAIN  * pan_error
tilt_command = -TILT_GAIN * tilt_error
```

### Step 5 — Handedness note

MediaPipe uses a **mirror convention** for hand labels.  For a person facing
the camera, MediaPipe's `right_hand` in `HandState` is typically the person's
anatomical *left* hand.  This does not affect finger following (you just use
whichever hand is raised) but matters for gesture interpretation.

### Step 6 — LLM mode activation

The finger-following loop should be a mode that the LLM activates and
deactivates in response to voice commands (e.g. "follow my finger" / "stop").
The LLM sets a flag; the RPi control loop reads that flag each cycle.  The LLM
should **not** be called inside the servo control loop — only at the
command/intent layer.

### Step 7 — Subscribing to PersonState (follow-me and point-and-navigate)

`PersonState` carries the full body skeleton, shoulder midpoint (for follow-me),
and the pointing ray (for point-and-navigate).

```python
from person_state_msgs.msg import PersonState

def person_callback(msg: PersonState):
    if not msg.person_visible:
        # Person lost — stop and search
        return

    # ── Follow-me ──────────────────────────────────────────────────────────
    # shoulder_midpoint is the average of both shoulders in camera frame (metres)
    # Z = distance to person,  X = lateral offset (positive = person to the right)
    if msg.shoulder_midpoint_valid:
        distance      = msg.shoulder_midpoint.z   # metres
        lateral_error = msg.shoulder_midpoint.x   # metres (positive = right)
        # Drive robot to maintain target distance; use lateral_error for steering

    # ── Point-and-navigate ──────────────────────────────────────────────────
    # pointing_arm:  0 = not pointing,  1 = person's left arm,  2 = person's right arm
    if msg.pointing_active:
        arm = msg.pointing_arm   # 1 or 2
        # pointing_ray is a unit vector in the camera optical frame
        # (Z forward, X right, Y down)
        ray_x = msg.pointing_ray.x
        ray_y = msg.pointing_ray.y
        ray_z = msg.pointing_ray.z
        # Transform this ray via TF into the robot base frame to get
        # the navigation direction.

    # ── Individual landmarks ────────────────────────────────────────────────
    # Each Landmark has:  .position.x / .y / .z  (metres, camera frame)
    #                     .visibility             (0-1 confidence)
    #                     .depth_valid            (True = z is metric)
    if msg.nose.depth_valid:
        nose_z = msg.nose.position.z   # distance to face
```

**Key PersonState fields summary**

| Field | When to use |
|---|---|
| `person_visible` | Gate all other fields — skip if False |
| `shoulder_midpoint` + `shoulder_midpoint_valid` | Primary follow-me reference point |
| `pointing_active` + `pointing_arm` + `pointing_ray` | Point-and-navigate gesture |
| `nose`, `left_shoulder`, `right_shoulder`, `left_wrist`, `right_wrist`, `left_elbow`, `right_elbow` | Any task needing specific body part positions |
| `left_hip`, `right_hip`, `left_knee`, `right_knee`, `left_ankle`, `right_ankle` | Lower body landmarks when visible |
| `header.frame_id` | Use with `tf2_ros.Buffer.transform()` once URDF/TF is set up |

### Step 8 — LLM mode activation for body-level tasks

Same pattern as finger following.  The LLM sets a mode string or enum
(e.g. `"follow_me"`, `"point_navigate"`, `"idle"`) in response to voice
commands.  The RPi control loop reads the mode each cycle and selects which
fields from `PersonState` and `HandState` to act on.  Never call the LLM inside
a control loop.

---

## Notes

- **`empy` version:** Running `pip install mediapipe` upgrades `empy` to 4.x
  which breaks `colcon build` for CMake packages.  If the build fails with a
  `TransientParseError`, fix it with:
  ```bash
  ~/venvs/jazzy/bin/pip install "empy==3.3.4"
  ```
- **Handedness:** MediaPipe hand handedness uses a mirror convention — for a
  person facing the camera, "Left" in the message is typically the person's
  anatomical right hand.
- **Pose left/right:** MediaPipe pose landmark labels are anatomical (relative
  to the person), so `left_shoulder` is the person's own left shoulder.
- **Model swap:** Replace `pose_landmarker_full.task` with
  `pose_landmarker_lite.task` in `config/` for faster but less accurate pose
  detection.  No code changes required.
