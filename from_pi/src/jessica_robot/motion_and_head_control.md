# Jessica — Motion & Head Control

Detailed documentation for Jessica's drive base and pan-tilt head, controlled
through `ros2_control` on an ESP32. This covers the hardware interface, the
controllers, the joystick (manual) and chatbot (autonomous) command paths, the
URDF, the launch file, and how to build, run, tune, and troubleshoot it.

> Adapted from an earlier robot's stack. The obsolete reference packages
> (`diffdrive_arduino`, `pan_tilt_hardware`, `autonomous_robot`,
> `pan_tilt_description`) were **not** brought over — only the parts Jessica
> needs were reused. The single C++ package we kept is
> `esp32_combined_hardware`.

---

## 1. Overview

Jessica has:

- **4 drive wheels** (differential drive — left pair vs. right pair), velocity
  controlled, with wheel-encoder feedback for odometry.
- **A pan-tilt mechanism** carrying her head (2 hobby servos: pan = yaw,
  tilt = pitch), position controlled, **open-loop** (no encoder feedback).

Both the wheels and the servos are driven by **one ESP32** over a single USB
serial link. On the ROS side, a single `ros2_control` hardware interface
(`esp32_combined_hardware`) exposes all 6 joints.

Two independent "drivers" can move Jessica:

| Source | Drive base | Head |
|--------|-----------|------|
| **Joystick** (manual) | `teleop_twist_joy` → `/cmd_vel_joy` | `pan_tilt_teleop` → trajectory |
| **Chatbot** (autonomous) | `jessica_chatbot` → `/cmd_vel` | `jessica_chatbot` → trajectory |

For driving, a **twist_mux** gives the joystick higher priority, so manual
control always overrides the chatbot. For the head, both publish to the same
controller topic — last message wins, so grabbing the stick interrupts a
gesture.

---

## 2. Packages

```
jessica_ws/src/
├── esp32_combined_hardware/      # C++ ros2_control hardware interface + joy_button_bridge
│   ├── src/esp32_combined_hardware.cpp
│   ├── include/.../esp32_combined_hardware.hpp
│   ├── src/joy_button_bridge.cpp
│   ├── config/                   # (its own controllers.yaml/joy maps — NOT used by Jessica)
│   ├── plugin.xml
│   └── CMakeLists.txt
└── jessica_robot/                # Jessica's own package
    ├── description/
    │   └── jessica.urdf.xacro     # minimal self-contained URDF + ros2_control block
    ├── config/
    │   ├── jessica_controllers.yaml
    │   ├── joystick.yaml
    │   ├── twist_mux.yaml
    │   └── joy_button_mappings.yaml
    ├── jessica_robot/
    │   ├── jessica_chatbot.py     # voice brain — now also publishes motion/head
    │   ├── hair_led_node.py
    │   └── pan_tilt_teleop.py     # joystick → head trajectories
    └── launch/jessica.launch.py   # brings everything up
```

> The reference robot's old code is kept under `tmp/` for reference and carries a
> `COLCON_IGNORE` so colcon never builds it.

---

## 3. ESP32 serial protocol

The hardware interface talks to the ESP32 with a simple newline-terminated ASCII
protocol on `/dev/esp32_motor` @ `115200` baud.

| Direction | Message | Meaning |
|-----------|---------|---------|
| ROS → ESP32 | `CMD,lf,lr,rf,rr,pan,tilt\n` | wheel velocities (cm/s) + servo positions (rad) |
| ROS → ESP32 | `GET\n` | request a state report |
| ESP32 → ROS | `STATE,lf,lr,rf,rr,lf_v,lr_v,rf_v,rr_v,pan,tilt\n` | 4 encoder positions (counts) + 4 wheel velocities (cm/s) + 2 servo positions (rad) |
| ROS → ESP32 | `AUX,name,arg\n` | one-shot auxiliary command (button actions) |

**Unit conversions** happen inside the hardware interface:

- Wheel position: `counts → rad` using `enc_counts_per_rev` (1425).
- Wheel velocity: `cm/s ↔ rad/s` using `wheel_radius` (7.2 cm — note this is the
  *protocol* radius in centimetres, separate from the controller's metres).

> **If Jessica's ESP32 firmware uses a different protocol, this is the first
> place things break.** The serial format above must match the firmware exactly.

---

## 4. The hardware interface (`esp32_combined_hardware`)

A `hardware_interface::SystemInterface` plugin
(`esp32_combined_hardware/ESP32CombinedHardware`) exporting **6 joints in a
fixed order**:

| Index | Joint | Command interface | State interface(s) |
|------:|-------|-------------------|--------------------|
| 0 | `left_front_wheel_joint`  | velocity | position, velocity |
| 1 | `left_rear_wheel_joint`   | velocity | position, velocity |
| 2 | `right_front_wheel_joint` | velocity | position, velocity |
| 3 | `right_rear_wheel_joint`  | velocity | position, velocity |
| 4 | `pan_joint`               | position | position |
| 5 | `tilt_joint`              | position | position |

The order and interface types are validated in `on_init()` — the URDF
`<ros2_control>` block must list the joints in exactly this order.

### Open-loop servo "feedback"

The hobby servos have **no position feedback**. JointTrajectoryController,
however, requires a position *state* interface and checks goal tolerances
against it. To satisfy this, `parse_state_message()` reports the **last
commanded** pan/tilt as the servo state ("last position moved to"):

```cpp
// Servos are open-loop (no encoder feedback): use the last commanded
// position as the reported state. JTC then sees perfect tracking.
hw_positions_[4] = hw_commands_position_[0];   // pan
hw_positions_[5] = hw_commands_position_[1];   // tilt
```

The wheels keep their **real** encoder feedback (used for odometry); only the
two servos are mirrored. This is the standard pattern for open-loop servos under
`ros2_control`.

### Auxiliary commands

`on_configure()` creates a small ROS node subscribing to `/esp32_aux_cmd`
(`std_msgs/String`). Messages are forwarded to the ESP32 as `AUX,<payload>\n` on
the next `write()`. This is how joystick buttons trigger one-shot actions
(centre head, toggle LED, e-stop) without going through a controller.

---

## 5. Controllers (`config/jessica_controllers.yaml`)

`controller_manager` runs at **50 Hz** and loads three controllers:

| Controller name | Type | Purpose |
|-----------------|------|---------|
| `diff_cont` | `diff_drive_controller/DiffDriveController` | wheels → odom + cmd_vel |
| `pan_tilt_controller` | `joint_trajectory_controller/JointTrajectoryController` | smooth head motion |
| `joint_broad` | `joint_state_broadcaster/JointStateBroadcaster` | publishes `/joint_states` |

### `diff_cont`

```yaml
left_wheel_names:  ['left_front_wheel_joint', 'left_rear_wheel_joint']
right_wheel_names: ['right_front_wheel_joint', 'right_rear_wheel_joint']
wheel_separation: 0.26   # metres — RE-TUNE (wheels are closer now)
wheel_radius:     0.072  # metres
use_stamped_vel: true    # subscribes to TwistStamped on /diff_cont/cmd_vel
```

Because `use_stamped_vel: true`, the controller listens for
**`geometry_msgs/TwistStamped`** on `/diff_cont/cmd_vel`. A `twist_stamper` node
converts the plain `Twist` coming out of `twist_mux` into a stamped message.

### `pan_tilt_controller` (JointTrajectoryController)

Chosen over a simple position controller because it **interpolates between
trajectory points at the controller rate**, giving genuinely smooth, slow head
motion. A gesture is just a list of `(pan, tilt, time_from_start)` points — the
controller handles the timing.

```yaml
joints: [pan_joint, tilt_joint]
command_interfaces: [position]
state_interfaces:   [position]
constraints:                 # open-loop servos → keep tolerances loose
  stopped_velocity_tolerance: 0.0
  goal_time: 0.0
```

It accepts `trajectory_msgs/JointTrajectory` on
**`/pan_tilt_controller/joint_trajectory`** (and also a FollowJointTrajectory
action, which we don't use).

---

## 6. Data flow

### Driving (mux + stamper)

```
                 (priority 100)
joystick ─► teleop_twist_joy ─► /cmd_vel_joy ─┐
                                              ├─► twist_mux ─► /diff_cont/cmd_vel_unstamped
chatbot ───────────────────────► /cmd_vel ────┘                       │
                 (priority 10)                                         ▼
                                                          twist_stamper (adds header)
                                                                      │
                                                                      ▼
                                                  /diff_cont/cmd_vel (TwistStamped)
                                                                      │
                                                                      ▼
                                                      DiffDriveController ─► ESP32 wheels
```

`twist_mux` (`config/twist_mux.yaml`) publishes whichever input is active and
highest priority; each input times out after 0.5 s. The chatbot therefore
**republishes its Twist at 20 Hz** for the duration of a move so the command
doesn't lapse mid-motion — and the joystick can pre-empt it at any instant.

### Head

```
joystick ─► pan_tilt_teleop ─┐
                             ├─► /pan_tilt_controller/joint_trajectory ─► JTC ─► ESP32 servos
chatbot ─────────────────────┘
```

Both publishers target the same topic; the most recent trajectory wins.

---

## 7. Joystick (manual control)

Manual control is the override path used when autonomous control is off. Three
nodes consume `/joy` (Xbox/PS layout assumed):

### Driving — `teleop_twist_joy` (`config/joystick.yaml`)

| Control | Action |
|---------|--------|
| Left stick vertical (axis 1) | forward / back (0.3 m/s, turbo 0.6) |
| Left stick horizontal (axis 0) | turn (1.0 rad/s, turbo 2.0) |
| **Back (button 6)** | hold-to-drive enable |
| **Start (button 7)** | hold-to-drive turbo |

`require_enable_button: true` — the base won't move unless the enable button is
held. Publishes to `/cmd_vel_joy`.

### Head — `pan_tilt_teleop` (`jessica_robot/pan_tilt_teleop.py`)

Reads `/joy`, integrates the right stick into target pan/tilt angles, and emits a
single short-duration `JointTrajectory` point per update (smoothly tracked by
JTC).

| Control | Action |
|---------|--------|
| Right stick horizontal (axis 2) | pan ±1.57 rad |
| Right stick vertical (axis 3) | tilt −1.5 … +0.87 rad |

Tunable via node parameters: `pan_axis`, `tilt_axis`, `invert_pan`,
`invert_tilt`, `pan_speed`, `tilt_speed`, `point_time`, limits, etc.

### Buttons — `joy_button_bridge` (`config/joy_button_mappings.yaml`)

Rising-edge button presses become `AUX` commands via `/esp32_aux_cmd`:

| Button | Command sent | Effect (firmware-dependent) |
|--------|--------------|------------------------------|
| A (0) | `center_pan_tilt,0` | recentre the head |
| B (1) | `toggle_led,1` | toggle ESP32 onboard LED |
| Y (3) | `estop,1` | emergency stop |

> The old robot's `fire` (watergun) mapping was removed — Jessica has a head
> there instead.

---

## 8. Chatbot (autonomous control)

`jessica_chatbot.py` parses the LLM's `robot_command` and acts on it. Robot
commands only fire when Jonny prefixes a request with **"Jessica darling"**
(enforced in the system prompt). Publishers created on startup:

- `/cmd_vel` (`geometry_msgs/Twist`) — autonomous drive (twist_mux priority 10)
- `/pan_tilt_controller/joint_trajectory` (`trajectory_msgs/JointTrajectory`) — head
- `/jessica/hair_hue` (`std_msgs/Int32`) — LED colour (pre-existing)

### Action → behaviour

| Action | Implementation |
|--------|----------------|
| `drive` (forward/backward) | hold Twist `linear.x = ±0.15 m/s` for `duration_s`, then stop |
| `turn` (left/right) | hold Twist `angular.z = ±0.6 rad/s` for `duration_s`, then stop |
| `stop` | publish zero Twist |
| `look` (l/r/u/d/centre) | single trajectory point to the target pose |
| `nod` | tilt down → up → back (3-point trajectory) |
| `shake_head` | pan left → right → left → centre (4-point) |
| `wave` | gentle pan wiggle (no arm — a friendly head wiggle) |
| `change_hair_color` | publish hue to `/jessica/hair_hue` |
| `none` | do nothing |

Speeds are intentionally gentle (the system prompt forbids fast movement). The
chatbot tracks the head's last commanded pose (`_head_pan`, `_head_tilt`) so
gestures build relative to wherever the head currently is.

Relevant tuning constants near the top of the file:

```python
DRIVE_SPEED  = 0.15   # m/s
TURN_SPEED   = 0.6    # rad/s
HEAD_PAN_MAX = 1.4    # rad
HEAD_TILT_UP = 0.8    # rad (positive = up)
HEAD_TILT_DN = -1.4   # rad (negative = down)
```

---

## 9. URDF (`description/jessica.urdf.xacro`)

A deliberately **minimal, self-contained** URDF — no lidar/camera/gazebo/inertial
clutter. It exists so `robot_state_publisher` has a valid tree and the
controller_manager can load the ESP32 hardware.

Link tree: `base_link → base_footprint`, `base_link → chassis → mast →
pan_link → head`, plus four wheel links on `base_link`. The
`<ros2_control>` block (Section 4) is included inline, gated on
`use_ros2_control:=true`.

Key geometry knobs at the top (re-tune to the real robot): `wheel_radius`,
`wheel_sep_x/y`, `chassis_*`, `mast_height`. Joint limits: pan ±1.57, tilt
−1.5…+0.87.

Render the URDF to check it:

```bash
xacro src/jessica_robot/description/jessica.urdf.xacro use_ros2_control:=true sim_mode:=false
```

---

## 10. Launch (`launch/jessica.launch.py`)

`ros2 launch jessica_robot jessica.launch.py` brings up everything:

| Node | Role |
|------|------|
| `robot_state_publisher` | publishes TF from the URDF |
| `controller_manager` (`ros2_control_node`) | loads hardware + controllers (delayed 3 s to let the serial port settle) |
| 3× `spawner` | `joint_broad`, `diff_cont`, `pan_tilt_controller` (started on controller_manager start) |
| `joy_node` | reads the gamepad |
| `teleop_twist_joy` | joystick → `/cmd_vel_joy` |
| `pan_tilt_teleop` | joystick → head trajectories |
| `joy_button_bridge` | joystick buttons → `/esp32_aux_cmd` |
| `twist_mux` + `twist_stamper` | velocity arbitration + stamping |
| `jessica_chatbot` | voice brain |
| `hair_led_node` | LEDs |

Jessica's own Python nodes are launched with `PYTHONUNBUFFERED=1` and
`emulate_tty=True` so their prints appear in the launch console.

---

## 11. Build & run

```bash
cd ~/jessica_ws
colcon build --packages-select esp32_combined_hardware jessica_robot
source install/setup.bash

# Full stack (needs the ESP32 + joystick connected):
ros2 launch jessica_robot jessica.launch.py

# Chatbot + LEDs only — no hardware required:
ros2 launch jessica_robot jessica.launch.py hardware:=false
```

The `hardware` launch argument defaults to `true`. Set `hardware:=false` to skip
the ESP32 / joystick / ros2_control nodes and run only `jessica_chatbot` +
`hair_led_node` (use this when the robot hardware isn't connected — the full
launch will otherwise error without the ESP32, a joystick, and
`ros-jazzy-teleop-twist-joy`).

(First full build also compiles the C++ hardware interface, which needs
`libserial-dev` and `pal_statistics` — both already present.)

---

## 12. Prerequisites / first-run setup

1. **Install the joystick teleop package** (the one missing dependency):
   ```bash
   sudo apt install ros-jazzy-teleop-twist-joy
   ```
2. **Serial device** — the URDF expects `/dev/esp32_motor`. Either add a udev
   rule that symlinks the ESP32 to that name, or change the `device` param in
   `description/jessica.urdf.xacro` to the raw port (e.g. `/dev/ttyUSB0`).

   Example udev rule (`/etc/udev/rules.d/99-esp32.rules`) — fill in your
   adapter's IDs from `udevadm info -a -n /dev/ttyUSB0`:
   ```
   SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", SYMLINK+="esp32_motor"
   ```
   Then `sudo udevadm control --reload && sudo udevadm trigger`.

---

## 13. Tuning after the first run

- **`wheel_separation`** (`jessica_controllers.yaml`) — 0.26 m is a guess since
  the wheels moved closer. Measure left-track to right-track centre distance and
  set it; this directly affects turn rate accuracy.
- **Pan/tilt direction** — if the head pans/tilts the wrong way, either flip
  `invert_pan` / `invert_tilt` (joystick teleop params) and/or swap the signs in
  the chatbot's `do_look` / `HEAD_*` constants. Servo wiring decides this.
- **Speeds** — `DRIVE_SPEED` / `TURN_SPEED` in the chatbot; `scale_linear` /
  `scale_angular` in `joystick.yaml`.
- **Gesture feel** — amplitudes and timings in `do_nod` / `do_shake_head` /
  `do_wave`.

---

## 14. Manual test cheatsheet

With the launch running (or `controller_manager` up), in another sourced
terminal:

```bash
# Controllers should be 'active'
ros2 control list_controllers

# Move the head directly (centre)
ros2 topic pub --once /pan_tilt_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [pan_joint, tilt_joint], points: [{positions: [0.0, 0.0], time_from_start: {sec: 1}}]}"

# Nudge pan to +0.5 over 1 s
ros2 topic pub --once /pan_tilt_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [pan_joint, tilt_joint], points: [{positions: [0.5, 0.0], time_from_start: {sec: 1}}]}"

# Drive forward slowly via the autonomous channel (mux priority 10)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# Watch joint states / odom
ros2 topic echo /joint_states
ros2 topic echo /diff_cont/odom

# Fire a button-style aux command by hand
ros2 topic pub --once /esp32_aux_cmd std_msgs/msg/String "{data: 'center_pan_tilt,0'}"
```

---

## 15. Troubleshooting

| Symptom | Likely cause / fix |
|---------|--------------------|
| `controller_manager` errors opening serial | ESP32 not on `/dev/esp32_motor`, wrong permissions, or busy. Check the udev symlink and `dialout` group. |
| Hardware loads but `STATE` parse warnings | ESP32 firmware protocol mismatch (Section 3). Confirm the firmware emits 10 comma-separated values prefixed `STATE`. |
| Head won't move from chatbot but joystick works | `pan_tilt_controller` not active, or chatbot publishing before the spawner is up. Check `ros2 control list_controllers`. |
| Base won't move | `diff_cont` inactive; or nothing converting Twist→TwistStamped (check `twist_stamper`); or joystick enable button required. |
| Joystick drives but head dead (or vice-versa) | `pan_tilt_teleop` axes (`pan_axis`/`tilt_axis`) don't match your pad. Echo `/joy` and adjust. |
| Movement is mirrored/backwards | Invert pan/tilt or wheel direction — see Section 13. |
| `teleop_twist_joy` package not found | `sudo apt install ros-jazzy-teleop-twist-joy`. |
| Colcon builds the old `tmp/` packages | Ensure `tmp/COLCON_IGNORE` exists (or delete `tmp/`). |
