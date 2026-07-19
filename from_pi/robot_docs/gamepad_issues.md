# Gamepad: mode issues + dual-mode support plan

## ⏭ NEXT: D-mode capture session (paused 2026-07-08, kid's bedtime)

Everything is ready; no launch needed. When Jonny is back:

1. **Jonny**: flip the pad to **D-input mode** (note the combo — still unrecorded!),
   confirm with `~/jessica_ws/gamepad_mode.sh` (should say D-INPUT, 3537:1041), say go.
2. **Claude**: starts a bare `joy_node` in the background and records `/joy` to a file.
   (Nothing else running → no robot reacts to presses. Pad is currently still X-input.)
3. **Jonny performs this sequence, pausing ~1 second between each action:**
   - Left stick: push **LEFT**, release — then push **UP**, release
   - Right stick: push **LEFT**, release — then push **UP**, release
   - Press and release, in order: **A, B, X, Y, LB, RB, LT, RT, View, Menu, Logo**
     (View = small button left of the logo; Menu = small button right of the logo —
     this pad uses Xbox-One-style labels for what 360 pads call Back/Start)
4. **Claude**: stops recording, derives the D-mode axis/button table, **writes the raw
   recorded table below in this file for Jonny to check**, then fills the verified
   numbers (sticks, deadman, and the A/B/Y estop trio) into `config/gamepad_dinput.yaml`
   and rebuilds.
5. **Jonny**: flips back to X-input, `gamepad_mode.sh` confirms, done.

### Recorded D-mode table (captured 2026-07-08, 1965 msgs, 8 axes / 19 buttons)

Raw event order recorded (t = seconds into capture):
sticks redone cleanly at t≈17–31s after a false start; buttons at t≈35–48s in
the order **0, 1, 3, 4, 8(+axis5), 9(+axis4), 6, 7, 10, 11, 12** — i.e. Jonny
pressed the triggers before the bumpers. Buttons 8/9 co-fired analog axes
resting at +1.0 (one a partial half-pull), proving 8/9 are the triggers; the
whole layout is the standard Linux HID BTN_ ordering.

| Control | D-mode index | Notes |
|---|---|---|
| Left stick X | axis 0 | left = + |
| Left stick Y | axis 1 | up = + |
| Right stick X | axis 2 | left = + (old config's guess confirmed) |
| Right stick Y | axis 3 | up = + |
| RT analog | axis 4 | rests +1.0, pulled → −1.0 |
| LT analog | axis 5 | rests +1.0, pulled → −1.0 |
| D-pad | axes 6/7 | assumed, not captured |
| A | button 0 | |
| B | button 1 | |
| X | button 3 | (button 2 is a reserved gap) |
| Y | button 4 | |
| LB | button 6 | old deadman — so it was LB all along, same as X-profile |
| RB | button 7 | (button 5 is a reserved gap) |
| LT digital | button 8 | fires alongside axis 5 |
| RT digital | button 9 | fires alongside axis 4 |
| View | button 10 | |
| Menu | button 11 | |
| Logo | button 12 | |

Applied to `config/gamepad_dinput.yaml` (sticks/deadman confirmed, estop trio
added as A=0, B=1, Y=4) — profile no longer a guess. Raw capture kept this
session at scratchpad/joy_capture.yaml (temporary).


*Written 2026-07-07. Status: **IMPLEMENTED 2026-07-08** — auto-detect built as planned, offline-tested, clean-rebuilt. Still outstanding: (1) the 5-minute D-mode capture session to replace the guessed values in `gamepad_dinput.yaml`, (2) record the pad's mode-switch combo here. Also fixed: `gamepad_mode.sh` no longer kills the shell when sourced with no dongle (uses `return`, not `exit`).*

## Background (what happened today)

The pad is a Zikway 2.4G "XBOX 360 For Windows" clone with **three modes**, distinguishable by USB product ID:

| USB ID | Mode | Kernel driver | Works? |
|---|---|---|---|
| `3537:1040` | **X-input** | `xpad` ("Generic X-Box pad") | ✅ what configs now expect |
| `3537:1041` | D-input | `usbhid` ("HID gamepad") | joystick exists, different axis/button numbering |
| `3537:2106` | Switch/Android? | none | ❌ dead — no js device at all, looks like a broken dongle |

It flipped from D-input to X-input during the battery recharge, which broke the old configs
(head pinned by LT resting at +1.0 on axis 2, wheels dead because the deadman button became an axis).
Configs were remapped to X-input: right stick = head (pan axis 3, tilt axis 4), **hold LB (button 4)**
= drive deadman, RB (5) = turbo, A/B/Y = recentre/estop-clear/estop.

Check the mode any time with: `~/jessica_ws/gamepad_mode.sh` (detects all three states).
Mode is toggled by a combo on the pad — **TODO: record the exact combo that worked** (not yet noted).
X-input axes: 0=LX 1=LY 2=LT 3=RX 4=RY 5=RT — never map 2/5, triggers rest at +1.0.

## The plan: auto-detect mode at launch (option 1, agreed, not yet built)

**Config files: consolidate to one yaml per pad personality.** Currently the mode-dependent
settings are smeared across `joystick.yaml` (drive axes + enable buttons) and
`joy_button_mappings.yaml` (A/B/Y indices), while `pan_tilt_teleop`'s axes are hard-coded
defaults in its Python. Replace with:

- `config/gamepad_xinput.yaml` — sections for all four joystick nodes: `joy_node` (device,
  deadzone), `teleop_node` (drive axes/scales, enable LB=4, turbo RB=5), `pan_tilt_teleop`
  (pan_axis 3, tilt_axis 4), `joy_button_bridge` (A=0 recentre, B=1 estop-clear, Y=3 estop)
- `config/gamepad_dinput.yaml` — the same sections with D-mode numbers (filled in properly
  after the capture session; until then the estop mappings stay empty rather than guessed)

So "one file completely describes one mode" — the old two yamls retire.

**Launch file: the only real code change.** The four joystick `Node(...)` definitions move
inside an `OpaqueFunction` — a launch construct that runs a Python function at launch time,
which lets us inspect the hardware before declaring the nodes. Roughly:

```python
def gamepad_nodes(context):
    mode = LaunchConfiguration("gamepad_mode").perform(context)   # auto | xinput | dinput
    if mode == "auto":
        out = subprocess.run(["udevadm", "info", "-q", "property", "/dev/input/js0"], ...)
        mode = "xinput" if "ID_USB_DRIVER=xpad" in out.stdout else "dinput"
        # no js0 at all (dead 2106 mode / unplugged) -> warn + fall back to xinput
    profile = os.path.join(pkg, "config", f"gamepad_{mode}.yaml")
    return [ joy_node, teleop_drive, pan_tilt_teleop, joy_button_bridge ]   # all with parameters=[profile]
```

Plus a `DeclareLaunchArgument("gamepad_mode", default_value="auto")` so you can force
`gamepad_mode:=xinput` if the detection ever misbehaves. That's ~35–40 lines of launch diff,
mostly moving existing node definitions.

**Node code: zero changes.** `pan_tilt_teleop` already declares `pan_axis`/`tilt_axis` as
parameters — it's just never been handed a yaml. The only touch is that `make_pynode` (the
launch helper) needs to accept a `parameters` argument, or use a plain `Node(...)` for it.

**Behaviour:** launch sniffs the pad once at startup and logs which profile it picked
(`[gamepad] detected X-input → gamepad_xinput.yaml`). Battery-swap flips get absorbed by the
next launch automatically. Mid-session flips still need a relaunch, and the dead `2106` mode
still needs the combo press — no yaml can fix a pad the kernel refuses to make a joystick for.

**Prerequisite:** before `gamepad_dinput.yaml` can hold real numbers, we need a **5-minute
capture session with the pad in D-mode** (Jonny presses each stick/button, Claude reads `/joy`
and writes down the real axis/button table). Until then the D profile ships with the sticks
best-guessed from the old config (pan 2, tilt 3, deadman 6, turbo 7) and the **estop
deliberately unmapped** (a wrongly-mapped estop is worse than a missing one).

**Estimate:** ~30–45 min total. 2 new yamls, 2 old yamls retired, ~40-line launch edit,
no node code changes.
