# Encoder failure detection — analysis 2026-07-13, IMPLEMENTED same day

**Implementation decisions** (details in the analysis below, which was
written first):
- Detector + two-tier response in `RobotController` (`checkWheelFaults()`,
  runs in `update()` on the motor task): trip on |duty| > 0.3 &&
  |speed| < 5 cm/s for 500ms; current ≥ 4.0A at trip → **stall latch**
  (robot brakes via the single-writer loop, like e-stop), otherwise →
  **limp mode** (that wheel feedforward-only, PID skipped; robot keeps
  driving). Limping wheel keeps a current-only stall check (200ms debounce).
  Thresholds are constants in RobotController.h — **tune on the bench**.
- Robot-wide speed cap at 50% while any wheel limps (both sides scaled
  together to preserve turn geometry).
- LEDs: stall latch → red; **limp → red+green both on**; else green
  (enabled) / red (disabled). Reconciled every cycle in
  `updateStatusLEDs()`, MCP written only on change.
- STATE gained 4 wheel-health flags (1 = OK, 0 = limping or latched),
  order lf,lr,rf,rr:
  `STATE,lf_pos,lr_pos,rf_pos,rr_pos,lf_vel,lr_vel,rf_vel,rr_vel,lf_ok,lr_ok,rf_ok,rr_ok`
  **Pi-side parser (esp32_combined_hardware.cpp) must be updated**, and
  should exclude flagged wheels from odometry.
- Manual clear only: `AUX,estopclear` and the dashboard CLEAR E-STOP
  button set `g_faultClearRequest`; the motor task calls
  `clearWheelFaults()` (single-writer pattern preserved).
- `maxSpeedMPS` set to 0.8 (main.cpp + struct default). Other max-speed
  definitions found and fixed: dashboard slider JS mapped 0–100% → 0–1.0
  m/s (now × MAX_SPEED_MPS = 0.8, must match main.cpp), and
  `setWheelSpeeds()` (ROS2 CMD path) never clamped to maxSpeedMPS — an
  unreachable target winds duty to the clamp just like a dead encoder;
  now clamped.

**Problem:** if an encoder fails (zero / frozen reading), the PID sees zero
speed, the error stays at full target value, and `applyPIDControl()` is
incremental (`duty += correction` every 10ms) — so the duty ratchets straight
up to the ±1.0 clamp. Full throttle until the target changes to zero.

## What a dead encoder looks like, per signal

Three per-motor signals are already in hand every cycle, plus one more —
the other three wheels:

| Scenario | Duty (commanded) | Encoder velocity | Motor current |
|---|---|---|---|
| Healthy driving | 0.3–1.0 | ≈ duty × 80 cm/s (± hill effects) | normal (load-dependent) |
| **Encoder dead, motor spinning** | ratchets to max | ~0 or frozen | **normal** |
| Wheel stalled (against wall, jammed) | ratchets to max | ~0 | **high** (locked rotor) |
| Motor/wiring dead | ratchets to max | ~0 | **≈ 0** |

Key observation: **the three failure rows don't need to be distinguished to
act** — in all of them, sustained high duty with near-zero measured speed
means the control loop has lost the plot, and the safe response is the same
(brake + disable). Current only matters for *reporting which* failure it was.

## Recommended detector

A per-wheel plausibility check in `motorControlTask` / `RobotController`:

- **Trigger:** `|duty| > 0.3` AND `|measured speed| < ~5 cm/s`,
  **sustained for ~500ms** (50 control cycles).
- At duty 0.3 expected speed is ~24 cm/s, so 5 cm/s is far outside any
  hill/load effect. A hill can slow the robot; it can't hold a wheel at zero
  while the driver pushes 30% duty — and if something *is* physically holding
  the wheel (stall), stopping is correct anyway before the motor cooks.
- The 500ms debounce absorbs spin-up transients and momentary snags.
- Self-arming: even if the encoder dies while creeping at low duty, PID
  windup drives the duty up through the 0.3 threshold within a fraction of a
  second, right into the detection window. No separate low-speed check needed.

**On detection:** set a per-motor fault latch and let the existing
single-writer machinery brake + disable — same pattern as `g_eStopActive`.
Current at the moment of trip classifies the cause (≈0 → wiring/driver,
high → stall, normal → encoder) for reporting.

Two supporting points from the code:

1. **The existing fault path is wrong for this.** The `hasFault()` handling
   in `motorControlTask` brakes, waits 1s, and *retries*. With a dead encoder
   that means: brake 1s → PID resumes → winds up → trips again → lurch,
   forever. An encoder-plausibility fault should **latch** and require an
   explicit clear (e.g. `AUX,estopclear` also clears it, or a dedicated
   clear). Open question: after a latched fault, is "dead until cleared"
   acceptable, or is a degraded open-loop limp mode wanted
   (`applyOpenLoopControl()` already exists)?

2. **The current reading is nearly free.** `faultCode()` already does a
   SEL-mux + ADC read per motor every 10ms — the same raw reading converts to
   amps. So a current-based "motor not drawing anything at duty > threshold"
   check costs no extra I2C/ADC traffic. Worth adding as a second independent
   detector: it catches a broken motor wire even when the wheel is dragged by
   the other three (encoder still turns, so the speed check stays happy).

## Not doing / deferred

- **Cross-checking wheel pairs** (FL vs RL should roughly agree): clever, but
  skid-steer wheels legitimately slip/scrub during turns, so thresholds get
  mushy. The duty-vs-speed check alone covers the runaway. Defer.
- **Direction plausibility** (velocity sign opposite duty sign → miswired
  encoder): cheap, worth adding at the same time, but second-order.
- **Partial encoder failure** (one quadrature channel dead → ~half counts):
  a tight "measured within X% of expected" band would catch it but
  false-positives on hills. PID compensates by driving faster than
  commanded — nastier to detect, lower stakes than full runaway. Defer.

## Before coding

- Thresholds (0.3 duty, 5 cm/s, 500ms, current floor) are educated guesses.
  `maxSpeedMPS` is 1.0 in `ROBOT_PARAMS` but real max is ~80 cm/s, so the
  duty→speed model is already ~20% off — fine for a loose plausibility bound,
  but a quick bench capture (duty vs. actual speed vs. current at a few
  operating points, flat ground) would set them with margin. Could be a
  temporary `DEBUG_LOG` build.
- When this trips, ROS2 currently has no way to know — the fault-reporting
  gap already noted in #4. A latched safety fault that silently ignores CMDs
  is exactly where an in-protocol status field earns its keep. Worth bundling.

---

# Firmware Issues — code review 2026-07-06

Found during a read-through of the esp32_bridge firmware. No changes made yet.
Ranked roughly by severity.

## Safety (both interact with the new e-stop)

### 1. Web dashboard bypasses the e-stop latch — FIXED 2026-07-07
`/api/enable` and `/api/cmd` now return 409 while `g_eStopActive`
(`/api/cmd` fully blocked — even `dir=stop` would have released the
brake via `setDuty(0)`). New `/api/estop?state=1|0` endpoint mirrors
the serial AUX handlers, and the page has E-STOP / CLEAR E-STOP
buttons plus an "E-STOP ACTIVE" banner driven by a new `estop` field
in the telemetry JSON. Note: the API is still unauthenticated (see
#11), so anyone on the network can also *clear* an e-stop.

Original finding:
`g_eStopActive` only guards the serial `CMD` path. The dashboard endpoints go
straight to the robot controller:
- `/api/enable?state=1` → `robot.enable(true)` — `src/WebDashboard.cpp:562-568`
- `/api/cmd?dir=fwd` → `robot.forward()` — `src/WebDashboard.cpp:580-596`

While e-stopped, an open dashboard tab (or anyone on the network — the API has
no auth) can re-enable and drive the robot. These handlers should respect the
latch if the e-stop is meant to be authoritative.

### 2. Serial watchdog quietly releases the e-stop brake — FIXED 2026-07-07
Redesigned as single-writer: the watchdog and the e-stop handlers only
set request flags (`g_watchdogBrake`, `g_eStopActive`);
`motorControlTask` — the only task that writes motor state — performs
`enable(false)` + `brake()` (hold, not freewheel), re-asserts the brake
every 10ms cycle, and re-enables when the condition clears. CMD now
feeds the watchdog timestamp even while e-stopped, so the watchdog
tracks link liveness; if the link dies during an e-stop the robot stays
braked until comms return. Encoder updates continue while braked so
STATE velocities stay live.

Original finding:
After `AUX,estop`, no more `lastMotorCommandMs` updates arrive, so within ~1s
the watchdog in `serialCommandTask` (`src/main.cpp:210-214`) fires
`robot->stop()`, which calls `setDuty(0)` on every motor — replacing the
active brake (INA=INB=1, shorted terminals) with drive-mode-at-zero-PWM
(freewheel). On a slope the robot could roll after an e-stop.
`Motor::setDuty()` doesn't check `_enabled`, so nothing prevents this.
Same applies to a fault-triggered brake: a watchdog `stop()` can follow and
release it.

## Concurrency

### 3. MCP23017 / I2C / ADC path has no locking, hit from 3–4 contexts on both cores — FIXED 2026-07-07
Three-part fix:
- `Mcp23017Bus` now owns a recursive FreeRTOS mutex; every shadow-register
  read-modify-write and every I2C transaction takes it, and `lock()`/
  `unlock()` are public for compound operations.
- `Motor::readCurrentAmps()` and `faultCode()` hold the bus lock across
  "set SEL mux → analogRead", so concurrent readers can't re-mux the
  shared SEL0/SEL1 lines mid-measurement (no more garbage current
  readings / spurious or masked faults).
- Dashboard `dir=stop` now zeroes targets (`setSpeed(0,0)`) instead of
  writing duties from the web task — motor duty writes stay single-writer
  (see #2 fix).
Remaining (accepted): `enable()` from the CMD handler / `/api/enable`
still runs cross-core, but it only writes flags + LEDs (now mutex-safe)
and resets PID state — worst case a one-cycle PID transient, no
hardware-state corruption.

Original finding:
Contexts: `motorControlTask` (core 1), `telemetryTask` (core 0),
`serialCommandTask` (core 0, via brake/enable), AsyncWebServer callbacks.
Two concrete races:
- Shadow-register read-modify-write in `Mcp23017Bus::writePin()`
  (`src/Mcp23017Bus.cpp:93-114`) isn't atomic — concurrent writes to different
  pins can clobber each other's bits, momentarily setting wrong INA/INB states
  on a motor driver.
- SEL0/SEL1 are **shared** across all four VNH7040s, and both
  `readCurrentAmps()` and `faultCode()` do "set SEL, then analogRead"
  (`src/Motor.cpp:136-155, 178-193`). Fault check runs every 10ms in the motor
  task while telemetry task and web handlers do current reads concurrently —
  one task can re-mux SEL between another's SEL-write and ADC-read. Result:
  garbage current telemetry, potentially a *spurious fault* (brakes the robot)
  or a *masked real fault*.

Fix idea: one mutex around "SEL + ADC read" and around the MCP shadow writes.

(Also fixed as part of this: e-stop/watchdog braking goes through
`motorControlTask` as the single writer of motor duty/direction state —
see #2.)

### 4. Debug prints share UART0 with the machine protocol — FIXED 2026-07-07
Survey showed only one print fires during operation: the motor fault
print in `motorControlTask`. It now goes through `DBG_PRINTF` (new
`include/DebugLog.h`), which compiles to nothing unless built with
`-DDEBUG_LOG=1` (commented example in `platformio.ini`; bench use only).
Boot-time prints in `setup()`/`begin()` are kept — they predate protocol
traffic and are the only visibility into init failures.
Future enhancement (not done): report motor faults in-protocol, e.g. a
fault field in the STATE message, so ROS2 knows when the robot brakes on
a fault. Needs a matching change in esp32_combined_hardware.cpp.

Original finding:
`motorControlTask` prints `[Motor Task] Motor %d FAULT...` (`src/main.cpp:246`)
and `PanTiltController` prints too, on the same `Serial` the ROS2 host parses.
A print from core 1 can interleave mid-`STATE` line with the serial task's
response on core 0. ROS2 side tolerates unparseable lines (warns only), but
during a fault this spams a print every ~1s and corrupts occasional STATE
reads exactly when things are going wrong.

## Minor / correctness

### 5. Encoder 64-bit count race
`src/QuadratureEncoder.cpp:125-132`: `_overflowCount + count` isn't atomic
against the overflow ISR — an overflow landing between the PCNT read and the
`_overflowCount` read produces a one-shot ±32768 glitch. Rare, but feeds
odometry.

### 6. `toggle` / `fire` state desync
`src/main.cpp:184-199`: `toggle` keeps its own `static bool` while `fire`'s
timer resets `g_auxPinHigh` — after a fire, the toggle's notion of pin state
can be inverted, so a toggle press appears to do nothing.

### 7. Duplicated AUX pin constant + stale comment
`PIN_AUX_OUT = 42` in `src/main.cpp` and `AUX_PIN = 42` in
`src/WebDashboard.cpp`. Comment at `src/WebDashboard.cpp:609` says
"pin 35 high for 2 s" while the code does pin 42 for 600ms.

### 8. `Motor::brake()` reports PWM as 0 while driving LEDC at max
`src/Motor.cpp:111-119`: writes `_maxPWM` to LEDC but sets `_currentPWM = 0`
— telemetry lies during braking.

### 9. Dead code: unused telemetry struct
The `telemetry` struct + mutex in `src/main.cpp` is populated by
`telemetryTask` but never read — the dashboard queries `RobotController`
directly.

### 10. Comment/code mismatch in `Mcp23017Bus::begin()`
`src/Mcp23017Bus.cpp:33-37`: comment says "Set all pins as outputs" but `0xFF`
in IODIR means all *inputs* — motor INA/INB lines float between MCP init and
each `Motor::begin()`. Harmless if the VNH7040 inputs have pulldowns.

### 11. WiFi credentials hardcoded
In `src/main.cpp` — fine for a hobby robot, but the dashboard API is
unauthenticated on whatever network it joins.

---

Suggested fix order: #2 (watchdog releasing the brake), #1 (dashboard
respecting the latch), then a mutex for #3.
