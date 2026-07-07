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
