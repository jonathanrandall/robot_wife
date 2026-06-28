# ESP32-S3 Quad Motor Robot Controller — Software Documentation

---

## Next Session Prompt

> Copy and paste this at the start of a new conversation to restore full context:
>
> *"I'm working on an ESP32-S3 quad motor robot controller (potential fields project) in PlatformIO/Arduino at the path `~/projects/kicad_esp32_design/esp32_s3_vnh5019_quad_motor_v3/software_base/potential_fields`. Please read DOCUMENTATION.md before we start. The robot is a 4-wheel differential drive with VNH7040 motor drivers controlled via a MCP23017 I2C GPIO expander. There are 4 independent PID controllers (one per wheel). Motor layout: M1=Rear Right (index 0), M2=Rear Left (index 1), M3=Front Left (index 2), M4=Front Right (index 3). Left side motors (M2, M3) have their encoder A/B pins swapped in the pin definitions because they are physically mounted in the opposite orientation. The firmware has a ROS2-compatible serial diff-drive interface (commands over USB-CDC Serial). A serial command watchdog stops motors after 1 second of no motor command. The web dashboard is served over WiFi and includes encoder, current, duty and PWM panels — it has NO watchdog. Current working PID gains: Kp=0.2, Ki=0.01, Kd=0.0 (derivative disabled — encoder velocity is too noisy)."*

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Hardware Architecture](#2-hardware-architecture)
3. [Software Architecture](#3-software-architecture)
4. [Module Reference](#4-module-reference)
   - [main.cpp — Entry Point & Configuration](#41-maincpp--entry-point--configuration)
   - [Mcp23017Bus — I2C GPIO Expander](#42-mcp23017bus--i2c-gpio-expander)
   - [QuadratureEncoder — Position & Velocity](#43-quadratureencoder--position--velocity)
   - [Motor — Single Motor Driver](#44-motor--single-motor-driver)
   - [PIDController — Speed Control Loop](#45-pidcontroller--speed-control-loop)
   - [RobotController — High-Level Movement](#46-robotcontroller--high-level-movement)
   - [WebDashboard — WiFi Web Interface](#47-webdashboard--wifi-web-interface)
5. [FreeRTOS Task Structure](#5-freertos-task-structure)
6. [Serial Command Protocol (ROS2 Interface)](#6-serial-command-protocol-ros2-interface)
7. [Configuration Guide](#7-configuration-guide)
8. [Web Dashboard Usage](#8-web-dashboard-usage)
9. [Web API Reference](#9-web-api-reference)
10. [PID Tuning Guide](#10-pid-tuning-guide)
11. [Status LEDs](#11-status-leds)
12. [Fault Handling](#12-fault-handling)
13. [Pin Assignments](#13-pin-assignments)
14. [Build & Flash](#14-build--flash)
15. [Known Gaps & Future Work](#15-known-gaps--future-work)

---

## 1. Project Overview

This firmware runs on an **ESP32-S3** microcontroller and provides closed-loop speed control for a **4-wheel differential drive robot** using four **VNH7040** motor driver ICs. The system features:

- **Independent PID speed control on every wheel** (four separate PID controllers)
- Quadrature encoder feedback via the ESP32-S3's hardware PCNT (pulse counter) peripheral
- Motor direction and current sensing via a **MCP23017** I2C GPIO expander
- A **ROS2-compatible serial diff-drive interface** over USB-CDC for high-level control
- A serial command **watchdog** — motors stop automatically if no command is received within 1 second
- A browser-based web dashboard served over WiFi for real-time telemetry and manual control
- FreeRTOS multi-core task scheduling (motor control on core 1, telemetry and serial command handling on core 0)
- Status LEDs (red/green) reflecting the enabled/disabled state of the motors

---

## 2. Hardware Architecture

```
                        ESP32-S3
                    ┌──────────────┐
            GPIO16  │ SDA ─────────┼──► MCP23017 (I2C addr 0x20)
            GPIO15  │ SCL ─────────┼──►   ├─ GPA0..GPA7 (motor INA/INB/SEL pins)
                    │              │      ├─ GPB0..GPB7 (motor INA/INB/SEL pins)
         GPIO8,48,  │ PWM (LEDC) ──┼──► VNH7040 drivers (x4) ──► Motors (x4)
          47, 9     │              │
      GPIO4,1,2,5   │ ADC inputs ──┼──► MultiSense (current sense, one per driver)
                    │              │
    GPIO17,7,       │ PCNT A/B ────┼──► Quadrature Encoders (x4)
    38,39,14,       │              │
    13,11,12        │              │
                    └──────────────┘
```

### Key Components

| Component | Role |
|-----------|------|
| ESP32-S3 | Main MCU. Dual-core Xtensa LX7. Runs FreeRTOS. |
| MCP23017 | 16-bit I2C GPIO expander. Controls motor INA/INB/SEL0/SEL1 pins and status LEDs. |
| VNH7040 | Automotive H-bridge motor driver. Controlled via INA, INB (direction), PWM (speed), MultiSense (current/fault). |
| Quadrature Encoders | 1425 CPR encoders on each motor. Read by ESP32 PCNT hardware. |
| Status LEDs | Red (GPA1) and Green (GPA2) on MCP23017. |

### Motor Physical Layout

```
         FRONT
   M3 (FL) | M4 (FR)
   ---------|---------
   M2 (RL) | M1 (RR)
         BACK
```

| Motor | Position | Array Index | Encoder orientation |
|-------|----------|-------------|---------------------|
| M1 | Rear Right | 0 | Normal |
| M2 | Rear Left | 1 | **A/B swapped** (physically reversed) |
| M3 | Front Left | 2 | **A/B swapped** (physically reversed) |
| M4 | Front Right | 3 | Normal |

> The left-side motors (M2, M3) are mounted in the opposite orientation to the right-side motors. Their encoder A and B pins are swapped in `main.cpp` so that forward motion produces positive counts on all four wheels.

---

## 3. Software Architecture

```
main.cpp
  │
  ├── Mcp23017Bus          (I2C GPIO expander driver)
  │
  ├── Motor  ×4            (per-motor: PWM, direction, current sense, encoder)
  │     ├── Mcp23017Bus    (shared reference)
  │     └── QuadratureEncoder
  │
  ├── RobotController      (kinematics, 4×PID, movement commands, LED control)
  │     ├── Motor[4]
  │     ├── PIDController  ×4  (one per wheel)
  │     └── Mcp23017Bus    (for LED control)
  │
  └── WebDashboard         (WiFi + async HTTP server + JSON telemetry)
        └── RobotController (reference for commands and telemetry)
```

### Class Dependency Summary

| Class | Depends On |
|-------|------------|
| `QuadratureEncoder` | ESP-IDF PCNT driver |
| `Mcp23017Bus` | Arduino Wire (I2C) |
| `Motor` | `Mcp23017Bus`, `QuadratureEncoder` |
| `PIDController` | (standalone) |
| `RobotController` | `Motor[4]`, `PIDController×4`, `Mcp23017Bus` |
| `WebDashboard` | `RobotController`, ESPAsyncWebServer, ArduinoJson |

---

## 4. Module Reference

### 4.1 `main.cpp` — Entry Point & Configuration

The top of `main.cpp` contains all user-facing configuration constants. **This is the only file you normally need to edit.**

#### WiFi
```cpp
const char* WIFI_SSID     = "your-network";
const char* WIFI_PASSWORD = "your-password";
const char* MDNS_HOSTNAME = "robot";   // Access via http://robot.local
```

#### Robot Physical Parameters (`RobotParams`)
```cpp
const RobotParams ROBOT_PARAMS = {
    0.144f,   // wheelDiameterM  — wheel diameter in metres (144 mm)
    1425.0f,  // encoderCPR      — encoder counts per revolution
    0.20f,    // wheelBaseM      — left-to-right wheel spacing in metres (200 mm)
    1.0f      // maxSpeedMPS     — maximum wheel speed in m/s
};
```
These values are used for all speed-to-encoder and speed-to-duty conversions.

#### Motor Configuration (`MotorConfig`)
```cpp
const MotorConfig MOTOR_CONFIG = {
    20000,    // pwmFreqHz            — PWM frequency (20 kHz, above audible range)
    10,       // pwmResolutionBits    — 10-bit = 0–1023 steps
    false,    // invertDirection      — flip motor polarity in software
    0.14f,    // csVoltsPerAmp        — current sense calibration (V/A)
    0.0f,     // csZeroOffset         — ADC zero-current voltage offset
    3.0f      // faultVoltageThreshold — MultiSense fault threshold (V)
};
```

#### Motor Pin Definitions
```cpp
// MotorPins: mcpINA, mcpINB, mcpSEL0, mcpSEL1, pwmGPIO, csAdcGPIO, encA, encB
const MotorPins M1_PINS = {3,  0,  5, 6, 8,  4, 17, 7};   // Rear Right — normal
const MotorPins M2_PINS = {15, 12, 5, 6, 48, 1, 38, 39};  // Rear Left  — encA/B swapped
const MotorPins M3_PINS = {11, 8,  5, 6, 47, 2, 14, 13};  // Front Left — encA/B swapped
const MotorPins M4_PINS = {7,  4,  5, 6, 9,  5, 11, 12};  // Front Right — normal
```

#### FreeRTOS Timing Constants
```cpp
constexpr uint32_t MOTOR_UPDATE_INTERVAL_MS    = 10;    // PID rate (ms)
constexpr uint32_t TELEMETRY_UPDATE_INTERVAL_MS = 50;  // Telemetry rate (ms)
constexpr uint32_t SERIAL_WATCHDOG_MS           = 1000; // Serial command timeout (ms)
```

#### `setup()` Sequence
1. Create telemetry mutex
2. Initialise MCP23017 (halt on failure)
3. Configure status LEDs (green on = ready)
4. Create and initialise four `Motor` objects
5. Create `RobotController` (passing MCP reference and LED pin numbers)
6. Connect to WiFi and start `WebDashboard`
7. Start mDNS
8. Start FreeRTOS tasks (`MotorControl` on core 1, `Telemetry` and `SerialCmd` on core 0)

#### `loop()`
The main loop is idle. Status printing is **disabled** because the Serial port is used exclusively for the ROS2 serial command protocol. All real work is done in FreeRTOS tasks.

---

### 4.2 `Mcp23017Bus` — I2C GPIO Expander

**File:** `include/Mcp23017Bus.h` / `src/Mcp23017Bus.cpp`

A driver for the MCP23017 16-bit I2C GPIO expander. All motor direction pins, SEL (MultiSense select) pins, and status LEDs are routed through this chip.

Pins 0–7 = Port A (GPA0–GPA7), pins 8–15 = Port B (GPB0–GPB7).

#### Key Methods

| Method | Description |
|--------|-------------|
| `begin(sda, scl, addr, i2cFreq)` | Initialise I2C bus and configure the chip. Returns `false` on failure. Default addr `0x20`, 400 kHz. |
| `pinMode(pin, OUTPUT/INPUT)` | Set a single pin direction. |
| `writePin(pin, HIGH/LOW)` | Write a single output pin. Uses shadow registers for efficiency. |
| `readPin(pin)` | Read a single input pin. |
| `writePortA(val)` / `writePortB(val)` | Write all 8 pins of a port at once. |
| `readPortA()` / `readPortB()` | Read all 8 pins of a port. |
| `writeRegister(reg, val)` / `readRegister(reg)` | Direct register access. |
| `enablePullup(pin, bool)` | Enable/disable internal pull-up on an input pin. |
| `flush()` | Force shadow registers to hardware (normally automatic). |
| `getShadowA()` / `getShadowB()` | Inspect cached output state. |

Shadow registers (`_shadowA`, `_shadowB`) cache the output state so individual pin writes don't require read-modify-write I2C transactions.

---

### 4.3 `QuadratureEncoder` — Position & Velocity

**File:** `include/QuadratureEncoder.h` / `src/QuadratureEncoder.cpp`

Uses the ESP32-S3 **PCNT (pulse counter) hardware peripheral** for interrupt-driven quadrature decoding. Up to 4 encoders are supported (one per PCNT unit).

#### How It Works

- Two PCNT channels per unit decode both edges of both encoder signals (full 4× quadrature).
- A hardware input filter (100 APB clock cycles) rejects glitches on the encoder lines.
- The hardware counter is 16-bit (−32768 to +32767). An ISR fires on overflow/underflow and accumulates the extra counts into a 64-bit software counter (`_overflowCount`).
- `getCount()` returns `_overflowCount + hardware_counter` for a seamless 64-bit position.
- `update()` must be called periodically (every 10–50 ms). It computes velocity in **counts/second** from the delta since the last call.

#### Key Methods

| Method | Description |
|--------|-------------|
| `begin(pinA, pinB, unit)` | Configure PCNT hardware. `unit` is `PCNT_UNIT_0`..`PCNT_UNIT_3`. |
| `getCount()` | 64-bit absolute position in encoder counts. |
| `resetCount()` | Zero the position counter. |
| `setCount(val)` | Set the position counter to an arbitrary value. |
| `getDirection()` | `true` if last motion was positive (forward). |
| `getVelocity()` | Last computed velocity in counts/second. |
| `update()` | Recalculate velocity — call every motor control cycle. |

---

### 4.4 `Motor` — Single Motor Driver

**File:** `include/Motor.h` / `src/Motor.cpp`

Controls one VNH7040 H-bridge driver. Each motor object owns a `QuadratureEncoder` instance and uses a shared `Mcp23017Bus` reference for direction and SEL control.

#### VNH7040 Control Truth Table

| INA | INB | PWM | Result |
|-----|-----|-----|--------|
| 1   | 0   | x   | Forward (CW) |
| 0   | 1   | x   | Reverse (CCW) |
| 1   | 1   | 1   | Brake (high-side short) |
| 0   | 0   | 0   | Coast (outputs floating) |

Direction is set via MCP23017 `mcpINA`/`mcpINB` pins; speed via ESP32 LEDC PWM on `pwmGPIO`.

#### SEL Pins for MultiSense

The VNH7040 MultiSense output multiplexes current sense and fault diagnostics. `SEL0`/`SEL1` select which internal signal is routed:

| Condition | SEL0 | SEL1 | MultiSense output |
|-----------|------|------|-------------------|
| Forward current | 1 | 0 | High-side A current |
| Reverse current | 0 | 0 | High-side B current |

`SEL0` and `SEL1` are **shared** across all four drivers via GPA5/GPA6 on the MCP23017 and are set each time current is read.

#### Key Methods

| Method | Description |
|--------|-------------|
| `begin(encoderUnit)` | Allocate LEDC channel, configure pins, init encoder. |
| `enable(bool)` | Software enable flag (VNH7040 has no dedicated EN pin). |
| `setDuty(float)` | Set speed+direction. Range −1.0 (full reverse) to +1.0 (full forward). |
| `setPWM(uint16_t)` | Set raw LEDC PWM value directly. |
| `setDirection(bool)` | Set direction without changing PWM. |
| `brake()` | Active brake: INA=INB=1, PWM=max. |
| `coast()` | Float outputs: INA=INB=0, PWM=0. Disables motor. |
| `readCurrentRaw()` | Raw 12-bit ADC value from current sense pin. |
| `readCurrentAmps()` | Read motor current via ADC, applying `csVoltsPerAmp` scaling. |
| `getDuty()` | Current duty cycle (−1.0 to +1.0). |
| `getPWM()` | Raw LEDC PWM value (0–1023 at 10-bit). |
| `getDirection()` | Current direction (`true` = forward). |
| `getEncoderCount()` | 64-bit encoder position. |
| `resetEncoderCount()` | Reset encoder position to zero. |
| `getEncoderVelocity()` | Encoder velocity in counts/second (from last `updateEncoder()` call). |
| `updateEncoder()` | Must be called every control cycle to update velocity. |
| `hasFault()` / `faultCode()` | Check MultiSense voltage against `faultVoltageThreshold`. |
| `clearFault()` | Clear internal fault state. |

#### LEDC Channel Allocation

Each motor uses one ESP32 LEDC channel. Channels are allocated sequentially via the static `_nextLedcChannel` counter. Motors 1–4 use channels 0–3. Up to 8 channels are available on ESP32.

---

### 4.5 `PIDController` — Speed Control Loop

**File:** `include/PIDController.h` / `src/PIDController.cpp`

A general-purpose discrete PID controller. `RobotController` instantiates **four** of these — one per wheel.

#### Algorithm

```
error       = setpoint − measurement
P           = Kp × error
integral   += error × dt          (clamped to ±maxIntegral for anti-windup)
I           = Ki × integral
D           = −Kd × d(measurement)/dt  (derivative on measurement, not error)
output      = clamp(P + I + D, minOutput, maxOutput)
```

Derivative is computed on the **measurement** rather than the error to avoid the "derivative kick" that occurs on step changes to the setpoint.

#### Current Tuned Gains

```
Kp = 0.2    Ki = 0.01    Kd = 0.0   (derivative disabled)
maxIntegral = 0.3    maxCorrection = ±0.5
```

> **Note:** Kd was set to zero because encoder velocity derived from the PCNT counter at 10 ms intervals is too noisy for stable derivative action. If Kd is needed in the future, add a low-pass filter to the velocity measurement first.

> **Note:** The `SpeedPIDConfig` struct defaults in `RobotController.h` (Kp=0.5, Ki=0.05, Kd=0.01) differ from the actual gains initialised in the `RobotController` constructor (Kp=0.2, Ki=0.01, Kd=0.0). The constructor values are the ones that take effect.

#### Key Methods

| Method | Description |
|--------|-------------|
| `compute(setpoint, measurement, dt)` | Run one PID iteration. Returns the control output. |
| `reset()` | Clear integral, derivative state — call when stopping or re-enabling. |
| `setGains(Kp, Ki, Kd)` | Update gains at runtime. |
| `setOutputLimits(min, max)` | Clamp output range. |
| `setMaxIntegral(val)` | Anti-windup limit on the integral accumulator. |
| `getProportional/Integral/Derivative/Error/Output()` | Inspect internal state for tuning/debugging. |
| `enableProportional/Integral/Derivative(bool)` | Selectively disable PID terms for testing. |

---

### 4.6 `RobotController` — High-Level Movement

**File:** `include/RobotController.h` / `src/RobotController.cpp`

The central control class. Implements differential drive kinematics, runs **four independent PID loops** (one per wheel), and controls the status LEDs.

#### Differential Drive Kinematics

```
leftSpeed  = linearSpeed − (angularSpeed × wheelBase/2)
rightSpeed = linearSpeed + (angularSpeed × wheelBase/2)
```

`_targetLeftSpeed` is shared by both left wheels; `_targetRightSpeed` is shared by both right wheels. Each wheel then runs its own PID independently against its own encoder.

#### MotorPosition Enum

```cpp
enum MotorPosition {
    FRONT_LEFT  = 2,   // M3
    FRONT_RIGHT = 3,   // M4
    REAR_LEFT   = 1,   // M2
    REAR_RIGHT  = 0    // M1
};
```

#### PID Speed Control (per wheel)

The controller runs in **feedforward + correction mode**:

```
On first non-zero cycle:
    duty = mpsToDuty(targetSpeed)   // feedforward seed

Each cycle:
    duty += PID.compute(target, actualWheelSpeed, dt)
    duty = clamp(duty, -1.0, +1.0)

If target == 0:
    duty = 0 immediately
```

Each wheel has its own duty accumulator: `_leftFrontDuty`, `_leftBackDuty`, `_rightFrontDuty`, `_rightBackDuty`.

#### Key Methods

| Method | Description |
|--------|-------------|
| `begin()` | Reset state, start timing. |
| `setSpeed(linear, angular)` | Primary movement command. linear in m/s, angular in rad/s. |
| `setWheelSpeeds(leftMPS, rightMPS)` | Set left/right wheel speeds directly (used by ROS2 serial interface). Updates `_command` for telemetry. |
| `forward(mps)` | Drive straight forward. |
| `backward(mps)` | Drive straight backward. |
| `turnLeft(mps)` | Spin left in place. |
| `turnRight(mps)` | Spin right in place. |
| `stop()` | Zero all targets, reset all 4 PIDs and duty accumulators. |
| `brake()` | Active brake all four motors, reset all 4 PIDs. |
| `update()` | Run one control cycle (auto dt from micros). |
| `update(dt)` | Run one control cycle with explicit dt (seconds). |
| `enable(bool)` | Enable/disable all motors and update LEDs automatically. |
| `isEnabled()` | Returns current enabled state. |
| `setLEDs(red, green)` | Directly set red/green LED states via MCP23017. |
| `enablePID(bool)` | Switch between closed-loop and open-loop control. |
| `isPIDEnabled()` | Returns current PID enabled state. |
| `setPIDGains(Kp, Ki, Kd)` | Update gains on all four PID controllers. |
| `setPIDGains(config)` | Update gains using a `SpeedPIDConfig` struct. |
| `resetPID()` | Clear all four PID integrators and zero all duty accumulators. |
| `getActualLinearSpeed()` | Average of left and right wheel speeds in m/s. |
| `getActualAngularSpeed()` | Angular velocity in rad/s from wheel speed difference. |
| `getLeftWheelSpeed()` | Average of front-left and rear-left wheel speeds in m/s. |
| `getRightWheelSpeed()` | Average of front-right and rear-right wheel speeds in m/s. |
| `getFrontLeftWheelSpeed()` | Individual wheel speed in m/s. |
| `getRearLeftWheelSpeed()` | Individual wheel speed in m/s. |
| `getFrontRightWheelSpeed()` | Individual wheel speed in m/s. |
| `getRearRightWheelSpeed()` | Individual wheel speed in m/s. |
| `getEncoderCount(i)` | Encoder count for motor index 0–3. |
| `getMotorCurrent(i)` | Current in amps for motor index 0–3. |
| `getMotorPWM(i)` | Raw PWM value (0–1023) for motor index 0–3. |
| `getMotorDuty(i)` | Duty cycle (−1.0 to +1.0) for motor index 0–3. |
| `getCommand()` | Returns the current `RobotCommand` (linearSpeed, angularSpeed). |
| `getParams()` / `setParams()` | Get/set robot physical parameters. |

#### PID Debug Getters

```cpp
getFrontLeftPIDOutput()  / getFrontLeftPIDError()
getRearLeftPIDOutput()   / getRearLeftPIDError()
getFrontRightPIDOutput() / getFrontRightPIDError()
getRearRightPIDOutput()  / getRearRightPIDError()
```

#### Unit Conversions

| Method | Conversion |
|--------|-----------|
| `mpsToEncoderCPS(mps)` | m/s → encoder counts/second |
| `encoderCPSToMPS(cps)` | Counts/second → m/s |
| `mpsToDuty(mps)` | m/s → duty cycle (−1.0 to +1.0), clamped by `maxSpeedMPS` |

---

### 4.7 `WebDashboard` — WiFi Web Interface

**File:** `include/WebDashboard.h` / `src/WebDashboard.cpp`

Hosts an asynchronous HTTP server using **ESPAsyncWebServer**. Serves a single-page dashboard and a small REST API. Uses **ArduinoJson** to format telemetry data.

#### Initialisation
```cpp
dashboard = new WebDashboard(*robot);
dashboard->begin(WIFI_SSID, WIFI_PASSWORD);
```
WiFi connection is attempted 30 times (15 seconds) before giving up. If WiFi fails, the robot continues to operate without the dashboard.

After connecting, mDNS is started so the dashboard is accessible at `http://robot.local`.

> **Note:** The web dashboard has **no command watchdog**. If the browser closes or WiFi drops mid-command, the robot continues at the last commanded speed. This is a known gap — see section 15.

---

## 5. FreeRTOS Task Structure

| Task | Core | Priority | Period | Stack |
|------|------|----------|--------|-------|
| `motorControlTask` | 1 | 5 (high) | 10 ms | 4096 B |
| `serialCommandTask` | 0 | 3 (medium) | 1 ms poll | 4096 B |
| `telemetryTask` | 0 | 2 (low) | 50 ms | 4096 B |
| `loop()` (Arduino main task) | — | — | 100 ms | — |

### `motorControlTask` (Core 1, every 10 ms)

1. Check all four motors for faults via `hasFault()`.
2. On fault: call `robot->brake()`, wait 1 second, retry.
3. Call `motors[i]->updateEncoder()` for all four motors (updates velocity).
4. Call `robot->update()` — runs 4 PIDs and applies duty cycles.
5. Sleep until next 10 ms tick via `vTaskDelayUntil`.

### `serialCommandTask` (Core 0, every 1 ms)

1. Check serial watchdog: if `lastMotorCommandMs > 0` and elapsed time > `SERIAL_WATCHDOG_MS` (1000 ms), call `robot->stop()` and reset the watchdog timestamp.
2. Read available bytes from Serial, accumulate into a line buffer.
3. On `\r` or `\n`: call `processSerialCommand()` and clear the buffer.
4. Sleep 1 ms via `vTaskDelay`.

### `telemetryTask` (Core 0, every 50 ms)

1. Acquire `telemetry.mutex`.
2. Collect encoder counts and motor currents from all four motors.
3. Read actual linear speed from `robot`.
4. Release mutex.
5. Sleep 50 ms.

> **Note:** The web server's telemetry endpoint reads directly from `RobotController` rather than the shared `telemetry` struct. The mutex-protected struct exists as a foundation for future use.

---

## 6. Serial Command Protocol (ROS2 Interface)

The firmware exposes a simple text-based serial protocol compatible with a ROS2 differential drive node. Communication is over the USB-CDC Serial port at **115200 baud**. All commands are terminated by `\r` or `\n`.

### Commands

| Command | Format | Description |
|---------|--------|-------------|
| Empty / ping | `\r` | Acknowledge — returns `\r\n` |
| Encoder query | `e\r` | Returns per-wheel velocities in cm/s as integers |
| Motor speed | `m_v0_v1_v2_v3_\r` | Set per-wheel speeds in cm/s |
| Stop | `stop\r` | Stop all motors |

### Encoder Response Format

```
<RR_cms> <RL_cms> <FL_cms> <FR_cms>\r\n
```

Index order: `v0` = M1 Rear Right, `v1` = M2 Rear Left, `v2` = M3 Front Left, `v3` = M4 Front Right.

Example: `12 11 12 11\r\n` (all wheels moving forward at ~12 cm/s)

### Motor Speed Command Format

```
m_<RR>_<RL>_<FL>_<FR>_\r
```

Values are integers in **cm/s** (positive = forward). Same index order as the encoder response.

**How wheel speeds are applied:**
```
rightMPS = (RR + FR) / 2 / 100    // average Rear Right + Front Right
leftMPS  = (RL + FL) / 2 / 100    // average Rear Left + Front Left
robot->setWheelSpeeds(leftMPS, rightMPS)
```

If the robot is not currently enabled, it will be auto-enabled on the first motor speed command.

### Serial Command Watchdog

`lastMotorCommandMs` is set to `millis()` each time a valid `m_...` command is received. If more than `SERIAL_WATCHDOG_MS` (1000 ms) elapses without a new motor command, `serialCommandTask` calls `robot->stop()` and resets the timestamp. This prevents runaway motion if the ROS2 node disconnects or crashes.

---

## 7. Configuration Guide

All configuration lives at the top of `src/main.cpp`.

### Changing WiFi Credentials
```cpp
const char* WIFI_SSID     = "MyNetwork";
const char* WIFI_PASSWORD = "MyPassword";
```

### Adjusting Robot Parameters

```cpp
const RobotParams ROBOT_PARAMS = {
    0.144f,   // wheelDiameterM — measure your wheel OD
    1425.0f,  // encoderCPR     — from your encoder's datasheet
    0.20f,    // wheelBaseM     — measure centre-to-centre wheel distance
    1.0f      // maxSpeedMPS    — mechanical/safety top speed
};
```

### Inverting a Motor Direction

If a motor runs backwards relative to expected:
```cpp
const MotorConfig MOTOR_CONFIG = { 20000, 10, true, ... };
//                                             ^^^^
//                                     invertDirection = true
```

### Encoder Direction

If a motor produces negative encoder counts when moving forward, swap its `encA` and `encB` values in its `MotorPins` definition in `main.cpp`. This is already done for M2 and M3 (left side).

### Control Loop Timing

```cpp
constexpr uint32_t MOTOR_UPDATE_INTERVAL_MS    = 10;    // PID rate (ms)
constexpr uint32_t TELEMETRY_UPDATE_INTERVAL_MS = 50;  // Telemetry rate (ms)
constexpr uint32_t SERIAL_WATCHDOG_MS           = 1000; // Serial command timeout (ms)
```

---

## 8. Web Dashboard Usage

Navigate to `http://robot.local` (or the IP printed on Serial) in any browser.

### Panels

**Motors Enable Toggle**
A slider-style toggle. Enabling the motors lights the green hardware LED; disabling lights the red LED.

**Direction Control**
A 3×3 button grid. Directional buttons are **press-and-hold** — the robot moves while held and stops on release. Stop button is single-click.

| Button | Action |
|--------|--------|
| `▲` | Forward |
| `▼` | Backward |
| `◄` | Turn left (spin in place) |
| `►` | Turn right (spin in place) |
| `■` | Stop |

**Speed Slider**
Sets target speed 0–1.0 m/s. Used for all subsequent direction commands.

**Telemetry Panel**
Updates every 200 ms: target speed, actual speed, direction.

**Motors Panel**
Per-motor encoder count and current (A). Layout matches physical robot:

```
Rear Right (M1)  |  Rear Left (M2)
-----------------|------------------
Front Right (M4) |  Front Left (M3)
```

**PWM Panel**
Same layout. Shows per-motor duty cycle (−1.0 to +1.0) and raw PWM value (0–1023).

### Keyboard Controls

| Key | Action |
|-----|--------|
| `W` or `↑` | Forward |
| `S` or `↓` | Backward |
| `A` or `←` | Turn left |
| `D` or `→` | Turn right |
| `Space` | Stop |

---

## 9. Web API Reference

All endpoints accept `HTTP GET`.

| Endpoint | Parameters | Description |
|----------|-----------|-------------|
| `GET /` | — | Serve the HTML dashboard. |
| `GET /api/enable` | `state=1` or `state=0` | Enable/disable all motors. |
| `GET /api/speed` | `value=<float>` | Set target speed in m/s (stored in dashboard; used on next direction cmd). |
| `GET /api/cmd` | `dir=fwd\|back\|left\|right\|stop` | Send a movement command. |
| `GET /api/telemetry` | — | Returns JSON telemetry (see below). |

### Telemetry JSON Format
```json
{
  "targetSpeed": 0.50,
  "actualSpeed": 0.48,
  "direction": "fwd",
  "encoders": [1234, 1230, 1228, 1225],
  "currents": [0.42, 0.45, 0.41, 0.44],
  "duties":   [0.61, 0.62, 0.61, 0.63],
  "pwms":     [624, 635, 624, 645]
}
```

Array order: index 0=M1(RR), 1=M2(RL), 2=M3(FL), 3=M4(FR).

---

## 10. PID Tuning Guide

There are **four** PID controllers sharing the same gains. Gains apply to all four wheels equally.

### Current Working Gains

```
Kp = 0.2    Ki = 0.01    Kd = 0.0
maxIntegral = 0.3    maxCorrection = ±0.5
```

These are set in the constructor initialiser list in `RobotController.cpp`. The `SpeedPIDConfig` struct defaults in `RobotController.h` should be kept in sync.

### Understanding the Control Architecture

- On first non-zero command: `duty = mpsToDuty(targetSpeed)` (feedforward seed per wheel)
- Each subsequent cycle: `duty += PID_correction` (per wheel independently)
- If target becomes 0: `duty = 0` immediately across that side

### Tuning Steps

1. **Start with Ki=0, Kd=0, low Kp** (e.g. Kp=0.1). Command a constant speed and observe actual speed on the dashboard.
2. **Increase Kp** until the speed tracks but oscillations start to appear.
3. **Reduce Kp slightly** below the oscillation threshold.
4. **Add Ki** (e.g. Ki=0.05) to eliminate steady-state error.
5. **Leave Kd=0** — encoder velocity is too noisy for stable derivative action at 10 ms. If Kd is revisited, add a low-pass filter to velocity first.
6. If integral winds up and causes overshoot after stops, reduce `maxIntegral`.

### Open-Loop Mode

```cpp
robot->enablePID(false);
```
Applies `mpsToDuty(targetSpeed)` directly without feedback. Useful for isolating mechanical vs control issues.

---

## 11. Status LEDs

| State | Red LED (GPA1) | Green LED (GPA2) |
|-------|----------------|------------------|
| Motors disabled | ON | OFF |
| Motors enabled | OFF | ON |

Automatically updated by `RobotController::enable()`. On power-up `setup()` sets green ON (system ready) before `RobotController` is created.

```cpp
// Direct control:
robot->setLEDs(true, false);   // red on, green off
robot->setLEDs(false, true);   // green on, red off
```

---

## 12. Fault Handling

### Motor Fault Detection

The VNH7040 signals faults by driving its MultiSense output above `faultVoltageThreshold` (default 3.0 V). Checked every motor control cycle via `Motor::hasFault()`.

| Fault Code | Meaning |
|------------|---------|
| `FAULT_NONE` | No fault |
| `FAULT_DIAG_PIN` | MultiSense voltage ≥ threshold (overcurrent, thermal, or short) |
| `FAULT_GLOBAL_LINE` | (defined, not currently set) |
| `FAULT_OVERCURRENT` | (defined, not currently set) |

### Fault Response

When any motor faults in `motorControlTask`:
1. `robot->brake()` called immediately.
2. Task waits 1 second.
3. Fault logged to Serial: `[Motor Task] Motor N FAULT (code X)`.
4. Normal operation resumes (fault auto-clears if condition resolves).

### MCP23017 Failure

`setup()` halts in an infinite loop if `mcp.begin()` fails — prevents uncontrolled motor outputs.

### WiFi Failure

Non-fatal. Robot operates normally; dashboard is unavailable.

### Serial Command Watchdog

If the ROS2 node (or any serial commander) stops sending motor commands for more than `SERIAL_WATCHDOG_MS` (1000 ms), `serialCommandTask` automatically calls `robot->stop()`.

### No Web Dashboard Watchdog

**The web dashboard has no command timeout.** If the browser closes or WiFi drops mid-command, the robot continues at the last commanded speed. See [section 15](#15-known-gaps--future-work).

---

## 13. Pin Assignments

### ESP32-S3 GPIO Pins

| GPIO | Function |
|------|----------|
| 15 | I2C SCL (MCP23017) |
| 16 | I2C SDA (MCP23017) |
| 8 | M1 PWM |
| 48 | M2 PWM |
| 47 | M3 PWM |
| 9 | M4 PWM |
| 4 | M1 Current Sense ADC |
| 1 | M2 Current Sense ADC |
| 2 | M3 Current Sense ADC |
| 5 | M4 Current Sense ADC |
| 17 | M1 Encoder A |
| 7 | M1 Encoder B |
| 38 | M2 Encoder A (software label; physically INverted — swapped from PINMAP) |
| 39 | M2 Encoder B (software label; physically INverted — swapped from PINMAP) |
| 14 | M3 Encoder A (software label; physically INverted — swapped from PINMAP) |
| 13 | M3 Encoder B (software label; physically INverted — swapped from PINMAP) |
| 11 | M4 Encoder A |
| 12 | M4 Encoder B |

### MCP23017 Pin Assignments (I2C addr 0x20)

| MCP Pin | GPA# | Function |
|---------|------|----------|
| GPA0 | 0 | M1 INB |
| GPA1 | 1 | Red Status LED |
| GPA2 | 2 | Green Status LED |
| GPA3 | 3 | M1 INA |
| GPA4 | 4 | M4 INB |
| GPA5 | 5 | SEL0 (shared, all motors) |
| GPA6 | 6 | SEL1 (shared, all motors) |
| GPA7 | 7 | M4 INA |
| GPB0 | 8 | M3 INB |
| GPB3 | 11 | M3 INA |
| GPB4 | 12 | M2 INB |
| GPB7 | 15 | M2 INA |

---

## 14. Build & Flash

### Build Environment

```ini
[env:esp32-s3-devkitm-1]
platform  = espressif32
board     = esp32-s3-devkitm-1
framework = arduino
monitor_speed = 115200
```

### Dependencies (auto-installed by PlatformIO)

| Library | Version | Purpose |
|---------|---------|---------|
| `mathieucarbou/ESPAsyncWebServer` | ^3.3.23 | Async HTTP server |
| `bblanchon/ArduinoJson` | ^7.0.0 | JSON telemetry serialisation |

### Build Flags

| Flag | Purpose |
|------|---------|
| `-std=gnu++17` | C++17 standard |
| `-O2` | Optimisation level 2 |
| `-DCORE_DEBUG_LEVEL=3` | ESP-IDF verbose logging |
| `ARDUINO_USB_CDC_ON_BOOT=1` | Serial over USB-CDC |
| `ARDUINO_USB_MODE=1` | USB mode configuration |

### Flash & Monitor

```bash
pio run --target upload
pio device monitor
```

---

## 15. Known Gaps & Future Work

| Item | Description |
|------|-------------|
| **Web dashboard watchdog** | No timeout for web commands. If the browser closes or WiFi drops mid-command the robot keeps moving. Fix: timestamp each `/api/cmd` call in `WebDashboard` and stop motors in `motorControlTask` if no command received within ~500 ms. The serial interface already has a working watchdog as a reference. |
| **Per-wheel PID gains** | All four PIDs share the same gains. If individual motors have different characteristics, per-motor gain tuning could improve balance. |
| **Velocity filtering** | Encoder velocity at 10 ms is noisy. A simple exponential moving average (EMA) filter on `getEncoderVelocity()` output would allow Kd > 0 and smoother speed readback on the dashboard. |
| **Telemetry mutex usage** | The `telemetry` struct collected by `telemetryTask` is not used by the web server (which reads directly from `RobotController`). The mutex infrastructure is in place for when thread-safe sharing is needed. |
| **SpeedPIDConfig defaults vs actual gains** | The `SpeedPIDConfig` struct defaults in `RobotController.h` (Kp=0.5, Ki=0.05, Kd=0.01) are out of sync with the constructor initialiser (Kp=0.2, Ki=0.01, Kd=0.0). The constructor wins; the struct defaults should be updated to match. |
