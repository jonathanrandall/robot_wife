# ESP32 Firmware Update Task: Serial Communication Protocol

## Context

You are updating the ESP32 firmware for a mobile robot with pan/tilt camera. The ESP32 communicates via serial with a ROS2 hardware interface running on a PC (Linux mini PC). The ROS2 hardware interface code is located at:
- `new_pcb_robot/src/esp32_combined_hardware/src/esp32_combined_hardware.cpp`

## Current Hardware Configuration

The robot has:
- **4 DC motors with encoders** (differential drive robot with 4-wheel layout)
  - Left front wheel
  - Left rear wheel
  - Right front wheel
  - Right rear wheel
- **2 servo motors** for pan/tilt camera mechanism
  - Pan joint (horizontal rotation)
  - Tilt joint (vertical rotation)

## Serial Communication Protocol

The ESP32 must handle **three types of messages FROM the PC** and send **one type of message TO the PC**.

### 1. Messages FROM PC → ESP32

#### Get State Message (GET)
**Format:** `GET\n`

**Description:** Request for current robot state. When the ESP32 receives this message, it should immediately respond with a STATE message.

**Parameters:** None

**Expected Response:** ESP32 should send a STATE message (see below)

**Example:**
```
GET\n
```

**Important:** This implements a request-response pattern where the PC requests data and the ESP32 responds. This ensures fresh data and prevents stale messages from accumulating.

#### Command Message (CMD)
**Format:** `CMD,lf_vel,lr_vel,rf_vel,rr_vel,pan_pos,tilt_pos\n`

**Description:** Main control command sent every control cycle (~10Hz)

**Parameters:**
- `lf_vel`: Left front wheel velocity in **cm/s** (floating point)
- `lr_vel`: Left rear wheel velocity in **cm/s** (floating point)
- `rf_vel`: Right front wheel velocity in **cm/s** (floating point)
- `rr_vel`: Right rear wheel velocity in **cm/s** (floating point)
- `pan_pos`: Pan servo position in **radians** (floating point)
- `tilt_pos`: Tilt servo position in **radians** (floating point)

**Example:**
```
CMD,15.5,15.5,15.5,15.5,0.0,0.5\n
```
This commands all wheels to move at 15.5 cm/s forward, pan at 0 radians (center), tilt at 0.5 radians.

#### Auxiliary Command Message (AUX)
**Format:** `AUX,command_name,arg\n`

**Description:** Special auxiliary commands for additional robot functions

**Parameters:**
- `command_name`: String identifier for the command
- `arg`: Optional argument (may or may not be present depending on command)

**Example:**
```
AUX,led,on\n
AUX,buzzer,short\n
```

### 2. Messages FROM ESP32 → PC

#### State Message (STATE)
**Format:** `STATE,lf_pos,lr_pos,rf_pos,rr_pos,lf_vel,lr_vel,rf_vel,rr_vel,pan_pos,tilt_pos\n`

**Description:** Robot state sent to PC every read cycle (~10Hz)

**Parameters:**
- `lf_pos`: Left front wheel position in **encoder counts** (integer)
- `lr_pos`: Left rear wheel position in **encoder counts** (integer)
- `rf_pos`: Right front wheel position in **encoder counts** (integer)
- `rr_pos`: Right rear wheel position in **encoder counts** (integer)
- `lf_vel`: Left front wheel velocity in **cm/s** (floating point)
- `lr_vel`: Left rear wheel velocity in **cm/s** (floating point)
- `rf_vel`: Right front wheel velocity in **cm/s** (floating point)
- `rr_vel`: Right rear wheel velocity in **cm/s** (floating point)
- `pan_pos`: Pan servo position in **radians** (floating point)
- `tilt_pos`: Tilt servo position in **radians** (floating point)

**Example:**
```
STATE,1234,1240,1235,1238,15.2,15.3,15.1,15.4,0.0,0.5\n
```

**Important Notes:**
- Wheel positions are cumulative encoder counts (not reset)
- Wheel velocities should match the commanded velocities (after control loop)
- Messages must end with newline character `\n`

## Implementation Requirements

### What You Need to Update

Update the `processSerialCommand()` function (or equivalent serial handling function) in your ESP32 firmware to:

1. **Parse incoming CMD messages**
   - Extract the 6 parameters
   - Apply wheel velocities to the 4 motors
   - Apply servo positions to pan/tilt servos (PLACEHOLDER - see below)

2. **Parse incoming AUX messages**
   - Handle auxiliary commands (PLACEHOLDER - see below)

3. **Send STATE messages**
   - Read encoder positions from all 4 wheels
   - Calculate current velocities from encoders
   - Read current pan/tilt positions (PLACEHOLDER - see below)
   - Format and send STATE message

### Implementation Status

#### ✅ FULLY IMPLEMENT: Motor Control
- Parse wheel velocity commands from CMD messages
- Control the 4 DC motors with the commanded velocities
- Read encoder values and calculate velocities
- Send encoder positions and velocities in STATE messages

#### ⚠️ PLACEHOLDER: Pan/Tilt Servos
**The pan/tilt servos are NOT YET IMPLEMENTED on the ESP32.**

For now, implement placeholders:

**For CMD message (write/control):**
```cpp
// Extract pan_pos and tilt_pos from CMD message
// For now, just print to Serial for debugging
Serial.print("Pan/Tilt CMD received: pan=");
Serial.print(pan_pos);
Serial.print(" tilt=");
Serial.println(tilt_pos);
// TODO: Implement actual servo control
```

**For STATE message (read/feedback):**
```cpp
// For pan/tilt positions, always return 0.0 for now
float pan_pos = 0.0;
float tilt_pos = 0.0;
// TODO: Read actual servo positions when implemented
```

#### ⚠️ PLACEHOLDER: AUX Commands
**Auxiliary commands are NOT YET IMPLEMENTED.**

For now, implement a placeholder:

```cpp
// Parse AUX message
// For now, just print to Serial for debugging
Serial.print("AUX command received: ");
Serial.print(command_name);
Serial.print(" arg: ");
Serial.println(arg);
// TODO: Implement actual auxiliary command handling
```

## Expected Behavior

1. ESP32 continuously reads serial input
2. When **GET** message arrives:
   - Immediately send a STATE message with current robot state
   - This implements a request-response pattern
3. When **CMD** message arrives, parse it and:
   - Control motors with velocities
   - Print pan/tilt values to Serial (placeholder)
4. When **AUX** message arrives:
   - Print to Serial (placeholder)

**Communication Flow:**
```
PC → ESP32: GET\n
ESP32 → PC: STATE,1234,1240,1235,1238,15.2,15.3,15.1,15.4,0.0,0.5\n

PC → ESP32: CMD,15.5,15.5,15.5,15.5,0.0,0.5\n
(ESP32 applies commands, no response needed)

PC → ESP32: GET\n
ESP32 → PC: STATE,1250,1256,1251,1254,15.5,15.5,15.5,15.5,0.0,0.5\n
```

**Note:** The PC sends GET messages at ~10Hz to request state updates, and CMD messages at ~10Hz to control the robot.

## Hardware Parameters

For reference, the ROS2 side uses these parameters:
- **Wheel radius:** 7.2 cm (0.072 m)
- **Encoder counts per revolution:** 1632 counts
- **Wheel separation:** 29.7 cm
- **Serial baud rate:** 115200 (typically)

## Testing Tips

1. Test with manual serial commands first
2. Verify STATE messages are properly formatted
3. Check that encoder counts increment correctly
4. Verify velocity control works for all 4 wheels
5. Confirm pan/tilt placeholders print to Serial but don't crash

## Questions to Consider

- What library are you using for motor control?
- What library for encoder reading?
- How often should STATE messages be sent? (Recommend 10-20Hz)
- Should velocities be in a PID control loop or open-loop?

## Your Task

Please update the ESP32 firmware's serial command processing function to implement the protocol described above, with:
- **GET message handling:** When GET is received, immediately send a STATE message with current robot state
- **Full motor control implementation:** Parse CMD messages and control the 4 motors
- **Placeholder pan/tilt servo handling:** Print received values to Serial on write, return 0.0 for read in STATE message
- **Placeholder AUX command handling:** Print to Serial for debugging

Provide the updated code for the command processing function.
