# Jessica — open issues

## 1. Encoder-health flags: decide what to do when one goes 0 (deferred by design)

The new motor firmware's STATE message carries 4 extra 1/0 flags — one per wheel —
where 0 means "this motor's encoder is not working properly".

What's implemented now (2026-07-14, `esp32_combined_hardware`):
- The 12-field STATE is parsed and the flags are stored in `encoder_ok_[4]`.
- On a 1→0 transition an ERROR is logged naming the wheel joint.
- Nothing else happens: velocities/positions from that wheel are still fed to
  diff_cont as-is.

Still to decide (next session material):
- Should a dead encoder trigger a soft-stop / e-stop, or just degrade odometry?
- diff_cont averages both wheels per side — a wheel with a dead encoder reports
  garbage velocity, polluting odom. Option: drop the bad wheel from the average
  (needs a custom controller or firmware-side handling).
- Expose the flags as custom state interfaces (e.g. for diagnostics /
  a dashboard) instead of just logging?

## 2. ⚠️ New motor firmware is NOT what's flashed (or in temp/) — verified live 2026-07-14

Sending `GET` to `/dev/esp32_motor` returns the OLD 10-field STATE
(`STATE,4×pos,4×vel,pan,tilt`) and the source in `temp/esp32_bridge_servo`
(dated Jul 7) also still prints the old format — no encoder-ok flags anywhere.
The 12-field firmware described for this upgrade exists somewhere else (PC?)
or hasn't been written yet.

The ROS interface handles both:
- 12-field STATE → new protocol, 4-field `CMD,lf,lr,rf,rr`.
- 10-field STATE → logs a one-time warning and falls back to the legacy
  6-field CMD (two trailing zeros) so the robot still drives on old firmware.

**TODO:** flash the real 12-field firmware to the motor ESP32 (and copy its
source into temp/ so it can be inspected here). Assumed new formats — verify
against the real firmware when it lands:
- `STATE,lf_pos,lr_pos,rf_pos,rr_pos,lf_vel,lr_vel,rf_vel,rr_vel,lf_ok,lr_ok,rf_ok,rr_ok`
- `CMD,lf_vel,lr_vel,rf_vel,rr_vel` (cm/s)

## 3. Servo controller (hiwonder head) — notes from bring-up 2026-07-14

- udev rule for `/dev/esp32_servo` written but needs sudo — see below.
- **Position feedback verified working** end-to-end through ros2_control
  (commanded 0.3/-0.2 rad, read back 0.283/-0.184 — quantized to servo units).
  BUT: earlier bench tests returned `position: [null, null]` for a while
  (probably servo power off at the time). If nulls persist, the interface
  logs a warning and reports commanded positions as state (open loop), so the
  head keeps working like the old PCA9685 setup did.
- The board **reboots when the serial port opens** (CH340 DTR) and homes both
  servos to centre; the hardware interface waits `boot_wait_ms` (3 s) for it.
  Every controller_manager start therefore recentres the head — same net
  effect as the launch's home_head step.
- Commands are rate-limited to one `ptr` per 100 ms (`min_send_period_ms`):
  at the full 30 Hz loop rate the board's UART overflows and commands arrive
  mangled (seen live: `{"error": "unknown command: pt9ptr"}`).
- Servo firmware clamps tilt to 220–780 units = ±1.32 rad; URDF tilt limits
  are -1.3..0.87 (tightened from -1.5 to stay inside the clamp).
- **Dead-head incident 2026-07-19, FIXED**: head stopped responding; stack
  looked healthy but /joint_states echoed commands exactly (= open-loop
  fallback). Cause: main.py on the servo board had died to the REPL — its
  loop catches `Exception` but a stray Ctrl-C byte (0x03) on the USB console
  raises `KeyboardInterrupt`, which escapes and kills it. The board then sits
  silent until a reset EDGE; merely reopening the port doesn't make one.
  **Permanent fix in `esp32_servo_hardware` on_configure**: esptool-style
  hard-reset pulse (DTR low + RTS high 100 ms = EN low, then both low = run)
  before the boot wait, so every launch boots fresh firmware and homes the
  head. Gotcha learned: pulsing DTR ALONE leaves (DTR low, RTS high) which
  HOLDS the board in reset on the two-transistor circuit — both lines must
  end deasserted. Verified live: real quantized feedback after launch.
  Bench tool: `temp/test_head_serial.py` (no-ROS serial test with the same
  reset logic + diagnosis; stop the stack first — it checks).
- **Direction signs verified on the real head 2026-07-14**: tilt was mirrored
  (forward/back swapped); fixed with a per-servo `DIRECTIONS` sign map in the
  servo firmware's `config.py` (uploaded to the board via mpremote, local copy
  in temp/micropython_servo_control matches). Pan was correct. User confirmed
  the full launch + head direction test all working. Any future mirrored axis:
  flip it in that map, never in ROS. Direction test script:
  `~/jessica_ws/temp/test_head_directions.sh` (needs the stack running).

## 4. Arducam ToF camera (front, CSI) — WORKING, verified live 2026-07-15

- New package `tof_publisher` publishes `/jessica/tof/image/compressed`
  (PNG, mono8) at 10 Hz; in jessica.launch.py behind `tof:=true` (default).
- Grayscale mapping: `gray = (2000 - depth_mm) * 255 / 2000` → 255 = 0 mm,
  0 = ≥2 m. Invalid pixels (no return/NaN) forced to 0 (far), not 255.
  Camera RANGE control set to 2000 mm. Later: navigation/object avoidance
  on the PC consumes this.
- **The camera is single-client**: a second process calling `start()` spins
  forever at 100% CPU. Close preview_jr.py before launching the stack.
- ⚠️ Killing a client mid-stream can wedge the kernel driver: the process
  survives `kill -9` (stuck in-kernel, R state) and `open()` then hangs for
  everyone until reboot. Seen 2026-07-15 with an orphaned preview_jr.py.
  (Only the second-client-spinning case wedges; a normal streaming process
  being SIGTERMed released the camera cleanly in testing.)
- If the sensor vanishes (SDK says "I2C bus name doesn't match any bus
  present!", dmesg shows `arducam-pivariety 10-000c: probe failed`), it's
  the CSI ribbon — sensor absent from I2C bus 10. Fixed 2026-07-15 by
  reseating the cable. Retry without reboot:
  `echo 10-000c | sudo tee /sys/bus/i2c/drivers/arducam-pivariety/bind`
- Live-verified: 240x180 @ 10.000 Hz, ~28 KB/frame PNG, correct grayscale,
  camera reopens cleanly across restarts.

## 5. udev rule — DONE 2026-07-14

`/etc/udev/rules.d/99-esp32-servo.rules`: CH340 1a86:7523 → `/dev/esp32_servo`.
Applied and verified; full ros2_control bring-up then tested on the REAL ports:
both hardware systems + diff_cont + pan_tilt_controller + joint_broad all
activated, /joint_states carries all 6 joints with live servo feedback, and
the legacy-firmware fallback warning fired as designed (issue 2 still open).
(Motor board keeps its rule: ESP32-S3 303a:1001 → /dev/esp32_motor.)
