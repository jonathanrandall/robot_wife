# esp32_servo_hardware

A `ros2_control` hardware interface plugin for the **head servos only** — an alternative to the combined interface in `esp32_combined_hardware`.

Communicates with the MicroPython ESP32 over a dedicated serial port using the same `ptr <pan_rad> <tilt_rad>` text protocol as the combined interface.

Use this plugin when the head servo ESP32 is on a separate serial port from the motor ESP32 (e.g. during development or when the motor board is not present).
