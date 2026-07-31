# Jessica Launcher

A standalone power-on menu for the Waveshare 7" touchscreen. Runs outside ROS (no `ros2` dependency at runtime) so it can start from systemd before anything else.

## What it does

Presents three touch buttons:

| Button | Action |
|--------|--------|
| **Start Jessica** | Full robot — hardware + cameras + chatbot + display |
| **Chatbot Only** | Mic/speakers/display, no ESP32s or joystick |
| **Drive Mode** | Hardware + cameras, no chatbot (gamepad control) |

While a stack is running, the launcher releases the display (KMS/DRM master) so `jessica_display` can use it. Hold the screen for ~3 seconds to stop the running stack and return to the menu.

Small corner buttons provide **Exit** (quit the launcher) and **Shutdown** (power off the Pi safely via `systemctl poweroff`).

## Logs

- `~/jessica_ws/logs/launcher.log` — launcher output
- `~/jessica_ws/logs/launcher_stack.log` — ros2 launch output (new file each start)

## Sudoers rule (one-time setup for shutdown button)

```bash
echo 'jonny ALL=(root) NOPASSWD: /usr/sbin/shutdown' | \
    sudo tee /etc/sudoers.d/010-jessica-shutdown >/dev/null
sudo chmod 440 /etc/sudoers.d/010-jessica-shutdown
```
