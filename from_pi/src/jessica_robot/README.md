# jessica_robot

Main ROS 2 package. Contains the AI chatbot, LED controller, head teleop, vision following nodes, and the top-level launch file.

## Nodes

| Node | Description |
|------|-------------|
| `jessica_chatbot` | Full conversation loop: listen → transcribe → LLM → execute → speak → log |
| `hair_led_node` | Drives 5× WS2811 LEDs via SPI from `/jessica/hair_hue` |
| `pan_tilt_teleop` | Joystick right stick → pan/tilt head trajectory commands |
| `person_follower` | Base + head follow a person's shoulders (visual servo); voice-toggled |
| `finger_follower` | Head tracks a raised index finger; voice-toggled |
| `stop_gesture` | Both arms raised → publishes `/jessica/stop` to halt all motion |

## Launch

```bash
ros2 launch jessica_robot jessica.launch.py             # full robot
ros2 launch jessica_robot jessica.launch.py hardware:=false  # chatbot + LEDs only
```

## Configuration

- `config/jessica_controllers.yaml` — ros2_control controller rates
- `config/gamepad_dinput.yaml` / `gamepad_xinput.yaml` — joystick axis/button mapping
- `config/twist_mux.yaml` — velocity source priority (joystick > chatbot > follower)

## Conversation logging

Every turn is logged to `~/jessica_ws/logs/jessica_YYYY-MM-DD.jsonl`. Use `tools/build_examples.py` to extract few-shot examples or fine-tuning pairs from the logs.

## Key topics

| Topic | Type | Notes |
|-------|------|-------|
| `/jessica/hair_hue` | `std_msgs/Int32` | 0–359 = hue, -1 = white, -2 = rainbow |
| `/jessica/ui_state` | `std_msgs/String` | listening / thinking / talking / idle |
| `/jessica/speech_env` | `std_msgs/Float32MultiArray` | speech amplitude envelope for display |
| `/jessica/person_follow/enable` | `std_msgs/Bool` | enable/disable person follower |
| `/jessica/finger_follow/enable` | `std_msgs/Bool` | enable/disable finger follower |
| `/jessica/stop` | `std_msgs/Empty` | emergency stop from gesture |
| `/cmd_vel` | `geometry_msgs/Twist` | autonomous drive commands |
