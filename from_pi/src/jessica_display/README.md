# jessica_display

Animated UI for the Waveshare 7" touchscreen. Uses **pygame over KMS/DRM** — no desktop or X11 required, works via SSH.

## Node: `display_node`

Subscribes to two topics and renders the appropriate animation:

| `/jessica/ui_state` value | Display |
|--------------------------|---------|
| `listening` | "Listening..." — hue-cycling text (pink → yellow) |
| `thinking` | "Thinking..." — hue-cycling text (green → blue) |
| `talking` | Animated soundwaves, amplitude from speech envelope |
| `idle` | Dim breathing dot, low frame rate (~10 fps) |

`/jessica/speech_env` carries the speech amplitude envelope as a `Float32MultiArray` — the chatbot publishes it just before playback begins so the wave animation is in sync with Jessica's voice.

Touch events are published on `/jessica/touch` as `geometry_msgs/Point`.

## Launch

```bash
ros2 launch jessica_display jessica_display.launch.py
```

Or it starts automatically as part of `jessica_robot jessica.launch.py`.

## Dependencies

- `pygame` — must be the **system** package (`apt install python3-pygame`), not the pip wheel, as the pip version lacks the KMS/DRM driver.
