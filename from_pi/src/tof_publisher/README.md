# tof_publisher

ROS 2 node that reads depth frames from the Arducam ToF camera (CSI) and publishes them as lossless PNG images.

## Node: `tof_publisher`

- **Topic:** `/jessica/tof/image/compressed` (`sensor_msgs/CompressedImage`)
- **Rate:** 10 Hz
- **Format:** PNG (lossless — preserves exact depth values for downstream processing)

### Depth encoding

Pixel brightness encodes distance: **255 = touching the lens, 0 = at/beyond max range**.

```
gray = (MAX_DISTANCE_MM - depth) * 255 / MAX_DISTANCE_MM
```

Invalid pixels (no return or low confidence) are forced to 0 (far), not 255, so a dropout never looks like an obstacle at the lens.

Default max range: 4000 mm (configurable in `tof_publisher.py`).

## Launch

```bash
ros2 launch tof_publisher tof_publisher.launch.py
```
