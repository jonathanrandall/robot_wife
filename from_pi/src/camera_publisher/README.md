# camera_publisher

ROS 2 node that captures frames from the USB stereo 3D camera and publishes them as compressed JPEG images.

## Node: `webcam_publisher`

- **Topic:** `/jessica/camera/image/compressed` (`sensor_msgs/CompressedImage`)
- **Rate:** 30 Hz
- **Resolution:** 640×240 px (half the native side-by-side frame: left eye | right eye)
- **Format:** JPEG at quality 80

The camera is identified by device name (`"3D USB Camera"`) via `v4l2-ctl`, so the `/dev/videoN` index doesn't need to be hardcoded.

## Launch

```bash
ros2 launch camera_publisher camera_publisher.launch.py
```

Or starts automatically as part of `jessica_robot jessica.launch.py`.
