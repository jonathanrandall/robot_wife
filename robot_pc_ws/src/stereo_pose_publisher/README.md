# stereo_pose_publisher

MediaPipe-based pose and hand estimation node. Subscribes to the Pi's stereo camera topic, runs landmark detection on both eyes, and publishes structured state messages used by the Pi's following nodes.

## Node: `stereo_pose_node`

**Subscribes:**
- `/jessica/camera/image/compressed` — side-by-side stereo frame (640×240 px, left | right)

**Publishes:**
- `/jessica/person_state` (`PersonState`) — pose landmarks + shoulder midpoint in 3D camera frame
- `/jessica/hand_state` (`HandState`) — hand landmarks + index fingertip in 3D camera frame
- Annotated stereo image (optional, for debugging)

## How it works

1. Splits each frame into left (320×240) and right (320×240) halves
2. Applies fisheye lens rectification using pre-computed remap tables from the calibration file
3. Runs MediaPipe `PoseLandmarker` and `HandLandmarker` on both halves
4. Computes metric 3D positions via stereo disparity: `Z = Q[2,3] / (Q[3,2]*d + Q[3,3])`
5. Publishes `PersonState` and `HandState` with 3D landmark positions

## Calibration

The stereo remap tables and Q matrix are loaded from a calibration file. Run the stereo calibration procedure once for your specific camera before using the 3D depth estimates.

## Launch

```bash
ros2 launch stereo_pose_publisher stereo_pose.launch.py
```
