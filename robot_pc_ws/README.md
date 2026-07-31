# robot_pc_ws — PC / Laptop ROS 2 Workspace

ROS 2 packages that run on the GPU-equipped laptop or desktop PC. Built with colcon (ROS 2 Jazzy).

## Build

```bash
cd robot_pc_ws
colcon build
source install/setup.bash
```

## Packages

| Package | Description |
|---------|-------------|
| `stereo_pose_publisher` | MediaPipe pose + hand estimation on stereo camera frames |
| `jessica_description` | Copy of the robot URDF model (for RViz on the PC) |
| `person_state_msgs` | Custom message definitions shared with the Pi |

## Why on the PC?

MediaPipe pose and hand estimation is too CPU-intensive for the Raspberry Pi 5 at useful frame rates. Running it on the PC with a GPU keeps latency low and leaves the Pi free for the control stack.

The PC communicates with the Pi over the LAN via standard ROS 2 DDS (same `ROS_DOMAIN_ID`).
