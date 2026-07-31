# test_ws

Scratch ROS 2 workspace used for testing and developing the robot URDF model in isolation, without needing the full Pi stack running.

Contains a copy of `jessica_description` for iterating on the robot model and previewing it in RViz.

```bash
cd test_ws
colcon build
source install/setup.bash
ros2 launch jessica_description display.launch.py
```
