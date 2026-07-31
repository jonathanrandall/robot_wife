# jessica_description

URDF/xacro robot model for Jessica. Used by `robot_state_publisher` and the ros2_control `controller_manager`.

## Files

| File | Description |
|------|-------------|
| `description/jessica.urdf.xacro` | Top-level xacro (includes all sub-xacros) |
| `description/frame.xacro` | Chassis and base link geometry |
| `description/wheels.xacro` | 4 drive wheels with differential drive joints |
| `description/pan_tilt.xacro` | Pan and tilt joints for the head |
| `description/stereo_camera.xacro` | Stereo camera link and joint |
| `description/ros2_control.xacro` | ros2_control hardware interface definitions |
| `description/dimensions.xacro` | Shared physical dimension constants |
| `description/inertial_macros.xacro` | Inertia helper macros |
| `description/materials.xacro` | Visual material colours |
| `description/meshes/` | Visual mesh files (STL/DAE) |

## Launch (RViz preview)

```bash
ros2 launch jessica_description display.launch.py
```
