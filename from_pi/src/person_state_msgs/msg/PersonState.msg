# Full person pose state derived from the stereo camera.
#
# All positions are expressed in the camera optical frame (Z forward, X right,
# Y down).  Use ROS TF to transform into the robot base frame or world frame.
#
# MediaPipe labels left/right relative to the *person*, not the camera.
# e.g. left_shoulder is the person's anatomical left shoulder.

# Timestamp and frame_id from the original camera image.
# frame_id matches the camera link in the TF tree so transforms work directly.
std_msgs/Header header

# True when at least one person was detected in this frame.
bool person_visible

# --- Landmarks (camera optical frame, metres) ---
person_state_msgs/Landmark nose

person_state_msgs/Landmark left_shoulder
person_state_msgs/Landmark right_shoulder

person_state_msgs/Landmark left_elbow
person_state_msgs/Landmark right_elbow

person_state_msgs/Landmark left_wrist
person_state_msgs/Landmark right_wrist

person_state_msgs/Landmark left_hip
person_state_msgs/Landmark right_hip

person_state_msgs/Landmark left_knee
person_state_msgs/Landmark right_knee

person_state_msgs/Landmark left_ankle
person_state_msgs/Landmark right_ankle

# --- Following reference ---
# Midpoint between the two shoulders in the camera optical frame.
# Primary reference point for the robot following task.
# Only valid when both shoulders have depth_valid == true.
geometry_msgs/Point shoulder_midpoint
bool shoulder_midpoint_valid

# --- Pointing gesture ---
# pointing_arm:  0 = not pointing, 1 = left arm, 2 = right arm
# pointing_ray:  unit direction vector (elbow → wrist) in the camera optical
#                frame.  Transform via TF to get a ray in the robot/world frame.
bool pointing_active
uint8 pointing_arm
geometry_msgs/Vector3 pointing_ray
