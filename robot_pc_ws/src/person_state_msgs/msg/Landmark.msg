# 3D position of a single pose landmark in the camera optical frame (metres).
# Only meaningful when depth_valid is true; position is zeroed otherwise.
geometry_msgs/Point position

# MediaPipe visibility score in [0.0, 1.0].
# Taken as the minimum of the left-eye and right-eye scores so that a landmark
# only gets a high score when it is clearly visible in both images.
float32 visibility

# True when stereo disparity was available and position.z is metric depth.
# False when the landmark was not detected in one or both eyes.
bool depth_valid
