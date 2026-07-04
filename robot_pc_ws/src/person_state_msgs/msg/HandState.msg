# Hand landmark state for both hands derived from the stereo camera.
#
# All positions are in the camera optical frame (Z forward, X right, Y down),
# in metres.  Use ROS TF to transform into other frames.
#
# Handedness note: MediaPipe reports handedness from the camera's point of view
# (mirror convention).  For a person facing the camera, MediaPipe's "Left" hand
# is typically the person's right hand and vice versa.  Label accordingly in
# the controller.
#
# Hand landmark indices (21 per hand):
#   0  wrist
#   1-4   thumb  (cmc, mcp, ip, tip)
#   5-8   index  (mcp, pip, dip, tip)   ← tip = 8
#   9-12  middle (mcp, pip, dip, tip)
#   13-16 ring   (mcp, pip, dip, tip)
#   17-20 pinky  (mcp, pip, dip, tip)

std_msgs/Header header

bool left_hand_detected
bool right_hand_detected

# All 21 landmarks for each hand (camera optical frame, metres).
person_state_msgs/Landmark[21] left_hand_landmarks
person_state_msgs/Landmark[21] right_hand_landmarks

# Convenience fields — index fingertip (landmark 8) for each hand.
# Primary reference point for finger-tracking and gesture tasks.
person_state_msgs/Landmark left_index_tip
person_state_msgs/Landmark right_index_tip
