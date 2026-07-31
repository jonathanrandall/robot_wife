# person_state_msgs

Custom ROS 2 message definitions for Jessica's vision pipeline.

## Messages

### `Landmark.msg`
A single 3D landmark in the camera optical frame (Z forward, X right, Y down, metres).

### `PersonState.msg`
Published by `stereo_pose_node` on `/jessica/person_state`. Contains:
- Pose landmarks (nose, shoulders, wrists, etc.)
- `shoulder_midpoint` — 3D position of the midpoint between both shoulders (the follow target for `person_follower`)
- Pointing gesture flag

### `HandState.msg`
Published by `stereo_pose_node` on `/jessica/hand_state`. Contains:
- Hand landmarks (wrist, finger joints, fingertips)
- Index fingertip position (used by `finger_follower`)

Used by both the Pi-side follower nodes and the PC-side `stereo_pose_node` that generates them.
