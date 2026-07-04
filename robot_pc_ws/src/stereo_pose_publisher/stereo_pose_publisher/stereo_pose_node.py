#!/usr/bin/env python3
"""
stereo_pose_node — MediaPipe pose and hand estimation on a rectified stereo
camera stream.

Overview
--------
Subscribes to the side-by-side compressed image published by the Jessica stereo
camera (left eye | right eye, 640×240 px total).  For every incoming frame it:

  1. Splits the frame into a 320×240 left half and a 320×240 right half.
  2. Applies fisheye lens rectification to each half using pre-computed remap
     tables from the calibration file.
  3. Runs MediaPipe PoseLandmarker and HandLandmarker on each rectified half.
  4. Draws the pose skeleton (green joints, red bones) and hand skeleton
     (cyan joints, magenta bones) on each half.
  5. For the nose, left shoulder and right shoulder, annotates metric depth (Z)
     on the left eye image.
  6. Re-combines the annotated halves side-by-side and annotates the live
     frame-rate in the top-left corner.
  7. Publishes the annotated image, a PersonState message (pose landmarks,
     shoulder midpoint, pointing gesture) and a HandState message (hand
     landmarks, index fingertips).

Depth calculation
-----------------
After rectification, matching 3-D points lie on the same horizontal scan-line
in both eye images.  For a landmark at pixel x_L (left) and x_R (right):

    disparity  d = x_L - x_R                       (pixels, positive for real objects)
    depth      Z = Q[2,3] / (Q[3,2]*d + Q[3,3])   (metres)

Back-projection to camera frame
--------------------------------
Given metric depth Z and the rectified projection matrix P1 (focal length f,
principal point cx, cy):

    X_cam = (x_px - cx) * Z / f
    Y_cam = (y_px - cy) * Z / f
    Z_cam = Z

The result is in the camera optical frame (Z forward, X right, Y down).

Pointing detection
------------------
An arm is classified as pointing when the elbow angle (shoulder–elbow–wrist)
exceeds 150°.  The pointing_ray (unit elbow→wrist in camera frame) is published
in PersonState so the controller can transform it into the robot frame via TF.

HandLandmarker — handedness convention
---------------------------------------
MediaPipe reports handedness from the camera's point of view (mirror
convention).  For a person facing the camera, MediaPipe's "Left" hand is
typically the person's anatomical right hand and vice versa.

Topics
------
  Subscribes : /jessica/camera/image/compressed   (sensor_msgs/CompressedImage)
  Publishes  : /jessica/camera/pose/compressed    (sensor_msgs/CompressedImage)
               /jessica/person_state              (person_state_msgs/PersonState)
               /jessica/hand_state                (person_state_msgs/HandState)

Parameters
----------
  use_gpu (bool, default: true)
      Run MediaPipe models on the GPU.  Set false to force CPU inference.

Running
-------
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ros2 launch stereo_pose_publisher stereo_pose.launch.py

  # Force CPU:
  ros2 launch stereo_pose_publisher stereo_pose.launch.py use_gpu:=false

  # View annotated stream:
  rqt  →  Plugins → Visualization → Image View
           topic: /jessica/camera/pose/compressed

Build
-----
  colcon build --packages-select person_state_msgs
  colcon build --packages-select stereo_pose_publisher --symlink-install

Configuration files (in the package share directory)
-----------------------------------------------------
  config/fisheye_stereo_charuco_calibration_v2.npz
  config/pose_landmarker_full.task
  config/hand_landmarker.task
"""

import sys
sys.path.insert(0, '/home/jonny/venvs/jazzy/lib/python3.12/site-packages')

import os
import time
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks.python.vision.pose_landmarker import (
    PoseLandmarker, PoseLandmarkerOptions, PoseLandmarksConnections,
)
from mediapipe.tasks.python.vision.hand_landmarker import (
    HandLandmarker, HandLandmarkerOptions, HandLandmarksConnections,
)
from mediapipe.tasks.python.vision.core.vision_task_running_mode import VisionTaskRunningMode as RunningMode
from mediapipe.tasks.python.vision.drawing_utils import draw_landmarks, DrawingSpec

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ament_index_python.packages import get_package_share_directory

from person_state_msgs.msg import PersonState
from person_state_msgs.msg import HandState
from person_state_msgs.msg import Landmark as LandmarkMsg


# ---------------------------------------------------------------------------
# Topics
# ---------------------------------------------------------------------------
INPUT_TOPIC        = "/jessica/camera/image/compressed"
OUTPUT_IMAGE_TOPIC = "/jessica/camera/pose/compressed"
OUTPUT_STATE_TOPIC = "/jessica/person_state"
OUTPUT_HAND_TOPIC  = "/jessica/hand_state"

JPEG_QUALITY = 65

EYE_WIDTH  = 320
EYE_HEIGHT = 240

# ---------------------------------------------------------------------------
# MediaPipe pose landmark indices
# Reference: https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker
# ---------------------------------------------------------------------------
_LM_NOSE           = 0
_LM_LEFT_SHOULDER  = 11
_LM_RIGHT_SHOULDER = 12
_LM_LEFT_ELBOW     = 13
_LM_RIGHT_ELBOW    = 14
_LM_LEFT_WRIST     = 15
_LM_RIGHT_WRIST    = 16
_LM_LEFT_HIP       = 23
_LM_RIGHT_HIP      = 24
_LM_LEFT_KNEE      = 25
_LM_RIGHT_KNEE     = 26
_LM_LEFT_ANKLE     = 27
_LM_RIGHT_ANKLE    = 28

# Hand landmark index for the index fingertip
_HAND_INDEX_TIP = 8

# Landmarks shown with a Z depth label on the image overlay
_DEPTH_DISPLAY = {
    _LM_NOSE:           "Nose",
    _LM_LEFT_SHOULDER:  "L.Shoulder",
    _LM_RIGHT_SHOULDER: "R.Shoulder",
}

# Arm joint triples for pointing detection (shoulder, elbow, wrist).
# pointing_arm value: 1 = left arm, 2 = right arm, 0 = none.
_ARM_JOINTS = {
    1: (_LM_LEFT_SHOULDER,  _LM_LEFT_ELBOW,  _LM_LEFT_WRIST),
    2: (_LM_RIGHT_SHOULDER, _LM_RIGHT_ELBOW, _LM_RIGHT_WRIST),
}

_POINTING_ANGLE_THRESHOLD = 150.0   # degrees — elbow angle above which arm is extended
_MIN_VISIBILITY           = 0.5
_MIN_DEPTH                = 0.1     # metres
_MAX_DEPTH                = 10.0    # metres

# FPS display — exponential moving average smoothing factor
_FPS_ALPHA = 0.1

# ---------------------------------------------------------------------------
# Drawing specs
# ---------------------------------------------------------------------------
_POSE_CONNECTIONS = list(PoseLandmarksConnections.POSE_LANDMARKS)
_POSE_LM_SPEC     = DrawingSpec(color=(0, 255, 0),   thickness=2, circle_radius=3)
_POSE_CONN_SPEC   = DrawingSpec(color=(0, 0, 255),   thickness=2)

_HAND_CONNECTIONS = list(HandLandmarksConnections.HAND_CONNECTIONS)
_HAND_LM_SPEC     = DrawingSpec(color=(255, 255, 0), thickness=2, circle_radius=3)  # cyan
_HAND_CONN_SPEC   = DrawingSpec(color=(255, 0, 255), thickness=2)                   # magenta

_LABEL_FONT      = cv2.FONT_HERSHEY_SIMPLEX
_LABEL_SCALE     = 0.4
_LABEL_THICKNESS = 1
_LABEL_COLOR     = (0, 230, 255)   # yellow
_LABEL_BG        = (0, 0, 0)

_FPS_COLOR = (0, 255, 0)           # green


# ---------------------------------------------------------------------------
# Module-level helpers
# ---------------------------------------------------------------------------

def _delegate(use_gpu: bool):
    return (mp.tasks.BaseOptions.Delegate.GPU if use_gpu
            else mp.tasks.BaseOptions.Delegate.CPU)


def _build_pose_landmarker(model_path: str, use_gpu: bool) -> PoseLandmarker:
    """Create a PoseLandmarker in VIDEO mode."""
    base_opts = mp.tasks.BaseOptions(model_asset_path=model_path,
                                     delegate=_delegate(use_gpu))
    opts = PoseLandmarkerOptions(
        base_options=base_opts,
        running_mode=RunningMode.VIDEO,
        num_poses=1,
        min_pose_detection_confidence=0.5,
        min_pose_presence_confidence=0.5,
        min_tracking_confidence=0.5,
        output_segmentation_masks=False,
    )
    return PoseLandmarker.create_from_options(opts)


def _build_hand_landmarker(model_path: str, use_gpu: bool) -> HandLandmarker:
    """Create a HandLandmarker in VIDEO mode detecting up to 2 hands."""
    base_opts = mp.tasks.BaseOptions(model_asset_path=model_path,
                                     delegate=_delegate(use_gpu))
    opts = HandLandmarkerOptions(
        base_options=base_opts,
        running_mode=RunningMode.VIDEO,
        num_hands=2,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    return HandLandmarker.create_from_options(opts)


def _draw_label(img: np.ndarray, text: str, px: int, py: int) -> None:
    """Draw text with a black background rectangle, clamped to image bounds."""
    (tw, th), baseline = cv2.getTextSize(text, _LABEL_FONT, _LABEL_SCALE, _LABEL_THICKNESS)
    px = max(0, min(px, img.shape[1] - tw - 4))
    py = max(th + 4, min(py, img.shape[0] - baseline - 2))
    cv2.rectangle(img, (px - 2, py - th - 2), (px + tw + 2, py + baseline), _LABEL_BG, -1)
    cv2.putText(img, text, (px, py), _LABEL_FONT, _LABEL_SCALE,
                _LABEL_COLOR, _LABEL_THICKNESS, cv2.LINE_AA)


def _find_hand_landmarks(result, handedness_name: str):
    """Return the landmark list for the first hand matching handedness_name, or None."""
    for i, hedness in enumerate(result.handedness):
        if hedness[0].category_name == handedness_name:
            return result.hand_landmarks[i]
    return None


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class StereoPoseNode(Node):
    """Stereo pose and hand estimation node with GPU/CPU selection and FPS display."""

    def __init__(self):
        super().__init__("stereo_pose_node")

        # --- Parameters ---
        # Pose and hand models are kept on separate delegates because running
        # both on the GPU simultaneously causes a tensor write-contention error
        # inside MediaPipe.  Default: pose on CPU, hand on GPU.
        self.declare_parameter('use_gpu_pose', False)
        self.declare_parameter('use_gpu_hand', False)
        use_gpu_pose = self.get_parameter('use_gpu_pose').get_parameter_value().bool_value
        use_gpu_hand = self.get_parameter('use_gpu_hand').get_parameter_value().bool_value
        self.get_logger().info(
            f"Pose model: {'GPU' if use_gpu_pose else 'CPU'} | "
            f"Hand model: {'GPU' if use_gpu_hand else 'CPU'}"
        )

        share_dir = get_package_share_directory("stereo_pose_publisher")

        # --- Calibration ---
        calib_path = os.path.join(
            share_dir, "config", "fisheye_stereo_charuco_calibration_v2.npz"
        )
        self.get_logger().info(f"Loading calibration from {calib_path}")
        cal = np.load(calib_path)

        self._map1x = cal["map1x"]
        self._map1y = cal["map1y"]
        self._map2x = cal["map2x"]
        self._map2y = cal["map2y"]
        self._Q    = cal["Q"]

        P1 = cal["P1"]
        self._f  = float(P1[0, 0])
        self._cx = float(P1[0, 2])
        self._cy = float(P1[1, 2])

        cal_w = int(cal["image_width"])
        cal_h = int(cal["image_height"])
        if cal_w != EYE_WIDTH or cal_h != EYE_HEIGHT:
            self.get_logger().warn(
                f"Calibration maps are {cal_w}x{cal_h} but node expects "
                f"{EYE_WIDTH}x{EYE_HEIGHT}. Rectification may be wrong."
            )

        # --- Pose landmarkers (one per eye) ---
        pose_model = os.path.join(share_dir, "config", "pose_landmarker_full.task")
        self.get_logger().info(f"Loading pose model from {pose_model}")
        self._pose_left  = _build_pose_landmarker(pose_model, use_gpu_pose)
        self._pose_right = _build_pose_landmarker(pose_model, use_gpu_pose)

        # --- Hand landmarkers (one per eye, up to 2 hands each) ---
        hand_model = os.path.join(share_dir, "config", "hand_landmarker.task")
        self.get_logger().info(f"Loading hand model from {hand_model}")
        self._hand_left  = _build_hand_landmarker(hand_model, use_gpu_hand)
        self._hand_right = _build_hand_landmarker(hand_model, use_gpu_hand)

        # --- Thread pool for parallel model inference ---
        # The four detectors (pose-left, pose-right, hand-left, hand-right) are
        # independent so we submit them all at once and collect results.
        self._executor = ThreadPoolExecutor(max_workers=4)

        # --- FPS tracking ---
        self._last_frame_time: float | None = None
        self._fps = 0.0

        # --- ROS I/O ---
        self._sub = self.create_subscription(
            CompressedImage, INPUT_TOPIC, self._image_callback, 10
        )
        self._pub_image = self.create_publisher(CompressedImage, OUTPUT_IMAGE_TOPIC, 10)
        self._pub_state = self.create_publisher(PersonState,     OUTPUT_STATE_TOPIC,  10)
        self._pub_hand  = self.create_publisher(HandState,       OUTPUT_HAND_TOPIC,   10)

        self.get_logger().info(
            f"Subscribed to {INPUT_TOPIC}\n"
            f"Publishing image  → {OUTPUT_IMAGE_TOPIC}\n"
            f"Publishing state  → {OUTPUT_STATE_TOPIC}\n"
            f"Publishing hands  → {OUTPUT_HAND_TOPIC}"
        )

    # ------------------------------------------------------------------
    # Geometry
    # ------------------------------------------------------------------

    def _stereo_depth(self, lm_left, lm_right) -> float | None:
        """Metric depth (metres) from the disparity of a matched landmark pair."""
        disparity = (lm_left.x - lm_right.x) * EYE_WIDTH
        if disparity <= 0:
            return None
        denom = self._Q[3, 2] * disparity + self._Q[3, 3]
        if denom == 0:
            return None
        z = float(self._Q[2, 3] / denom)
        return z if _MIN_DEPTH <= z <= _MAX_DEPTH else None

    def _to_3d(self, lm_left, lm_right) -> np.ndarray | None:
        """Back-project a matched landmark pair to a 3-D camera-frame point (metres)."""
        z = self._stereo_depth(lm_left, lm_right)
        if z is None:
            return None
        px = lm_left.x * EYE_WIDTH
        py = lm_left.y * EYE_HEIGHT
        return np.array([
            (px - self._cx) * z / self._f,
            (py - self._cy) * z / self._f,
            z,
        ])

    # ------------------------------------------------------------------
    # Detection
    # ------------------------------------------------------------------

    def _detect_pose(self, landmarker: PoseLandmarker,
                     bgr_img: np.ndarray, ts_ms: int):
        rgb    = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        return landmarker.detect_for_video(mp_img, ts_ms)

    def _detect_hand(self, landmarker: HandLandmarker,
                     bgr_img: np.ndarray, ts_ms: int):
        rgb    = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        return landmarker.detect_for_video(mp_img, ts_ms)

    # ------------------------------------------------------------------
    # Image annotation
    # ------------------------------------------------------------------

    def _rectify(self, img: np.ndarray, map_x: np.ndarray, map_y: np.ndarray) -> np.ndarray:
        return cv2.remap(img, map_x, map_y,
                         interpolation=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT)

    def _annotate_pose(self, img: np.ndarray, landmarks: list) -> np.ndarray:
        out = img.copy()
        draw_landmarks(out, landmarks, _POSE_CONNECTIONS,
                       landmark_drawing_spec=_POSE_LM_SPEC,
                       connection_drawing_spec=_POSE_CONN_SPEC)
        return out

    def _annotate_hand(self, img: np.ndarray, hand_result) -> np.ndarray:
        """Draw all detected hands onto img (modifies in-place, returns img)."""
        for hand_lms in hand_result.hand_landmarks:
            draw_landmarks(img, hand_lms, _HAND_CONNECTIONS,
                           landmark_drawing_spec=_HAND_LM_SPEC,
                           connection_drawing_spec=_HAND_CONN_SPEC)
        return img

    def _draw_depth_labels(self, img: np.ndarray,
                           lms_left: list, lms_right: list) -> None:
        for idx, name in _DEPTH_DISPLAY.items():
            if idx >= len(lms_left) or idx >= len(lms_right):
                continue
            lm_l, lm_r = lms_left[idx], lms_right[idx]
            vis_l = lm_l.visibility if lm_l.visibility is not None else 0.0
            vis_r = lm_r.visibility if lm_r.visibility is not None else 0.0
            if vis_l < _MIN_VISIBILITY or vis_r < _MIN_VISIBILITY:
                continue
            z = self._stereo_depth(lm_l, lm_r)
            if z is None:
                continue
            px = int(lm_l.x * EYE_WIDTH)
            py = int(lm_l.y * EYE_HEIGHT)
            _draw_label(img, f"{name} Z:{z:.2f}m", px + 6, py - 6)

    def _draw_fps(self, img: np.ndarray) -> None:
        """Draw FPS counter in the top-left corner of the combined image."""
        text = f"FPS: {self._fps:.1f}"
        cv2.putText(img, text, (6, 16), _LABEL_FONT, 0.5,
                    _LABEL_BG, 3, cv2.LINE_AA)          # dark outline
        cv2.putText(img, text, (6, 16), _LABEL_FONT, 0.5,
                    _FPS_COLOR, 1, cv2.LINE_AA)          # green text

    # ------------------------------------------------------------------
    # PersonState building
    # ------------------------------------------------------------------

    def _build_landmark_msg(self, lm_left, lm_right) -> LandmarkMsg:
        msg      = LandmarkMsg()
        vis_l    = lm_left.visibility  if lm_left.visibility  is not None else 0.0
        vis_r    = lm_right.visibility if lm_right.visibility is not None else 0.0
        msg.visibility = float(min(vis_l, vis_r))
        pos = self._to_3d(lm_left, lm_right)
        if pos is not None:
            msg.position.x = float(pos[0])
            msg.position.y = float(pos[1])
            msg.position.z = float(pos[2])
            msg.depth_valid = True
        return msg

    def _safe_lm(self, lms_left: list, lms_right: list, idx: int) -> LandmarkMsg:
        if idx < len(lms_left) and idx < len(lms_right):
            return self._build_landmark_msg(lms_left[idx], lms_right[idx])
        return LandmarkMsg()

    def _detect_pointing(self, lms_left: list, lms_right: list):
        """Return (active, arm_id, unit_ray) for the first extended arm found."""
        cos_thresh = np.cos(np.radians(_POINTING_ANGLE_THRESHOLD))
        for arm_id, (s_idx, e_idx, w_idx) in _ARM_JOINTS.items():
            visible = True
            for idx in (s_idx, e_idx, w_idx):
                if idx >= len(lms_left) or idx >= len(lms_right):
                    visible = False; break
                if min(lms_left[idx].visibility  or 0.0,
                       lms_right[idx].visibility or 0.0) < _MIN_VISIBILITY:
                    visible = False; break
            if not visible:
                continue

            s = self._to_3d(lms_left[s_idx], lms_right[s_idx])
            e = self._to_3d(lms_left[e_idx], lms_right[e_idx])
            w = self._to_3d(lms_left[w_idx], lms_right[w_idx])
            if s is None or e is None or w is None:
                continue

            es = s - e
            ew = w - e
            n_es, n_ew = np.linalg.norm(es), np.linalg.norm(ew)
            if n_es < 0.01 or n_ew < 0.01:
                continue

            if np.dot(es / n_es, ew / n_ew) < cos_thresh:
                return True, arm_id, ew / n_ew

        return False, 0, np.zeros(3)

    def _build_person_state(self, lms_left: list, lms_right: list,
                            stamp, frame_id: str) -> PersonState:
        msg = PersonState()
        msg.header.stamp    = stamp
        msg.header.frame_id = frame_id
        msg.person_visible  = True

        msg.nose            = self._safe_lm(lms_left, lms_right, _LM_NOSE)
        msg.left_shoulder   = self._safe_lm(lms_left, lms_right, _LM_LEFT_SHOULDER)
        msg.right_shoulder  = self._safe_lm(lms_left, lms_right, _LM_RIGHT_SHOULDER)
        msg.left_elbow      = self._safe_lm(lms_left, lms_right, _LM_LEFT_ELBOW)
        msg.right_elbow     = self._safe_lm(lms_left, lms_right, _LM_RIGHT_ELBOW)
        msg.left_wrist      = self._safe_lm(lms_left, lms_right, _LM_LEFT_WRIST)
        msg.right_wrist     = self._safe_lm(lms_left, lms_right, _LM_RIGHT_WRIST)
        msg.left_hip        = self._safe_lm(lms_left, lms_right, _LM_LEFT_HIP)
        msg.right_hip       = self._safe_lm(lms_left, lms_right, _LM_RIGHT_HIP)
        msg.left_knee       = self._safe_lm(lms_left, lms_right, _LM_LEFT_KNEE)
        msg.right_knee      = self._safe_lm(lms_left, lms_right, _LM_RIGHT_KNEE)
        msg.left_ankle      = self._safe_lm(lms_left, lms_right, _LM_LEFT_ANKLE)
        msg.right_ankle     = self._safe_lm(lms_left, lms_right, _LM_RIGHT_ANKLE)

        ls, rs = msg.left_shoulder, msg.right_shoulder
        if ls.depth_valid and rs.depth_valid:
            msg.shoulder_midpoint.x    = (ls.position.x + rs.position.x) / 2.0
            msg.shoulder_midpoint.y    = (ls.position.y + rs.position.y) / 2.0
            msg.shoulder_midpoint.z    = (ls.position.z + rs.position.z) / 2.0
            msg.shoulder_midpoint_valid = True

        active, arm_id, ray = self._detect_pointing(lms_left, lms_right)
        msg.pointing_active = active
        msg.pointing_arm    = arm_id
        msg.pointing_ray.x  = float(ray[0])
        msg.pointing_ray.y  = float(ray[1])
        msg.pointing_ray.z  = float(ray[2])

        return msg

    # ------------------------------------------------------------------
    # HandState building
    # ------------------------------------------------------------------

    def _build_hand_landmark_msg(self, lm_left, lm_right) -> LandmarkMsg:
        """Build a Landmark message from a matched hand landmark pair."""
        msg      = LandmarkMsg()
        # Hand landmarks don't have visibility; use presence or default 1.0
        msg.visibility = 1.0
        pos = self._to_3d(lm_left, lm_right)
        if pos is not None:
            msg.position.x = float(pos[0])
            msg.position.y = float(pos[1])
            msg.position.z = float(pos[2])
            msg.depth_valid = True
        return msg

    def _build_hand_landmarks_array(self, lms_left: list,
                                    lms_right: list) -> list[LandmarkMsg]:
        """Build the 21-element landmark array for one hand from matched eye results."""
        out = []
        for i in range(21):
            if i < len(lms_left) and i < len(lms_right):
                out.append(self._build_hand_landmark_msg(lms_left[i], lms_right[i]))
            else:
                out.append(LandmarkMsg())
        return out

    def _build_hand_state(self, hand_left_eye, hand_right_eye,
                          stamp, frame_id: str) -> HandState:
        msg = HandState()
        msg.header.stamp    = stamp
        msg.header.frame_id = frame_id

        # Match hands between eyes by handedness label
        for side, attr_detected, attr_lms, attr_tip in [
            ('Left',  'left_hand_detected',  'left_hand_landmarks',  'left_index_tip'),
            ('Right', 'right_hand_detected', 'right_hand_landmarks', 'right_index_tip'),
        ]:
            lms_l = _find_hand_landmarks(hand_left_eye,  side)
            lms_r = _find_hand_landmarks(hand_right_eye, side)

            if lms_l is not None and lms_r is not None:
                setattr(msg, attr_detected, True)
                lm_array = self._build_hand_landmarks_array(lms_l, lms_r)
                setattr(msg, attr_lms, lm_array)
                # Index fingertip convenience field
                if _HAND_INDEX_TIP < len(lms_l) and _HAND_INDEX_TIP < len(lms_r):
                    tip = self._build_hand_landmark_msg(
                        lms_l[_HAND_INDEX_TIP], lms_r[_HAND_INDEX_TIP]
                    )
                    setattr(msg, attr_tip, tip)

        return msg

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------

    def _image_callback(self, msg: CompressedImage):
        """Decode → rectify → pose + hand detection → annotate → publish."""
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("Failed to decode incoming image")
            return

        # FPS calculation using monotonic wall clock
        now = time.monotonic()
        if self._last_frame_time is not None:
            dt = now - self._last_frame_time
            if dt > 0:
                self._fps = _FPS_ALPHA * (1.0 / dt) + (1.0 - _FPS_ALPHA) * self._fps
        self._last_frame_time = now

        ts_ms = int(self.get_clock().now().nanoseconds // 1_000_000)

        # Split and rectify
        left_raw  = frame[:, :EYE_WIDTH]
        right_raw = frame[:, EYE_WIDTH:]
        left_rect  = self._rectify(left_raw,  self._map1x, self._map1y)
        right_rect = self._rectify(right_raw, self._map2x, self._map2y)

        # Run all four model inferences in parallel — they are independent.
        fut_pose_l = self._executor.submit(self._detect_pose, self._pose_left,  left_rect,  ts_ms)
        fut_pose_r = self._executor.submit(self._detect_pose, self._pose_right, right_rect, ts_ms)
        fut_hand_l = self._executor.submit(self._detect_hand, self._hand_left,  left_rect,  ts_ms)
        fut_hand_r = self._executor.submit(self._detect_hand, self._hand_right, right_rect, ts_ms)

        pose_result_l = fut_pose_l.result()
        pose_result_r = fut_pose_r.result()
        hand_result_l = fut_hand_l.result()
        hand_result_r = fut_hand_r.result()

        lms_pose_l = pose_result_l.pose_landmarks[0] if pose_result_l.pose_landmarks else None
        lms_pose_r = pose_result_r.pose_landmarks[0] if pose_result_r.pose_landmarks else None

        # --- Annotate left eye ---
        left_ann = (self._annotate_pose(left_rect, lms_pose_l)
                    if lms_pose_l is not None else left_rect.copy())
        self._annotate_hand(left_ann, hand_result_l)
        if lms_pose_l is not None and lms_pose_r is not None:
            self._draw_depth_labels(left_ann, lms_pose_l, lms_pose_r)

        # --- Annotate right eye ---
        right_ann = (self._annotate_pose(right_rect, lms_pose_r)
                     if lms_pose_r is not None else right_rect.copy())
        self._annotate_hand(right_ann, hand_result_r)

        # Combine and add FPS overlay
        combined = np.hstack([left_ann, right_ann])
        self._draw_fps(combined)

        # Publish annotated image
        _, buffer = cv2.imencode(".jpg", combined, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        img_out = CompressedImage()
        img_out.header.stamp    = msg.header.stamp
        img_out.header.frame_id = msg.header.frame_id
        img_out.format          = "jpeg"
        img_out.data            = buffer.tobytes()
        self._pub_image.publish(img_out)

        # Publish PersonState
        if lms_pose_l is not None and lms_pose_r is not None:
            state = self._build_person_state(
                lms_pose_l, lms_pose_r, msg.header.stamp, msg.header.frame_id
            )
        else:
            state = PersonState()
            state.header.stamp    = msg.header.stamp
            state.header.frame_id = msg.header.frame_id
            state.person_visible  = False
        self._pub_state.publish(state)

        # Publish HandState
        hand_state = self._build_hand_state(
            hand_result_l, hand_result_r, msg.header.stamp, msg.header.frame_id
        )
        self._pub_hand.publish(hand_state)

    def destroy_node(self):
        self._executor.shutdown(wait=False)
        self._pose_left.close()
        self._pose_right.close()
        self._hand_left.close()
        self._hand_right.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StereoPoseNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
