"""ROS side of the GUI: subscribes to the pipeline topics and keeps the latest values.

The node spins in a background thread; the Qt thread reads copies through snapshot().
Nothing here touches Qt.
"""
import math
import threading
import time

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import Bool, Empty, Float32MultiArray, Float64, String

from finger_node.common import (
    FINGERS, HW_MOTION_ENABLE_TOPIC, HW_MOTION_STATE_TOPIC, HW_STATE_TOPIC, PROXIMAL_FINGERS, SIM_FULL_STATE_TOPIC, TACTILE_DOT_MATRIX,
    TACTILE_FORCE3D, TACTILE_REGIONS, finger_command_topic, finger_state_topic,
    finger_tactile_proximal_topic, finger_tactile_topic, hw_tactile_topic,
)

WEBCAM_TOPIC = '/webcam/image_annotated/compressed'
ROBOT_CAMERA_TOPIC = '/gripper_camera/color/image_raw'
LANDMARKS_TOPIC = '/hand_landmarks'
RESET_CALIBRATION_TOPIC = '/pose_command/reset_calibration'
NAN = float('nan')
# fingertips + palm, then the proximal phalanx sensors (same order as /feedback/tactile_summary)
SENSORS = TACTILE_REGIONS + tuple(f'{f}_proximal' for f in PROXIMAL_FINGERS)


def tactile_summary(kind, data):
    """(normal, tangential, direction_deg) of one sensor.

    force3d: data = [normal N, tangential N, direction_deg] per point; NaN / -1 = invalid.
             normal is summed, shear is summed as vectors.
    normal / dot_matrix: values per point / taxel (NaN = none); normal = sum, no shear.
    """
    values = np.asarray(data, dtype=np.float64)
    if kind == TACTILE_FORCE3D and values.size >= 3:
        points = values[: values.size // 3 * 3].reshape(-1, 3)
        valid = np.isfinite(points).all(axis=1) & (points[:, 0] >= 0) & (points[:, 1] >= 0)
        if not valid.any():
            return NAN, NAN, NAN
        points = points[valid]
        angle = np.radians(points[:, 2])
        shear = np.array([np.sum(points[:, 1] * np.cos(angle)), np.sum(points[:, 1] * np.sin(angle))])
        tangential = float(np.hypot(*shear))
        direction = math.degrees(math.atan2(shear[1], shear[0])) if tangential > 0 else 0.0
        return float(points[:, 0].sum()), tangential, direction
    valid = values[np.isfinite(values) & (values >= 0)]
    return (float(valid.sum()) if valid.size else NAN), NAN, NAN


def decode_image(msg):
    """sensor_msgs/Image rgb8 / bgr8 / mono8 -> RGB uint8 array (other encodings: None)."""
    channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        return None
    image = np.frombuffer(bytes(msg.data), np.uint8)
    image = image.reshape(msg.height, msg.step)[:, : msg.width * channels]
    image = image.reshape(msg.height, msg.width, channels)
    if msg.encoding == 'bgr8':
        image = image[:, :, ::-1]
    elif channels == 1:
        image = np.repeat(image, 3, axis=2)
    return np.ascontiguousarray(image)


class RosBridge(Node):
    def __init__(self):
        super().__init__('hand_gui')
        p = self.declare_parameter
        self.robot_camera_topic = p('robot_camera_topic', ROBOT_CAMERA_TOPIC).value
        # display only, counter-clockwise degrees (0 / 90 / 180 / 270); recordings stay as captured
        self.robot_camera_rotation = int(p('robot_camera_rotation', 90).value) % 360
        self.webcam_topic = p('webcam_topic', WEBCAM_TOPIC).value
        self.lock = threading.Lock()

        self.fingers = {f: dict(command=NAN, angle=NAN, velocity=NAN, effort=NAN, stamp=0.0)
                        for f in FINGERS}
        self.tactile = {r: dict(kind='', raw=None, normal=NAN, tangential=NAN, direction=NAN,
                                stamp=0.0) for r in SENSORS}
        self.full_state = (None, 0.0)   # joint name -> qpos, monotonic time
        self.hw_state = (None, 0.0)     # finger -> actuator rad
        self.landmarks = (None, 0.0)
        self.webcam_jpeg = (None, 0.0)
        self.robot_image = (None, 0.0)

        for f in FINGERS:
            self.create_subscription(JointState, finger_state_topic(f),
                                     lambda m, f=f: self.on_state(f, m), 10)
            self.create_subscription(Float64, finger_command_topic(f),
                                     lambda m, f=f: self.on_command(f, m), 10)
        for r in SENSORS:
            if r == 'palm':
                topic = hw_tactile_topic(r)
            elif r.endswith('_proximal'):
                topic = finger_tactile_proximal_topic(r[:-len('_proximal')])
            else:
                topic = finger_tactile_topic(r)
            self.create_subscription(Float32MultiArray, topic,
                                     lambda m, r=r: self.on_tactile(r, m), 10)
        self.create_subscription(JointState, SIM_FULL_STATE_TOPIC, self.on_full_state, 10)
        self.create_subscription(JointState, HW_STATE_TOPIC, self.on_hw_state, 10)
        self.create_subscription(Float32MultiArray, LANDMARKS_TOPIC, self.on_landmarks, 10)
        self.create_subscription(CompressedImage, self.webcam_topic, self.on_webcam, 2)
        self.create_subscription(Image, self.robot_camera_topic, self.on_robot_image, 2)
        self.reset_publisher = self.create_publisher(Empty, RESET_CALIBRATION_TOPIC, 1)
        self.motion_publisher = self.create_publisher(Bool, HW_MOTION_ENABLE_TOPIC, 10)
        self.motion_state = (None, 0.0)  # running | stopped | read_only, monotonic time
        self.create_subscription(String, HW_MOTION_STATE_TOPIC, self.on_motion_state, 10)
        self.get_logger().info(f'GUI listening: webcam {self.webcam_topic}, '
                               f'robot camera {self.robot_camera_topic}')

    # ---- callbacks (executor thread) ------------------------------------------
    def on_state(self, finger, msg):
        def first(values):
            return float(values[0]) if len(values) else NAN
        with self.lock:
            entry = self.fingers[finger]
            entry.update(angle=first(msg.position), velocity=first(msg.velocity),
                         effort=first(msg.effort), stamp=time.monotonic())

    def on_command(self, finger, msg):
        with self.lock:
            self.fingers[finger]['command'] = float(msg.data)

    def on_tactile(self, region, msg):
        kind = msg.layout.dim[0].label if msg.layout.dim else TACTILE_DOT_MATRIX
        normal, tangential, direction = tactile_summary(kind, msg.data)
        with self.lock:
            self.tactile[region].update(kind=kind, raw=np.asarray(msg.data, np.float32),
                                        normal=normal, tangential=tangential,
                                        direction=direction, stamp=time.monotonic())

    def on_full_state(self, msg):
        with self.lock:
            self.full_state = (dict(zip(msg.name, msg.position)), time.monotonic())

    def on_hw_state(self, msg):
        with self.lock:
            self.hw_state = (dict(zip(msg.name, msg.position)), time.monotonic())

    def on_landmarks(self, msg):
        with self.lock:
            self.landmarks = (np.asarray(msg.data, np.float32), time.monotonic())

    def on_motion_state(self, msg):
        with self.lock:
            self.motion_state = (msg.data, time.monotonic())

    def on_webcam(self, msg):
        with self.lock:
            self.webcam_jpeg = (bytes(msg.data), time.monotonic())

    def on_robot_image(self, msg):
        image = decode_image(msg)
        with self.lock:
            self.robot_image = (image, time.monotonic())

    # ---- Qt thread -------------------------------------------------------------
    def set_motion(self, enabled):
        """START (True) / STOP (False) the robot hand. Sent 3 times, a STOP must not get lost."""
        for _ in range(3):
            self.motion_publisher.publish(Bool(data=enabled))

    def reset_calibration(self):
        """Ask pose_command_node to forget its auto calibration."""
        self.reset_publisher.publish(Empty())

    def snapshot(self):
        """Copy of everything the GUI needs, taken under the lock."""
        with self.lock:
            return dict(
                time=time.monotonic(),
                fingers={f: dict(v) for f, v in self.fingers.items()},
                tactile={r: dict(v) for r, v in self.tactile.items()},
                full_state=self.full_state,
                hw_state=self.hw_state,
                landmarks=self.landmarks,
                webcam_jpeg=self.webcam_jpeg,
                robot_image=self.robot_image,
                motion_state=self.motion_state,
            )
