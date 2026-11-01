import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, Float32MultiArray, Float64, Float64MultiArray

from finger_node.common import FINGER_LIMITS, FINGERS, finger_command_topic


# Thumb rotation angle grows opposite to the robot's thumb_root
# (checked with the webcam), so the normalized value is flipped.
REVERSE_THUMB_ROOT = True

INPUT_TIMEOUT = 0.5
# EMA on measured joint angles (same value as windows_sender/Py_webCamReader.py)
FILTER_ALPHA = 0.7

# Auto calibration: each channel maps its observed [min, max] angle to 0~1
# (0 = open). The range starts from these defaults (deg) and only widens,
# so opening and closing the hand once calibrates it.
# Order == FINGERS: [index, middle, ring, little, thumb_root, thumb]
DEFAULT_RANGES = [
    (15.0, 60.0),
    (15.0, 60.0),
    (15.0, 60.0),
    (15.0, 60.0),
    None,           # thumb rotation: no sensible default, learned only
    (10.0, 40.0),
]
MIN_RANGE = 10.0    # deg; narrower ranges output 0 (open)

SAFE_TARGETS = [0.0] * 6


def clamp(value, low=0.0, high=1.0):
    return max(low, min(high, value))


def subtract(a, b):
    return [a[i] - b[i] for i in range(3)]


def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def normalize_vector(v):
    norm = math.sqrt(dot(v, v))

    if norm < 1e-8:
        return None

    return [c / norm for c in v]


def calculate_joint_angle(p1, p2, p3):
    v1 = subtract(p1, p2)
    v2 = subtract(p3, p2)

    norm1 = math.sqrt(dot(v1, v1))
    norm2 = math.sqrt(dot(v2, v2))

    if norm1 < 1e-8 or norm2 < 1e-8:
        return None

    cosine = clamp(
        dot(v1, v2) / (norm1 * norm2), -1.0, 1.0
    )

    return math.degrees(math.acos(cosine))


# Measurements below follow windows_sender/Py_webCamReader.py (person 1).

def finger_bend_angle(landmarks, base):
    """Weighted MCP / PIP / DIP flexion (deg) of index..little."""

    mcp = calculate_joint_angle(
        landmarks[0], landmarks[base], landmarks[base + 1]
    )
    pip = calculate_joint_angle(
        landmarks[base], landmarks[base + 1], landmarks[base + 2]
    )
    dip = calculate_joint_angle(
        landmarks[base + 1], landmarks[base + 2], landmarks[base + 3]
    )

    if mcp is None or pip is None or dip is None:
        return None

    return (
        0.35 * (180.0 - mcp)
        + 0.45 * (180.0 - pip)
        + 0.20 * (180.0 - dip)
    )


def thumb_bend_angle(landmarks):
    """Mean MCP / IP flexion (deg) of the thumb."""

    mcp = calculate_joint_angle(landmarks[1], landmarks[2], landmarks[3])
    ip = calculate_joint_angle(landmarks[2], landmarks[3], landmarks[4])

    if mcp is None or ip is None:
        return None

    return 0.5 * (180.0 - mcp) + 0.5 * (180.0 - ip)


def palm_axes(landmarks):
    """x: pinky MCP -> index MCP, y: wrist -> palm centre, z: palm normal."""

    wrist, index_mcp, pinky_mcp = landmarks[0], landmarks[5], landmarks[17]

    x_axis = normalize_vector(subtract(index_mcp, pinky_mcp))

    if x_axis is None:
        return None

    palm_center = [(index_mcp[i] + pinky_mcp[i]) / 2.0 for i in range(3)]
    y_temp = subtract(palm_center, wrist)
    along_x = dot(y_temp, x_axis)
    y_axis = normalize_vector(
        [y_temp[i] - along_x * x_axis[i] for i in range(3)]
    )

    if y_axis is None:
        return None

    z_axis = normalize_vector(cross(x_axis, y_axis))

    if z_axis is None:
        return None

    return x_axis, y_axis, z_axis


def thumb_rotation_angle(landmarks):
    """Angle (deg) of the thumb CMC -> MCP direction in the palm plane."""

    axes = palm_axes(landmarks)

    if axes is None:
        return None

    x_axis, y_axis, z_axis = axes
    thumb = subtract(landmarks[2], landmarks[1])
    along_z = dot(thumb, z_axis)
    projected = normalize_vector(
        [thumb[i] - along_z * z_axis[i] for i in range(3)]
    )

    if projected is None:
        return None

    return math.degrees(
        math.atan2(dot(projected, x_axis), dot(projected, y_axis))
    )


def measure_hand(landmarks):
    """Joint angles (deg) in FINGERS order; None where geometry is invalid."""

    return [
        finger_bend_angle(landmarks, 5),
        finger_bend_angle(landmarks, 9),
        finger_bend_angle(landmarks, 13),
        finger_bend_angle(landmarks, 17),
        thumb_rotation_angle(landmarks),
        thumb_bend_angle(landmarks),
    ]


class AutoRange:
    """Observed [min, max] of one channel, widened by every new value."""

    def __init__(self, default):
        self.default = default
        self.reset()

    def reset(self):
        if self.default is None:
            self.low, self.high = math.inf, -math.inf
        else:
            self.low, self.high = self.default

    def update(self, value):
        self.low = min(self.low, value)
        self.high = max(self.high, value)

    def normalize(self, value):
        if self.high - self.low < MIN_RANGE:
            return 0.0

        return clamp((value - self.low) / (self.high - self.low))


class PoseCommandNode(Node):

    def __init__(self):
        super().__init__('pose_command_node')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/hand_landmarks',
            self.landmarks_callback,
            10
        )

        # 0~1 targets for hardware_connection_test/rohand_sim_ros2.py
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/rohand_sim/target',
            10
        )

        # Finger nodes: one Float64 (rad) per actuator.
        # targets order == FINGERS order:
        # [index, middle, ring, little, thumb_root, thumb]
        self.finger_publishers = [
            self.create_publisher(
                Float64,
                finger_command_topic(finger),
                10
            )
            for finger in FINGERS
        ]

        self.create_subscription(
            Empty,
            '/pose_command/reset_calibration',
            self.reset_calibration,
            10
        )

        self.ranges = [AutoRange(default) for default in DEFAULT_RANGES]

        self.last_valid_input = None
        self.filtered_angles = None
        self.timed_out = False
        self.frame_count = 0

        self.create_timer(0.05, self.watchdog_callback)

        self.get_logger().info(
            'Pose Command Node - Stabilized Version'
        )
        self.get_logger().info(
            'SIMULATION ONLY'
        )
        self.get_logger().info(
            '6-axis control enabled'
        )
        self.get_logger().info(
            f'Input timeout: {INPUT_TIMEOUT}s'
        )
        self.get_logger().info(
            f'Filter alpha: {FILTER_ALPHA}'
        )
        self.get_logger().info(
            'Auto calibration: open and close the hand fully once '
            '(reset: /pose_command/reset_calibration)'
        )

    def reset_calibration(self, _msg):
        for auto_range in self.ranges:
            auto_range.reset()

        self.get_logger().info('Calibration reset')

    def publish_targets(self, targets):
        msg = Float64MultiArray()
        msg.data = [float(value) for value in targets]
        self.publisher.publish(msg)

        # 0~1 (0 = open) -> joint angle in rad within each finger's range
        for finger, value, publisher in zip(
            FINGERS, targets, self.finger_publishers
        ):
            low, high = FINGER_LIMITS[finger]
            publisher.publish(
                Float64(data=low + clamp(value) * (high - low))
            )

    def watchdog_callback(self):
        now = time.monotonic()

        if self.last_valid_input is None:
            # Before receiving the first valid frame,
            # periodically request the default pose.
            self.publish_targets(SAFE_TARGETS)
            return

        elapsed = now - self.last_valid_input

        if elapsed >= INPUT_TIMEOUT:
            if not self.timed_out:
                self.timed_out = True
                self.filtered_angles = None

                self.get_logger().warn(
                    'HAND INPUT TIMEOUT - default pose'
                )

            self.publish_targets(SAFE_TARGETS)

    def landmarks_callback(self, msg):
        if len(msg.data) != 63:
            self.get_logger().warn(
                f'Invalid data length: {len(msg.data)}'
            )
            return

        if not all(math.isfinite(v) for v in msg.data):
            self.get_logger().warn(
                'Invalid landmark values'
            )
            return

        landmarks = [
            tuple(msg.data[i * 3:i * 3 + 3])
            for i in range(21)
        ]

        angles = measure_hand(landmarks)

        for finger, angle in zip(FINGERS, angles):
            if angle is None:
                self.get_logger().warn(
                    f'Invalid {finger} geometry'
                )
                return

        now = time.monotonic()

        # Record the timestamp only after successful
        # validation and calculation.
        recovered = self.timed_out

        self.last_valid_input = now
        self.timed_out = False

        if recovered:
            self.get_logger().info(
                'HAND INPUT RECOVERED'
            )

        # First valid frame starts from the measured pose.
        if self.filtered_angles is None:
            self.filtered_angles = list(angles)
        else:
            self.filtered_angles = [
                FILTER_ALPHA * current
                + (1.0 - FILTER_ALPHA) * previous
                for current, previous in zip(
                    angles, self.filtered_angles
                )
            ]

        targets = []

        for k, angle in enumerate(self.filtered_angles):
            self.ranges[k].update(angle)
            targets.append(self.ranges[k].normalize(angle))

        thumb_root = FINGERS.index('thumb_root')

        if REVERSE_THUMB_ROOT and self.ranges[thumb_root].high - \
                self.ranges[thumb_root].low >= MIN_RANGE:
            targets[thumb_root] = 1.0 - targets[thumb_root]

        self.publish_targets(targets)

        self.frame_count += 1

        if self.frame_count % 20 == 0:
            self.get_logger().info(
                'Targets: '
                + str([round(v, 3) for v in targets])
                + ' | ranges(deg): '
                + str([
                    (round(r.low), round(r.high))
                    if r.high >= r.low else None
                    for r in self.ranges
                ])
            )


def main(args=None):
    rclpy.init(args=args)
    node = PoseCommandNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise  # a real error; otherwise Humble's take_message raced with Ctrl+C
    finally:
        node.destroy_node()
        # Ctrl+C under ros2 launch may already have shut the context down
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
