
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray


THUMB_OPEN_DISTANCE = 1.8
THUMB_CLOSED_DISTANCE = 0.85
REVERSE_THUMB_ROOT = False

INPUT_TIMEOUT = 0.5
FILTER_ALPHA = 0.25

SAFE_TARGETS = [0.0] * 6


def clamp(value, low=0.0, high=1.0):
    return max(low, min(high, value))


def calculate_distance(p1, p2):
    return math.sqrt(
        sum((p1[i] - p2[i]) ** 2 for i in range(3))
    )


def calculate_joint_angle(p1, p2, p3):
    v1 = [p1[i] - p2[i] for i in range(3)]
    v2 = [p3[i] - p2[i] for i in range(3)]

    dot = sum(v1[i] * v2[i] for i in range(3))
    norm1 = math.sqrt(sum(v * v for v in v1))
    norm2 = math.sqrt(sum(v * v for v in v2))

    if norm1 < 1e-8 or norm2 < 1e-8:
        return None

    cosine = clamp(
        dot / (norm1 * norm2), -1.0, 1.0
    )

    return math.degrees(math.acos(cosine))


def normalize_flexion(angle):
    flexion = 180.0 - angle
    target = clamp(flexion / 90.0)
    return flexion, target


def calculate_thumb_root(landmarks):
    thumb_tip = landmarks[4]
    index_mcp = landmarks[5]
    pinky_mcp = landmarks[17]

    palm_width = calculate_distance(
        index_mcp, pinky_mcp
    )

    if palm_width < 1e-8:
        return None

    thumb_distance = calculate_distance(
        thumb_tip, pinky_mcp
    )

    ratio = thumb_distance / palm_width

    denominator = (
        THUMB_OPEN_DISTANCE - THUMB_CLOSED_DISTANCE
    )

    if denominator <= 0:
        return None

    target = clamp(
        (THUMB_OPEN_DISTANCE - ratio) / denominator
    )

    if REVERSE_THUMB_ROOT:
        target = 1.0 - target

    return ratio, target


class PoseCommandNode(Node):

    def __init__(self):
        super().__init__('pose_command_node')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/hand_landmarks',
            self.landmarks_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/rohand_sim/target',
            10
        )

        self.finger_joints = {
            'Index': (5, 6, 7),
            'Middle': (9, 10, 11),
            'Ring': (13, 14, 15),
            'Pinky': (17, 18, 19)
        }

        self.last_valid_input = None
        self.filtered_targets = None
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

    def publish_targets(self, targets):
        msg = Float64MultiArray()
        msg.data = [float(value) for value in targets]
        self.publisher.publish(msg)

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
                self.filtered_targets = None

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

        targets = []

        for finger, joints in self.finger_joints.items():
            mcp, pip, dip = joints

            angle = calculate_joint_angle(
                landmarks[mcp],
                landmarks[pip],
                landmarks[dip]
            )

            if angle is None:
                self.get_logger().warn(
                    f'Invalid {finger} geometry'
                )
                return

            _, target = normalize_flexion(angle)
            targets.append(target)

        thumb_root_result = calculate_thumb_root(
            landmarks
        )

        if thumb_root_result is None:
            self.get_logger().warn(
                'Invalid thumb root geometry'
            )
            return

        thumb_ratio, thumb_root_target = (
            thumb_root_result
        )

        thumb_angle = calculate_joint_angle(
            landmarks[2],
            landmarks[3],
            landmarks[4]
        )

        if thumb_angle is None:
            self.get_logger().warn(
                'Invalid thumb flexion geometry'
            )
            return

        _, thumb_flex_target = normalize_flexion(
            thumb_angle
        )

        targets.extend([
            thumb_root_target,
            thumb_flex_target
        ])

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
        if self.filtered_targets is None:
            self.filtered_targets = list(targets)
        else:
            self.filtered_targets = [
                clamp(
                    FILTER_ALPHA * current
                    + (1.0 - FILTER_ALPHA) * previous
                )
                for current, previous in zip(
                    targets, self.filtered_targets
                )
            ]

        self.publish_targets(self.filtered_targets)

        self.frame_count += 1

        if self.frame_count % 20 == 0:
            self.get_logger().info(
                'Targets: '
                + str([
                    round(v, 3)
                    for v in self.filtered_targets
                ])
                + f' | thumb_ratio={thumb_ratio:.3f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PoseCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
