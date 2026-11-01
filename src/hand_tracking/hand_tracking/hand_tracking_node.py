
import json
import math
import socket

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


UDP_HOST = "0.0.0.0"
UDP_PORT = 5005

# MediaPipe world landmarks 21 x (x, y, z) -> pose_command_node
LANDMARKS_TOPIC = "/hand_landmarks"
LANDMARK_COUNT = 21 * 3

# Calibrated finger values 0~1 (monitoring / recording only)
# [thumb, index, middle, ring, pinky, thumb_rotation], 1.0 = open
TOPIC_NAME = "/hand/finger_angles"
FG_COUNT = 6


def parse_floats(values, count, low=None, high=None):
    """Return a float list of the given length, or None if invalid."""

    if not isinstance(values, list) or len(values) != count:
        return None

    values = [float(v) for v in values]

    if not all(math.isfinite(v) for v in values):
        return None

    if low is not None and not all(low <= v <= high for v in values):
        return None

    return values


class HandTrackingNode(Node):

    def __init__(self):
        super().__init__("hand_tracking_node")

        # ROS 2 Publishers
        self.landmarks_publisher = self.create_publisher(
            Float32MultiArray,
            LANDMARKS_TOPIC,
            10
        )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            TOPIC_NAME,
            10
        )

        # UDP 소켓 생성 및 바인딩
        self.udp_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.udp_socket.bind(
            (UDP_HOST, UDP_PORT)
        )

        self.udp_socket.setblocking(False)

        # 100Hz 주기로 UDP 데이터 확인
        self.timer = self.create_timer(
            0.01,
            self.receive_data
        )

        self.get_logger().info(
            f"UDP receiver started: {UDP_HOST}:{UDP_PORT}"
        )

        self.get_logger().info(
            f"Publishing topics: {LANDMARKS_TOPIC}, {TOPIC_NAME}"
        )

    def receive_data(self):

        for _ in range(20):

            try:
                data, _ = self.udp_socket.recvfrom(65536)

            except BlockingIOError:
                break

            except OSError as error:
                self.get_logger().error(
                    f"UDP receive error: {error}"
                )
                break

            try:
                payload = json.loads(
                    data.decode("utf-8")
                )

                if not isinstance(payload, dict):
                    continue

                landmarks = parse_floats(
                    payload.get("landmarks"),
                    LANDMARK_COUNT
                )

                if landmarks is not None:
                    self.landmarks_publisher.publish(
                        Float32MultiArray(data=landmarks)
                    )

                values = parse_floats(
                    payload.get("values"),
                    FG_COUNT,
                    0.0,
                    1.0
                )

                if values is not None:
                    self.publisher.publish(
                        Float32MultiArray(data=values)
                    )

            except (
                ValueError,
                TypeError,
                UnicodeDecodeError,
                AttributeError
            ):
                continue

    def destroy_node(self):
        self.udp_socket.close()
        return super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = HandTrackingNode()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    except RuntimeError:
        if rclpy.ok():
            raise  # a real error; otherwise Humble's take_message raced with Ctrl+C

    finally:
        if node is not None:
            node.destroy_node()

        # Ctrl+C under ros2 launch may already have shut the context down
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
