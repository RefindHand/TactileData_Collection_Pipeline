
import json
import math
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


UDP_HOST = "0.0.0.0"
UDP_PORT = 5005
TOPIC_NAME = "/hand/finger_angles"
FG_COUNT = 6


class HandTrackingNode(Node):

    def __init__(self):
        super().__init__("hand_tracking_node")

        # ROS 2 Publisher
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
            f"Publishing topic: {TOPIC_NAME}"
        )

    def receive_data(self):

        for _ in range(20):

            try:
                data, _ = self.udp_socket.recvfrom(4096)

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

                values = payload["values"]

                if not isinstance(values, list):
                    continue

                if len(values) != FG_COUNT:
                    continue

                values = [
                    float(v)
                    for v in values
                ]

                if not all(
                    math.isfinite(v) and 0.0 <= v <= 1.0
                    for v in values
                ):
                    continue

                msg = Float32MultiArray()
                msg.data = values

                self.publisher.publish(msg)

            except (
                ValueError,
                TypeError,
                KeyError,
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

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
