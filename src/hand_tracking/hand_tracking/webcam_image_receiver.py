#!/usr/bin/env python3
"""Publish webcam frames streamed over UDP so hand_gui can show a camera on another OS.

windows_sender/windows_webcam_sender.py sends one JPEG per datagram to UDP 5006; each one
becomes a sensor_msgs/CompressedImage on /webcam/image_annotated/compressed, the same topic
linux_webcam_sender --publish-image uses. Idle until frames arrive.
"""
import socket

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

UDP_HOST = '0.0.0.0'
UDP_PORT = 5006
IMAGE_TOPIC = '/webcam/image_annotated/compressed'
JPEG_START = b'\xff\xd8'


class WebcamImageReceiver(Node):
    def __init__(self):
        super().__init__('webcam_image_receiver')
        self.publisher = self.create_publisher(CompressedImage, IMAGE_TOPIC, 2)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((UDP_HOST, UDP_PORT))
        self.udp_socket.setblocking(False)
        self.create_timer(0.01, self.receive_frames)
        self.get_logger().info(f'UDP receiver started: {UDP_HOST}:{UDP_PORT} -> {IMAGE_TOPIC}')

    def receive_frames(self):
        latest = None
        while True:  # keep only the newest frame if several arrived
            try:
                data, _ = self.udp_socket.recvfrom(65536)
            except BlockingIOError:
                break
            except OSError as error:
                self.get_logger().error(f'UDP receive error: {error}')
                break
            if data.startswith(JPEG_START):
                latest = data
        if latest is None:
            return
        msg = CompressedImage(format='jpeg', data=latest)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'webcam'
        self.publisher.publish(msg)

    def destroy_node(self):
        self.udp_socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WebcamImageReceiver()
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


if __name__ == '__main__':
    main()
