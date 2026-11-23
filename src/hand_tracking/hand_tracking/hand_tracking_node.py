
import json
import math
import socket
import argparse
import threading
from decimal import Decimal, ROUND_DOWN

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
DEFAULT_DECIMALS = 3
MAX_DECIMALS = 6


def truncate_values(values, decimals):
    """Truncate towards zero, never round up."""
    unit = Decimal(1).scaleb(-decimals)
    return [float(Decimal(str(value)).quantize(unit, rounding=ROUND_DOWN)) for value in values]


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

    def __init__(self, decimals=DEFAULT_DECIMALS):
        super().__init__("hand_tracking_node")
        self.decimals = decimals

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
                        Float32MultiArray(data=truncate_values(values, self.decimals))
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


def run_filter_window(node, stop_event):
    """Tk GUI on the main thread; ROS spins on a worker thread."""
    import tkinter as tk
    from tkinter import ttk

    root = tk.Tk()
    root.title("Hand Tracking - Decimal Filter")
    root.resizable(False, False)

    frame = ttk.Frame(root, padding=18)
    frame.grid(row=0, column=0)
    ttk.Label(frame, text="소수점 이하 유지할 자릿수 (이후 버림)").grid(
        row=0, column=0, columnspan=2, sticky="w", pady=(0, 12)
    )

    current = tk.IntVar(value=node.decimals)
    entry_text = tk.StringVar(value=str(node.decimals))
    message = tk.StringVar(value=f"현재 설정: 소수점 {node.decimals}자리까지 유지")

    def apply_decimals(value):
        value = int(value)
        if not 0 <= value <= MAX_DECIMALS:
            message.set(f"0~{MAX_DECIMALS} 사이의 정수를 입력하세요.")
            return
        node.decimals = value
        current.set(value)
        entry_text.set(str(value))
        message.set(f"현재 설정: 소수점 {value}자리까지 유지 (이후 버림)")

    slider = tk.Scale(
        frame, from_=0, to=MAX_DECIMALS, orient="horizontal", length=300,
        variable=current, showvalue=True,
        command=lambda value: apply_decimals(int(float(value))),
    )
    slider.grid(row=1, column=0, sticky="ew", padx=(0, 12))

    entry = ttk.Entry(frame, width=6, textvariable=entry_text, justify="center")
    entry.grid(row=1, column=1)

    def apply_entry(_event=None):
        raw = entry_text.get().strip()
        try:
            value = int(raw)
        except ValueError:
            message.set(f"0~{MAX_DECIMALS} 사이의 정수를 입력하세요.")
            return
        apply_decimals(value)

    entry.bind("<Return>", apply_entry)
    entry.bind("<FocusOut>", apply_entry)
    ttk.Label(frame, textvariable=message).grid(
        row=2, column=0, columnspan=2, sticky="w", pady=(12, 0)
    )

    def close():
        stop_event.set()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close)

    def check_shutdown():
        if stop_event.is_set() or not rclpy.ok():
            root.destroy()
        else:
            root.after(100, check_shutdown)

    root.after(100, check_shutdown)
    root.mainloop()


def main(args=None):
    parser = argparse.ArgumentParser(description="UDP hand tracking ROS2 receiver")
    parser.add_argument(
        "--decimals", type=int, default=DEFAULT_DECIMALS,
        help=f"Digits to keep after decimal point (0-{MAX_DECIMALS})",
    )
    parser.add_argument(
        "--no-gui", action="store_true",
        help="Run without a window (useful for headless/SSH/ros2 launch)",
    )
    options, ros_args = parser.parse_known_args(args)
    if not 0 <= options.decimals <= MAX_DECIMALS:
        parser.error(f"--decimals must be between 0 and {MAX_DECIMALS}")

    rclpy.init(args=ros_args)
    node = None
    stop_event = threading.Event()
    worker = None
    try:
        node = HandTrackingNode(decimals=options.decimals)
        node.get_logger().info(
            f"Finger angle truncation: keep {node.decimals} decimal digits"
        )
        if options.no_gui:
            rclpy.spin(node)
        else:
            def spin_ros():
                try:
                    rclpy.spin(node)
                except (ExternalShutdownException, RuntimeError):
                    pass
                finally:
                    stop_event.set()

            worker = threading.Thread(target=spin_ros, daemon=True)
            worker.start()
            try:
                run_filter_window(node, stop_event)
            except Exception as error:
                node.get_logger().warning(
                    f"GUI unavailable ({error}). Continuing without GUI. "
                    "Install python3-tk and configure DISPLAY to enable it."
                )
                while worker.is_alive():
                    worker.join(timeout=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        stop_event.set()
        if rclpy.ok():
            rclpy.try_shutdown()
        if worker is not None:
            worker.join(timeout=2)
        if node is not None:
            node.destroy_node()


if __name__ == "__main__":
    main()
