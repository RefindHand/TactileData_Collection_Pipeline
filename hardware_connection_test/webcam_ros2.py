#!/usr/bin/env python3
"""Read a V4L2 webcam, display it, and publish ROS2 sensor_msgs/Image.

Only use this when the WSL2/Linux system has a working /dev/videoN.
For Orbbec Gemini 305 depth data, use the dedicated Orbbec ROS2 driver later.
"""
import argparse
import sys


def main():
    parser = argparse.ArgumentParser(description="ROS2 웹캠 미리보기와 이미지 발행")
    parser.add_argument("--camera", default="0", help="카메라 번호 또는 /dev/video0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    try:
        import cv2
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
    except ImportError as exc:
        sys.exit(f"의존성 부족: {exc}. ROS2 Humble을 source하고 python3-opencv 설치를 확인하세요.")

    source = int(args.camera) if args.camera.isdigit() else args.camera
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        sys.exit("카메라 열기 실패: WSL /dev/video*가 존재하는지 확인하세요.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    rclpy.init()
    node = Node("webcam_test")
    pub = node.create_publisher(Image, "/webcam/image_raw", 10)
    node.get_logger().info("웹캠 발행: /webcam/image_raw (sensor_msgs/Image, bgr8)")
    print("웹캠 창을 클릭하고 q 키로 종료하세요.")
    try:
        while rclpy.ok():
            ok, frame = cap.read()
            if not ok:
                print("카메라 프레임을 읽지 못했습니다.")
                break
            if not frame.flags["C_CONTIGUOUS"]:
                frame = frame.copy()
            height, width = frame.shape[:2]
            msg = Image()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "webcam_frame"
            msg.height = height
            msg.width = width
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = width * 3
            msg.data = frame.tobytes()
            pub.publish(msg)
            cv2.imshow("ROS2 webcam (q: quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
