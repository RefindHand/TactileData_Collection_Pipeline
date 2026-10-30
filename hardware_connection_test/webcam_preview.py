#!/usr/bin/env python3
"""Basic USB/built-in webcam live preview; does not require ROS2.

Works on Windows directly and on Linux/WSL2 only when a video device is exposed.
"""
import argparse
import os
import sys


def main():
    parser = argparse.ArgumentParser(description="웹캠 영상 보기 (q 키로 종료)")
    parser.add_argument("--camera", default="0", help="카메라 번호 또는 /dev/video0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()
    try:
        import cv2
    except ImportError:
        sys.exit("OpenCV 미설치: Ubuntu는 sudo apt install python3-opencv / Windows는 py -m pip install opencv-python")
    source = int(args.camera) if args.camera.isdigit() else args.camera
    backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_V4L2
    cap = cv2.VideoCapture(source, backend)
    if not cap.isOpened():
        cap.release()
        sys.exit("카메라 열기 실패. WSL에서는 /dev/video* 확인, Windows에서는 다른 --camera 번호를 시험하세요.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    print("웹캠 미리보기 실행 중. 영상 창에서 q를 누르면 종료합니다.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("영상 프레임 수신 실패")
                break
            cv2.imshow("Webcam preview (q: quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
