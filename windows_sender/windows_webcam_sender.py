#!/usr/bin/env python3
"""Run Py_webCamReader.py unchanged on Windows and stream its frames to WSL for the GUI.

Same idea as src/hand_tracking/hand_tracking/linux_webcam_sender.py. Two parts are patched:
  cv2.VideoCapture  camera index from --camera (the script always opens camera 0)
  cv2.imshow        the annotated frame goes out as one JPEG per UDP datagram instead of a window

  python windows_webcam_sender.py                # camera 0, no window (Ctrl+C to quit)
  python windows_webcam_sender.py --camera 1
  python windows_webcam_sender.py --window       # also keep the window and its F/R/S/C/Q keys

Landmarks still go to hand_tracking_node on UDP 5005 from the script itself. Frames go to
webcam_image_receiver on UDP 5006, which publishes the topic hand_gui shows.
"""
import argparse
import runpy
import socket
import time
from pathlib import Path

import cv2

from network_config import get_network_config

IMAGE_PORT = 5006
IMAGE_MAX_WIDTH = 640
MAX_DATAGRAM = 60000  # one UDP datagram carries at most 65507 bytes
JPEG_QUALITIES = (80, 60, 40)  # lowered only when a frame does not fit in a datagram
SCRIPT = Path(__file__).resolve().parent / 'Py_webCamReader.py'


def make_frame_sender(host, port, rate):
    """Return send(frame_bgr): JPEG over UDP, throttled to `rate` Hz."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    last = [0.0]

    def send(frame):
        now = time.monotonic()
        if now - last[0] < 1.0 / rate:
            return
        last[0] = now
        height, width = frame.shape[:2]
        if width > IMAGE_MAX_WIDTH:
            frame = cv2.resize(frame, (IMAGE_MAX_WIDTH, int(height * IMAGE_MAX_WIDTH / width)))
        for quality in JPEG_QUALITIES:
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if ok and len(jpeg) <= MAX_DATAGRAM:
                sock.sendto(jpeg.tobytes(), (host, port))
                return

    return send


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--camera', type=int, default=0, help='Windows camera index')
    parser.add_argument('--image-port', type=int, default=IMAGE_PORT, help='UDP port for frames')
    parser.add_argument('--image-rate', type=float, default=15.0, help='max frames per second')
    parser.add_argument('--window', action='store_true', help='also show the OpenCV window')
    args = parser.parse_args()

    host, _ = get_network_config()
    send = make_frame_sender(host, args.image_port, args.image_rate)
    print(f'카메라 {args.camera} -> WSL {host}:{args.image_port} (종료: Ctrl+C)')

    open_capture = cv2.VideoCapture

    def video_capture(_index=0, backend=cv2.CAP_DSHOW):
        return open_capture(args.camera, backend)

    show_window = cv2.imshow

    def imshow(name, frame):
        send(frame)
        if args.window:
            show_window(name, frame)

    cv2.VideoCapture = video_capture
    cv2.imshow = imshow
    try:
        runpy.run_path(str(SCRIPT), run_name='__main__')
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
