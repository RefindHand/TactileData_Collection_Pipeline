#!/usr/bin/env python3
"""Run person 1's windows_sender/Py_webCamReader.py unchanged on Ubuntu (desktop, Jetson AGX Orin).

The Windows script differs from Linux in two places only, both patched here before it runs:
  cv2.CAP_DSHOW  (DirectShow, Windows only)  -> cv2.CAP_V4L2, camera index from --device
  WSL IP lookup  (runs wsl.exe)               -> WSL_IP env, default 127.0.0.1 (same machine)

  ros2 run hand_tracking linux_webcam_sender --device 0
  ros2 run hand_tracking linux_webcam_sender --script path/to/Py_webCamReader.py
  ros2 run hand_tracking linux_webcam_sender --publish-image --no-window   # for the GUI / headless

Keys in the window are the script's own: F fix calibration, R record, S save CSV, C reset, Q quit.

--publish-image also publishes the annotated window frame (landmarks drawn) as JPEG on
/webcam/image_annotated/compressed (sensor_msgs/CompressedImage, at most --image-rate Hz).
--no-window skips the OpenCV window (no display needed; keys are then unavailable).
"""
import argparse
import os
import runpy
import sys
from pathlib import Path

SCRIPT_RELATIVE_PATH = Path('windows_sender') / 'Py_webCamReader.py'
IMAGE_TOPIC = '/webcam/image_annotated/compressed'
IMAGE_MAX_WIDTH = 640


def find_script():
    # cwd and its parents first, then the source tree (colcon --symlink-install)
    source_root = Path(__file__).resolve().parents[3]
    for base in (Path.cwd(), *Path.cwd().parents, source_root):
        candidate = base / SCRIPT_RELATIVE_PATH
        if candidate.is_file():
            return candidate
    return SCRIPT_RELATIVE_PATH


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--script', type=Path, default=None, help='Py_webCamReader.py path')
    parser.add_argument('--device', type=int, default=0, help='webcam index (/dev/videoN)')
    parser.add_argument('--host', default='127.0.0.1', help='machine running hand_tracking_node')
    parser.add_argument('--publish-image', action='store_true', help=f'publish frames on {IMAGE_TOPIC}')
    parser.add_argument('--image-rate', type=float, default=15.0, help='max image publish rate (Hz)')
    parser.add_argument('--no-window', action='store_true', help='do not open the OpenCV window')
    args, _ = parser.parse_known_args()  # ignore --ros-args when started from ros2 launch

    script = (args.script or find_script()).resolve()
    if not script.is_file():
        sys.exit(f'{SCRIPT_RELATIVE_PATH} not found (run from the project root or pass --script)')

    os.environ['WSL_IP'] = os.environ.get('WSL_IP', args.host)

    import cv2
    open_capture = cv2.VideoCapture

    def video_capture(_index=0, _backend=None):
        return open_capture(args.device, cv2.CAP_V4L2)

    cv2.VideoCapture = video_capture

    publish = make_image_publisher(args.image_rate) if args.publish_image else None
    show_window = cv2.imshow

    def imshow(name, frame):
        if publish is not None:
            publish(frame)
        if not args.no_window:
            show_window(name, frame)

    cv2.imshow = imshow
    sys.path.insert(0, str(script.parent))  # for 'from network_config import ...'
    try:
        runpy.run_path(str(script), run_name='__main__')
    except KeyboardInterrupt:  # Ctrl+C from the terminal or ros2 launch
        pass
    finally:
        if publish is not None:
            import rclpy
            rclpy.try_shutdown()


def make_image_publisher(rate):
    """Return publish(frame_bgr): JPEG on IMAGE_TOPIC, throttled to `rate` Hz."""
    import time

    import cv2
    import rclpy
    from sensor_msgs.msg import CompressedImage

    rclpy.init()
    node = rclpy.create_node('linux_webcam_sender')
    publisher = node.create_publisher(CompressedImage, IMAGE_TOPIC, 2)
    last = [0.0]

    def publish(frame):
        now = time.monotonic()
        if now - last[0] < 1.0 / rate or not rclpy.ok():
            return
        last[0] = now
        height, width = frame.shape[:2]
        if width > IMAGE_MAX_WIDTH:
            frame = cv2.resize(frame, (IMAGE_MAX_WIDTH, int(height * IMAGE_MAX_WIDTH / width)))
        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            msg = CompressedImage(format='jpeg', data=jpeg.tobytes())
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'webcam'
            publisher.publish(msg)

    return publish


if __name__ == '__main__':
    main()
