#!/usr/bin/env python3
"""Run person 1's windows_sender/Py_webCamReader.py unchanged on Ubuntu (desktop, Jetson AGX Orin).

The Windows script differs from Linux in two places only, both patched here before it runs:
  cv2.CAP_DSHOW  (DirectShow, Windows only)  -> cv2.CAP_V4L2, camera index from --device
  WSL IP lookup  (runs wsl.exe)               -> WSL_IP env, default 127.0.0.1 (same machine)

  ros2 run hand_tracking linux_webcam_sender --device 0
  ros2 run hand_tracking linux_webcam_sender --script path/to/Py_webCamReader.py

Keys in the window are the script's own: F fix calibration, R record, S save CSV, C reset, Q quit.
"""
import argparse
import os
import runpy
import sys
from pathlib import Path

SCRIPT_RELATIVE_PATH = Path('windows_sender') / 'Py_webCamReader.py'


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
    sys.path.insert(0, str(script.parent))  # for 'from network_config import ...'
    try:
        runpy.run_path(str(script), run_name='__main__')
    except KeyboardInterrupt:  # Ctrl+C from the terminal or ros2 launch
        pass


if __name__ == '__main__':
    main()
