"""Webcam sources the GUI can switch between. Nothing here touches Qt.

  /dev/videoN  USB camera on this machine: the GUI runs linux_webcam_sender for it
  network      camera of a Windows host: windows_sender/windows_webcam_sender.py is started in
               PowerShell and its frames arrive through webcam_image_receiver
  none         the GUI starts nothing and shows whatever is on the webcam topic

Either way the frames arrive on the same topic and landmarks on the same UDP port, so the rest
of the pipeline does not know which source is active.
"""
import os
import platform
import signal
import subprocess
from pathlib import Path

NONE = 'none'
NETWORK = 'network'
VIDEO_DEVICES = Path('/sys/class/video4linux')
STOP_TIMEOUT = 3.0  # s for the sender to release the camera before it is killed


def is_wsl():
    return 'microsoft' in platform.uname().release.lower()


def usb_cameras():
    """[(device, name)] of this machine; index 0 skips a camera's extra metadata nodes."""
    cameras = []
    nodes = sorted(VIDEO_DEVICES.glob('video*'), key=lambda node: int(node.name[len('video'):]))
    for node in nodes:
        try:
            if (node / 'index').read_text().strip() == '0':
                cameras.append((f'/dev/{node.name}', (node / 'name').read_text().strip()))
        except OSError:
            continue  # unplugged while reading
    return cameras


def resolve_source(source, cameras):
    """The requested source, or the best one here when that camera does not exist."""
    if source in (NONE, NETWORK) or source in [device for device, _ in cameras]:
        return source
    if is_wsl():
        return NETWORK
    return cameras[0][0] if cameras else NONE


class LocalSender:
    """linux_webcam_sender as a child process, for one USB camera at a time."""

    def __init__(self):
        self.process = None

    def start(self, device):
        self.stop()
        index = device[len('/dev/video'):]
        self.process = subprocess.Popen(
            ['ros2', 'run', 'hand_tracking', 'linux_webcam_sender', '--device', index,
             '--publish-image', '--no-window'],
            start_new_session=True)  # own process group: stop() reaches the child of ros2 run

    def stop(self):
        if self.process is None:
            return
        process, self.process = self.process, None
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=STOP_TIMEOUT)
        except subprocess.TimeoutExpired:
            pass
        except ProcessLookupError:
            return
        try:  # whatever is left in the group would keep the camera open
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()

    def exit_code(self):
        """Exit code once if the sender ended by itself (camera busy, mediapipe missing, ...)."""
        if self.process is None:
            return None
        code = self.process.poll()
        if code is not None:
            self.process = None
        return code
