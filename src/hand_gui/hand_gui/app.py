#!/usr/bin/env python3
"""Tactile data collection GUI.

  ros2 run hand_gui hand_gui
  ros2 run hand_gui hand_gui --ros-args -p data_dir:=~/tactile_dataset -p test_object:=ball

Parameters: data_dir (episode folder root), test_object (none | ball: match hand_sim to see
the ball), hand_xml, robot_camera_topic, webcam_topic, webcam_source (none | network | /dev/videoN:
camera selected at start, see webcam_source.py).
"""
import os
import signal
import sys
import threading

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from PyQt5.QtCore import QLibraryInfo, QTimer
from PyQt5.QtWidgets import QApplication

from hand_gui import theme
from hand_gui.main_window import MainWindow
from hand_gui.ros_bridge import RosBridge
from hand_gui.webcam_source import NONE, is_wsl


def use_pyqt_plugins():
    """pip's opencv sets QT_QPA_PLATFORM_PLUGIN_PATH to its own bundled Qt plugins on import,
    which PyQt5 cannot load ("Could not load the Qt platform plugin xcb"). Point it back."""
    plugins = QLibraryInfo.location(QLibraryInfo.PluginsPath)
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = os.path.join(plugins, 'platforms')
    os.environ.pop('QT_PLUGIN_PATH', None)


def use_software_gl_on_wsl():
    """WSL2 (Mesa d3d12) returns broken offscreen MuJoCo frames; software GL works.

    Must run before the first mujoco.Renderer (GL context) is created.
    """
    if is_wsl():
        os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')


def spin(executor):
    try:
        executor.spin()
    except (ExternalShutdownException, RuntimeError):
        pass  # shutting down


def main():
    rclpy.init()
    bridge = RosBridge()
    data_dir = bridge.declare_parameter('data_dir', '~/tactile_dataset').value
    test_object = bridge.declare_parameter('test_object', 'none').value
    hand_xml = bridge.declare_parameter('hand_xml', '').value
    webcam_source = bridge.declare_parameter('webcam_source', NONE).value
    bridge.get_logger().info(f'data_dir={data_dir}, test_object={test_object}')
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    thread = threading.Thread(target=spin, args=(executor,), daemon=True)
    thread.start()

    use_software_gl_on_wsl()
    use_pyqt_plugins()  # after cv2 was imported (by main_window), before QApplication
    app = QApplication(sys.argv[:1])
    theme.apply_dark(app)
    window = MainWindow(bridge, data_dir, test_object=test_object, hand_xml=hand_xml,
                        webcam_source=webcam_source)
    window.show()

    # Ctrl+C in the terminal closes the window (Qt blocks Python signals without a timer tick)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    keepalive = QTimer()
    keepalive.timeout.connect(lambda: None)
    keepalive.start(200)

    code = app.exec_()
    executor.shutdown()
    bridge.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
