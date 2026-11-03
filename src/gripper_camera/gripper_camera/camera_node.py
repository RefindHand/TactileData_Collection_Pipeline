#!/usr/bin/env python3
"""ROS2 node for the virtual Gemini 305 (MuJoCo passive stereo RGB-D).

Only the selected stream (parameter `mode`, or 1-5 typed in the terminal) is rendered
and published, and only while it has a subscriber:
  rgb        ~/color/image_raw      rgb8                 ~/color/camera_info
  left       ~/left/image_raw       rgb8                 ~/left/camera_info
  right      ~/right/image_raw      rgb8                 ~/right/camera_info
  disparity  ~/disparity/image_raw  32FC1, px            ~/disparity/camera_info (left camera)
             ~/disparity/image_color bgr8 (colorized, for viewing)
  depth      ~/depth/image_raw      32FC1, m (REP 118)   ~/depth/camera_info (left camera)
             ~/depth/image_color    bgr8 (colorized, for viewing)
Invalid disparity/depth pixels are NaN. Disparity and depth are aligned to the left camera.
Scene, rendering and stereo matching are reused from gemini305_sim.

Hand pose (parameter follow, default true) follows the running pipeline:
  /hand_sim/full_joint_states  all MuJoCo joints (hand_sim)  -> copied, same pose as the simulation
  /hand_hw/state               6 actuators, rad (hand_sim or hand_serial, i.e. also the real hand)
                               -> used as actuator targets, the camera's own physics follows
Until either topic arrives (or with follow:=false) the fingers open and close by themselves
(static:=true stops them). Once followed, a stalled pipeline holds the last pose.
"""
import array
import queue
import signal
import sys
import threading
import time
from pathlib import Path

import mujoco
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image, JointState

from finger_node.common import HW_STATE_TOPIC, SIM_FULL_STATE_TOPIC
from finger_node.sim_backend import ACTUATORS
from gripper_camera import gemini305_sim as sim

FOLLOW_TIMEOUT = 0.5  # s; full joint states older than this fall back to /hand_hw/state

# mode -> (menu number, topic prefix, MuJoCo camera the stream is aligned to)
MODES = {
    'rgb': (1, 'color', 'rgb'),
    'left': (2, 'left', 'stereo_left'),
    'right': (3, 'right', 'stereo_right'),
    'disparity': (4, 'disparity', 'stereo_left'),
    'depth': (5, 'depth', 'stereo_left'),
}
MODE_BY_NUMBER = {number: mode for mode, (number, _, _) in MODES.items()}
FRAME_IDS = {
    'rgb': 'gemini305_color_optical_frame',
    'stereo_left': 'gemini305_left_optical_frame',
    'stereo_right': 'gemini305_right_optical_frame',
}


def image_msg(image, encoding, header):
    msg = Image(header=header, height=image.shape[0], width=image.shape[1], encoding=encoding,
                is_bigendian=int(sys.byteorder == 'big'), step=image.strides[0])
    msg.data = array.array('B', np.ascontiguousarray(image).tobytes())  # array: fast path in rclpy
    return msg


def invalid_to_nan(values):
    """float32 image with NaN where the stereo match failed (value <= 0)."""
    return np.where(values > 0, values, np.nan).astype(np.float32)


def camera_info_msg(width, height, fx, frame_id, tx=0.0):
    """Ideal rectified pinhole; MuJoCo puts the principal point at the image center."""
    cx, cy = (width - 1) / 2, (height - 1) / 2
    msg = CameraInfo(width=width, height=height, distortion_model='plumb_bob')
    msg.header.frame_id = frame_id
    msg.d = [0.0] * 5
    msg.k = [fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, tx, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


class CameraNode(Node):
    def __init__(self):
        super().__init__('gripper_camera')
        read_only = ParameterDescriptor(read_only=True)
        self.mode = self.declare_parameter('mode', 'rgb').value
        resolution = self.declare_parameter('resolution', '848x530', read_only).value
        rate = self.declare_parameter('rate', 30.0, read_only).value
        hand_xml = self.declare_parameter('hand_xml', '', read_only).value or sim.find_hand_xml()
        self.animate = not self.declare_parameter('static', False, read_only).value
        follow = self.declare_parameter('follow', True, read_only).value
        if self.mode not in MODES:
            raise ValueError(f'mode must be one of {list(MODES)}, got {self.mode!r}')
        if resolution not in sim.RESOLUTIONS:
            raise ValueError(f'resolution must be one of {list(sim.RESOLUTIONS)}, got {resolution!r}')
        if not Path(hand_xml).is_file():
            raise FileNotFoundError(f'AP002 model not found: {hand_xml} (set hand_xml or ROHAND_MODEL_XML)')

        width, height = sim.RESOLUTIONS[resolution]
        sim.use_software_gl_on_wsl()
        self.model = sim.build_model(hand_xml, width, height)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.camera = sim.VirtualGemini305(self.model, width, height)

        fx = {name: self.camera.focal(name) for name in FRAME_IDS}
        self.infos = {
            'rgb': camera_info_msg(width, height, fx['rgb'], FRAME_IDS['rgb']),
            'stereo_left': camera_info_msg(width, height, fx['stereo_left'], FRAME_IDS['stereo_left']),
            # ROS stereo convention: right P[0, 3] = -fx * baseline
            'stereo_right': camera_info_msg(width, height, fx['stereo_right'], FRAME_IDS['stereo_right'],
                                            tx=-fx['stereo_right'] * self.camera.baseline),
        }
        self.pubs = {}
        for mode, (_, prefix, _) in MODES.items():
            pubs = {'image': self.create_publisher(Image, f'~/{prefix}/image_raw', 5),
                    'info': self.create_publisher(CameraInfo, f'~/{prefix}/camera_info', 5)}
            if mode in ('disparity', 'depth'):
                pubs['color'] = self.create_publisher(Image, f'~/{prefix}/image_color', 5)
            self.pubs[mode] = pubs

        # joint name -> qpos address, finger -> actuator id (for following the pipeline hand)
        self.qpos_addr = {self.model.joint(j).name: int(self.model.jnt_qposadr[j])
                          for j in range(self.model.njnt) if self.model.joint(j).name}
        self.actuator_id = {finger: self.model.actuator(name).id for finger, name in ACTUATORS.items()}
        self.full_state = (None, -1.0)  # (qpos by name, monotonic time)
        self.hw_state = (None, -1.0)    # (ctrl by finger, monotonic time)
        if follow:
            self.create_subscription(JointState, SIM_FULL_STATE_TOPIC, self.on_full_state, 10)
            self.create_subscription(JointState, HW_STATE_TOPIC, self.on_hw_state, 10)

        self.add_on_set_parameters_callback(self.on_set_parameters)
        self.start = time.monotonic()
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'Gemini 305 virtual camera {width}x{height}, fx={self.camera.fx:.1f}px, '
            f'baseline={self.camera.baseline * 1000:.1f}mm, mode={self.mode}, '
            f'topics: {self.get_fully_qualified_name()}/<color|left|right|disparity|depth>/...')

        # terminal selection (skipped when stdin is not a terminal, e.g. under ros2 launch)
        self.commands = queue.Queue()
        self.done = False
        if sys.stdin.isatty():
            sim.print_menu()
            threading.Thread(target=sim.read_stdin, args=(self.commands,), daemon=True).start()

    def on_full_state(self, msg):
        self.full_state = (dict(zip(msg.name, msg.position)), time.monotonic())

    def on_hw_state(self, msg):
        self.hw_state = (dict(zip(msg.name, msg.position)), time.monotonic())

    def update_hand(self):
        """Pose the hand from the pipeline if its state is fresh, else animate."""
        now = time.monotonic()
        positions, stamp = self.full_state
        if positions is not None and now - stamp < FOLLOW_TIMEOUT:
            for name, value in positions.items():
                if name in self.qpos_addr:
                    self.data.qpos[self.qpos_addr[name]] = value
            mujoco.mj_forward(self.model, self.data)
            return
        targets, _ = self.hw_state
        if targets is not None:  # latest actuator targets; held if the pipeline stalls
            for finger, value in targets.items():
                if finger in self.actuator_id:
                    a = self.actuator_id[finger]
                    lo, hi = self.model.actuator_ctrlrange[a]
                    self.data.ctrl[a] = min(max(value, lo), hi)
            sim.step_physics(self.model, self.data, now - self.start, animate=False)
            return
        if positions is not None:
            return  # followed the simulation before: keep its last pose
        sim.step_physics(self.model, self.data, now - self.start, self.animate)

    def on_set_parameters(self, params):
        for param in params:
            if param.name == 'mode':
                if param.value not in MODES:
                    return SetParametersResult(successful=False, reason=f'mode must be one of {list(MODES)}')
                self.mode = param.value
                self.get_logger().info(f'mode -> {self.mode}')
        return SetParametersResult(successful=True)

    def handle_terminal(self):
        while not self.commands.empty():
            text = self.commands.get()
            if text == '0':
                self.done = True
                return
            if text.isdigit() and int(text) in MODE_BY_NUMBER:
                self.set_parameters([Parameter('mode', Parameter.Type.STRING, MODE_BY_NUMBER[int(text)])])
            else:
                print('0~5 중에서 입력하세요.')
            sim.prompt()

    def tick(self):
        self.handle_terminal()
        self.update_hand()

        mode = self.mode
        pubs = self.pubs[mode]
        if not any(pub.get_subscription_count() for pub in pubs.values()):
            return  # nobody is listening: skip rendering and stereo matching
        _, _, camera = MODES[mode]
        info = self.infos[camera]
        info.header.stamp = self.get_clock().now().to_msg()
        header = info.header

        if mode in ('rgb', 'left', 'right'):
            pubs['image'].publish(image_msg(self.camera.render(self.data, camera), 'rgb8', header))
        elif mode == 'disparity':
            disp = self.camera.disparity(self.data)
            pubs['image'].publish(image_msg(invalid_to_nan(disp), '32FC1', header))
            if pubs['color'].get_subscription_count():
                pubs['color'].publish(image_msg(sim.disparity_image(disp), 'bgr8', header))
        else:
            depth = self.camera.depth(self.data)
            pubs['image'].publish(image_msg(invalid_to_nan(depth), '32FC1', header))
            if pubs['color'].get_subscription_count():
                pubs['color'].publish(image_msg(sim.depth_image(depth), 'bgr8', header))
        pubs['info'].publish(info)

    def destroy_node(self):
        self.camera.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise  # a real error; otherwise Humble's take_message raced with Ctrl+C
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # ros2 launch forwards a second Ctrl+C
        node.destroy_node()
        # Ctrl+C under ros2 launch may already have shut the context down
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
