#!/usr/bin/env python3
"""Virtual Orbbec Gemini 305 (passive stereo RGB-D) looking at the ROHand AP002 in MuJoCo.

Standalone test program (no ROS). Pick a stream number in the terminal; it is shown
live in its own OpenCV window. Only the selected stream is rendered/computed:
  1 RGB           render "rgb"
  2 Stereo Left   render "stereo_left"
  3 Stereo Right  render "stereo_right"
  4 Disparity     render left + right, StereoSGBM
  5 Depth         same as 4, then depth = fx * baseline / disparity
Camera geometry and intrinsics live in model/gemini305.xml.
The ROS2 node (camera_node.py) reuses the scene, rendering and stereo matching from here.
"""
import argparse
import math
import os
import platform
import queue
import threading
import time
from pathlib import Path

import cv2
import mujoco
import mujoco.viewer
import numpy as np

HERE = Path(__file__).resolve().parent
CAMERA_XML = HERE / 'model' / 'gemini305.xml'
HAND_XML_RELATIVE = Path('external/rohand_mujoco/AP002/model/rohand_gen2_AP002_right.xml')

CAMERA_BODY = 'gemini305'
CAMERA_GEOM_GROUP = 3  # housing mesh group, hidden from the virtual cameras
# Gemini 305 16:10 stream sizes (848x530 native, 640x400 = 1280x800 with decimation 2)
RESOLUTIONS = {'1280x800': (1280, 800), '848x530': (848, 530), '640x400': (640, 400)}
NUM_DISPARITIES = 128  # disparity search range (px); min depth = fx * B / 128
DEPTH_RANGE = (0.07, 0.50)  # datasheet ideal range (m), used for the colormap
MOTION_PERIOD = 4.0  # s, fingers open -> close -> open

STREAMS = {1: 'RGB', 2: 'Stereo Left', 3: 'Stereo Right', 4: 'Disparity Map', 5: 'Depth Map'}


def find_hand_xml():
    """Official AP002 right-hand MJCF: $ROHAND_MODEL_XML, else ./external/... from cwd or this file upward."""
    if os.environ.get('ROHAND_MODEL_XML'):
        return os.environ['ROHAND_MODEL_XML']
    for base in (Path.cwd(), *Path.cwd().parents, *HERE.parents):
        if (base / HAND_XML_RELATIVE).is_file():
            return str(base / HAND_XML_RELATIVE)
    return str(HAND_XML_RELATIVE)


def build_model(hand_xml, width, height, pos=None, euler=None):
    """AP002 right hand + textured backdrop + Gemini 305 body (mesh and cameras)."""
    spec = mujoco.MjSpec.from_file(str(hand_xml))
    spec.visual.quality.offsamples = 0  # no MSAA: a sensor image is not anti-aliased, and it is faster
    spec.visual.global_.offwidth = max(width, spec.visual.global_.offwidth)
    spec.visual.global_.offheight = max(height, spec.visual.global_.offheight)

    # Random-dot backdrop behind the hand: passive stereo needs texture to match.
    tex = spec.add_texture(name='backdrop', type=mujoco.mjtTexture.mjTEXTURE_2D,
                           builtin=mujoco.mjtBuiltin.mjBUILTIN_FLAT, mark=mujoco.mjtMark.mjMARK_RANDOM,
                           random=0.4, rgb1=[0.35, 0.38, 0.42], markrgb=[0.9, 0.9, 0.85],
                           width=512, height=512)
    mat = spec.add_material(name='backdrop', texrepeat=[4, 4])
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = tex.name
    spec.worldbody.add_geom(name='backdrop', type=mujoco.mjtGeom.mjGEOM_PLANE, material='backdrop',
                            pos=[-0.10, -0.02, 0.15], quat=[0, 1, 0, 0], size=[0.6, 0.6, 0.01],
                            contype=0, conaffinity=0)
    spec.worldbody.add_light(name='key', pos=[-0.10, 0.10, -0.40], dir=[0, -0.3, 1], diffuse=[0.7, 0.7, 0.7])

    camera = mujoco.MjSpec.from_file(str(CAMERA_XML))
    body = camera.body(CAMERA_BODY)
    body.mocap = True  # can be dragged in the MuJoCo viewer (--viewer); cameras follow the body
    if pos is not None:
        body.pos = pos
    if euler is not None:
        quat = np.zeros(4)
        mujoco.mju_euler2Quat(quat, np.radians(euler), 'xyz')
        body.quat = quat
    spec.worldbody.add_frame().attach_body(body, '', '')
    return spec.compile()


class VirtualGemini305:
    """Renders the Gemini 305 streams; disparity and depth come from stereo matching."""

    def __init__(self, model, width, height):
        self.model = model
        self.width, self.height = width, height
        self.renderer = mujoco.Renderer(model, height, width)
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False  # halves render time
        self.option = mujoco.MjvOption()
        self.option.geomgroup[CAMERA_GEOM_GROUP] = 0
        left, right = model.camera('stereo_left'), model.camera('stereo_right')
        # both cameras hang off the same body, so the local offset is the baseline
        self.baseline = float(np.linalg.norm(right.pos - left.pos))
        self.fx = self.focal('stereo_left')
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=NUM_DISPARITIES, blockSize=5,
            P1=8 * 5 * 5, P2=32 * 5 * 5, disp12MaxDiff=1, uniquenessRatio=10,
            speckleWindowSize=100, speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    def focal(self, camera):
        """Focal length in px from the MuJoCo fovy (square pixels: fx = fy)."""
        return 0.5 * self.height / math.tan(math.radians(self.model.camera(camera).fovy[0]) / 2)

    def render(self, data, camera):
        """RGB uint8 image of one MuJoCo camera."""
        self.renderer.update_scene(data, camera=camera, scene_option=self.option)
        return self.renderer.render()

    def disparity(self, data):
        """Left-referenced disparity in px (float32, <= 0 where invalid)."""
        left = cv2.cvtColor(self.render(data, 'stereo_left'), cv2.COLOR_RGB2GRAY)
        right = cv2.cvtColor(self.render(data, 'stereo_right'), cv2.COLOR_RGB2GRAY)
        return self.sgbm.compute(left, right).astype(np.float32) / 16.0  # SGBM output is fixed-point x16

    def depth(self, data):
        """Depth (Z, m) in the left camera frame from disparity; 0 where invalid."""
        disp = self.disparity(data)
        valid = disp > 0
        depth = np.zeros_like(disp)
        depth[valid] = self.fx * self.baseline / disp[valid]
        return depth

    def frame(self, data, stream):
        """BGR image of the selected stream for display."""
        if stream == 1:
            return cv2.cvtColor(self.render(data, 'rgb'), cv2.COLOR_RGB2BGR)
        if stream in (2, 3):
            camera = 'stereo_left' if stream == 2 else 'stereo_right'
            return cv2.cvtColor(self.render(data, camera), cv2.COLOR_RGB2BGR)
        if stream == 4:
            return disparity_image(self.disparity(data))
        return depth_image(self.depth(data))

    def close(self):
        self.renderer.close()


def disparity_image(disp):
    """Colorized disparity (BGR) with the center value."""
    image = colorize(disp / NUM_DISPARITIES, disp > 0)
    return label(image, f'center {disp[disp.shape[0] // 2, disp.shape[1] // 2]:.1f} px')


def depth_image(depth):
    """Colorized depth (BGR, near = red, far = blue) with the center value."""
    near, far = DEPTH_RANGE
    valid = (depth >= near) & (depth <= far)
    image = colorize((far - depth) / (far - near), valid)
    z = depth[depth.shape[0] // 2, depth.shape[1] // 2]
    return label(image, f'center {z * 1000:.0f} mm' if z > 0 else 'center invalid')


def colorize(normalized, valid):
    image = cv2.applyColorMap((np.clip(normalized, 0, 1) * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    image[~valid] = 0
    return image


def label(image, text):
    h, w = image.shape[:2]
    cv2.drawMarker(image, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
    cv2.putText(image, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def animate_fingers(model, data, t):
    """Open/close all fingers slowly so the streams show motion."""
    phase = 0.5 * (1 - math.cos(2 * math.pi * t / MOTION_PERIOD))
    lo, hi = model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1]
    data.ctrl[:] = lo + (hi - lo) * phase


def step_physics(model, data, now, animate=True):
    """Advance physics to wall-clock time `now` (s), capped so slow frames do not pile up."""
    if animate:
        animate_fingers(model, data, now)
    target = min(now, data.time + 0.1)
    while data.time < target:
        mujoco.mj_step(model, data)


def print_menu():
    print('\n출력할 영상 번호를 입력하세요 (실행 중에도 바꿀 수 있음)')
    for number, name in STREAMS.items():
        print(f'  {number}. {name}')
    print('  0. 종료')
    prompt()


def prompt():
    print('선택 > ', end='', flush=True)


def read_stdin(commands):
    while True:
        try:
            commands.put(input().strip())
        except (EOFError, KeyboardInterrupt):
            commands.put('0')
            return


def use_software_gl_on_wsl():
    """WSL2 (Mesa d3d12) returns black offscreen MuJoCo frames; software GL works.

    Must run before the first mujoco.Renderer (GL context) is created.
    """
    if 'microsoft' in platform.uname().release.lower():
        os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')


def main():
    use_software_gl_on_wsl()
    parser = argparse.ArgumentParser(description='Gemini 305 virtual RGB-D camera (MuJoCo, no ROS)')
    parser.add_argument('--hand-xml', default=find_hand_xml(),
                        help='AP002 MJCF (default: $ROHAND_MODEL_XML or external/rohand_mujoco/...)')
    parser.add_argument('--resolution', choices=RESOLUTIONS, default='848x530')
    parser.add_argument('--pos', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                        help='camera body position in world (m); default in model/gemini305.xml')
    parser.add_argument('--euler', type=float, nargs=3, metavar=('RX', 'RY', 'RZ'),
                        help='camera body orientation, intrinsic xyz Euler (deg)')
    parser.add_argument('--static', action='store_true', help='do not move the fingers')
    parser.add_argument('--viewer', action='store_true',
                        help='also open the MuJoCo 3D viewer (camera model visible, Ctrl+drag moves it)')
    args = parser.parse_args()

    if not Path(args.hand_xml).is_file():
        raise SystemExit(f'AP002 모델을 찾을 수 없음: {args.hand_xml}\n'
                         '프로젝트 루트의 external/rohand_mujoco 또는 --hand-xml / ROHAND_MODEL_XML 지정')
    width, height = RESOLUTIONS[args.resolution]
    model = build_model(args.hand_xml, width, height, args.pos, args.euler)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = VirtualGemini305(model, width, height)
    min_depth = camera.fx * camera.baseline / NUM_DISPARITIES
    print(f'Gemini 305 가상 카메라: {width}x{height}, fx={camera.fx:.1f}px, '
          f'baseline={camera.baseline * 1000:.1f}mm, 최소 거리={min_depth * 1000:.0f}mm')

    viewer = None
    if args.viewer:
        viewer = mujoco.viewer.launch_passive(model, data)
        with viewer.lock():
            viewer.opt.geomgroup[CAMERA_GEOM_GROUP] = 1
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CAMERA] = True

    print_menu()
    commands = queue.Queue()
    threading.Thread(target=read_stdin, args=(commands,), daemon=True).start()

    stream, window = None, None
    start = time.monotonic()
    frames, fps, fps_time = 0, 0.0, start
    try:
        while True:
            while not commands.empty():
                text = commands.get()
                if text == '0':
                    return
                if text.isdigit() and int(text) in STREAMS:
                    if window:
                        cv2.destroyWindow(window)
                    stream, window = int(text), f'Gemini 305 - {STREAMS[int(text)]}'
                    frames, fps, fps_time = 0, 0.0, time.monotonic()
                    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
                    print(f'[{STREAMS[stream]}] 출력 중 (창에서 q/ESC: 창 닫기)')
                else:
                    print('0~5 중에서 입력하세요.')
                prompt()

            step_physics(model, data, time.monotonic() - start, not args.static)
            if viewer is not None and viewer.is_running():
                viewer.sync()

            if stream is None:
                time.sleep(0.02)
                continue
            image = camera.frame(data, stream)
            frames += 1
            if time.monotonic() - fps_time >= 1.0:
                fps = frames / (time.monotonic() - fps_time)
                frames, fps_time = 0, time.monotonic()
            cv2.putText(image, f'{fps:.1f} fps', (width - 100, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(window, image)
            key = cv2.waitKey(1) & 0xFF
            closed = cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1
            if key in (ord('q'), 27) or closed:
                if not closed:
                    cv2.destroyWindow(window)
                stream, window = None, None
                print('\n창을 닫았습니다. 다른 번호를 입력하거나 0으로 종료하세요.')
                prompt()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        camera.close()
        if viewer is not None:
            viewer.close()
        print('종료')


if __name__ == '__main__':
    main()
