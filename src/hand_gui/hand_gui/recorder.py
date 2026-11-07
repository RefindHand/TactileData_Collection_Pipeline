"""Episode recorder: samples the latest pipeline values at a fixed rate and writes one folder.

<root>/episode_000001/
  data.csv          one row per frame: time, command / angle / velocity / force per finger,
                    tactile normal / tangential / direction per region, human hand landmarks
  tactile_raw.npz   raw tactile arrays per region, shape [frames, values] (NaN = no data)
  webcam.mp4        annotated webcam frames (if received)
  robot_camera.mp4  robot hand camera frames (if received)
  meta.json         task, notes, fps, units, finger / region order, tactile kinds, files
Values older than STALE_AFTER seconds are written as NaN, so gaps stay visible.
start/stop come from the Qt thread and tick from the sampler thread, so all three take a lock.
"""
import csv
import json
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from finger_node.common import FINGERS, TACTILE_REGIONS

FORMAT_VERSION = 1
STALE_AFTER = 0.5  # s
NAN = float('nan')
LANDMARK_COUNT = 21


def header():
    columns = ['frame', 'time_s', 'unix_time']
    for f in FINGERS:
        columns += [f'{f}.command_rad', f'{f}.angle_rad', f'{f}.velocity_rad_s', f'{f}.force_n']
    for r in TACTILE_REGIONS:
        columns += [f'{r}.tactile_normal', f'{r}.tactile_tangential', f'{r}.tactile_direction_deg']
    for i in range(LANDMARK_COUNT):
        columns += [f'landmark{i}.{axis}' for axis in 'xyz']
    return columns


def fresh(value, stamp, now):
    return value if now - stamp <= STALE_AFTER else NAN


class Recorder:
    def __init__(self, fps=30.0):
        self.fps = fps
        self.active = False
        self.folder = None
        self.lock = threading.Lock()

    def next_folder(self, root):
        root.mkdir(parents=True, exist_ok=True)
        numbers = [int(p.name.split('_')[1]) for p in root.glob('episode_*')
                   if p.is_dir() and p.name.split('_')[1].isdigit()]
        return root / f'episode_{max(numbers, default=0) + 1:06d}'

    def start(self, root, task, notes):
        with self.lock:
            return self._start(root, task, notes)

    def _start(self, root, task, notes):
        self.folder = self.next_folder(Path(root).expanduser())
        self.folder.mkdir()
        self.task, self.notes = task, notes
        self.started_unix = time.time()
        self.started = time.monotonic()
        self.frames = 0
        self.csv_file = open(self.folder / 'data.csv', 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(header())
        self.raw = {r: [] for r in TACTILE_REGIONS}
        self.kinds = {}
        self.videos = {}
        self.video_size = {}
        self.active = True
        return self.folder

    def video(self, name, image_bgr):
        """Append a frame; the writer is created with the first frame's size."""
        if image_bgr is None:
            return
        writer = self.videos.get(name)
        if writer is None:
            height, width = image_bgr.shape[:2]
            writer = cv2.VideoWriter(str(self.folder / f'{name}.mp4'),
                                     cv2.VideoWriter_fourcc(*'mp4v'), self.fps, (width, height))
            self.videos[name] = writer
            self.video_size[name] = (width, height)
        width, height = self.video_size[name]
        if image_bgr.shape[1] != width or image_bgr.shape[0] != height:
            image_bgr = cv2.resize(image_bgr, (width, height))
        writer.write(image_bgr)

    def tick(self, snap, webcam_bgr=None, robot_rgb=None):
        with self.lock:
            if self.active:
                self._tick(snap, webcam_bgr, robot_rgb)

    def _tick(self, snap, webcam_bgr, robot_rgb):
        now = snap['time']
        row = [self.frames, round(now - self.started, 4), round(time.time(), 4)]
        for f in FINGERS:
            e = snap['fingers'][f]
            row += [e['command']] + [fresh(e[k], e['stamp'], now)
                                     for k in ('angle', 'velocity', 'effort')]
        for r in TACTILE_REGIONS:
            e = snap['tactile'][r]
            row += [fresh(e[k], e['stamp'], now) for k in ('normal', 'tangential', 'direction')]
            raw = e['raw'] if e['raw'] is not None and now - e['stamp'] <= STALE_AFTER else None
            self.raw[r].append(raw)
            if e['kind']:
                self.kinds[r] = e['kind']
        landmarks, stamp = snap['landmarks']
        if landmarks is not None and landmarks.size == LANDMARK_COUNT * 3 and now - stamp <= STALE_AFTER:
            row += [float(v) for v in landmarks]
        else:
            row += [NAN] * (LANDMARK_COUNT * 3)
        self.csv.writerow(row)
        self.video('webcam', webcam_bgr)
        if robot_rgb is not None:
            self.video('robot_camera', cv2.cvtColor(robot_rgb, cv2.COLOR_RGB2BGR))
        self.frames += 1

    def stop(self):
        """Close files; returns (folder, frames)."""
        with self.lock:
            return self._stop()

    def _stop(self):
        if not self.active:
            return None, 0
        self.active = False
        self.csv_file.close()
        for writer in self.videos.values():
            writer.release()
        arrays = {}
        for r, frames in self.raw.items():
            width = max((len(a) for a in frames if a is not None), default=0)
            stacked = np.full((len(frames), width), np.nan, np.float32)
            for i, a in enumerate(frames):
                if a is not None:
                    stacked[i, : len(a)] = a
            arrays[r] = stacked
        np.savez_compressed(self.folder / 'tactile_raw.npz', **arrays)
        meta = dict(
            format_version=FORMAT_VERSION,
            created=datetime.fromtimestamp(self.started_unix).isoformat(timespec='seconds'),
            task=self.task,
            notes=self.notes,
            fps=self.fps,
            frames=self.frames,
            duration_s=round(time.monotonic() - self.started, 3),
            measured_fps=round(self.frames / max(time.monotonic() - self.started, 1e-6), 2),
            fingers=list(FINGERS),
            tactile_regions=list(TACTILE_REGIONS),
            tactile_kinds=self.kinds,
            units=dict(
                angle='rad (0 = open)', velocity='rad/s', force='N (actuator)',
                tactile_force3d='sensor units: sum of normal, vector-summed shear, direction deg '
                                'in the sensor / fingertip frame (simulation: N)',
                tactile_dot_matrix='sum of taxels 0..255',
                landmarks='MediaPipe world landmarks, m',
            ),
            stale_after_s=STALE_AFTER,
            files=sorted(p.name for p in self.folder.iterdir()),
        )
        (self.folder / 'meta.json').write_text(json.dumps(meta, indent=2, ensure_ascii=False))
        return self.folder, self.frames
