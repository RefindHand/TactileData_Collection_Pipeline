"""Per-person calibration profile: human joint angle (deg) -> robot target (0~1).

The webcam skeleton follows the centre line of each finger, so when two fingertips touch
(pinch) the measured angles still look slightly open. A single [min, max] range per finger
(AutoRange) cannot fix that, because the robot needs a different amount of flexion than
the human angle suggests.

A profile stores reference poses (open, fist, pinch, ...). For each pose and each channel it
keeps the human angle measured while holding the pose and the robot target (0~1) that makes
the robot hand take the same pose. Between those points the mapping is piecewise linear and
outside them it is clamped, so every recorded pose is reproduced exactly.

No ROS imports: this file can be tested on any PC with plain Python.
"""
import json
from pathlib import Path

# Same order as finger_node.common.FINGERS (kept here so the file needs no ROS package).
CHANNELS = ('index', 'middle', 'ring', 'little', 'thumb_root', 'thumb')

# Two poses whose human angles on a channel are closer than this cannot be told apart
# (MediaPipe jitters a few degrees), so they are merged into one averaged point.
MIN_GAP_DEG = 3.0

PROFILE_VERSION = 1

# One JSON file per person: ~/hand_calibration_profiles/<name>.json
# (kept out of ~/tactile_dataset, which the GUI uploads to Hugging Face as a whole).
PROFILE_DIR = Path.home() / 'hand_calibration_profiles'


def profile_path(name):
    """Path of a profile by person name; rejects names that would leave PROFILE_DIR."""
    name = name.strip()

    if not name or any(c in name for c in '/\\:') or name.startswith('.'):
        raise ValueError(f'invalid profile name: {name!r}')

    return PROFILE_DIR / f'{name}.json'


def clean_points(pairs):
    """Sorted, merged, monotonic (human_deg, robot) points of one channel."""
    merged = []

    for human, robot in sorted(pairs):
        if merged and human - merged[-1][0] < MIN_GAP_DEG:
            previous_human, previous_robot = merged[-1]
            merged[-1] = (
                (previous_human + human) / 2.0,
                (previous_robot + robot) / 2.0,
            )
        else:
            merged.append((human, robot))

    if len(merged) < 2:
        return merged

    # Direction comes from the two outermost points: flexion channels grow with the
    # human angle, thumb_root may shrink. A point that goes back against it is dropped,
    # otherwise the robot would reverse while the human keeps closing.
    increasing = merged[-1][1] >= merged[0][1]
    monotonic = [merged[0]]

    for point in merged[1:]:
        if (point[1] >= monotonic[-1][1]) == increasing or point[1] == monotonic[-1][1]:
            monotonic.append(point)

    return monotonic


def interpolate(points, x):
    """Piecewise-linear y at x through sorted points, clamped at both ends."""
    if x <= points[0][0]:
        return points[0][1]

    if x >= points[-1][0]:
        return points[-1][1]

    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)

    return points[-1][1]


class CalibrationProfile:
    """Reference poses of one person and the mapping built from them."""

    def __init__(self, name):
        self.name = name
        self.poses = {}     # pose name -> {'human': [deg x6], 'robot': [0~1 x6]}
        self._points = [[] for _ in CHANNELS]

    def set_pose(self, pose, human, robot):
        """Record (or overwrite) one reference pose."""
        if len(human) != len(CHANNELS) or len(robot) != len(CHANNELS):
            raise ValueError(f'need {len(CHANNELS)} values for human and robot')

        self.poses[pose] = {
            'human': [float(v) for v in human],
            'robot': [min(1.0, max(0.0, float(v))) for v in robot],
        }
        self._rebuild()

    def remove_pose(self, pose):
        self.poses.pop(pose, None)
        self._rebuild()

    def _rebuild(self):
        self._points = [
            clean_points([
                (p['human'][k], p['robot'][k]) for p in self.poses.values()
            ])
            for k in range(len(CHANNELS))
        ]

    def channel_ready(self, k):
        """True when channel k has at least two distinct points to map with."""
        return len(self._points[k]) >= 2

    def map_channel(self, k, angle):
        """Robot target (0~1) of channel k, or None if the profile cannot map it yet."""
        if not self.channel_ready(k):
            return None

        return interpolate(self._points[k], angle)

    def map(self, angles):
        """Robot targets in CHANNELS order; None where the caller should fall back."""
        return [self.map_channel(k, angle) for k, angle in enumerate(angles)]

    # ---- save / load --------------------------------------------------------------
    def to_dict(self):
        return {
            'version': PROFILE_VERSION,
            'name': self.name,
            'channels': list(CHANNELS),
            'poses': self.poses,
        }

    @classmethod
    def from_dict(cls, data):
        if list(data.get('channels', [])) != list(CHANNELS):
            raise ValueError('profile channel order does not match CHANNELS')

        profile = cls(data['name'])

        for pose, values in data['poses'].items():
            profile.set_pose(pose, values['human'], values['robot'])

        return profile

    def save(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(self.to_dict(), ensure_ascii=False, indent=2),
            encoding='utf-8',
        )

    @classmethod
    def load(cls, path):
        return cls.from_dict(json.loads(Path(path).read_text(encoding='utf-8')))
