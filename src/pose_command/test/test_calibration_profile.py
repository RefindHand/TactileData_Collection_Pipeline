"""Tests for pose_command.calibration_profile (plain Python, no ROS needed).

Run from the repository root:
    python src/pose_command/test/test_calibration_profile.py
or with pytest:
    python -m pytest src/pose_command/test/test_calibration_profile.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from pose_command.calibration_profile import (  # noqa: E402
    CalibrationProfile, clean_points, interpolate,
)

# Example numbers (deg / 0~1). Real values come from recording each pose.
# order: index, middle, ring, little, thumb_root, thumb
OPEN_HUMAN = [20.0, 18.0, 17.0, 19.0, 40.0, 10.0]
OPEN_ROBOT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
FIST_HUMAN = [110.0, 112.0, 108.0, 105.0, 10.0, 45.0]
FIST_ROBOT = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
# Pinch: index only half bent on the webcam, but the robot needs 0.55 to touch the thumb.
PINCH_HUMAN = [50.0, 25.0, 24.0, 26.0, 20.0, 25.0]
PINCH_ROBOT = [0.55, 0.05, 0.05, 0.05, 0.8, 0.6]


def make_profile():
    profile = CalibrationProfile('tester')
    profile.set_pose('open', OPEN_HUMAN, OPEN_ROBOT)
    profile.set_pose('fist', FIST_HUMAN, FIST_ROBOT)
    profile.set_pose('pinch', PINCH_HUMAN, PINCH_ROBOT)
    return profile


def test_interpolate_and_clamp():
    points = [(0.0, 0.0), (10.0, 1.0)]
    assert interpolate(points, 5.0) == 0.5
    assert interpolate(points, -3.0) == 0.0
    assert interpolate(points, 99.0) == 1.0


def test_recorded_poses_are_reproduced():
    profile = make_profile()
    for human, robot in [(OPEN_HUMAN, OPEN_ROBOT), (FIST_HUMAN, FIST_ROBOT),
                         (PINCH_HUMAN, PINCH_ROBOT)]:
        mapped = profile.map(human)
        assert all(abs(m - r) < 1e-9 for m, r in zip(mapped, robot)), mapped


def test_pinch_is_stronger_than_linear_range():
    # A plain open..fist linear range would give the index (50-20)/(110-20) = 0.33.
    linear = (50.0 - 20.0) / (110.0 - 20.0)
    assert make_profile().map_channel(0, 50.0) > linear


def test_thumb_root_can_decrease():
    # thumb_root: human angle shrinks while the robot value grows.
    profile = make_profile()
    assert profile.map_channel(4, 40.0) == 0.0
    assert profile.map_channel(4, 10.0) == 1.0


def test_close_poses_are_merged_and_reversal_dropped():
    assert clean_points([(20.0, 0.0), (21.0, 0.2)]) == [(20.5, 0.1)]
    # (50, 0.9) then (80, 0.5) would make the robot open while the human closes.
    assert clean_points([(20.0, 0.0), (50.0, 0.9), (80.0, 0.5), (110.0, 1.0)]) == [
        (20.0, 0.0), (50.0, 0.9), (110.0, 1.0)]


def test_not_ready_until_two_poses():
    profile = CalibrationProfile('tester')
    profile.set_pose('open', OPEN_HUMAN, OPEN_ROBOT)
    assert profile.map(OPEN_HUMAN) == [None] * 6


def test_save_and_load():
    profile = make_profile()
    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, 'tester.json')
        profile.save(path)
        loaded = CalibrationProfile.load(path)
    assert loaded.name == 'tester'
    assert loaded.map(PINCH_HUMAN) == profile.map(PINCH_HUMAN)


if __name__ == '__main__':
    tests = [value for name, value in sorted(globals().items()) if name.startswith('test_')]
    for test in tests:
        test()
        print(f'ok  {test.__name__}')
    print(f'{len(tests)} tests passed')
