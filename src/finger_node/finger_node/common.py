"""Definitions shared by the finger nodes and the hardware backends (hand_sim, hand_serial)."""
import math

# Finger (actuator) order used everywhere: same as the existing keyboard tools
# (1 index, 2 middle, 3 ring, 4 little, 5 thumb_root, 6 thumb).
FINGERS = ('index', 'middle', 'ring', 'little', 'thumb_root', 'thumb')

# Tactile regions follow the ROH Gen2 firmware order (thumb first) plus palm.
# thumb_root has no tactile pad.
TACTILE_REGIONS = ('thumb', 'index', 'middle', 'ring', 'little', 'palm')

# Joint-angle command limits in rad (MuJoCo AP002 actuator ctrlrange).
# 0 = open, upper bound = fully flexed / fully rotated thumb.
FINGER_LIMITS = {
    'index': (0.0, 1.45),
    'middle': (0.0, 1.5),
    'ring': (0.0, 1.44),
    'little': (0.0, 1.44),
    'thumb_root': (0.0, 1.57),
    'thumb': (0.0, 1.327),
}


# ---- finger node topics (other nodes use these) -------------------------------
def finger_command_topic(finger):
    return f'/finger/{finger}/command'            # std_msgs/Float64, target angle rad


def finger_state_topic(finger):
    return f'/finger/{finger}/state'              # sensor_msgs/JointState, 1 joint: rad, rad/s, N


def finger_angle_force_topic(finger):
    return f'/finger/{finger}/angle_force'        # std_msgs/Float64MultiArray [deg, N, target deg]


def finger_tactile_topic(finger):
    return f'/finger/{finger}/tactile'            # std_msgs/Float32MultiArray, see tactile layout


# ---- hardware backend topics (finger nodes <-> hand_sim / hand_serial) --------
HW_STATE_TOPIC = '/hand_hw/state'                 # sensor_msgs/JointState, FINGERS order
SIM_FULL_STATE_TOPIC = '/hand_sim/full_joint_states'  # sensor_msgs/JointState, all MuJoCo joints
# STOP / START (GUI): False = hold the current pose and ignore targets, True = follow targets
HW_MOTION_ENABLE_TOPIC = '/hand_hw/motion_enable'  # std_msgs/Bool, command
HW_MOTION_STATE_TOPIC = '/hand_hw/motion_state'    # std_msgs/String: running | stopped | read_only


def hw_target_topic(finger):
    return f'/hand_hw/{finger}/target'            # std_msgs/Float64, rad


def hw_tactile_topic(region):
    return f'/hand_hw/tactile/{region}'           # std_msgs/Float32MultiArray: fingertip / palm


def hw_tactile_proximal_topic(finger):
    return f'/hand_hw/tactile_proximal/{finger}'  # std_msgs/Float32MultiArray: proximal phalanx


def finger_tactile_proximal_topic(finger):
    return f'/finger/{finger}/tactile_proximal'   # std_msgs/Float32MultiArray: proximal phalanx


# ---- tactile layout (ROH-AP002 manual 7.4, checked on the real hand) -------------
# fingertip (distal phalanx), all 5 fingers : 3D force  -> /…/tactile/<finger>
# proximal phalanx, index / middle / ring   : normal    -> /…/tactile_proximal/<finger>
# palm                                      : 11 x 5 dot matrix -> /…/tactile/palm
#
# Float32MultiArray.layout.dim[0].label tells the format of data[]:
#   'force3d'    : [normal N, tangential N, direction_deg]  (one fingertip point)
#   'normal'     : normal force N per point (proximal phalanx; simulated palm cells)
#   'dot_matrix' : raw taxel values 0..255 (real palm; fingers of the dot-matrix hand model)
# Palm messages have two dims: [PALM_GRID[0] columns, PALM_GRID[1] rows], data index
# = column * 5 + row. Columns run from the index-finger side to the little-finger side, rows
# from the wrist side to the finger side. PALM_DEAD_CELLS have no taxel (NaN).
# Invalid / missing values are NaN.
TACTILE_FORCE3D = 'force3d'
TACTILE_NORMAL = 'normal'
TACTILE_DOT_MATRIX = 'dot_matrix'
PROXIMAL_FINGERS = ('index', 'middle', 'ring')
PALM_GRID = (11, 5)
PALM_DEAD_CELLS = (0, 1, 5, 6, 10)  # thenar corner (right hand), always 0 on the real hand


def make_tactile_msg(kind, values, grid=None):
    """grid=(columns, rows) adds a second layout dimension (palm)."""
    from std_msgs.msg import Float32MultiArray, MultiArrayDimension
    msg = Float32MultiArray()
    if grid is None:
        msg.layout.dim = [MultiArrayDimension(label=kind, size=len(values), stride=len(values))]
    else:
        columns, rows = grid
        msg.layout.dim = [MultiArrayDimension(label=kind, size=columns, stride=columns * rows),
                          MultiArrayDimension(label='rows', size=rows, stride=rows)]
    msg.data = [float(v) for v in values]
    return msg


def palm_values(values):
    """First 55 values with the dead cells set to NaN."""
    cells = [float(v) for v in list(values)[:PALM_GRID[0] * PALM_GRID[1]]]
    cells += [float('nan')] * (PALM_GRID[0] * PALM_GRID[1] - len(cells))
    for k in PALM_DEAD_CELLS:
        cells[k] = float('nan')
    return cells


MODEL_RELATIVE_PATH = 'external/rohand_mujoco/AP002/model/rohand_gen2_AP002_right.xml'


def default_model_xml():
    """Official AP002 right-hand MJCF: $ROHAND_MODEL_XML, else ./external/... from cwd upward."""
    import os
    from pathlib import Path
    if os.environ.get('ROHAND_MODEL_XML'):
        return os.environ['ROHAND_MODEL_XML']
    for base in (Path.cwd(), *Path.cwd().parents):
        candidate = base / MODEL_RELATIVE_PATH
        if candidate.is_file():
            return str(candidate)
    return MODEL_RELATIVE_PATH


def is_finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)
