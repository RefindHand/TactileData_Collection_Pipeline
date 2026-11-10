#!/usr/bin/env python3
"""MuJoCo hardware backend for AP002 (right hand).

Owns the physics engine; finger nodes talk to it instead of to the real hand.
  sub  /hand_hw/<finger>/target        std_msgs/Float64 (rad, clamped to ctrlrange)
  pub  /hand_hw/state                  sensor_msgs/JointState (6 actuated joints)
  pub  /hand_hw/tactile/<finger>       std_msgs/Float32MultiArray, fingertip 'force3d' [N, N, deg]
  pub  /hand_hw/tactile_proximal/<f>   std_msgs/Float32MultiArray, proximal 'normal' [N] (index/middle/ring)
  pub  /hand_hw/tactile/palm           std_msgs/Float32MultiArray, 'normal' N per cell, 11 x 5 grid
  pub  /hand_sim/full_joint_states     sensor_msgs/JointState (all joints, for robot camera)
  sub  /hand_hw/motion_enable          std_msgs/Bool: STOP (False) holds the current pose, START (True)
  pub  /hand_hw/motion_state           std_msgs/String: running | stopped

Tactile = MuJoCo contact forces between a sensor link and any non-hand body, laid out like the
real AP002 (common.py): fingertip = *_distal_link (normal + shear, direction in the distal link
frame), proximal phalanx = *_proximal_link of index / middle / ring (normal only), palm =
base_link contacts binned into the 11 x 5 cells of PalmGrid. Contacts on the other links
(connecting links, thumb base) have no sensor and are ignored. Only a rough stand-in for the
real sensors; values are in N (the real palm reports raw 0..255).

Gravity is off by default (parameter gravity): the model's position actuators (kp = 1) are
too weak to hold the fingers against it (thumb_root sagged 16 deg at target 0), and higher
gains go unstable with these light links. The real hand's geared fingers do not sag either.
"""
import math
import signal
import time

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, Float64, String

from finger_node.common import (
    FINGERS, HW_MOTION_ENABLE_TOPIC, HW_MOTION_STATE_TOPIC, HW_STATE_TOPIC, PALM_DEAD_CELLS, PALM_GRID, PROXIMAL_FINGERS, SIM_FULL_STATE_TOPIC,
    TACTILE_FORCE3D, TACTILE_NORMAL, TACTILE_REGIONS, default_model_xml, hw_tactile_proximal_topic,
    hw_tactile_topic, hw_target_topic, is_finite, make_tactile_msg,
)

ACTUATORS = dict(index='index_finger', middle='middle_finger', ring='ring_finger',
                 little='little_finger', thumb_root='thumb_root', thumb='thumb')
BODY_PREFIX_REGION = (('if_', 'index'), ('mf_', 'middle'), ('rf_', 'ring'),
                      ('lf_', 'little'), ('th_', 'thumb'), ('base_link', 'palm'))
SEGMENT_TIP, SEGMENT_PROXIMAL, SEGMENT_PALM = 'tip', 'proximal', 'palm'
# Palm taxel grid placement, read off the manual's sensor figure (ROH-AP002 manual 7.4):
# columns run between the index and little knuckles, minus this fraction at both ends; rows use
# the same pitch (square taxels), centred at this fraction of the wrist -> knuckle distance.
PALM_COLUMN_INSET = 0.08
PALM_ROWS_CENTER = 0.60
PALM_SURFACE_RADIUS = 0.006  # m; mesh vertices this close to a cell's normal line give its height
# Fixed ball in front of the palm: closing the hand produces contacts on most fingers.
TEST_OBJECTS = {
    'ball': dict(pos=(-0.10, -0.01, -0.045), size=(0.03, 0, 0), type=mujoco.mjtGeom.mjGEOM_SPHERE),
}


def load_model(xml_path, test_object):
    spec = mujoco.MjSpec.from_file(xml_path)
    if test_object != 'none':
        obj = TEST_OBJECTS[test_object]
        body = spec.worldbody.add_body(name='test_object', pos=obj['pos'])
        body.add_geom(name='test_object', type=obj['type'], size=obj['size'],
                      rgba=[0.9, 0.4, 0.2, 1.0])
    return spec.compile()


def hand_bodies(model):
    """Ids of all bodies that belong to the hand."""
    return {b for b in range(model.nbody)
            if any(model.body(b).name.startswith(prefix) for prefix, _ in BODY_PREFIX_REGION)}


def sensor_of_body(name):
    """Body name -> (region, segment) of the tactile sensor on it, or None (no sensor)."""
    if name.startswith('base_link'):
        return 'palm', SEGMENT_PALM
    for prefix, region in BODY_PREFIX_REGION:
        if region != 'palm' and name.startswith(prefix):
            if 'distal' in name:
                return region, SEGMENT_TIP
            if 'proximal' in name and region in PROXIMAL_FINGERS:
                return region, SEGMENT_PROXIMAL
    return None


def unit(v):
    return v / np.linalg.norm(v)


class PalmGrid:
    """The 11 x 5 palm taxel grid on the model (the palm is fixed to the world).

    points[k] is the world position of taxel k = column * rows + row on the palm surface;
    column 0 is at the index-finger side, row 0 at the wrist side (common.PALM_GRID).
    """

    def __init__(self, model, data):
        self.columns, self.rows = PALM_GRID
        body = lambda name: data.xpos[model.body(name).id].copy()
        self.origin = body('base_link')  # wrist end of the palm
        index, little = body('if_proximal_link'), body('lf_proximal_link')
        knuckles = np.mean([body(f'{p}_proximal_link') for p in ('if', 'mf', 'rf', 'lf')], axis=0)
        self.toward = unit(knuckles - self.origin)
        across = little - index
        self.across = unit(across - (across @ self.toward) * self.toward)
        normal = unit(np.cross(self.across, self.toward))
        if normal @ (body('th_distal_link') - self.origin) < 0:  # palm side = where the thumb is
            normal = -normal
        self.normal = normal

        a_index, a_little = (index - self.origin) @ self.across, (little - self.origin) @ self.across
        inset = PALM_COLUMN_INSET * (a_little - a_index)
        self.cols_at = np.linspace(a_index + inset, a_little - inset, self.columns)
        self.du = self.cols_at[1] - self.cols_at[0]
        length = (knuckles - self.origin) @ self.toward
        self.rows_at = PALM_ROWS_CENTER * length + (np.arange(self.rows) - (self.rows - 1) / 2) * self.du
        self.dv = self.du

        vertices = []
        palm = model.body('base_link').id
        for g in range(model.ngeom):
            if model.geom_bodyid[g] == palm and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                mesh = model.geom_dataid[g]
                start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
                local = model.mesh_vert[start:start + count]
                vertices.append(data.geom_xpos[g] + local @ data.geom_xmat[g].reshape(3, 3).T)
        vertices = np.concatenate(vertices) if vertices else np.zeros((0, 3))
        points = []
        for c in self.cols_at:
            for r in self.rows_at:
                p = self.origin + c * self.across + r * self.toward
                rel = vertices - p
                height = rel @ self.normal
                lateral = np.linalg.norm(rel - np.outer(height, self.normal), axis=1)
                near = height[lateral < PALM_SURFACE_RADIUS]
                points.append(p + self.normal * ((near.max() if near.size else 0.0) + 0.001))
        self.points = np.array(points)

    def cell(self, position):
        """Taxel index under a world point, or None (outside the grid or a dead cell)."""
        rel = np.asarray(position) - self.origin
        c = int(round((rel @ self.across - self.cols_at[0]) / self.du))
        r = int(round((rel @ self.toward - self.rows_at[0]) / self.dv))
        if 0 <= c < self.columns and 0 <= r < self.rows:
            k = c * self.rows + r
            return None if k in PALM_DEAD_CELLS else k
        return None


def disable_gravity(model):
    """No gravity: the weak (kp = 1) finger actuators reach their targets exactly."""
    model.opt.gravity[:] = 0.0


class SimBackend(Node):
    def __init__(self, model, data):
        super().__init__('hand_sim')
        self.model = model
        self.data = data
        self.act_ids = []
        self.joint_qpos = []
        self.joint_dof = []
        for finger in FINGERS:
            act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ACTUATORS[finger])
            if act < 0:
                raise ValueError(f'actuator not found: {ACTUATORS[finger]}')
            joint = int(model.actuator_trnid[act, 0])
            self.act_ids.append(act)
            self.joint_qpos.append(int(model.jnt_qposadr[joint]))
            self.joint_dof.append(int(model.jnt_dofadr[joint]))
        self.ctrl = [float(model.actuator_ctrlrange[a, 0]) for a in self.act_ids]

        self.hand = hand_bodies(model)
        self.sensor = {}      # body id -> (region, segment)
        self.tip_body = {}    # finger -> distal link: frame of the shear direction
        for b in self.hand:
            found = sensor_of_body(model.body(b).name)
            if found is not None:
                self.sensor[b] = found
                if found[1] == SEGMENT_TIP:
                    self.tip_body[found[0]] = b
        mujoco.mj_forward(model, data)
        self.palm = PalmGrid(model, data)
        self.all_joint_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
                                for j in range(model.njnt)]
        self.all_joint_qpos = [int(model.jnt_qposadr[j]) for j in range(model.njnt)]

        for k, finger in enumerate(FINGERS):
            self.create_subscription(Float64, hw_target_topic(finger),
                                     lambda msg, k=k: self.on_target(k, msg), 10)
        self.running = True  # STOP / START
        self.create_subscription(Bool, HW_MOTION_ENABLE_TOPIC, self.on_motion_enable, 10)
        self.pub_motion_state = self.create_publisher(String, HW_MOTION_STATE_TOPIC, 10)
        self.create_timer(0.5, self.publish_motion_state)
        self.pub_state = self.create_publisher(JointState, HW_STATE_TOPIC, 10)
        self.pub_full = self.create_publisher(JointState, SIM_FULL_STATE_TOPIC, 10)
        self.pub_tactile = {r: self.create_publisher(Float32MultiArray, hw_tactile_topic(r), 10)
                            for r in TACTILE_REGIONS}
        self.pub_proximal = {f: self.create_publisher(Float32MultiArray, hw_tactile_proximal_topic(f), 10)
                             for f in PROXIMAL_FINGERS}
        self.get_logger().info('MuJoCo backend ready; targets on /hand_hw/<finger>/target')

    def on_motion_enable(self, msg):
        if msg.data == self.running:
            return
        self.running = msg.data
        if not self.running:  # hold the current pose
            self.ctrl = [float(self.data.qpos[q]) for q in self.joint_qpos]
        self.get_logger().warning('START: following targets' if self.running else 'STOP: holding pose')
        self.publish_motion_state()

    def publish_motion_state(self):
        self.pub_motion_state.publish(String(data='running' if self.running else 'stopped'))

    def on_target(self, k, msg):
        if not self.running or not is_finite(msg.data):
            return
        lo, hi = self.model.actuator_ctrlrange[self.act_ids[k]]
        self.ctrl[k] = float(min(max(msg.data, lo), hi))

    def apply_ctrl(self):
        for a, value in zip(self.act_ids, self.ctrl):
            self.data.ctrl[a] = value

    def contact_forces(self):
        """(fingertip {finger: [N, N, deg]}, proximal {finger: [N]}, palm [55 cells N, dead NaN])."""
        tip_normal = {r: 0.0 for r in self.tip_body}
        tip_shear = {r: np.zeros(3) for r in self.tip_body}
        proximal = {f: 0.0 for f in PROXIMAL_FINGERS}
        palm = np.zeros(self.palm.columns * self.palm.rows)
        f6 = np.zeros(6)
        for i in range(self.data.ncon):
            con = self.data.contact[i]
            b1 = int(self.model.geom_bodyid[con.geom1])
            b2 = int(self.model.geom_bodyid[con.geom2])
            if (b1 in self.hand) == (b2 in self.hand):
                continue  # hand-hand or object-object contact
            sensor = self.sensor.get(b1 if b1 in self.hand else b2)
            if sensor is None:
                continue  # link without a tactile sensor
            region, segment = sensor
            mujoco.mj_contactForce(self.model, self.data, i, f6)
            if segment == SEGMENT_TIP:
                frame = con.frame.reshape(3, 3)  # rows: normal, tangent1, tangent2 (world)
                tip_normal[region] += abs(f6[0])
                tip_shear[region] += f6[1] * frame[1] + f6[2] * frame[2]
            elif segment == SEGMENT_PROXIMAL:
                proximal[region] += abs(f6[0])
            else:
                k = self.palm.cell(con.pos)
                if k is not None:
                    palm[k] += abs(f6[0])
        tips = {}
        for region, body in self.tip_body.items():
            local = self.data.xmat[body].reshape(3, 3).T @ tip_shear[region]
            tips[region] = [tip_normal[region], float(np.linalg.norm(tip_shear[region])),
                            math.degrees(math.atan2(local[1], local[0])) % 360.0]
        palm[list(PALM_DEAD_CELLS)] = np.nan
        return tips, {f: [n] for f, n in proximal.items()}, palm.tolist()

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        state = JointState()
        state.header.stamp = stamp
        state.name = list(FINGERS)
        state.position = [float(self.data.qpos[q]) for q in self.joint_qpos]
        state.velocity = [float(self.data.qvel[d]) for d in self.joint_dof]
        state.effort = [float(self.data.actuator_force[a]) for a in self.act_ids]
        self.pub_state.publish(state)

        full = JointState()
        full.header.stamp = stamp
        full.name = self.all_joint_names
        full.position = [float(self.data.qpos[q]) for q in self.all_joint_qpos]
        self.pub_full.publish(full)

        tips, proximal, palm = self.contact_forces()
        for region, values in tips.items():
            self.pub_tactile[region].publish(make_tactile_msg(TACTILE_FORCE3D, values))
        for finger, values in proximal.items():
            self.pub_proximal[finger].publish(make_tactile_msg(TACTILE_NORMAL, values))
        self.pub_tactile['palm'].publish(make_tactile_msg(TACTILE_NORMAL, palm, grid=PALM_GRID))


def main():
    rclpy.init()
    probe = Node('hand_sim_params')
    xml = probe.declare_parameter('model_xml', default_model_xml()).value
    use_viewer = probe.declare_parameter('viewer', True).value
    test_object = probe.declare_parameter('test_object', 'ball').value
    rate = float(probe.declare_parameter('publish_rate', 60.0).value)  # capped by 60 Hz render loop
    gravity = probe.declare_parameter('gravity', False).value
    probe.destroy_node()

    node = None
    try:
        model = load_model(xml, test_object)
        if not gravity:
            disable_gravity(model)
        data = mujoco.MjData(model)
        node = SimBackend(model, data)
        frame = 1.0 / 60.0
        steps = max(1, round(frame / model.opt.timestep))
        publish_every = 1.0 / rate
        last_pub = 0.0
        viewer = mujoco.viewer.launch_passive(model, data) if use_viewer else None
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            started = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0)
            node.apply_ctrl()
            for _ in range(steps):
                mujoco.mj_step(model, data)
            if started - last_pub >= publish_every - frame / 2:
                node.publish()
                last_pub = started
            if viewer is not None:
                viewer.sync()
            remaining = frame - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
        if viewer is not None:
            viewer.close()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise  # a real error; otherwise Humble's take_message raced with Ctrl+C
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # ros2 launch forwards a second Ctrl+C
        try:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:  # second Ctrl+C (e.g. from ros2 launch) during cleanup
            pass


if __name__ == '__main__':
    main()
