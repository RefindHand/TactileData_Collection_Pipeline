#!/usr/bin/env python3
"""MuJoCo hardware backend for AP002 (right hand).

Owns the physics engine; finger nodes talk to it instead of to the real hand.
  sub  /hand_hw/<finger>/target        std_msgs/Float64 (rad, clamped to ctrlrange)
  pub  /hand_hw/state                  sensor_msgs/JointState (6 actuated joints)
  pub  /hand_hw/tactile/<region>       std_msgs/Float32MultiArray, force3d [N, N, deg]
  pub  /hand_sim/full_joint_states     sensor_msgs/JointState (all joints, for robot camera)

Tactile = MuJoCo contact forces between a hand region and any non-hand body,
summed per region. Only a rough stand-in for the real tactile sensor.
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
from std_msgs.msg import Float32MultiArray, Float64

from finger_node.common import (
    FINGERS, HW_STATE_TOPIC, SIM_FULL_STATE_TOPIC, TACTILE_FORCE3D, TACTILE_REGIONS,
    default_model_xml, hw_tactile_topic, hw_target_topic, is_finite, make_tactile_msg,
)

ACTUATORS = dict(index='index_finger', middle='middle_finger', ring='ring_finger',
                 little='little_finger', thumb_root='thumb_root', thumb='thumb')
BODY_PREFIX_REGION = (('if_', 'index'), ('mf_', 'middle'), ('rf_', 'ring'),
                      ('lf_', 'little'), ('th_', 'thumb'), ('base_link', 'palm'))
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

        self.body_region = {}
        for b in range(model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or ''
            for prefix, region in BODY_PREFIX_REGION:
                if name.startswith(prefix):
                    self.body_region[b] = region
        # shear direction is expressed in the fingertip (distal link) frame; palm uses base_link
        self.region_frame_body = {}
        for b, region in self.body_region.items():
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
            if region not in self.region_frame_body or 'distal' in name:
                self.region_frame_body[region] = b
        self.all_joint_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
                                for j in range(model.njnt)]
        self.all_joint_qpos = [int(model.jnt_qposadr[j]) for j in range(model.njnt)]

        for k, finger in enumerate(FINGERS):
            self.create_subscription(Float64, hw_target_topic(finger),
                                     lambda msg, k=k: self.on_target(k, msg), 10)
        self.pub_state = self.create_publisher(JointState, HW_STATE_TOPIC, 10)
        self.pub_full = self.create_publisher(JointState, SIM_FULL_STATE_TOPIC, 10)
        self.pub_tactile = {r: self.create_publisher(Float32MultiArray, hw_tactile_topic(r), 10)
                            for r in TACTILE_REGIONS}
        self.get_logger().info('MuJoCo backend ready; targets on /hand_hw/<finger>/target')

    def on_target(self, k, msg):
        if not is_finite(msg.data):
            return
        lo, hi = self.model.actuator_ctrlrange[self.act_ids[k]]
        self.ctrl[k] = float(min(max(msg.data, lo), hi))

    def apply_ctrl(self):
        for a, value in zip(self.act_ids, self.ctrl):
            self.data.ctrl[a] = value

    def contact_forces(self):
        """Per region: summed normal force, summed tangential force, direction in region frame."""
        normal = {r: 0.0 for r in TACTILE_REGIONS}
        tangential = {r: np.zeros(3) for r in TACTILE_REGIONS}
        f6 = np.zeros(6)
        for i in range(self.data.ncon):
            con = self.data.contact[i]
            b1 = int(self.model.geom_bodyid[con.geom1])
            b2 = int(self.model.geom_bodyid[con.geom2])
            r1, r2 = self.body_region.get(b1), self.body_region.get(b2)
            if (r1 is None) == (r2 is None):
                continue  # hand-hand or object-object contact
            region = r1 or r2
            mujoco.mj_contactForce(self.model, self.data, i, f6)
            frame = con.frame.reshape(3, 3)  # rows: normal, tangent1, tangent2 (world)
            normal[region] += abs(f6[0])
            tangential[region] += f6[1] * frame[1] + f6[2] * frame[2]
        out = {}
        for region in TACTILE_REGIONS:
            rot = self.data.xmat[self.region_frame_body[region]].reshape(3, 3)
            local = rot.T @ tangential[region]
            out[region] = [normal[region], float(np.linalg.norm(tangential[region])),
                           math.degrees(math.atan2(local[1], local[0]))]
        return out

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

        for region, values in self.contact_forces().items():
            self.pub_tactile[region].publish(make_tactile_msg(TACTILE_FORCE3D, values))


def main():
    rclpy.init()
    probe = Node('hand_sim_params')
    xml = probe.declare_parameter('model_xml', default_model_xml()).value
    use_viewer = probe.declare_parameter('viewer', True).value
    test_object = probe.declare_parameter('test_object', 'ball').value
    rate = float(probe.declare_parameter('publish_rate', 60.0).value)  # capped by 60 Hz render loop
    probe.destroy_node()

    node = None
    try:
        model = load_model(xml, test_object)
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
