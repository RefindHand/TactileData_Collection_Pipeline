#!/usr/bin/env python3
"""Feedback node: collects angle / force / tactile feedback from the six finger nodes.

  sub  <prefix>/<finger>/state      sensor_msgs/JointState      angle (rad), velocity, actuator effort
  sub  <prefix>/<finger>/command    std_msgs/Float64            target angle (rad), for tracking error
  sub  <prefix>/<finger>/tactile    std_msgs/Float32MultiArray  fingertip tactile (no thumb_root)
  sub  <palm_tactile_topic>         std_msgs/Float32MultiArray  palm tactile
  (prefix = parameter finger_topic_prefix, default /finger)

  pub  /feedback/joint_states       sensor_msgs/JointState      6 fingers in one stamped snapshot
  pub  /feedback/tracking_error     std_msgs/Float64MultiArray  command - measured angle (rad)
  pub  /feedback/tactile_summary    std_msgs/Float64MultiArray  pressure total per region
                                    [thumb, index, middle, ring, little, palm]
  pub  /feedback/contact            std_msgs/UInt8MultiArray    1 = region in contact
  pub  /feedback/diagnostics        diagnostic_msgs/DiagnosticArray  per-finger / per-region health

Missing values are NaN. A source is 'stale' if nothing arrived for stale_timeout seconds.

Tactile Float32MultiArray: layout.dim[0].label gives the data format
  'force3d'    : [normal, tangential, direction_deg] repeated per sensing point
  'dot_matrix' : taxel values 0..255 (default when no label)
"""
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64, Float64MultiArray, UInt8MultiArray

# Same order as the keyboard tools: 1 index, 2 middle, 3 ring, 4 little, 5 thumb_root, 6 thumb
FINGERS = ('index', 'middle', 'ring', 'little', 'thumb_root', 'thumb')
# ROH Gen2 firmware tactile order (thumb first) plus palm; thumb_root has no tactile pad
TACTILE_REGIONS = ('thumb', 'index', 'middle', 'ring', 'little', 'palm')
TACTILE_FORCE3D = 'force3d'
TACTILE_DOT_MATRIX = 'dot_matrix'
NAN = float('nan')


def tactile_kind(msg):
    return msg.layout.dim[0].label if msg.layout.dim else TACTILE_DOT_MATRIX


def tactile_total(kind, data):
    """Scalar 'how hard is this region pressed' used for contact detection."""
    if kind == TACTILE_FORCE3D:
        return float(sum(v for v in data[0::3] if v >= 0))
    return float(sum(v for v in data if v >= 0))


class Sample:
    """Latest value of one input plus its arrival time (monotonic)."""

    def __init__(self):
        self.value = None
        self.stamp = None

    def set(self, value):
        self.value = value
        self.stamp = time.monotonic()

    def age(self):
        return math.inf if self.stamp is None else time.monotonic() - self.stamp


class FeedbackNode(Node):
    def __init__(self):
        super().__init__('feedback_node')
        p = self.declare_parameter
        rate = float(p('rate', 30.0).value)
        self.stale_timeout = float(p('stale_timeout', 0.5).value)
        self.contact_threshold = {
            TACTILE_FORCE3D: float(p('contact_threshold_force3d', 0.5).value),
            TACTILE_DOT_MATRIX: float(p('contact_threshold_dot_matrix', 50.0).value),
        }
        prefix = p('finger_topic_prefix', '/finger').value.rstrip('/')
        palm_topic = p('palm_tactile_topic', '/hand_hw/tactile/palm').value

        self.state = {f: Sample() for f in FINGERS}
        self.command = {f: Sample() for f in FINGERS}
        self.tactile = {r: Sample() for r in TACTILE_REGIONS}

        for f in FINGERS:
            self.create_subscription(JointState, f'{prefix}/{f}/state',
                                     lambda m, f=f: self.on_state(f, m), 10)
            self.create_subscription(Float64, f'{prefix}/{f}/command',
                                     lambda m, f=f: self.command[f].set(m.data), 10)
        for r in TACTILE_REGIONS:
            topic = palm_topic if r == 'palm' else f'{prefix}/{r}/tactile'
            self.create_subscription(Float32MultiArray, topic,
                                     lambda m, r=r: self.on_tactile(r, m), 10)

        self.pub_joints = self.create_publisher(JointState, '/feedback/joint_states', 10)
        self.pub_error = self.create_publisher(Float64MultiArray, '/feedback/tracking_error', 10)
        self.pub_tactile = self.create_publisher(Float64MultiArray, '/feedback/tactile_summary', 10)
        self.pub_contact = self.create_publisher(UInt8MultiArray, '/feedback/contact', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/feedback/diagnostics', 10)
        self.create_timer(1.0 / rate, self.publish_feedback)
        self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().info(
            f'feedback node: {rate:g} Hz, fingers from {prefix}/<finger>/*, palm from {palm_topic}')

    # ---- inputs ---------------------------------------------------------------
    def on_state(self, finger, msg):
        def first(values):
            return float(values[0]) if len(values) else NAN
        self.state[finger].set((first(msg.position), first(msg.velocity), first(msg.effort)))

    def on_tactile(self, region, msg):
        kind = tactile_kind(msg)
        total = tactile_total(kind, msg.data)
        self.tactile[region].set((kind, total, len(msg.data)))

    # ---- derived values -------------------------------------------------------
    def fresh(self, sample):
        return sample.age() <= self.stale_timeout

    def angle(self, finger):
        s = self.state[finger]
        return s.value[0] if self.fresh(s) else NAN

    def tracking_error(self, finger):
        c = self.command[finger]
        if c.value is None:
            return NAN
        return c.value - self.angle(finger)

    def pressure(self, region):
        s = self.tactile[region]
        return s.value[1] if self.fresh(s) else NAN

    def in_contact(self, region):
        s = self.tactile[region]
        if not self.fresh(s):
            return False
        kind, total, _ = s.value
        return total >= self.contact_threshold.get(kind, math.inf)

    # ---- outputs --------------------------------------------------------------
    def publish_feedback(self):
        joints = JointState()
        joints.header.stamp = self.get_clock().now().to_msg()
        joints.name = list(FINGERS)
        for f in FINGERS:
            s = self.state[f]
            pos, vel, eff = s.value if self.fresh(s) else (NAN, NAN, NAN)
            joints.position.append(pos)
            joints.velocity.append(vel)
            joints.effort.append(eff)
        self.pub_joints.publish(joints)
        self.pub_error.publish(Float64MultiArray(data=[self.tracking_error(f) for f in FINGERS]))
        self.pub_tactile.publish(Float64MultiArray(data=[self.pressure(r) for r in TACTILE_REGIONS]))
        self.pub_contact.publish(UInt8MultiArray(
            data=[int(self.in_contact(r)) for r in TACTILE_REGIONS]))

    def status_for(self, sample, name, hardware_id):
        st = DiagnosticStatus(name=name, hardware_id=hardware_id)
        if sample.value is None:
            st.level, st.message = DiagnosticStatus.ERROR, 'no data'
        elif not self.fresh(sample):
            st.level, st.message = DiagnosticStatus.WARN, f'stale ({sample.age():.1f}s)'
        else:
            st.level, st.message = DiagnosticStatus.OK, 'ok'
        return st

    def publish_diagnostics(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        lines = []
        for f in FINGERS:
            st = self.status_for(self.state[f], f'feedback/finger/{f}', f)
            angle, err = self.angle(f), self.tracking_error(f)
            effort = self.state[f].value[2] if self.fresh(self.state[f]) else NAN
            st.values = [KeyValue(key='angle_deg', value=f'{math.degrees(angle):.2f}'),
                         KeyValue(key='error_deg', value=f'{math.degrees(err):.2f}'),
                         KeyValue(key='effort', value=f'{effort:.3f}')]
            array.status.append(st)
            lines.append(f'{f:10} angle={math.degrees(angle):7.2f}deg '
                         f'err={math.degrees(err):7.2f}deg effort={effort:7.3f} [{st.message}]')
        for r in TACTILE_REGIONS:
            st = self.status_for(self.tactile[r], f'feedback/tactile/{r}', r)
            if self.tactile[r].value is not None:
                kind, total, size = self.tactile[r].value
                st.values = [KeyValue(key='kind', value=kind), KeyValue(key='size', value=str(size)),
                             KeyValue(key='total', value=f'{total:.3f}'),
                             KeyValue(key='contact', value=str(self.in_contact(r)))]
            array.status.append(st)
        self.pub_diag.publish(array)
        lines.append('tactile ' + ' '.join(
            f"{r}={self.pressure(r):.2f}{'*' if self.in_contact(r) else ''}"
            for r in TACTILE_REGIONS))
        self.get_logger().info('\n' + '\n'.join(lines))


def main():
    rclpy.init()
    node = None
    try:
        node = FeedbackNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:  # second Ctrl+C (e.g. from ros2 launch) during cleanup
            pass


if __name__ == '__main__':
    main()
