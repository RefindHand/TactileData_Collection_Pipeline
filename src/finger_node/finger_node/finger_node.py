#!/usr/bin/env python3
"""Finger node: one instance per actuator, outputs that actuator's angle and force.

Parameter finger: index | middle | ring | little | thumb_root | thumb

  sub  /finger/<finger>/command        std_msgs/Float64            target angle (rad)
  pub  /finger/<finger>/state          sensor_msgs/JointState      1 joint: angle (rad), velocity (rad/s),
                                                                   effort = actuator force (N)
  pub  /finger/<finger>/angle_force    std_msgs/Float64MultiArray  [angle_deg, force_N, target_deg]
  pub  /finger/<finger>/tactile        std_msgs/Float32MultiArray  fingertip tactile (not for thumb_root)

Hardware side (hand_sim or hand_serial in this package; one backend owns MuJoCo / the serial port,
so the six finger nodes never open it themselves):
  pub  /hand_hw/<finger>/target        std_msgs/Float64            clamped target angle (rad)
  sub  /hand_hw/state                  sensor_msgs/JointState      all 6 actuators
  sub  /hand_hw/tactile/<finger>       std_msgs/Float32MultiArray

If the backend gives no velocity (real hand), it is estimated from consecutive angles.
"""
import math
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64, Float64MultiArray

from finger_node.common import (
    FINGER_LIMITS, FINGERS, HW_STATE_TOPIC, TACTILE_REGIONS, finger_angle_force_topic,
    finger_command_topic, finger_state_topic, finger_tactile_topic, hw_tactile_topic,
    hw_target_topic,
)

NAN = float('nan')


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def element(values, k):
    return float(values[k]) if k < len(values) else NAN


class FingerNode(Node):
    def __init__(self):
        super().__init__('finger_node')
        p = self.declare_parameter
        self.finger = p('finger', 'index').value
        if self.finger not in FINGERS:
            raise ValueError(f'finger must be one of {FINGERS}: {self.finger}')
        lo, hi = FINGER_LIMITS[self.finger]
        self.lo = float(p('min_angle', lo).value)
        self.hi = float(p('max_angle', hi).value)
        log_period = float(p('log_period', 1.0).value)  # 0 = no periodic log
        hw_state_topic = p('hw_state_topic', HW_STATE_TOPIC).value
        f = self.finger

        self.target = NAN
        self.last = None  # (time_s, angle_rad) for velocity estimate
        self.latest = (NAN, NAN, NAN)  # angle rad, velocity rad/s, force N

        self.pub_target = self.create_publisher(Float64, hw_target_topic(f), 10)
        self.pub_state = self.create_publisher(JointState, finger_state_topic(f), 10)
        self.pub_angle_force = self.create_publisher(
            Float64MultiArray, finger_angle_force_topic(f), 10)
        self.create_subscription(Float64, finger_command_topic(f), self.on_command, 10)
        self.create_subscription(JointState, hw_state_topic, self.on_hw_state, 10)
        if f in TACTILE_REGIONS:  # thumb_root has no tactile pad
            self.pub_tactile = self.create_publisher(Float32MultiArray, finger_tactile_topic(f), 10)
            self.create_subscription(Float32MultiArray, hw_tactile_topic(f),
                                     self.pub_tactile.publish, 10)
        if log_period > 0:
            self.create_timer(log_period, self.log_status)
        self.get_logger().info(f'finger={f} limits=[{self.lo:.3f}, {self.hi:.3f}] rad')

    def on_command(self, msg):
        if not math.isfinite(msg.data):
            self.get_logger().warning(f'ignored non-finite command: {msg.data}')
            return
        self.target = max(self.lo, min(self.hi, msg.data))
        self.pub_target.publish(Float64(data=self.target))

    def on_hw_state(self, msg):
        names = list(msg.name)
        k = names.index(self.finger) if self.finger in names else FINGERS.index(self.finger)
        angle = element(msg.position, k)
        velocity = element(msg.velocity, k)
        force = element(msg.effort, k)

        now = stamp_seconds(msg.header.stamp) or self.get_clock().now().nanoseconds * 1e-9
        if math.isnan(velocity) and self.last is not None and now > self.last[0]:
            velocity = (angle - self.last[1]) / (now - self.last[0])
        self.last = (now, angle)
        self.latest = (angle, velocity, force)

        out = JointState()
        out.header = msg.header
        out.name = [self.finger]
        out.position = [angle]
        out.velocity = [velocity]
        out.effort = [force]
        self.pub_state.publish(out)
        self.pub_angle_force.publish(Float64MultiArray(
            data=[math.degrees(angle), force, math.degrees(self.target)]))

    def log_status(self):
        angle, velocity, force = self.latest
        self.get_logger().info(
            f'{self.finger:10} angle={math.degrees(angle):7.2f}deg '
            f'target={math.degrees(self.target):7.2f}deg '
            f'vel={math.degrees(velocity):8.2f}deg/s force={force:7.3f}N')


def main():
    rclpy.init()
    node = None
    try:
        node = FingerNode()
        rclpy.spin(node)
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
