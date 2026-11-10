#!/usr/bin/env python3
"""Real AP002 hardware backend over Modbus-RTU (single owner of the serial port).

Same topics as the MuJoCo backend, so finger nodes work unchanged:
  sub  /hand_hw/<finger>/target        std_msgs/Float64 (rad)
  pub  /hand_hw/state                  sensor_msgs/JointState (angle rad, effort N)
  pub  /hand_hw/firmware_angle         sensor_msgs/JointState (ROH_FINGER_ANGLE, rad, reference only)
  pub  /hand_hw/tactile/<region>       std_msgs/Float32MultiArray: fingertip / palm (see below)
  pub  /hand_hw/tactile_proximal/<f>   std_msgs/Float32MultiArray: proximal phalanx (index, middle, ring)

Tactile (3D-force hand, sub_model 1; layout checked on the real hand with tactile_probe):
  FORCE_EX0..4 (thumb, index, middle, ring, little): point 0 = fingertip [normal, tangential mN,
    direction deg] -> 'force3d' [N, N, deg]; point 1 (index, middle, ring) = proximal phalanx
    [normal mN, 0, 0] -> 'normal' [N]. 0xFFFF = invalid -> NaN.
  FORCE_EX5 (palm): 28 registers, high then low byte = 56 taxels 0..255; the first 55 are the
    11 x 5 grid (common.PALM_GRID, dead cells NaN) -> 'dot_matrix' with two dims.
  (the OYMotion demo reads only the high palm byte; the low bytes do change on the real hand)
Dot-matrix hand (sub_model 0): finger taxels as 'dot_matrix', palm as above, no proximal topic.

command_mode
  position (default): ROH_FINGER_POS_TARGET / ROH_FINGER_POS, logical 0..65535 (0 = open), the
                      registers already tested with hardware_connection_test/rohand_real_ros2.py.
                      rad <-> logical through FINGER_LIMITS, so 0..1 means the same as in MuJoCo.
  angle             : ROH_FINGER_ANGLE_TARGET / ROH_FINGER_ANGLE in firmware degrees (untested;
                      firmware angle = first phalanx vs palm, not the MuJoCo joint angle).

Targets are buffered and written once per poll (all 6 registers, Modbus 0x10), only when changed.
Read-only unless the parameter enable_motion:=true is given.
STOP / START: /hand_hw/motion_enable (Bool). False writes the measured positions as targets once
(the hand holds where it is) and ignores targets until True. State on /hand_hw/motion_state.
Register map: oymotion/roh_gen2_demos common/roh_registers_v2.py
Tactile decoding: oymotion/roh_gen2_demos force_on_rohand/force_chart.py
"""
import math
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, Float64, String

from finger_node.common import (
    FINGER_LIMITS, FINGERS, HW_MOTION_ENABLE_TOPIC, HW_MOTION_STATE_TOPIC, HW_STATE_TOPIC, PALM_GRID, PROXIMAL_FINGERS, TACTILE_DOT_MATRIX,
    TACTILE_FORCE3D, TACTILE_NORMAL, TACTILE_REGIONS, hw_tactile_proximal_topic, hw_tactile_topic,
    hw_target_topic, is_finite, make_tactile_msg, palm_values,
)

SLAVE_ID = 2
PROTO_ADDR = 1000
HW_ADDR = 1003
RESET_FORCE_ADDR = 1016
POS_TARGET_ADDR = 1135     # uint16, logical 0..65535
POS_ADDR = 1145            # uint16, logical 0..65535
MAX_LOGICAL = 65535
ANGLE_TARGET_ADDR = 1155
ANGLE_ADDR = 1165          # int16, deg * 100
FORCE_ADDR = 1175          # uint16, mN
MANU_DATA_ADDR = 1265
FORCE_EX_ADDR = 2000
FORCE_GROUP_SIZE = 100
EXPECTED_HW = 0x2001
MAX_READ_FAILURES = 3

# FINGERS order -> firmware index (firmware: 0 thumb, 1 index, ..., 4 little, 5 thumb_root)
FIRMWARE_INDEX = dict(index=1, middle=2, ring=3, little=4, thumb_root=5, thumb=0)
# sub_model (high byte of ROH_MANU_DATA0) -> register count per region (TACTILE_REGIONS order)
TACTILE_MODELS = {
    0: (TACTILE_DOT_MATRIX, (18, 30, 30, 30, 16, 28)),
    1: (TACTILE_FORCE3D, (3, 6, 6, 6, 3, 28)),
}


def to_int16(value):
    return value - 0x10000 if value >= 0x8000 else value


def decode_dot_matrix(regs):
    """Each register packs two 8-bit taxels, high byte first."""
    values = []
    for reg in regs:
        values.append((reg >> 8) & 0xFF)
        values.append(reg & 0xFF)
    return values


def decode_force3d(regs):
    """Each register is one byte-swapped 16-bit value; 0xFFFF marks invalid data (-> NaN)."""
    values = []
    for reg in regs:
        value = ((reg & 0xFF) << 8) | ((reg >> 8) & 0xFF)
        values.append(math.nan if value == 0xFFFF else value)
    return values


def split_force3d(values):
    """[n, t, dir, (n_proximal, -, -)] in mN / deg -> fingertip [N, N, deg], proximal [N] or None."""
    tip = [values[0] / 1000.0, values[1] / 1000.0, values[2]]
    proximal = [values[3] / 1000.0] if len(values) >= 6 else None
    return tip, proximal


class RealBackend(Node):
    def __init__(self):
        super().__init__('hand_serial')
        port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.enable_motion = self.declare_parameter('enable_motion', False).value
        rate = float(self.declare_parameter('rate', 10.0).value)
        tactile = self.declare_parameter('tactile', 'auto').value  # auto|dot_matrix|force3d|off
        tare = self.declare_parameter('tare_tactile', False).value
        self.mode = self.declare_parameter('command_mode', 'position').value
        if self.mode not in ('position', 'angle'):
            raise ValueError(f'command_mode must be position or angle: {self.mode}')

        from pymodbus.client import ModbusSerialClient  # only needed for the real hand
        self.client = ModbusSerialClient(port=port, baudrate=115200, timeout=1)
        if not self.client.connect():
            raise RuntimeError(f'serial port open failed: {port}')
        proto = self.read(PROTO_ADDR)[0]
        hardware = self.read(HW_ADDR)[0]
        self.get_logger().info(f'Modbus device: protocol=0x{proto:04X}, hardware=0x{hardware:04X}')
        if proto >> 8 != 2 or hardware != EXPECTED_HW:
            raise RuntimeError('unexpected ROH Gen2 AP identification; refusing to run')

        self.tactile_model = self.detect_tactile(tactile)
        if tare and self.tactile_model is not None:
            self.write(RESET_FORCE_ADDR, 1)
            self.get_logger().info('tactile sensors zeroed (hand must not touch anything)')

        # Start from the targets already in the hand so untouched fingers do not move
        self.target_addr = POS_TARGET_ADDR if self.mode == 'position' else ANGLE_TARGET_ADDR
        self.targets = self.read(self.target_addr, 6)
        self.dirty = False

        self.active = True
        self.failures = 0
        self.read_angle = True
        self.running = True        # STOP / START
        self.hold_pending = False  # STOP pressed: write the measured pose as target on next poll
        self.pub_state = self.create_publisher(JointState, HW_STATE_TOPIC, 10)
        self.pub_fw_angle = self.create_publisher(JointState, '/hand_hw/firmware_angle', 10)
        self.pub_tactile = {r: self.create_publisher(Float32MultiArray, hw_tactile_topic(r), 10)
                            for r in TACTILE_REGIONS}
        self.pub_proximal = {f: self.create_publisher(Float32MultiArray, hw_tactile_proximal_topic(f), 10)
                             for f in PROXIMAL_FINGERS}
        for finger in FINGERS:
            self.create_subscription(Float64, hw_target_topic(finger),
                                     lambda msg, f=finger: self.on_target(f, msg), 10)
        self.create_subscription(Bool, HW_MOTION_ENABLE_TOPIC, self.on_motion_enable, 10)
        self.pub_motion_state = self.create_publisher(String, HW_MOTION_STATE_TOPIC, 10)
        self.create_timer(0.5, self.publish_motion_state)
        self.create_timer(1.0 / rate, self.poll)
        self.get_logger().info(
            f"motion={'ENABLED' if self.enable_motion else 'READ ONLY'}; mode={self.mode}; "
            f"rate={rate:g}Hz; "
            f"tactile={TACTILE_MODELS[self.tactile_model][0] if self.tactile_model is not None else 'off'}"
        )

    def read(self, address, count=1):
        result = self.client.read_holding_registers(address=address, count=count, slave=SLAVE_ID)
        if result is None or result.isError() or len(result.registers) != count:
            raise RuntimeError(f'Modbus read error at {address}: {result}')
        return list(result.registers)

    def write_block(self, address, values):
        result = self.client.write_registers(address=address, values=values, slave=SLAVE_ID)
        if result is None or result.isError():
            raise RuntimeError(f'Modbus write error at {address}: {result}')

    def write(self, address, value):
        result = self.client.write_register(address=address, value=value, slave=SLAVE_ID)
        if result is None or result.isError():
            raise RuntimeError(f'Modbus write error at {address}: {result}')

    def detect_tactile(self, requested):
        if requested == 'off':
            return None
        if requested != 'auto':
            return {TACTILE_DOT_MATRIX: 0, TACTILE_FORCE3D: 1}[requested]
        try:
            sub_model = (self.read(MANU_DATA_ADDR)[0] >> 8) & 0xFF
        except RuntimeError as exc:
            self.get_logger().warning(f'tactile type detection failed, tactile off: {exc}')
            return None
        if sub_model not in TACTILE_MODELS:
            self.get_logger().warning(f'unknown tactile sub_model={sub_model}, tactile off')
            return None
        return sub_model

    def on_motion_enable(self, msg):
        if msg.data == self.running:
            return
        self.running = msg.data
        if not self.running:
            self.hold_pending = True
        self.get_logger().warning('START: following targets' if self.running else 'STOP: holding pose')
        self.publish_motion_state()

    def publish_motion_state(self):
        state = ('read_only' if not self.enable_motion or not self.active
                 else 'running' if self.running else 'stopped')
        self.pub_motion_state.publish(String(data=state))

    def on_target(self, finger, msg):
        if not self.running:
            return  # STOP: hold the pose
        if not self.enable_motion or not self.active:
            self.get_logger().warning('read-only or comm error: target not sent',
                                      throttle_duration_sec=2.0)
            return
        if not is_finite(msg.data):
            return
        lo, hi = FINGER_LIMITS[finger]
        if self.mode == 'position':
            fraction = max(0.0, min(1.0, (msg.data - lo) / (hi - lo)))
            value = round(fraction * MAX_LOGICAL)
        else:
            value = round(math.degrees(msg.data) * 100) & 0xFFFF
        k = FIRMWARE_INDEX[finger]
        if self.targets[k] != value:
            self.targets[k] = value
            self.dirty = True

    def position_rad(self, finger, logical):
        lo, hi = FINGER_LIMITS[finger]
        return lo + logical / MAX_LOGICAL * (hi - lo)

    def poll(self):
        if not self.active:
            return
        try:
            if self.hold_pending and self.enable_motion:
                # STOP: the measured pose becomes the target, so the fingers stay where they are
                self.hold_pending = False
                self.targets = self.read(POS_ADDR if self.mode == 'position' else ANGLE_ADDR, 6)
                self.dirty = True
            if self.dirty and self.enable_motion:
                self.dirty = False
                self.write_block(self.target_addr, self.targets)
            # one read per register group: the real firmware rejects a span that crosses the
            # unused finger slots 6..9 between them (IllegalAddress)
            blocks = {POS_ADDR: self.read(POS_ADDR, 6), FORCE_ADDR: self.read(FORCE_ADDR, 6)}
            if self.read_angle:
                try:
                    blocks[ANGLE_ADDR] = self.read(ANGLE_ADDR, 6)
                except RuntimeError as exc:
                    if self.mode == 'angle':
                        raise
                    self.read_angle = False  # reference only: stop asking
                    self.get_logger().warning(f'firmware angle not readable, skipped: {exc}')
            tactile = self.read_tactile() if self.tactile_model is not None else None
            self.failures = 0
        except RuntimeError as exc:
            self.failures += 1
            if self.failures >= MAX_READ_FAILURES:
                self.active = False
                self.get_logger().error(f'Modbus failed {self.failures}x; backend stopped: {exc}')
            else:
                self.get_logger().warning(f'Modbus failed ({self.failures}): {exc}')
            return

        def reg(addr, finger):
            return blocks[addr][FIRMWARE_INDEX[finger]]

        stamp = self.get_clock().now().to_msg()
        fw_angle = ([math.radians(to_int16(reg(ANGLE_ADDR, f)) / 100.0) for f in FINGERS]
                    if ANGLE_ADDR in blocks else None)
        state = JointState()
        state.header.stamp = stamp
        state.name = list(FINGERS)
        if self.mode == 'position':
            state.position = [self.position_rad(f, reg(POS_ADDR, f)) for f in FINGERS]
        else:
            state.position = fw_angle
        state.effort = [reg(FORCE_ADDR, f) / 1000.0 for f in FINGERS]
        self.pub_state.publish(state)

        if fw_angle is not None:
            fw = JointState()
            fw.header.stamp = stamp
            fw.name = list(FINGERS)
            fw.position = fw_angle
            self.pub_fw_angle.publish(fw)

        if tactile is not None:
            kind = TACTILE_MODELS[self.tactile_model][0]
            for region, regs in zip(TACTILE_REGIONS, tactile):
                if region == 'palm':
                    msg = make_tactile_msg(TACTILE_DOT_MATRIX, palm_values(decode_dot_matrix(regs)),
                                           grid=PALM_GRID)
                    self.pub_tactile[region].publish(msg)
                elif kind == TACTILE_FORCE3D:
                    tip, proximal = split_force3d(decode_force3d(regs))
                    self.pub_tactile[region].publish(make_tactile_msg(TACTILE_FORCE3D, tip))
                    if proximal is not None and region in self.pub_proximal:
                        self.pub_proximal[region].publish(make_tactile_msg(TACTILE_NORMAL, proximal))
                else:
                    self.pub_tactile[region].publish(
                        make_tactile_msg(TACTILE_DOT_MATRIX, decode_dot_matrix(regs)))

    def read_tactile(self):
        """Raw registers per region (TACTILE_REGIONS order)."""
        _, counts = TACTILE_MODELS[self.tactile_model]
        return [self.read(FORCE_EX_ADDR + region * FORCE_GROUP_SIZE, count)
                for region, count in enumerate(counts)]

    def destroy_node(self):
        self.client.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = RealBackend()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if rclpy.ok():  # otherwise Humble's take_message raced with Ctrl+C
            print(f'hand_serial failed: {exc}')
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
