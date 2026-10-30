#!/usr/bin/env python3
"""ROHand AP002: ROS 2 <-> Modbus-RTU bridge for six physical axes.

Read-only by default. --enable-motion + ENABLE permits user-initiated goals.
Input topic: /rohand_real/step (std_msgs/String)
  '1 50'  = index absolute 50% logical position
  '1 +10' = index +10 percentage points from measured position
  '1 -10' = index -10 percentage points from measured position
  'all 0' = all six fingers absolute 0%
No movement is sent automatically on startup/shutdown.
These percentages refer to device logical range [0, 65535], not joint angles.
"""

import argparse
import math
import sys

from pymodbus.client import ModbusSerialClient
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray, String

# Console order = original MuJoCo UI order; register index = ROH Gen2 firmware order
JOINTS = (("index", 1), ("middle", 2), ("ring", 3),
          ("little", 4), ("thumb_root", 5), ("thumb", 0))
SLAVE_ID = 2
PROTO_ADDR = 1000
HW_ADDR = 1003
STATUS_ADDR = 1085
TARGET_ADDR = 1135
POSITION_ADDR = 1145
MAX_LOGICAL = 65535
STUCK_STATUS = 5


def read_regs(client, address, count=1):
    result = client.read_holding_registers(address=address, count=count, slave=SLAVE_ID)
    if result is None or result.isError() or len(result.registers) != count:
        raise RuntimeError(f"Modbus read error at {address}: {result}")
    return list(result.registers)


def as_percent(value):
    return value * 100.0 / MAX_LOGICAL


def to_register(percent):
    return round(percent * MAX_LOGICAL / 100.0)


def parse_command(raw, positions):
    """Return list of (register index, register value); never writes anything."""
    parts = raw.strip().split()
    if len(parts) != 2:
        raise ValueError("사용법: '1 50', '1 +10', '1 -10' 또는 'all 0'")

    key, value_str = parts
    try:
        value = float(value_str)
    except ValueError as exc:
        raise ValueError("위치는 숫자여야 합니다") from exc
    if not math.isfinite(value):
        raise ValueError("숫자는 유한한 값이어야 합니다")

    if key.lower() == "all":
        if value_str.startswith(('+', '-')):
            raise ValueError("all은 절대 위치만 허용합니다: all 0")
        if not 0 <= value <= 100:
            raise ValueError("절대 위치는 0~100%만 허용합니다")
        return [(i, to_register(value)) for i in range(6)]

    try:
        number = int(key)
    except ValueError as exc:
        raise ValueError("손가락 번호는 1~6 또는 all입니다") from exc
    if not 1 <= number <= 6:
        raise ValueError("손가락 번호는 1~6입니다")
    name, i = JOINTS[number - 1]

    relative = value_str.startswith(('+', '-'))
    target_pct = as_percent(positions[i]) + value if relative else value
    if not 0 <= target_pct <= 100:
        raise ValueError(f"{name} 명령 목표 {target_pct:.2f}%가 0~100% 범위를 벗어남")
    return [(i, to_register(target_pct))]


class RealHandNode(Node):
    def __init__(self, client, allow_motion):
        super().__init__('rohand_ap002_real')
        self.client = client
        self.allow_motion = allow_motion
        self.active = True
        self.positions = read_regs(client, POSITION_ADDR, 6)
        self.targets = read_regs(client, TARGET_ADDR, 6)
        self.status = read_regs(client, STATUS_ADDR, 6)
        self.pub_pos = self.create_publisher(Float64MultiArray, '/rohand_real/positions', 10)
        self.pub_target = self.create_publisher(Float64MultiArray, '/rohand_real/targets', 10)
        self.pub_status = self.create_publisher(Int32MultiArray, '/rohand_real/status', 10)
        self.create_subscription(String, '/rohand_real/step', self.on_command, 10)
        self.create_timer(1.0, self.poll)
        self.get_logger().info(
            f"AP002 ROS2 /dev/ttyUSB bridge; motion={'ENABLED' if allow_motion else 'READ ONLY'}; "
            "topic=/rohand_real/step"
        )
        self.publish_feedback()

    def publish_feedback(self):
        msg = Float64MultiArray()
        msg.data = [self.positions[i] / MAX_LOGICAL for _, i in JOINTS]
        self.pub_pos.publish(msg)
        msg2 = Float64MultiArray()
        msg2.data = [self.targets[i] / MAX_LOGICAL for _, i in JOINTS]
        self.pub_target.publish(msg2)
        msg3 = Int32MultiArray()
        msg3.data = [self.status[i] for _, i in JOINTS]
        self.pub_status.publish(msg3)
        for number, (name, i) in enumerate(JOINTS, 1):
            self.get_logger().info(
                f"{number} {name:10} actual={as_percent(self.positions[i]):6.2f}% "
                f"target={as_percent(self.targets[i]):6.2f}% status={self.status[i]}"
            )

    def refresh(self):
        self.positions = read_regs(self.client, POSITION_ADDR, 6)
        self.targets = read_regs(self.client, TARGET_ADDR, 6)
        self.status = read_regs(self.client, STATUS_ADDR, 6)

    def poll(self):
        if not self.active:
            return
        try:
            self.refresh()
            self.publish_feedback()
        except Exception as exc:
            self.active = False
            self.get_logger().error(f"Modbus read failed; commands disabled: {exc}")

    def on_command(self, message):
        if not self.allow_motion or not self.active:
            self.get_logger().warning("읽기 전용 또는 통신 오류 상태: 명령을 전송하지 않았습니다")
            return
        try:
            self.refresh()  # relative moves use fresh measured positions
            writes = parse_command(message.data, self.positions)
            for i, _ in writes:
                if self.status[i] == STUCK_STATUS and dict(writes)[i] >= self.positions[i]:
                    raise RuntimeError(
                        f'모터 {i} status=5 (STUCK): 추가 굽힘 명령 거부. 펴는 방향은 허용'
                    )
            # all: one Modbus write of six adjacent target registers; one finger: single register
            if len(writes) == 6:
                vals = [v for _, v in writes]  # in physical register order 0..5
                res = self.client.write_registers(address=TARGET_ADDR, values=vals, slave=SLAVE_ID)
                if res is None or res.isError():
                    raise RuntimeError(f"write_registers failure: {res}")
                confirm = read_regs(self.client, TARGET_ADDR, 6)
                self.get_logger().warning(
                    f"실물 전체 관절 명령: {message.data!r}; readback={confirm}"
                )
            else:
                i, val = writes[0]
                res = self.client.write_register(address=TARGET_ADDR + i, value=val, slave=SLAVE_ID)
                if res is None or res.isError():
                    raise RuntimeError(f"write_register failure: {res}")
                actual_target = read_regs(self.client, TARGET_ADDR + i)[0]
                joint_name = next(n for n, j in JOINTS if j == i)
                self.get_logger().warning(
                    f"실물 {joint_name} 명령 {message.data!r}: "
                    f"requested={val} ({as_percent(val):.2f}%), target_readback={actual_target}"
                )
        except Exception as exc:
            self.get_logger().error(f"명령 거부 또는 통신 실패: {exc}")


def main():
    parser = argparse.ArgumentParser(description='ROHand AP002 ROS2 physical controller')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--enable-motion', action='store_true')
    args = parser.parse_args()

    if args.enable_motion:
        print('실물 6축 제어 모드: 명령 시 0~100% 논리 위치를 전송할 수 있습니다.')
        print('손 주변 장애물을 치우고 정격 전원·장치 고정을 확인하세요.')
        if input('실제 모터 제어를 허용하려면 ENABLE 입력 > ').strip() != 'ENABLE':
            sys.exit('승인되지 않았습니다')

    client = ModbusSerialClient(port=args.port, baudrate=115200, timeout=1)
    if not client.connect():
        sys.exit(f"직렬 포트 연결 실패: {args.port}")
    node = None
    try:
        proto = read_regs(client, PROTO_ADDR)[0]
        hardware = read_regs(client, HW_ADDR)[0]
        print(f"Modbus 장치 연결: protocol=0x{proto:04X}, hardware=0x{hardware:04X}")
        if proto >> 8 != 2 or hardware != 0x2001:
            raise RuntimeError("이전에 확인된 Gen2 AP 하드웨어 식별값과 다릅니다: 명령 금지")
        rclpy.init()
        node = RealHandNode(client, args.enable_motion)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"실행 실패: {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        client.close()


if __name__ == '__main__':
    main()
