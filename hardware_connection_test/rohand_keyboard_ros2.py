#!/usr/bin/env python3
"""Interactive ROS2 control of the AP002 *simulation only*.

Input examples:
  1 50     -> index finger at 50% of simulation control range
  5 100    -> thumb root at 100%
  all 0    -> set all six targets to zero
  show     -> print current targets
  q        -> quit
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

ACTUATORS = ("index_finger", "middle_finger", "ring_finger",
             "little_finger", "thumb_root", "thumb")
TOPIC = "/rohand_sim/target"


def print_targets(targets):
    for i, (name, value) in enumerate(zip(ACTUATORS, targets), 1):
        print(f"  {i}. {name:<16}: {value * 100:5.1f}%")


def main():
    rclpy.init()
    node = Node("rohand_keyboard")
    pub = node.create_publisher(Float64MultiArray, TOPIC, 10)
    targets = [0.0] * 6
    print("\nAP002 MuJoCo ROS2 키보드 제어 (실물 로봇 제어 아님)")
    print("손가락 번호(1~6)와 퍼센트(0~100)를 입력: 예) 1 50")
    print("all 0 / all 100 / show / q")
    print_targets(targets)
    try:
        while rclpy.ok():
            try:
                line = input("명령 > ").strip().lower()
            except EOFError:
                break
            if line in {"q", "quit", "exit"}:
                break
            if line == "show":
                print_targets(targets)
                continue
            parts = line.split()
            if len(parts) != 2:
                print("형식: 1 50 또는 all 0")
                continue
            key, percentage = parts
            try:
                value = float(percentage)
                if not 0 <= value <= 100:
                    raise ValueError
                if key == "all":
                    new_targets = [value / 100.0] * 6
                else:
                    index = int(key)
                    if not 1 <= index <= 6:
                        raise ValueError
                    new_targets = targets.copy()
                    new_targets[index - 1] = value / 100.0
            except ValueError:
                print("손가락 번호는 1~6, 퍼센트는 0~100이야.")
                continue

            targets = new_targets
            if pub.get_subscription_count() == 0:
                print("[알림] 현재 시뮬레이터 구독자가 보이지 않습니다. 시뮬레이터가 켜져 있는지 확인하세요.")
            msg = Float64MultiArray()
            msg.data = list(targets)
            # Send a few times to be robust while ROS2 peer discovery completes.
            import time
            for _ in range(3):
                pub.publish(msg)
                time.sleep(0.08)
            print_targets(targets)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
