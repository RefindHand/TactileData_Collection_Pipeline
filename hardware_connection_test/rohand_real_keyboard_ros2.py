#!/usr/bin/env python3
"""ROS2 terminal for AP002 real hand, six joints, logical 0..100% position."""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node('rohand_real_keyboard')
    publisher = node.create_publisher(String, '/rohand_real/step', 10)
    print('=== ROHand AP002 실물 6축 제어 ===')
    print('1 검지, 2 중지, 3 약지, 4 소지, 5 엄지 뿌리, 6 엄지 굽힘')
    print('1 50 = 검지 절대 50%; 1 +10 / 1 -10 = 현재 위치 기준 상대 이동')
    print('all 0 = 6축 모두 절대 0%; q = 종료')
    try:
        while rclpy.ok():
            try:
                line = input('실물 명령 > ').strip()
            except EOFError:
                break
            if line.lower() in ('q', 'quit', 'exit'):
                break
            parts = line.split()
            if len(parts) != 2:
                print('형식: 1 50 / 1 +10 / 1 -10 / all 0')
                continue
            key, number = parts
            if key.lower() != 'all' and key not in ('1','2','3','4','5','6'):
                print('번호는 1~6 또는 all만 허용합니다')
                continue
            try:
                val = float(number)
            except ValueError:
                print('숫자 형식 오류')
                continue
            if not math.isfinite(val):
                print('유한한 숫자만 허용합니다')
                continue
            if key.lower() == 'all' and (number.startswith(('+', '-')) or not 0 <= val <= 100):
                print('all은 절대 위치 0~100%만 가능합니다 (예: all 0)')
                continue
            if key.lower() != 'all' and not number.startswith(('+', '-')) and not 0 <= val <= 100:
                print('절대 위치는 0~100%만 가능합니다')
                continue
            if key.lower() != 'all' and number.startswith(('+', '-')) and not -100 <= val <= 100:
                print('상대 이동량은 -100~+100%p만 가능합니다')
                continue
            rclpy.spin_once(node, timeout_sec=0.1)
            if publisher.get_subscription_count() == 0:
                print('구독 중인 브리지 노드가 없어 전송하지 않았습니다')
                continue
            msg = String()
            msg.data = line
            publisher.publish(msg)
            print('ROS2 명령 전송. 브리지 로그에서 결과를 확인하세요.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
