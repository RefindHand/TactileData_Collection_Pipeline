#!/usr/bin/env python3
"""OYMotion AP002 (right) MuJoCo viewer controlled by ROS 2.

Subscribe: /rohand_sim/target (std_msgs/Float64MultiArray)
  Six normalized values in [0, 1] in actuator order.
Publish: /rohand_sim/joint_states (sensor_msgs/JointState)
  The six actuator-driven joints in their native MuJoCo coordinates.

This simulator node NEVER opens a serial port or talks to real hardware.
"""
import argparse
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

ACTUATORS = (
    "index_finger",
    "middle_finger",
    "ring_finger",
    "little_finger",
    "thumb_root",
    "thumb",
)
COMMAND_TOPIC = "/rohand_sim/target"
STATE_TOPIC = "/rohand_sim/joint_states"
DEFAULT_XML = (
    Path(__file__).resolve().parents[1]
    / "external/rohand_mujoco/AP002/model/rohand_gen2_AP002_right.xml"
)


class AP002Simulator(Node):
    def __init__(self, model, data):
        super().__init__("rohand_ap002_mujoco")
        self.model = model
        self.data = data
        self.actuator_ids = []
        self.joint_ids = []
        self.joint_names = []
        self.target = [0.0] * len(ACTUATORS)

        for name in ACTUATORS:
            idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if idx == -1:
                raise ValueError(f"AP002 액추에이터를 찾을 수 없습니다: {name}")
            self.actuator_ids.append(idx)
            joint_id = int(model.actuator_trnid[idx, 0])
            if joint_id < 0:
                raise ValueError(f"{name}: 구동 관절이 연결되지 않았습니다")
            self.joint_ids.append(joint_id)
            self.joint_names.append(
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            )

        self.create_subscription(Float64MultiArray, COMMAND_TOPIC, self.on_command, 10)
        self.state_pub = self.create_publisher(JointState, STATE_TOPIC, 10)
        self.create_timer(0.1, self.publish_state)
        self.get_logger().info(f"명령 수신: {COMMAND_TOPIC} (0~1 정규화 값 6개)")
        self.get_logger().info(f"현재 상태: {STATE_TOPIC}")
        self.get_logger().info("실물 로봇 핸드와의 통신은 수행하지 않습니다.")

    def on_command(self, msg):
        if len(msg.data) != len(ACTUATORS):
            self.get_logger().warning("손가락 6개 목표값이 필요합니다. 예: [0, 0, 0, 0, 0, 0]")
            return
        import math
        values = list(msg.data)
        if any(not math.isfinite(v) or v < 0.0 or v > 1.0 for v in values):
            self.get_logger().warning("목표값은 각각 0.0 ~ 1.0 범위여야 합니다.")
            return
        self.target = values
        self.get_logger().info("목표 변경: " + ", ".join(
            f"{name}={value:.2f}" for name, value in zip(ACTUATORS, values)
        ))

    def apply_target(self):
        # Mapping normalized 0..1 to official AP002 MJCF ctrlrange.
        for actuator_id, value in zip(self.actuator_ids, self.target):
            lo, hi = self.model.actuator_ctrlrange[actuator_id]
            self.data.ctrl[actuator_id] = float(lo + (hi - lo) * value)

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [
            float(self.data.qpos[int(self.model.jnt_qposadr[jid])])
            for jid in self.joint_ids
        ]
        # Positions are MuJoCo joint coordinates (radians for hinges).
        self.state_pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="ROS2 AP002 MuJoCo test")
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML,
                        help="AP002 오른손 MuJoCo XML 파일")
    args = parser.parse_args()
    xml_path = args.xml.expanduser().resolve()
    if not xml_path.is_file():
        raise SystemExit(f"모델 파일이 없습니다: {xml_path}")

    rclpy.init()
    node = None
    try:
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        data = mujoco.MjData(model)
        node = AP002Simulator(model, data)
        node.apply_target()
        steps_per_frame = max(1, round((1.0 / 60.0) / float(model.opt.timestep)))
        frame_period = 1.0 / 60.0
        print("MuJoCo 창을 닫거나 Ctrl+C로 종료할 수 있습니다.")
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and rclpy.ok():
                started = time.monotonic()
                rclpy.spin_once(node, timeout_sec=0)
                node.apply_target()
                for _ in range(steps_per_frame):
                    mujoco.mj_step(model, data)
                viewer.sync()
                remaining = frame_period - (time.monotonic() - started)
                if remaining > 0:
                    time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
