# finger_node — 액추에이터별 각도·힘 출력 노드

ROHand AP002의 액추에이터 6개마다 하나씩 실행하는 노드다. 자세 명령 노드가 보낸 목표 각도를 하드웨어 백엔드로 전달하고, 백엔드가 측정한 **각도와 힘**을 손가락별 토픽으로 출력한다.

손가락 이름(`finger` 파라미터): `index, middle, ring, little, thumb_root, thumb`

## 토픽

| 토픽 | 타입 | 방향 | 내용 |
| --- | --- | --- | --- |
| `/finger/<f>/command` | `std_msgs/Float64` | 구독 | 목표 각도 (rad). 범위를 벗어나면 잘라서 전달 |
| `/finger/<f>/state` | `sensor_msgs/JointState` | 발행 | 관절 1개: `position` 각도 rad, `velocity` rad/s, `effort` 힘 N |
| `/finger/<f>/angle_force` | `std_msgs/Float64MultiArray` | 발행 | `[각도(deg), 힘(N), 목표 각도(deg)]` 확인용 |
| `/finger/<f>/tactile` | `std_msgs/Float32MultiArray` | 발행 | 손끝 촉각 (`thumb_root`는 없음), 백엔드 값을 그대로 전달 |
| `/hand_hw/<f>/target` | `std_msgs/Float64` | 발행 | 백엔드로 보내는 목표 각도 (rad) |
| `/hand_hw/state` | `sensor_msgs/JointState` | 구독 | 백엔드가 측정한 6개 액추에이터 상태 |
| `/hand_hw/tactile/<f>` | `std_msgs/Float32MultiArray` | 구독 | 백엔드 촉각 |

- 각도는 0 = 펴짐, 값이 클수록 굽힘(엄지는 회전).
- 실물 손은 펌웨어 `ROH_FINGER_FORCE`(mN)를 N으로 바꾼 값, 시뮬레이션은 MuJoCo 액추에이터 힘이다.
- 실물 백엔드는 속도를 주지 않으므로, 연속된 각도 차이로 속도를 계산한다.

## 파라미터

| 이름 | 기본값 | 설명 |
| --- | --- | --- |
| `finger` | `index` | 담당 손가락 |
| `min_angle`, `max_angle` | 손가락별 MuJoCo 범위 | 목표 각도 제한 (rad) |
| `log_period` | `1.0` | 터미널에 각도·힘을 출력하는 주기 (초, 0이면 끔) |
| `hw_state_topic` | `/hand_hw/state` | 백엔드 상태 토픽 |

## 빌드·실행

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select finger_node
source install/setup.bash

# 손가락 하나
ros2 run finger_node finger_node --ros-args -p finger:=index -r __node:=finger_index

# 확인
ros2 topic echo /finger/index/angle_force
ros2 topic pub --once /finger/index/command std_msgs/msg/Float64 "{data: 1.0}"
```

백엔드(`hand_sim` 또는 `hand_serial`)가 함께 실행 중이어야 값이 나온다.
