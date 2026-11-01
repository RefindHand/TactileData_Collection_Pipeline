# Pose Command Node

## 1. Overview

ROS 2 Humble 기반 ROHand AP002 로봇 핸드의 자세 명령 노드입니다.

MediaPipe에서 추출한 21개 손 관절 좌표를 입력받아 손가락의 굽힘 정도와 엄지의 움직임을 계산하고, 이를 MuJoCo 시뮬레이터에서 사용하는 6축 목표값으로 변환합니다.

현재 버전은 **시뮬레이션 전용**이며 실제 로봇 하드웨어 제어는 지원하지 않습니다.

## 2. Development Environment

- OS: Ubuntu 22.04 (WSL2)
- ROS 2: Humble
- Language: Python
- ROS 2 Package: `pose_command`
- Dependencies: `rclpy`, `std_msgs`

## 3. ROS 2 Topics

**Input**

- Topic: `/hand_landmarks`
- Type: `std_msgs/msg/Float32MultiArray`
- Data: 21개의 손 관절 × (x, y, z) = 총 63개 실수
- Landmark Order: MediaPipe Hand Landmarker 기준 0~20번

**Output**

- Topic: `/finger/<finger>/command` (`index`, `middle`, `ring`, `little`, `thumb_root`, `thumb`)
- Type: `std_msgs/msg/Float64`
- Data: 손가락 노드로 보내는 목표 각도 (rad). 0.0~1.0 목표값을 손가락별 범위(`finger_node.common.FINGER_LIMITS`, 예: 검지 0~1.45 rad)로 변환한 값

**Output (기존 시뮬레이터 호환)**

- Topic: `/rohand_sim/target`
- Type: `std_msgs/msg/Float64MultiArray`
- Data: 6개 액추에이터 목표값 (각각 0.0~1.0, 0.0 = 펴짐). `hardware_connection_test/rohand_sim_ros2.py`용

출력 배열 순서:

| Index | Actuator |
|---|---|
| 0 | Index finger |
| 1 | Middle finger |
| 2 | Ring finger |
| 3 | Little finger |
| 4 | Thumb root |
| 5 | Thumb flexion |

## 4. Main Features

- MediaPipe 손 관절 좌표 유효성 검사
- 검지·중지·약지·소지 PIP 관절 기반 굽힘 계산
- 엄지 IP 관절 기반 굽힘 계산
- 엄지 끝과 새끼손가락 뿌리 거리 기반 엄지 뿌리 움직임 근사
- 6축 목표값 정규화 (0.0~1.0)
- 저역통과 필터를 이용한 움직임 안정화
- 0.5초 입력 타임아웃 감지
- 입력 중단 시 시뮬레이터 기본 목표값 전송
- 손 재인식 시 명령 재개

## 5. Build

ROS 2 Humble이 설치되어 있어야 합니다.

손가락 범위를 `finger_node` 패키지에서 가져오므로 함께 빌드합니다.

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select finger_node pose_command
source install/setup.bash
```

## 6. Run

```bash
source ~/TactileData_Collection_Pipeline/install/setup.bash
ros2 run pose_command pose_command_node
```

입력 토픽 `/hand_landmarks`를 발행하는 웹캠 노드(`hand_tracking_node`)와, 출력을 받는 손가락 노드(`ros2 launch finger_node fingers.launch.py`)가 별도로 실행되어 있어야 합니다. 기존 MuJoCo 시뮬레이터(`hardware_connection_test/rohand_sim_ros2.py`)는 `/rohand_sim/target`으로 계속 사용할 수 있습니다.

## 7. Configuration

`pose_command_node.py` 상단의 설정값으로 동작을 조정할 수 있습니다.

| Parameter | Default | Description |
|---|---|---|
| `INPUT_TIMEOUT` | `0.5` | 입력 중단 판정 시간(초) |
| `FILTER_ALPHA` | `0.25` | 저역통과 필터의 새 입력 반영 비율 |
| `THUMB_OPEN_DISTANCE` | `1.8` | 엄지 뿌리 매핑을 위한 초기 거리 기준 |
| `THUMB_CLOSED_DISTANCE` | `0.85` | 엄지 뿌리 매핑을 위한 초기 거리 기준 |
| `REVERSE_THUMB_ROOT` | `False` | 엄지 뿌리 명령 방향 반전 |

## 8. Limitations

- 엄지 뿌리 회전은 관절 거리 기반 근사값이며, 엄지 끝마디 굽힘에 영향을 받을 수 있습니다.
- 손 크기와 카메라 방향에 따라 리타게팅 성능이 달라질 수 있습니다.
- 입력 복구 시 첫 번째 유효 프레임의 목표값으로 즉시 전환될 수 있습니다.
- 타임아웃 처리는 자세 명령 노드가 정상 실행 중일 때만 동작합니다.
- 시뮬레이터의 기본 명령값 `0.0`은 실제 하드웨어의 안전 자세를 보장하지 않습니다.
- 실제 ROHand 하드웨어와의 통합 및 안전성 검증은 별도로 필요합니다.

## 9. Integration Notes

전체 흐름: 웹캠(`windows_sender/Py_webCamReader.py` → UDP → `hand_tracking_node`) → `/hand_landmarks` → **pose_command_node** → `/finger/<finger>/command` → 손가락 노드 ×6 → 로봇 핸드

웹캠 노드는 MediaPipe world landmark를 매 프레임 `/hand_landmarks`로 발행합니다 (보정 불필요).

ROS 2 패키지 자체는 웹캠 영상 처리 및 UDP 통신 기능을 포함하지 않습니다.
