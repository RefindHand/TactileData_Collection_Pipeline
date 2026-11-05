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
- 검지·중지·약지·소지 굽힘: MCP·PIP·DIP 굽힘 각도의 가중합 (0.35 / 0.45 / 0.20)
- 엄지 굽힘: MCP·IP 굽힘 각도의 평균
- 엄지 뿌리 회전: 손목·검지 뿌리·소지 뿌리로 만든 손바닥 좌표계에서 엄지 첫 마디(1→2번) 방향의 각도
  (굽힘·회전 계산은 `windows_sender/Py_webCamReader.py`와 동일)
- 자동 보정: 채널마다 **실제로 관찰된** 최소·최대 각도를 0.0~1.0 (0.0 = 펴짐)으로 정규화. 가장 펴진 자세가 정확히 0.0이 됨 (완전히 편 손도 MediaPipe로는 15~25° 굽힘으로 측정되기 때문). 손을 한 번 쥐기 전에는 최대값을 `최소 + DEFAULT_SPANS`로 둠. 시작 후 손을 한 번 끝까지 쥐었다 펴고 엄지를 돌리면 보정됨
- 보정 초기화: `ros2 topic pub --once /pose_command/reset_calibration std_msgs/msg/Empty "{}"`
- 측정 각도에 지수 이동 평균 필터를 적용해 떨림 완화
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
| `FILTER_ALPHA` | `0.7` | 측정 각도 필터의 새 입력 반영 비율 (클수록 빠르고 떨림 증가) |
| `DEFAULT_SPANS` | 손가락 `45°`, 엄지 굽힘 `30°`, 엄지 회전 없음 | 쥐어 보기 전 임시 범위 폭 (deg) |
| `MIN_RANGE` | `10.0` | 이보다 좁은 범위는 보정 전으로 보고 0.0 (펴짐) 출력 |
| `DEADBAND` | `5.0` | 양 끝 데드존 (deg). 가장 펴진·쥔 값에서 이 안쪽은 정확히 0.0 / 1.0 (MediaPipe 떨림 흡수) |
| `REVERSE_THUMB_ROOT` | `True` | 엄지 뿌리 명령 방향 반전 (웹캠 테스트로 반대 방향 확인) |

## 8. Limitations

- 자동 보정은 범위가 넓어지기만 하므로, MediaPipe가 손을 잘못 인식한 순간의 값이 범위를 넓힐 수 있습니다. 동작이 둔해지면 보정을 초기화합니다.
- 엄지 회전은 보정 전(범위 10° 미만)에는 0.0을 출력합니다.
- 손 크기와 카메라 방향에 따라 리타게팅 성능이 달라질 수 있습니다.
- 입력 복구 시 첫 번째 유효 프레임의 목표값으로 즉시 전환될 수 있습니다.
- 타임아웃 처리는 자세 명령 노드가 정상 실행 중일 때만 동작합니다.
- 시뮬레이터의 기본 명령값 `0.0`은 실제 하드웨어의 안전 자세를 보장하지 않습니다.
- 실제 ROHand 하드웨어와의 통합 및 안전성 검증은 별도로 필요합니다.

## 9. Integration Notes

전체 흐름: 웹캠(`windows_sender/Py_webCamReader.py` → UDP → `hand_tracking_node`) → `/hand_landmarks` → **pose_command_node** → `/finger/<finger>/command` → 손가락 노드 ×6 → 로봇 핸드

웹캠 노드는 MediaPipe world landmark를 매 프레임 `/hand_landmarks`로 발행합니다 (보정 불필요).

ROS 2 패키지 자체는 웹캠 영상 처리 및 UDP 통신 기능을 포함하지 않습니다.
