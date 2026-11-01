# Feedback Node

ROS 2 Humble 기반 ROHand AP002 피드백 노드입니다. 손가락 노드 6개가 발행하는 **관절 각도·구동력·촉각**과 손바닥 촉각을 모아, 데이터 수집과 통합에 쓰기 좋은 형태로 다시 발행합니다.

## 1. 기능

- 손가락 6개 상태를 하나의 타임스탬프 메시지로 묶어 발행 (30 Hz)
- 자세 명령과 실측 각도의 차이(추종 오차) 계산
- 부위별 촉각 합산 및 접촉 판정 (손가락 5개 + 손바닥)
- 입력별 상태 진단: 정상 / 끊김(stale) / 미수신, 1초마다 로그 출력
- 입력이 끊기면 해당 값은 NaN으로 발행

## 2. 입력 토픽 (손가락 노드와 협의 필요)

손가락 이름과 순서: `index, middle, ring, little, thumb_root, thumb` (키보드 도구의 1~6번과 동일)

| 토픽 | 타입 | 내용 |
| --- | --- | --- |
| `/finger/<손가락>/state` | `sensor_msgs/JointState` | 관절 1개: `position` 각도(rad), `velocity`, `effort` 구동력 |
| `/finger/<손가락>/command` | `std_msgs/Float64` | 목표 각도(rad), 추종 오차 계산용 |
| `/finger/<손가락>/tactile` | `std_msgs/Float32MultiArray` | 손끝 촉각 (`thumb_root` 제외) |
| `/hand_hw/tactile/palm` | `std_msgs/Float32MultiArray` | 손바닥 촉각 |

촉각 배열의 형식은 `layout.dim[0].label`로 구분합니다.

| label | `data` 구성 |
| --- | --- |
| `force3d` | 측정점마다 `[법선력, 전단력, 전단 방향(deg)]` 반복 (3D 힘 센서) |
| `dot_matrix` | 점별 압력 0~255 (점 배열 센서, label이 없을 때 기본값) |

## 3. 출력 토픽

| 토픽 | 타입 | 내용 |
| --- | --- | --- |
| `/feedback/joint_states` | `sensor_msgs/JointState` | 손가락 6개 각도·속도·구동력 |
| `/feedback/tracking_error` | `std_msgs/Float64MultiArray` | 명령 − 실측 각도(rad), 6개 |
| `/feedback/tactile_summary` | `std_msgs/Float64MultiArray` | 부위별 압력 합 `[thumb, index, middle, ring, little, palm]` |
| `/feedback/contact` | `std_msgs/UInt8MultiArray` | 부위별 접촉 여부 1/0 |
| `/feedback/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 손가락·촉각 입력별 상태 (1 Hz) |

## 4. 빌드 및 실행

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select feedback_node
source install/setup.bash
ros2 run feedback_node feedback_node
```

입력 없이 실행하면 모든 입력이 `no data`로 표시됩니다. 손가락 노드 없이 확인하려면 다른 터미널에서 가짜 값을 보냅니다.

```bash
ros2 topic pub -r 10 /finger/index/state sensor_msgs/msg/JointState "{name: [index], position: [0.5], velocity: [0.0], effort: [0.2]}"
ros2 topic pub -r 10 /finger/index/tactile std_msgs/msg/Float32MultiArray "{layout: {dim: [{label: force3d, size: 3, stride: 3}]}, data: [2.0, 0.3, 45.0]}"
ros2 topic echo /feedback/contact
```

## 5. 파라미터

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `rate` | `30.0` | 출력 주기 (Hz) |
| `stale_timeout` | `0.5` | 입력이 이 시간(초) 이상 없으면 끊김으로 판정 |
| `contact_threshold_force3d` | `0.5` | `force3d` 법선력 합의 접촉 기준 |
| `contact_threshold_dot_matrix` | `50.0` | `dot_matrix` 압력 합의 접촉 기준 |
| `finger_topic_prefix` | `/finger` | 손가락 입력 토픽 접두사 |
| `palm_tactile_topic` | `/hand_hw/tactile/palm` | 손바닥 촉각 토픽 |

예: `ros2 run feedback_node feedback_node --ros-args -p stale_timeout:=1.0`

## 6. 한계 및 통합 시 확인 사항

- 입력 토픽 이름·메시지 타입은 손가락 노드 담당자와 아직 확정하지 않은 **제안안**입니다. 다르게 정해지면 파라미터나 ROS 2 remap으로 맞추거나 코드를 수정합니다.
- 현재 자세 명령 노드는 `/rohand_sim/target` (0~1 정규화 6개)을 발행하므로, 추종 오차를 쓰려면 rad 단위 손가락별 명령과의 변환을 통합 단계에서 정해야 합니다.
- 접촉 기준값은 임시값이며 실물 촉각 센서의 원시값 범위에 맞춰 조정해야 합니다.
- 촉각 데이터 해석(미끄러짐·쥐는 세기 등)과 데이터 기록은 아직 포함하지 않습니다.
