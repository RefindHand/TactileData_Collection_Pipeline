# hand_bringup — 전체 파이프라인 실행

팀 노드를 한 번에 실행하는 launch 패키지다. 노드 코드는 없고 각 패키지의 노드를 묶기만 한다.

```text
웹캠 (hand_tracking) ──/hand_landmarks──▶ 자세 명령 (pose_command) ──/finger/<f>/command──▶ 손가락 노드 ×6 (finger_node)
                                                                                              │  ▲
                                                                                              ▼  │
                                                                      하드웨어 백엔드: MuJoCo (hand_sim) 또는 실물 (hand_serial)
피드백 노드 (feedback_node) ◀──/finger/<f>/state, /finger/<f>/tactile──┘
로봇 핸드 카메라 (gripper_camera, 가상 Gemini 305) ◀──/hand_sim/full_joint_states, /hand_hw/state (영상 속 손이 실제 자세를 따라감)
```

카메라는 현재 MuJoCo 가상 카메라이며 실물 Gemini 305 연결은 아직 없다. 영상 확인: `ros2 run rqt_image_view rqt_image_view /gripper_camera/color/image_raw`

## 실행

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# MuJoCo 시뮬레이션 + 이 PC의 웹캠 (Ubuntu 데스크탑, AGX Orin)
ros2 launch hand_bringup pipeline.launch.py

# 웹캠은 Windows에서 실행 (WSL): Windows에서 windows_sender/Py_webCamReader.py를 따로 실행
ros2 launch hand_bringup pipeline.launch.py team_sender:=remote

# 실물 AP002: 먼저 읽기 전용으로 값 확인 → 동작 허용
ros2 launch hand_bringup pipeline.launch.py backend:=real
ros2 launch hand_bringup pipeline.launch.py backend:=real enable_motion:=true
```

1. 웹캠 창이 뜨면 손을 보여 주고 **한 번 끝까지 쥐었다 펴고 엄지를 크게 돌린다** (자세 명령 노드 자동 보정).
2. 이후 손 동작을 로봇 핸드가 따라간다. 보정 초기화: `ros2 topic pub --once /pose_command/reset_calibration std_msgs/msg/Empty "{}"`
3. 종료: 웹캠 창에서 `Q`, 터미널에서 `Ctrl+C`.

## launch 인자

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `backend` | `sim` | `sim` (MuJoCo) / `real` (실물 AP002) |
| `viewer` | `true` | MuJoCo 창 표시 (sim) |
| `test_object` | `ball` | `ball` / `none`: 손바닥 앞 고정 공 (sim) |
| `port` | `/dev/ttyUSB0` | 시리얼 포트 (real) |
| `enable_motion` | `false` | 실물에 목표값 쓰기 허용 (real) |
| `real_mode` | `position` | `position` (검증된 위치 레지스터) / `angle` (미검증) |
| `log_period` | `1.0` | 손가락 노드 각도·힘 출력 주기 (초, 0이면 끔) |
| `team_sender` | `local` | `local`: 이 PC에서 웹캠 프로그램 실행 / `remote`: 다른 PC(Windows)가 UDP 5005로 전송 |
| `camera_device` | `0` | 웹캠 번호 `/dev/videoN` (`team_sender:=local`) |
| `gripper_camera` | `true` | 가상 Gemini 305 카메라 실행 |
| `camera_mode` | `rgb` | 카메라 영상: `rgb` / `left` / `right` / `disparity` / `depth` (실행 중 변경: `ros2 param set /gripper_camera mode depth`) |

## 주의

- 웹캠에서 손이 0.5초 이상 사라지면 자세 명령 노드가 **모든 손가락을 펴는 명령**을 보낸다. 실물로 물체를 쥘 때 주의한다.
- 실물 시리얼 포트는 한 프로그램만 쓸 수 있다. `hardware_connection_test`의 프로그램과 동시에 실행하지 않는다.
- 각 노드의 토픽·준비물은 패키지별 README를 참고한다: [hand_tracking](../hand_tracking/README.md), [pose_command](../pose_command/README.md), [finger_node](../finger_node/README.md), [feedback_node](../feedback_node/README.md), [gripper_camera](../gripper_camera/README.md).
