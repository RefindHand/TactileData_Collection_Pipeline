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
| `hand_camera` | `virtual` | 로봇 핸드 카메라: `virtual` (MuJoCo 가상 Gemini 305) / `d405` (실물 RealSense D405, 현재 그리퍼 카메라) / `d415` (실물 RealSense D415, 이전 카메라) / `none` |
| `hand_camera_rotation` | `auto` | GUI 표시 회전(반시계, 도): `auto` (가상 90, D405·D415 0) / `0` / `90` / `180` / `270`. 녹화는 원본 방향 |
| `gripper_camera` | `true` | 가상 Gemini 305 카메라 실행 (`hand_camera:=virtual`일 때) |
| `camera_mode` | `rgb` | 카메라 영상: `rgb` / `left` / `right` / `disparity` / `depth` (실행 중 변경: `ros2 param set /gripper_camera mode depth`) |
| `gui` | `false` | 데이터 수집 GUI ([hand_gui](../hand_gui/README.md)): 로봇 손 힘 표시, 웹캠·로봇 카메라, 힘 그래프, 녹화, Hugging Face 업로드. `true`이면 MuJoCo 창과 웹캠 창은 열지 않음 |
| `data_dir` | `~/tactile_dataset` | GUI 녹화 저장 폴더 |

## 실물 RealSense D405 (그리퍼 카메라)

```bash
sudo apt install ros-humble-realsense2-camera     # 최초 1회
ros2 launch hand_bringup pipeline.launch.py backend:=real hand_camera:=d405 gui:=true
```

- 컬러 `/hand_camera/d405/color/image_raw` (640×480, 30 Hz), 컬러에 맞춘 깊이 `/hand_camera/d405/aligned_depth_to_color/image_raw`. GUI·녹화는 컬러 영상을 쓴다.
- D405는 별도 RGB 센서가 없고 깊이 모듈이 컬러도 낸다. 그래서 해상도는 `depth_module.color_profile`로 정한다 (D415의 `rgb_camera.color_profile`은 D405에서 무시됨). 근거리용(약 7~50 cm)이라 그리퍼 카메라에 맞다.
- USB 3 포트에 연결한다 (`lsusb -t`에서 5000M).
- 이전 카메라 D415는 `hand_camera:=d415`로 그대로 쓸 수 있다 (토픽 `/hand_camera/d415/...`). 두 카메라가 함께 꽂혀 있어도 `device_type`으로 고른 모델만 연다.

## 주의

- 웹캠에서 손이 0.5초 이상 사라지면 자세 명령 노드가 **모든 손가락을 펴는 명령**을 보낸다. 실물로 물체를 쥘 때 주의한다.
- 실물 시리얼 포트는 한 프로그램만 쓸 수 있다. `hardware_connection_test`의 프로그램과 동시에 실행하지 않는다.
- 각 노드의 토픽·준비물은 패키지별 README를 참고한다: [hand_tracking](../hand_tracking/README.md), [pose_command](../pose_command/README.md), [finger_node](../finger_node/README.md), [feedback_node](../feedback_node/README.md), [gripper_camera](../gripper_camera/README.md), [hand_gui](../hand_gui/README.md).
