# TactileData_Collection_Pipeline

로봇 핸드의 **관절 상태·촉각 데이터**와 카메라 영상을 수집하기 위한 ROS2 기반 프로젝트입니다.

웹캠으로 사람 손 동작을 인식해 **OYMotion ROHand AP002**를 움직이고, 손가락별 각도·힘·촉각 피드백과 로봇 핸드 카메라 영상을 함께 수집합니다. MuJoCo 시뮬레이션과 실물 AP002 모두를 대상으로 합니다.

## 개발 환경

- **OS:** Ubuntu 22.04 (데스크탑, Jetson AGX Orin) 또는 Windows + WSL2 Ubuntu 22.04
- **ROS:** ROS2 Humble
- **로봇 핸드:** OYMotion ROHand AP002 (오른손)
- **통신:** USB-Serial(CH340), Modbus-RTU
- **시뮬레이터:** MuJoCo
- **손 인식:** MediaPipe Hands
- **카메라(예정):** Orbbec Gemini 305

## 노드 구성

| 역할 | 패키지 | 안내서 |
| --- | --- | --- |
| 웹캠 노드 (손 인식) | `src/hand_tracking` + `windows_sender/` | [README](src/hand_tracking/README.md) |
| 손가락 노드 ×6 (액추에이터별 각도·힘) | `src/finger_node` | 추가 예정 |
| 자세 명령 노드 | `src/pose_command` | [README](src/pose_command/README.md) |
| 피드백 노드 (각도·힘·촉각 피드백) | `src/feedback_node` | [README](src/feedback_node/README.md) |
| 로봇 핸드 카메라 노드 | 미정 | 작업 중 |

```text
웹캠 ──▶ 자세 명령 ──▶ 손가락 노드 ×6 ──▶ 로봇 핸드 (MuJoCo 또는 실물)
                            │
                            └──▶ 피드백 노드 (각도·힘·촉각)
로봇 핸드 카메라 ──▶ 영상
```

## 프로젝트 구조

```text
TactileData_Collection_Pipeline/
├── src/                       # ROS2 패키지 (모든 패키지는 여기에 둔다)
│   ├── hand_tracking/         #   웹캠 손 인식 UDP 수신 노드
│   ├── pose_command/          #   자세 명령 노드
│   └── feedback_node/         #   피드백 노드
├── windows_sender/            # Windows 웹캠 손 인식 프로그램 (ROS 패키지 아님)
├── hardware_connection_test/  # 로봇 핸드 시뮬레이션·실물 연결 테스트
├── external/                  # 제조사 등 외부 소스 (Git 추적 제외)
├── build/                     # ROS2 빌드 결과 (Git 추적 제외)
├── install/                   # ROS2 설치 결과 (Git 추적 제외)
└── log/                       # ROS2 빌드 로그 (Git 추적 제외)
```

## 빌드

프로젝트 루트에서 모든 패키지를 한 번에 빌드합니다 (`src/` 아래 패키지를 자동으로 찾음).

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

특정 패키지만 빌드하려면 `colcon build --symlink-install --packages-select feedback_node`처럼 지정합니다.

## 실행 환경별 차이

| 항목 | Ubuntu 22.04 (데스크탑, AGX Orin) | Windows + WSL2 |
| --- | --- | --- |
| 웹캠 | Ubuntu에서 직접 사용 | Windows에서 `windows_sender/`를 실행해 UDP로 전송 |
| 실물 USB (`/dev/ttyUSB0`) | 바로 인식 | Windows에서 `usbipd`로 WSL에 연결 ([안내서](hardware_connection_test/README.md)) |
| MuJoCo 창 | 바로 표시 | WSLg 필요 (`wsl --update`) |
| 다른 PC와 ROS2 통신 | 같은 `ROS_DOMAIN_ID` | `.wslconfig`에 `networkingMode=mirrored` 설정 + 같은 `ROS_DOMAIN_ID` |

AGX Orin은 ARM(aarch64)이므로 `mediapipe`, `mujoco` 설치 가능 여부를 먼저 확인합니다. 어떤 환경이든 `numpy`는 1.x를 유지해야 합니다 (numpy 2는 ROS2 Humble과 충돌).

## 공통 규약

- 손가락(액추에이터) 순서: `index, middle, ring, little, thumb_root, thumb`
- 굽힘 방향: **0 = 펴짐, 1(또는 최대 각도) = 굽힘** (MuJoCo·실물 공통)
- 각도 단위: rad
- 노드별 토픽은 각 패키지 README를 참고합니다.

## 로봇 핸드 테스트

ROS2 Humble 환경을 불러온 뒤 테스트 프로그램을 실행합니다.

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash

# 실물 AP002 연결 상태 및 관절 위치 확인 (읽기 전용)
python3 hardware_connection_test/rohand_real_ros2.py
```

실물 손가락 제어, MuJoCo 시뮬레이터 실행 및 Windows–WSL2 USB 연결 방법은 **[Hardware Connection Test 안내서](hardware_connection_test/README.md)** 를 참고하세요.

> **참고:** `external/` 및 ROS2 빌드 결과물은 GitHub 저장소에 포함되지 않으므로 개발환경에 맞게 별도 준비해야 합니다.
