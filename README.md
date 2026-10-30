# TactileData_Collection_Pipeline

로봇 핸드의 **관절 상태·촉각 데이터**와 카메라 영상을 수집하기 위한 ROS2 기반 프로젝트입니다.

현재는 **OYMotion ROHand AP002**의 MuJoCo 시뮬레이션 및 실물 제어 통신을 테스트하고 있으며, 카메라 연동과 데이터 수집 기능은 추후 구현할 예정입니다.

## 개발 환경

- **OS:** Windows + WSL2 Ubuntu 22.04
- **ROS:** ROS2 Humble
- **로봇 핸드:** OYMotion ROHand AP002 (오른손)
- **통신:** USB-Serial(CH340), Modbus-RTU
- **시뮬레이터:** MuJoCo
- **카메라(예정):** Orbbec Gemini 305

## 프로젝트 구조

```text
TactileData_Collection_Pipeline/
├── hardware_connection_test/  # 로봇 핸드 시뮬레이션·실물 연결 테스트
├── src/                       # ROS2 패키지 작업 공간
├── external/                  # 제조사 등 외부 소스 (Git 추적 제외)
├── build/                     # ROS2 빌드 결과 (Git 추적 제외)
├── install/                   # ROS2 설치 결과 (Git 추적 제외)
└── log/                       # ROS2 빌드 로그 (Git 추적 제외)
```

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
