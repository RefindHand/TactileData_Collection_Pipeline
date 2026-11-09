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
- **그리퍼 카메라:** Intel RealSense D415 (시뮬레이션은 MuJoCo 가상 카메라)

## 노드 구성

| 역할 | 패키지 | 안내서 |
| --- | --- | --- |
| 웹캠 노드 (손 인식) | `src/hand_tracking` + `windows_sender/` | [README](src/hand_tracking/README.md) |
| 손가락 노드 ×6 (액추에이터별 각도·힘) + 하드웨어 백엔드 (MuJoCo / 실물) | `src/finger_node` | [README](src/finger_node/README.md) |
| 자세 명령 노드 | `src/pose_command` | [README](src/pose_command/README.md) |
| 피드백 노드 (각도·힘·촉각 피드백) | `src/feedback_node` | [README](src/feedback_node/README.md) |
| 그리퍼 카메라: 시뮬레이션 (MuJoCo 가상 카메라) | `src/gripper_camera` | [README](src/gripper_camera/README.md) |
| 그리퍼 카메라: 실물 (RealSense D415) | `realsense2_camera` (ROS 패키지) | [실행 방법](#실물-로봇-핸드--그리퍼-카메라-실행-wsl) |
| 전체 실행 (통합) | `src/hand_bringup` | [README](src/hand_bringup/README.md) |
| 데이터 수집 GUI (힘 표시·녹화·Hugging Face 업로드) | `src/hand_gui` | [README](src/hand_gui/README.md) |

```text
웹캠 ──/hand_landmarks──▶ 자세 명령 ──/finger/<f>/command──▶ 손가락 노드 ×6 ──▶ 로봇 핸드 (MuJoCo 또는 실물)
(관절 21개 좌표)                      (목표 각도 rad)              │
                                                                  └──/finger/<f>/state, tactile──▶ 피드백 노드
로봇 핸드 카메라 ──▶ 영상
```

## 프로젝트 구조

```text
TactileData_Collection_Pipeline/
├── src/                       # ROS2 패키지 (모든 패키지는 여기에 둔다)
│   ├── hand_bringup/          #   전체 파이프라인 launch
│   ├── hand_gui/              #   데이터 수집 GUI (힘 표시, 녹화, 업로드)
│   ├── hand_tracking/         #   웹캠 손 인식 UDP 수신 노드 + Ubuntu용 웹캠 실행
│   ├── finger_node/           #   손가락 노드 + 하드웨어 백엔드 (MuJoCo / 실물 AP002)
│   ├── pose_command/          #   자세 명령 노드
│   ├── feedback_node/         #   피드백 노드
│   └── gripper_camera/        #   Gemini 305 가상 RGB-D 카메라 노드 (MuJoCo)
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

## 전체 실행

```bash
ros2 launch hand_bringup pipeline.launch.py                                    # MuJoCo + 이 PC 웹캠
ros2 launch hand_bringup pipeline.launch.py team_sender:=remote                # 웹캠은 Windows에서 (WSL)
ros2 launch hand_bringup pipeline.launch.py backend:=real enable_motion:=true  # 실물 AP002
```

웹캠 창이 뜨면 손을 한 번 끝까지 쥐었다 펴고 엄지를 크게 돌린 뒤(자동 보정) 손을 움직입니다. 자세한 내용은 [hand_bringup README](src/hand_bringup/README.md)를 참고합니다.

## 데이터 수집 GUI

```bash
pip install --user "pyqtgraph==0.13.7" "numpy<2"     # 최초 1회
ros2 launch hand_bringup pipeline.launch.py gui:=true                                    # 시뮬레이션
ros2 launch hand_bringup pipeline.launch.py backend:=real enable_motion:=true gui:=true gripper_camera:=false  # 실물
```

GUI 창 하나에 로봇 손(힘 표시), 웹캠, 로봇 핸드 카메라, 힘 그래프가 나옵니다 (MuJoCo 창·웹캠 창은 열지 않음).

1. 왼쪽 **녹화** 탭에서 작업(MW / LP / TFP / PS 등)과 메모를 입력합니다.
2. **● 녹화 시작** → 동작 → **■ 녹화 정지**. 한 번이 에피소드 하나이며 `~/tactile_dataset/episode_NNNNNN/`에 저장됩니다.
3. **업로드** 탭에서 Hugging Face 데이터셋 저장소로 올립니다 (`pip install --user "huggingface_hub<1"`, `huggingface-cli login` 필요).
4. 로봇 손가락이 덜/더 굽으면 **보정** 탭의 "손 보정 초기화" 후 손을 한 번 끝까지 폈다 쥡니다.

자세한 내용은 [hand_gui README](src/hand_gui/README.md)를 참고합니다.

## 실물 로봇 핸드 + 그리퍼 카메라 실행 (WSL)

노트북 웹캠으로 실물 AP002를 움직이고, 웹캠과 그리퍼 카메라(RealSense D415) 영상을 GUI에서 확인합니다. Windows + WSL2 Ubuntu 22.04에서 동작을 확인했습니다.

| 장치 | 연결 방법 | 실행 위치 |
| --- | --- | --- |
| 노트북 웹캠 | `windows_webcam_sender.py`가 손 좌표·영상을 UDP로 WSL에 전송 | PowerShell |
| AP002 (CH340) | `usbipd`로 WSL에 연결 → `/dev/ttyUSB0` | WSL |
| RealSense D415 | `usbipd`로 WSL에 연결 → `realsense2_camera` 노드 | WSL |

### 처음 한 번만

WSL에서 패키지를 설치하고 USB 권한을 설정합니다.

```bash
sudo apt update
sudo apt install ros-humble-librealsense2 ros-humble-realsense2-camera python3-pyqt5 fonts-nanum
pip install --user pymodbus==3.7.2 pyserial==3.5 "pyqtgraph==0.13.7" "numpy<2"
sudo usermod -aG dialout $USER     # 적용하려면 터미널을 새로 연다

sudo tee /etc/udev/rules.d/99-realsense-d415.rules <<'RULES'
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad3", MODE="0666"
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad3", MODE="0666"
RULES
sudo udevadm control --reload-rules

cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

**관리자 PowerShell**에서 두 장치를 WSL과 공유합니다. `usbipd list`에서 각 장치의 BUSID를 확인합니다. 웹캠용 Python 환경은 [hand_tracking README](src/hand_tracking/README.md)의 Windows 환경 구성을 따릅니다.

```powershell
usbipd list
usbipd bind --busid <핸드 BUSID>
usbipd bind --busid <D415 BUSID>
```

### 매번 실행하는 순서

핸드의 24V 전원을 켠 뒤 아래 순서로 실행합니다.

**1. USB 연결 (PowerShell 1).** BUSID는 다시 꽂으면 바뀔 수 있습니다. 마지막 명령은 창을 띄워 둔 채로 유지합니다 (카메라가 재시작해도 자동으로 다시 연결).

```powershell
usbipd attach --wsl --busid <핸드 BUSID>
usbipd attach --wsl --busid <D415 BUSID> --auto-attach
```

**2. 노트북 웹캠 (PowerShell 2).** `Ubuntu-22.04`와 `<사용자이름>`은 환경에 맞게 바꿉니다.

```powershell
conda activate rohand_mp
cd "\\wsl.localhost\Ubuntu-22.04\home\<사용자이름>\TactileData_Collection_Pipeline\windows_sender"
python windows_webcam_sender.py --camera 0
```

**3. 장치 확인과 그리퍼 카메라 (WSL 터미널 1).** `/dev/ttyUSB0`와 `Intel RealSense D415`가 나오는지 확인한 뒤 카메라를 실행하고 켜 둡니다.

```bash
source /opt/ros/humble/setup.bash
ls /dev/ttyUSB0
rs-enumerate-devices -s

ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __node:=gripper_camera -r __ns:=/ \
  -p enable_depth:=false -p enable_infra1:=false -p enable_infra2:=false \
  -p rgb_camera.color_profile:=640x480x15
```

**4. 읽기 전용으로 전체 실행 (WSL 터미널 2).** 핸드는 움직이지 않습니다. GUI 아래쪽에 `손가락 6/6`, `웹캠 ✓`, `로봇 카메라 ✓`가 나오는지 확인합니다.

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hand_bringup pipeline.launch.py backend:=real gui:=true gripper_camera:=false
```

`gripper_camera:=false`는 필수입니다. 빠지면 MuJoCo 가상 카메라가 같은 토픽에 영상을 보내 D415 영상과 섞입니다.

**5. 실제 동작 (WSL 터미널 2).** 4단계를 Ctrl+C로 끄고 다시 실행합니다.

```bash
ros2 launch hand_bringup pipeline.launch.py backend:=real enable_motion:=true gui:=true gripper_camera:=false
```

손을 웹캠에 보여 주고, 끝까지 쥐었다 펴고 엄지를 크게 돌려 자동 보정한 뒤 천천히 움직입니다. 종료는 터미널 2와 1에서 Ctrl+C, PowerShell 웹캠도 Ctrl+C입니다.

> **안전 주의**
> - 실행하는 순간 핸드가 완전히 펴집니다. 주변을 비우고 물체 없이 시작합니다.
> - 손이 0.5초 이상 인식되지 않아도 펴집니다. 물체를 쥔 상태에서 손이 화면 밖으로 나가면 물체를 놓칩니다.
> - Ctrl+C를 눌러도 핸드는 마지막 자세를 유지합니다. 급히 멈출 때는 24V 전원을 끕니다.

### 문제가 생기면

| 증상 | 확인할 것 |
| --- | --- |
| `/dev/ttyUSB0`이 없음 | 핸드의 `usbipd attach`, BUSID |
| `rs-enumerate-devices`에 장치가 없거나 `Permission denied` | D415의 `usbipd attach`, udev 규칙. 규칙을 추가한 뒤에는 `sudo udevadm trigger --subsystem-match=video4linux` |
| GUI에 로봇 카메라 영상이 안 나옴 | `ros2 topic hz /gripper_camera/color/image_raw`. 없으면 카메라 명령의 `-r __node:=gripper_camera -r __ns:=/` 확인 |
| GUI에 `손가락 6/6`이 안 나옴 | `ros2 topic hz /hand_hw/state` (정상이면 약 10 Hz), 24V 전원, 배선, 시리얼 포트를 쓰는 다른 프로그램 |
| GUI에 웹캠 영상이 안 나옴 | PowerShell의 `windows_webcam_sender.py` 실행 여부, GUI 웹캠 목록이 `Windows 카메라`인지 |
| 카메라 영상이 끊김 | `rgb_camera.color_profile:=640x480x6`으로 낮춰 보기 |

D415의 Depth 스트림은 WSL 커널이 형식을 일부 지원하지 않아 동작을 확인하지 않았습니다. 네이티브 Ubuntu에서는 `usbipd`와 PowerShell 단계 없이 USB 웹캠을 GUI 목록에서 고르면 됩니다.

## 로봇 핸드만 움직이기 (손가락 노드)

```bash
ros2 launch finger_node fingers.launch.py                                   # MuJoCo 시뮬레이션
ros2 launch finger_node fingers.launch.py backend:=real                     # 실물 (읽기 전용)
ros2 launch finger_node fingers.launch.py backend:=real enable_motion:=true # 실물 (동작 허용)

# 다른 터미널: 검지에 1.0 rad 명령, 각도·힘 확인
ros2 topic pub --once /finger/index/command std_msgs/msg/Float64 "{data: 1.0}"
ros2 topic echo /finger/index/angle_force
```

준비물(MuJoCo 모델, pip 패키지)과 토픽은 [finger_node README](src/finger_node/README.md)를 참고합니다.

## 실행 환경별 차이

| 항목 | Ubuntu 22.04 (데스크탑, AGX Orin) | Windows + WSL2 |
| --- | --- | --- |
| 웹캠 | Ubuntu에서 직접 사용 (`team_sender:=local`, 기본). GUI에서는 USB 카메라 선택 | Windows에서 `windows_sender/`를 실행해 UDP로 전송 (`team_sender:=remote`). GUI에는 `windows_webcam_sender.py`로 영상까지 전송 |
| 실물 USB (`/dev/ttyUSB0`) | 바로 인식 | Windows에서 `usbipd`로 WSL에 연결 ([안내서](hardware_connection_test/README.md)) |
| 그리퍼 카메라 (D415) | USB로 바로 연결 | Windows에서 `usbipd`로 WSL에 연결 ([실행 방법](#실물-로봇-핸드--그리퍼-카메라-실행-wsl)) |
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
