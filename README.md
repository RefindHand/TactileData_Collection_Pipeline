# TactileData_Collection_Pipeline

로봇 핸드의 **관절 상태·촉각 데이터**와 카메라 영상을 수집하기 위한 ROS2 기반 프로젝트입니다.

웹캠으로 사람 손 동작을 인식해 **OYMotion ROHand AP002**를 움직이고, 손가락별 각도·힘·촉각 피드백과 로봇 핸드 카메라 영상을 함께 수집합니다. MuJoCo 시뮬레이션과 실물 AP002 모두를 대상으로 합니다.

## 개발 환경

- **OS:** Ubuntu 22.04 (데스크탑, Jetson AGX Orin) 또는 Windows + WSL2 Ubuntu 22.04
  - Jetson AGX Orin (JetPack 6, L4T R36.5)에서 실물 AP002 + D405 + GUI 동작을 확인했다. 준비는 [Jetson AGX Orin 설정](#jetson-agx-orin-설정) 참고
- **ROS:** ROS2 Humble
- **로봇 핸드:** OYMotion ROHand AP002 (오른손)
- **통신:** USB-Serial(CH340), Modbus-RTU
- **시뮬레이터:** MuJoCo
- **손 인식:** MediaPipe Hands
- **로봇 핸드 카메라:** Intel RealSense D405 (실물, 이전에는 D415), Orbbec Gemini 305 (MuJoCo 가상 카메라)
- **촉각 센서 (AP002):** 손끝 3D 힘 ×5, 안쪽 마디 수직항력 ×3 (검지·중지·약지), 손바닥 11×5 점 배열

## 노드 구성

| 역할 | 패키지 | 안내서 |
| --- | --- | --- |
| 웹캠 노드 (손 인식) | `src/hand_tracking` + `windows_sender/` | [README](src/hand_tracking/README.md) |
| 손가락 노드 ×6 (액추에이터별 각도·힘) + 하드웨어 백엔드 (MuJoCo / 실물) | `src/finger_node` | [README](src/finger_node/README.md) |
| 자세 명령 노드 | `src/pose_command` | [README](src/pose_command/README.md) |
| 피드백 노드 (각도·힘·촉각 피드백) | `src/feedback_node` | [README](src/feedback_node/README.md) |
| 로봇 핸드 카메라 노드 (가상 Gemini 305 / 실물 D405는 `hand_camera:=d405`) | `src/gripper_camera` | [README](src/gripper_camera/README.md) |
| 전체 실행 (통합) | `src/hand_bringup` | [README](src/hand_bringup/README.md) |
| 데이터 수집 GUI (힘 표시·녹화·Hugging Face 업로드) | `src/hand_gui` | [README](src/hand_gui/README.md) |

```text
웹캠 ──/hand_landmarks──▶ 자세 명령 ──/finger/<f>/command──▶ 손가락 노드 ×6 ──▶ 로봇 핸드 (MuJoCo 또는 실물)
(관절 21개 좌표)                      (목표 각도 rad)              │
                                                                  └──/finger/<f>/state, tactile, tactile_proximal──▶ 피드백 노드
로봇 핸드 카메라 (가상 Gemini 305 또는 실물 D405) ──▶ 영상
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
ros2 launch hand_bringup pipeline.launch.py backend:=real enable_motion:=true hand_camera:=d405 gui:=true  # 실물 + D405 + GUI
```

실물 D405(이전 D415는 `hand_camera:=d415`)는 `sudo apt install ros-humble-realsense2-camera`가 필요합니다.

웹캠 창이 뜨면 손을 한 번 끝까지 쥐었다 펴고 엄지를 크게 돌린 뒤(자동 보정) 손을 움직입니다. 자세한 내용은 [hand_bringup README](src/hand_bringup/README.md)를 참고합니다.

## 데이터 수집 GUI

```bash
pip install --user "pyqtgraph==0.13.7" "numpy<2"     # 최초 1회
ros2 launch hand_bringup pipeline.launch.py gui:=true                                    # 시뮬레이션
ros2 launch hand_bringup pipeline.launch.py backend:=real enable_motion:=true gui:=true  # 실물
ros2 launch hand_bringup pipeline.launch.py backend:=real enable_motion:=true hand_camera:=d405 gui:=true  # 실물 + D405
```

GUI 창 하나에 로봇 손(손끝·안쪽 마디·손바닥 11×5 힘 표시), 웹캠, 로봇 핸드 카메라, 힘 그래프가 나옵니다 (MuJoCo 창·웹캠 창은 열지 않음). 왼쪽 맨 위 **■ STOP / ▶ START** 버튼(또는 Esc = STOP)으로 로봇 손을 멈추고 다시 움직입니다.

1. 왼쪽 **녹화** 탭에서 작업(MW / LP / TFP / PS 등)과 메모를 입력합니다.
2. **● 녹화 시작** → 동작 → **■ 녹화 정지**. 한 번이 에피소드 하나이며 `~/tactile_dataset/episode_NNNNNN/`에 저장됩니다.
3. **업로드** 탭에서 Hugging Face 데이터셋 저장소로 올립니다 (`pip install --user "huggingface_hub<1"`, `huggingface-cli login` 필요).
4. 로봇 손가락이 덜/더 굽으면 **보정** 탭의 "손 보정 초기화" 후 손을 한 번 끝까지 폈다 쥡니다.

자세한 내용은 [hand_gui README](src/hand_gui/README.md)를 참고합니다.

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

| 항목 | Ubuntu 22.04 데스크탑 | Jetson AGX Orin (JetPack 6) | Windows + WSL2 |
| --- | --- | --- | --- |
| 웹캠 | Ubuntu에서 직접 사용 (`team_sender:=local`, 기본) | 데스크탑과 같음. RealSense도 `/dev/video*`를 만들어 웹캠 번호가 바뀔 수 있음 → `camera_device:=N` | Windows에서 `windows_sender/`를 실행해 UDP로 전송 (`team_sender:=remote`) |
| 실물 USB (`/dev/ttyUSB0`) | 바로 인식 | 커널에 CH340 드라이버가 없어 직접 빌드 ([아래](#4-ch340-usb-시리얼-드라이버-ch341)) | Windows에서 `usbipd`로 WSL에 연결 ([안내서](hardware_connection_test/README.md)) |
| MuJoCo 창 · GUI | 바로 표시 | Jetson에 연결한 화면에 표시. 원격 접속에서는 `DISPLAY=:1` ([아래](#7-원격-접속-vs-code-remote-ssh에서-실행)) | WSLg 필요 (`wsl --update`) |
| 다른 PC와 ROS2 통신 | 같은 `ROS_DOMAIN_ID` | 같은 `ROS_DOMAIN_ID` | `.wslconfig`에 `networkingMode=mirrored` 설정 + 같은 `ROS_DOMAIN_ID` |

**Ubuntu 22.04에서 `/dev/ttyUSB0`이 안 생길 때:** CH340은 인식되는데(`lsusb`) 포트가 없으면 점자 단말 프로그램 `brltty`가 장치를 가져간 것입니다. 점자 단말기를 쓰지 않으면 `sudo apt remove brltty` 후 USB를 다시 꽂습니다. Jetson은 이것 외에 드라이버도 없습니다 (아래).

어떤 환경이든 `numpy`는 1.x를 유지해야 합니다 (numpy 2는 ROS2 Humble과 충돌).

## Jetson AGX Orin 설정

JetPack 6 (L4T R36.5, Ubuntu 22.04, 커널 `5.15.x-tegra`, aarch64) 기준입니다. 처음 한 번만 합니다.

### 1. ROS2 Humble, RealSense 드라이버, apt 패키지

[ROS2 Humble 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)대로 apt 저장소를 추가한 뒤 설치합니다.

```bash
sudo apt install ros-humble-desktop ros-dev-tools \
    ros-humble-realsense2-camera ros-humble-rqt-image-view \
    python3-pip python3-pyqt5 python3-opencv
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

### 2. pip 패키지

모두 aarch64용 wheel이 있습니다 (mediapipe는 0.10.14까지 aarch64 wheel이 있고 0.10.21은 없음).

```bash
python3 -m pip install --user "numpy<2" "pyqtgraph==0.13.7" mujoco "mediapipe==0.10.14" \
    "opencv-contrib-python<4.12" "pymodbus==3.7.2" "pyserial==3.5" "huggingface_hub<1"
```

### 3. RealSense udev 규칙 (root 없이 D405 사용)

```bash
sudo curl -fsSL -o /etc/udev/rules.d/99-realsense-libusb.rules \
    https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 4. CH340 USB-시리얼 드라이버 (`ch341`)

JetPack 커널에는 CH340 드라이버가 없어서 AP002를 꽂아도(`lsusb`에 `1a86:7523`) `/dev/ttyUSB0`이 생기지 않습니다 (`modinfo ch341` → not found). 같은 버전의 리눅스 소스에서 드라이버 하나만 빌드해 넣습니다. 커널 헤더(`/lib/modules/$(uname -r)/build`)는 JetPack에 들어 있습니다.

```bash
mkdir -p ~/ch341 && cd ~/ch341
KVER=$(uname -r | cut -d- -f1)          # 예: 5.15.199
curl -fsSL -o ch341.c "https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain/drivers/usb/serial/ch341.c?h=v${KVER}"
echo 'obj-m += ch341.o' > Makefile
make -C /lib/modules/$(uname -r)/build M=$PWD modules
sudo install -D -m 644 ch341.ko /lib/modules/$(uname -r)/updates/ch341.ko
sudo depmod -a && sudo modprobe ch341
echo ch341 | sudo tee /etc/modules-load.d/ch341.conf    # 부팅 때 자동으로 불러옴
```

JetPack 커널을 업데이트하면 다시 빌드합니다.

### 5. 시리얼 포트 권한, `brltty` 제거

```bash
sudo usermod -aG dialout $USER
sudo apt remove brltty        # 점자 단말기를 쓰지 않을 때
sudo reboot
```

**재부팅해야 합니다.** VS Code Remote-SSH 같은 원격 접속이 열려 있으면 Jetson 화면에서 로그아웃했다가 다시 로그인해도 새 그룹이 적용되지 않습니다 (원격 접속이 사용자 세션을 계속 붙잡고 있음). 확인: `id -nG`에 `dialout`이 있어야 합니다. 권한이 없으면 `hand_serial`이 `Permission denied: '/dev/ttyUSB0'`을 출력하고 바로 끝나서, GUI에 촉각이 안 나오고 손도 움직이지 않습니다.

### 6. AP002 MuJoCo 모델

GUI의 로봇 손 화면과 시뮬레이션에 필요합니다.

```bash
cd ~/TactileData_Collection_Pipeline
git clone https://github.com/oymotion/rohand_mujoco.git external/rohand_mujoco
```

그다음 [빌드](#빌드)합니다.

### 7. 원격 접속 (VS Code Remote-SSH)에서 실행

원격 터미널에는 화면이 없어서 GUI가 뜨지 않습니다. Jetson 화면에 로그인되어 있으면 아래처럼 Jetson 화면에 띄울 수 있습니다. Jetson 화면의 터미널에서 직접 실행해도 됩니다.

```bash
export DISPLAY=:1 XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
```

### 8. 실행

```bash
# 웹캠 번호 확인: RealSense가 /dev/video*를 여러 개 만들어 부팅마다 바뀔 수 있음
for v in /sys/class/video4linux/video*; do echo "$(basename $v): $(cat $v/name)"; done

ros2 launch hand_bringup pipeline.launch.py backend:=real hand_camera:=d405 gui:=true camera_device:=6                      # 읽기 전용
ros2 launch hand_bringup pipeline.launch.py backend:=real enable_motion:=true hand_camera:=d405 gui:=true camera_device:=6  # 동작 허용
```

- `camera_device`는 웹캠 이름(예: `Logitech StreamCam`)이 붙은 장치 중 **번호가 작은 쪽**입니다.
- 파이프라인은 **한 번에 하나만** 실행합니다. 이미 실행 중이면 새로 띄운 쪽의 `hand_serial`(시리얼 포트), 손 인식 노드(UDP 5005), 웹캠 노드가 바로 끝나서 손이 움직이지 않습니다. 확인: `pgrep -af "ros2 launch"`
- 확인한 속도: D405 컬러·깊이 640×480 30 Hz, 실물 손 상태·촉각 약 10 Hz, 웹캠 손 인식 약 10 Hz (MediaPipe CPU).

## 공통 규약

- 손가락(액추에이터) 순서: `index, middle, ring, little, thumb_root, thumb`
- 굽힘 방향: **0 = 펴짐, 1(또는 최대 각도) = 굽힘** (MuJoCo·실물 공통)
- 각도 단위: rad, 힘 단위: N
- 촉각 (ROH-AP002 매뉴얼 7.4, 실물로 확인):

  | 위치 | 센서 | 토픽 |
  | --- | --- | --- |
  | 손끝 (5개) | 3D 힘 `[수직 N, 접선 N, 방향°]` | `/finger/<f>/tactile` |
  | 안쪽 마디 (검지·중지·약지) | 수직항력 `[N]` | `/finger/<f>/tactile_proximal` |
  | 손바닥 | 11×5 점 배열 (실물 0~255) | `/hand_hw/tactile/palm` |

- 노드별 토픽은 각 패키지 README를 참고합니다.

## 로봇 핸드 테스트

ROS2 Humble 환경을 불러온 뒤 테스트 프로그램을 실행합니다.

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash

# 실물 AP002 연결 상태 및 관절 위치 확인 (읽기 전용)
python3 hardware_connection_test/rohand_real_ros2.py

# 실물 촉각 센서 원시값 확인 (읽기 전용, 누른 곳이 어떤 값으로 들어오는지)
ros2 run finger_node tactile_probe
```

실물 손가락 제어, MuJoCo 시뮬레이터 실행 및 Windows–WSL2 USB 연결 방법은 **[Hardware Connection Test 안내서](hardware_connection_test/README.md)** 를 참고하세요.

> **참고:** `external/` 및 ROS2 빌드 결과물은 GitHub 저장소에 포함되지 않으므로 개발환경에 맞게 별도 준비해야 합니다. AP002 모델은 `git clone https://github.com/oymotion/rohand_mujoco.git external/rohand_mujoco`로 받습니다.
