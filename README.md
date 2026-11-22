# TactileData_Collection_Pipeline

로봇 핸드의 **관절 상태·촉각 데이터**와 카메라 영상을 수집하기 위한 ROS2 기반 프로젝트입니다.

웹캠으로 사람 손 동작을 인식해 **OYMotion ROHand AP002**를 움직이고, 손가락별 각도·힘·촉각 피드백과 로봇 핸드 카메라 영상을 함께 수집합니다. MuJoCo 시뮬레이션과 실물 AP002 모두를 대상으로 합니다.

## 개발 환경

개발과 실행은 **Jetson AGX Orin** 한 대에서 합니다.

- **보드:** NVIDIA Jetson AGX Orin (JetPack 6, L4T R36.5, Ubuntu 22.04, aarch64)
- **접속:** Jetson에 연결한 모니터, 또는 다른 PC에서 VS Code Remote-SSH ([원격 접속에서 GUI 띄우기](#원격-접속에서-gui-띄우기))
- **ROS:** ROS2 Humble
- **로봇 핸드:** OYMotion ROHand AP002 (오른손), USB-Serial(CH340) + Modbus-RTU
- **웹캠 (사람 손 인식):** Logitech StreamCam, MediaPipe Hands
- **로봇 핸드 카메라:** Intel RealSense **D405와 D415 모두 지원** (`hand_camera:=d405` / `hand_camera:=d415`). 현재 사용하는 실물 카메라는 **D405**입니다. 인자를 주지 않으면 MuJoCo 가상 카메라(Orbbec Gemini 305, `virtual`)를 씁니다.
- **시뮬레이터:** MuJoCo
- **촉각 센서 (AP002):** 손끝 3D 힘 ×5, 안쪽 마디 수직항력 ×3 (검지·중지·약지), 손바닥 11×5 점 배열

### 하드웨어 연결

| 장치 | 연결 | 확인 |
| --- | --- | --- |
| RealSense D405 (또는 D415) | USB 3 포트 (USB-A) | `lsusb -t`에서 5000M |
| AP002 (CH340) | USB-A 포트 + 24 V 전원 | `ls /dev/ttyUSB0` |
| 웹캠 (StreamCam) | USB-A 포트 (USB-C 케이블은 C→A 젠더 사용) | `lsusb`에 `046d:0893` |

웹캠을 Jetson의 Type-C 포트에 꽂았을 때는 연결·전원은 감지되지만 USB 장치로 인식되지 않았습니다. USB-A 포트에 꽂습니다.

## 노드 구성

| 역할 | 패키지 |
| --- | --- |
| 웹캠 노드 (손 인식) | `src/hand_tracking` |
| 손가락 노드 ×6 (액추에이터별 각도·힘) + 하드웨어 백엔드 (MuJoCo / 실물) | `src/finger_node` |
| 자세 명령 노드 | `src/pose_command` |
| 피드백 노드 (각도·힘·촉각 피드백) | `src/feedback_node` |
| 로봇 핸드 카메라 (가상 Gemini 305 노드 / 실물 D405·D415는 RealSense 드라이버) | `src/gripper_camera`, `src/hand_bringup` |
| 전체 실행 (통합) | `src/hand_bringup` |
| 데이터 수집 GUI (힘 표시·녹화·Hugging Face 업로드) | `src/hand_gui` |

패키지별 안내서는 아래 [패키지별 안내서](#패키지별-안내서)에 요약해 두었습니다.

```text
웹캠 ──/hand_landmarks──▶ 자세 명령 ──/finger/<f>/command──▶ 손가락 노드 ×6 ──▶ 로봇 핸드 (MuJoCo 또는 실물)
(관절 21개 좌표)                      (목표 각도 rad)              │
                                                                  └──/finger/<f>/state, tactile, tactile_proximal──▶ 피드백 노드
로봇 핸드 카메라 (가상 Gemini 305 또는 실물 D405 / D415) ──▶ 영상 ──▶ GUI·녹화
```

## 프로젝트 구조

```text
TactileData_Collection_Pipeline/
├── src/                       # ROS2 패키지 (모든 패키지는 여기에 둔다)
│   ├── hand_bringup/          #   전체 파이프라인 launch
│   ├── hand_gui/              #   데이터 수집 GUI (힘 표시, 녹화, 업로드)
│   ├── hand_tracking/         #   웹캠 손 인식 (UDP 수신 노드 + Ubuntu용 웹캠 실행)
│   ├── finger_node/           #   손가락 노드 + 하드웨어 백엔드 (MuJoCo / 실물 AP002)
│   ├── pose_command/          #   자세 명령 노드
│   ├── feedback_node/         #   피드백 노드
│   └── gripper_camera/        #   Gemini 305 가상 RGB-D 카메라 노드 (MuJoCo)
├── hardware_connection_test/  # 로봇 핸드 시뮬레이션·실물 연결 테스트
├── windows_sender/            # 이전 환경(WSL)용 Windows 웹캠 프로그램. Ubuntu 웹캠 노드도 이 코드를 그대로 씀
├── external/                  # 제조사 등 외부 소스 (Git 추적 제외)
├── build/ install/ log/       # ROS2 빌드 결과 (Git 추적 제외)
```

## 환경 준비 (처음 한 번)

JetPack 6 (L4T R36.5, 커널 `5.15.x-tegra`) 기준입니다. 순서대로 한 번만 합니다.

### 1. ROS2 Humble, RealSense 드라이버, apt 패키지

[ROS2 Humble 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)대로 apt 저장소를 추가한 뒤 설치합니다.

```bash
sudo apt install ros-humble-desktop ros-dev-tools \
    ros-humble-realsense2-camera ros-humble-rqt-image-view \
    python3-pip python3-pyqt5 python3-opencv
```

### 2. pip 패키지

모두 aarch64용 wheel이 있습니다 (mediapipe는 0.10.14까지 aarch64 wheel이 있고 0.10.21은 없음). `numpy`는 1.x를 유지합니다 (numpy 2는 ROS2 Humble과 충돌).

```bash
python3 -m pip install --user "numpy<2" "pyqtgraph==0.13.7" mujoco "mediapipe==0.10.14" \
    "opencv-contrib-python<4.12" "pymodbus==3.7.2" "pyserial==3.5" "huggingface_hub<1"
```

### 3. RealSense udev 규칙 (root 없이 D405·D415 사용)

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

`brltty`(점자 단말 프로그램)는 CH340 장치를 가져가므로 점자 단말기를 쓰지 않으면 지웁니다.

```bash
sudo usermod -aG dialout $USER
sudo apt remove brltty
sudo reboot
```

**재부팅해야 합니다.** VS Code Remote-SSH 같은 원격 접속이 열려 있으면 Jetson 화면에서 로그아웃했다가 다시 로그인해도 새 그룹이 적용되지 않습니다 (원격 접속이 사용자 세션을 계속 붙잡고 있음). 재부팅 후 `id -nG`에 `dialout`이 있어야 합니다.

### 6. AP002 MuJoCo 모델

GUI의 로봇 손 화면과 시뮬레이션에 필요합니다. `external/`은 Git에 포함되지 않습니다.

```bash
cd ~/TactileData_Collection_Pipeline
git clone https://github.com/oymotion/rohand_mujoco.git external/rohand_mujoco
```

### 7. 터미널 환경 자동 설정

새 터미널마다 ROS2와 이 프로젝트 환경을 불러오도록 `~/.bashrc`에 추가합니다. 프로젝트 환경이 없으면 `Package 'hand_bringup' not found`가 납니다.

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo '[ -f ~/TactileData_Collection_Pipeline/install/setup.bash ] && source ~/TactileData_Collection_Pipeline/install/setup.bash' >> ~/.bashrc
```

## 빌드

프로젝트 루트에서 모든 패키지를 한 번에 빌드합니다 (`src/` 아래 패키지를 자동으로 찾음).

```bash
cd ~/TactileData_Collection_Pipeline
colcon build --symlink-install
source install/setup.bash
```

- 특정 패키지만 빌드하려면 `colcon build --symlink-install --packages-select feedback_node`처럼 지정합니다.
- `--symlink-install`이라 Python 코드만 고쳤으면 다시 빌드하지 않아도 됩니다. 새 파일·launch·`setup.py`를 바꿨을 때만 다시 빌드합니다.

<a id="8-실행"></a>
## 실행

### 1. 웹캠 번호 확인

RealSense도 `/dev/video*`를 여러 개 만들기 때문에 웹캠 번호가 0이 아니고, 부팅·재연결마다 바뀔 수 있습니다.

```bash
for v in /sys/class/video4linux/video*; do echo "$(basename $v): $(cat $v/name)"; done
```

웹캠 이름(`Logitech StreamCam`)이 붙은 장치 중 **번호가 작은 쪽**을 `camera_device`로 줍니다 (아래 예시는 `/dev/video6`).

### 2. 전체 실행

```bash
ros2 launch hand_bringup pipeline.launch.py camera_device:=6                                     # MuJoCo 시뮬레이션
ros2 launch hand_bringup pipeline.launch.py camera_device:=6 gui:=true                           # 시뮬레이션 + GUI
ros2 launch hand_bringup pipeline.launch.py backend:=real hand_camera:=d405 gui:=true camera_device:=6                      # 실물, 읽기 전용
ros2 launch hand_bringup pipeline.launch.py backend:=real enable_motion:=true hand_camera:=d405 gui:=true camera_device:=6  # 실물, 동작 허용
```

- 실물은 먼저 **읽기 전용**으로 촉각·각도가 들어오는지 확인한 뒤 `enable_motion:=true`로 실행합니다.
- D415를 쓸 때는 `hand_camera:=d415`로 바꿉니다. 영상이 돌아가 보이면 `hand_camera_rotation:=90`처럼 회전을 줍니다.
- 웹캠에 손 전체가 보이게 하고, 처음에 손을 한 번 끝까지 쥐었다 펴고 엄지를 크게 돌립니다 (자동 보정).
- 파이프라인은 **한 번에 하나만** 실행합니다. 확인: `pgrep -af "ros2 launch"`
- 웹캠에서 손이 0.5초 이상 사라지면 자세 명령 노드가 **모든 손가락을 펴는 명령**을 보냅니다. 실물로 물체를 쥘 때 주의합니다.

launch 인자 전체는 [hand_bringup README](src/hand_bringup/README.md)에 있습니다.

<a id="7-원격-접속-vs-code-remote-ssh에서-실행"></a>
### 원격 접속에서 GUI 띄우기

VS Code Remote-SSH 터미널에는 화면이 없어서 GUI가 뜨지 않습니다. Jetson 화면에 로그인되어 있으면 아래를 먼저 실행해 Jetson 화면에 띄웁니다. Jetson 화면의 터미널에서 직접 실행해도 됩니다.

```bash
export DISPLAY=:1 XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
```

### 확인한 속도 (Jetson AGX Orin)

D405 컬러·깊이 640×480 30 Hz, 실물 손 상태·촉각 약 10 Hz, 웹캠 손 인식 약 10 Hz (MediaPipe CPU).

## 데이터 수집 GUI

`gui:=true`로 실행하면 GUI 창 하나에 로봇 손(손끝·안쪽 마디·손바닥 11×5 힘 표시), 웹캠, 로봇 핸드 카메라, 힘 그래프가 나옵니다 (MuJoCo 창·웹캠 창은 열지 않음). 왼쪽 맨 위 **■ STOP / ▶ START** 버튼(또는 Esc = STOP)으로 로봇 손을 멈추고 다시 움직입니다. 버튼 아래에 로봇 상태(`● 동작 중` / `■ 정지됨` / `읽기 전용` / `연결 없음`)가 나옵니다.

1. 왼쪽 **녹화** 탭에서 작업(MW / LP / TFP / PS 등)과 메모를 입력합니다.
2. **● 녹화 시작** → 동작 → **■ 녹화 정지**. 한 번이 에피소드 하나이며 `~/tactile_dataset/episode_NNNNNN/`에 저장됩니다.
3. **업로드** 탭에서 Hugging Face 데이터셋 저장소로 올립니다 (처음 한 번 `huggingface-cli login`).
4. 로봇 손가락이 덜/더 굽으면 **보정** 탭의 "손 보정 초기화" 후 손을 한 번 끝까지 폈다 쥡니다.

## 로봇 핸드만 움직이기 (손가락 노드)

```bash
ros2 launch finger_node fingers.launch.py                                   # MuJoCo 시뮬레이션
ros2 launch finger_node fingers.launch.py backend:=real                     # 실물 (읽기 전용)
ros2 launch finger_node fingers.launch.py backend:=real enable_motion:=true # 실물 (동작 허용)

# 다른 터미널: 검지에 1.0 rad 명령, 각도·힘 확인
ros2 topic pub --once /finger/index/command std_msgs/msg/Float64 "{data: 1.0}"
ros2 topic echo /finger/index/angle_force
```

## 로봇 핸드 테스트

파이프라인이 실행 중이 아닐 때 합니다 (시리얼 포트는 한 프로그램만 쓸 수 있음).

```bash
cd ~/TactileData_Collection_Pipeline
source install/setup.bash          # ~/.bashrc에 추가했으면 생략

# 실물 촉각 센서 원시값 확인 (읽기 전용, 누른 곳이 어떤 값으로 들어오는지)
ros2 run finger_node tactile_probe

# 실물 AP002 연결 상태 및 관절 위치 확인 (명령 토픽을 보내지 않으면 손에 쓰지 않음)
python3 hardware_connection_test/rohand_real_ros2.py
```

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

- 실물 손끝은 미끄러지는 힘이 없으면 방향을 무효로 보내 `[수직 N, 0, NaN]`으로 들어옵니다 (수직 N은 유효).

## 문제 해결

| 증상 | 원인 | 해결 |
| --- | --- | --- |
| `Package 'hand_bringup' not found` | 이 터미널에 프로젝트 환경을 불러오지 않음 | `source ~/TactileData_Collection_Pipeline/install/setup.bash` ([환경 준비 7](#7-터미널-환경-자동-설정)) |
| VS Code 터미널에서 실행하면 GUI가 안 뜸 | 원격 터미널에 화면이 없음 | [원격 접속에서 GUI 띄우기](#원격-접속에서-gui-띄우기) |
| `/dev/ttyUSB0`이 없음 (`lsusb`에는 CH340 있음) | CH340 드라이버 없음 또는 `brltty`가 가져감 | [환경 준비 4·5](#4-ch340-usb-시리얼-드라이버-ch341) |
| GUI에 촉각이 안 나오고 손이 안 움직임, 로그에 `Permission denied: '/dev/ttyUSB0'` | `dialout` 권한 없음 (`hand_serial`이 바로 끝남) | `id -nG` 확인, 그룹 추가 후 **재부팅** |
| 화면은 정상인데 손이 안 움직임, `hand_serial`·`hand_tracking_node`가 바로 끝남 | 파이프라인이 이미 하나 실행 중 (시리얼 포트·UDP 5005·웹캠을 먼저 차지) | `pgrep -af "ros2 launch"`로 찾아 끄고 다시 실행 |
| `웹캠을 열 수 없습니다` | `camera_device` 번호가 RealSense 장치를 가리킴 | [웹캠 번호 확인](#1-웹캠-번호-확인) |
| 웹캠이 `lsusb`에 안 나옴 | Type-C 포트에서 USB 장치로 인식되지 않음 | USB-A 포트에 꽂음 (C→A 젠더) |

## 패키지별 안내서

각 패키지 폴더의 README 요약입니다. 일부 안내서는 아직 이전 환경(Windows + WSL2) 기준으로 쓰여 있습니다 (표시함).

<details>
<summary><b>hand_bringup</b> — 전체 파이프라인 실행</summary>

- [src/hand_bringup/README.md](src/hand_bringup/README.md)
- 노드 코드 없이 모든 노드를 묶어 실행하는 launch 패키지.
- launch 인자 전체 (`backend`, `enable_motion`, `hand_camera`, `hand_camera_rotation`, `camera_device`, `gui`, `data_dir` 등), 실물 D405·D415 토픽과 설정, 실행 시 주의 사항.

</details>

<details>
<summary><b>hand_gui</b> — 데이터 수집 GUI</summary>

- [src/hand_gui/README.md](src/hand_gui/README.md)
- 로봇 손 힘 표시(손끝 원·화살표, 안쪽 마디, 손바닥 격자), 웹캠·로봇 핸드 카메라, 최근 10초 힘 그래프.
- STOP / START, 녹화 파일 구성(에피소드 폴더), Hugging Face 업로드, 파라미터.

</details>

<details>
<summary><b>finger_node</b> — 손가락 노드 + 하드웨어 백엔드</summary>

- [src/finger_node/README.md](src/finger_node/README.md)
- 액추에이터별 손가락 노드 ×6의 토픽, 촉각 센서 구성(매뉴얼 7.4), MuJoCo 백엔드(`hand_sim`)와 실물 백엔드(`hand_serial`, Modbus-RTU) 파라미터.
- 실물 레지스터 주소와 실물로 확인한 항목 / 아직 확인하지 않은 항목.

</details>

<details>
<summary><b>pose_command</b> — 자세 명령 노드</summary>

- [src/pose_command/README.md](src/pose_command/README.md)
- MediaPipe 손 관절 좌표 → 손가락 6개 목표 각도 계산 방법, 자동 보정과 보정 초기화, 떨림 필터, 0.5초 입력 타임아웃.
- 개발 환경 항목은 이전 환경(WSL) 기준.

</details>

<details>
<summary><b>feedback_node</b> — 피드백 노드</summary>

- [src/feedback_node/README.md](src/feedback_node/README.md)
- 손가락 노드의 각도·구동력·촉각을 모아 `/feedback/joint_states`, `tracking_error`, `tactile_summary`, `contact`, `diagnostics`로 다시 발행.
- 입력·출력 토픽, 파라미터, 통합 시 확인할 점.

</details>

<details>
<summary><b>gripper_camera</b> — Gemini 305 가상 RGB-D 카메라</summary>

- [src/gripper_camera/README.md](src/gripper_camera/README.md)
- MuJoCo에 Gemini 305 모델과 가상 스테레오 RGB-D 카메라를 배치해 로봇 손을 찍는 노드 (`hand_camera:=virtual`). 손 자세를 파이프라인에 맞춰 따라감.
- 토픽·파라미터, 단독 테스트, 카메라 모델 배치, CAD 변환. 성능 수치는 이전 환경(WSL2) 기준.

</details>

<details>
<summary><b>hand_tracking</b> — 웹캠 손 인식 (이전 환경 기준)</summary>

- [src/hand_tracking/README.md](src/hand_tracking/README.md)
- UDP 5005로 받은 손 관절 좌표를 `/hand_landmarks`로 발행하는 노드와, Ubuntu에서 웹캠을 직접 여는 `linux_webcam_sender`.
- 본문 대부분은 Windows 웹캠 + WSL 수신 구성(콘솔 3개) 기준. Jetson에서는 `hand_bringup`이 두 노드를 함께 실행하므로 따로 실행할 일이 거의 없음.

</details>

<details>
<summary><b>hardware_connection_test</b> — 로봇 핸드 연결 테스트 (이전 환경 기준)</summary>

- [hardware_connection_test/README.md](hardware_connection_test/README.md)
- 파이프라인과 별개인 단독 테스트 프로그램: MuJoCo 시뮬레이터 키보드 제어, 실물 AP002 Modbus 읽기·키보드 제어.
- AP002 통신 설정(115200 bps, slave 2)과 Modbus 레지스터 표. USB 연결 절차는 Windows → WSL2(`usbipd`) 기준.

</details>

## 이전 환경 (WSL)

이전에는 Windows 노트북 + WSL2 Ubuntu 22.04에서 개발했습니다. 코드는 남아 있지만 지금은 확인하지 않습니다.

- 웹캠: Windows에서 `windows_sender/Py_webCamReader.py`를 실행해 UDP 5005로 보내고, `ros2 launch hand_bringup pipeline.launch.py team_sender:=remote`로 받음 ([hand_tracking README](src/hand_tracking/README.md))
- 실물 USB: Windows에서 `usbipd`로 CH340을 WSL에 연결 ([hardware_connection_test README](hardware_connection_test/README.md))
- MuJoCo 창: WSLg 필요 (`wsl --update`). 다른 PC와 ROS2 통신은 `.wslconfig`의 `networkingMode=mirrored` + 같은 `ROS_DOMAIN_ID`
