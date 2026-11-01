# Hand Tracking → ROS 2 데이터 전송

Windows 웹캠에서 MediaPipe로 손 관절 좌표를 측정하고, UDP를 통해 WSL(Ubuntu)의 ROS 2 토픽으로 전송하는 프로그램입니다. 관절 좌표는 자세 명령 노드(`pose_command`)가 받아 로봇 손가락 각도로 바꿉니다.

## 1. 실행 구조

```text
Windows 웹캠 → Py_webCamReader.py (MediaPipe)
             → UDP / 5005
             → WSL hand_tracking_node (ROS 2 Humble)
             → /hand_landmarks      (Float32MultiArray, 관절 21개 × xyz)  → pose_command_node
             → /hand/finger_angles  (Float32MultiArray, 보정된 손가락 값 6개, 확인·기록용)
```

| 토픽 | 내용 | 전송 조건 |
|---|---|---|
| `/hand_landmarks` | MediaPipe world landmark 21개 × (x, y, z) = 63개 (m). 순서는 MediaPipe 기준 0~20번 | 손이 인식되면 **매 프레임** (보정 불필요) |
| `/hand/finger_angles` | 보정된 손가락 값 6개 (0~1) | 6개 손가락 모두 보정 범위가 생긴 뒤 |

UDP 메시지 형식 (JSON): `{"landmarks": [63개], "values": [6개]}`. `values`는 보정 전에는 없습니다.

**총 3개의 콘솔**을 사용합니다. 콘솔 ①과 ③은 **WSL Ubuntu**, 콘솔 ②는 **Windows PowerShell**입니다.

| 콘솔 | 환경 | 역할 |
|---|---|---|
| ① | WSL Ubuntu | ROS 2 UDP 수신 노드 실행 |
| ② | Windows PowerShell | 웹캠·손 추적 프로그램 실행 |
| ③ | WSL Ubuntu | ROS 2 토픽 데이터 확인 |

## 2. 최초 1회 준비

### WSL: ROS 2 패키지 빌드

ROS 2 Humble과 `colcon`이 설치되어 있어야 합니다. 프로젝트가 아래 경로에 있다고 가정합니다. 이 패키지는 `src/hand_tracking/`에 있습니다.

```bash
~/TactileData_Collection_Pipeline
```

WSL에서 프로젝트 루트로 이동해 다음을 실행합니다.

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash
colcon build --packages-select hand_tracking --symlink-install
```

> 빌드는 처음 설치하거나 ROS 2 패키지 코드를 수정했을 때만 다시 수행하면 됩니다.

### Windows: Python 환경 준비

Windows PowerShell에서 실행합니다. 아래는 기존에 작동 확인한 Python 3.11 환경 구성 예시입니다.

```powershell
conda create -n rohand_mp python=3.11 -y
conda activate rohand_mp
python -m pip install "mediapipe==0.10.21" "opencv-python" "numpy<2"
```

이미 작동하는 환경이 있다면 새로 만들지 않고 해당 환경을 활성화하면 됩니다. `Py_webCamReader.py`는 기존 `mp.solutions.hands` API를 사용하므로 호환되는 MediaPipe 버전이 필요합니다.

`windows_sender` 폴더에 다음 파일이 있는지 확인합니다.

```text
windows_sender/
├── Py_webCamReader.py
└── network_config.py
```

`network_config.py`는 WSL IP와 UDP 포트 설정을 가져오는 파일입니다. 기본 UDP 포트는 **5005**입니다. 프로젝트 루트에 `.env` 파일이 있다면 `WSL_IP` 및 `UDP_PORT`를 지정할 수 있으며, 설정하지 않은 경우 WSL IP 자동 탐지를 사용합니다.

## 3. 매번 실행하는 방법

### 콘솔 ① — WSL Ubuntu: ROS 2 수신 노드

**첫 번째 WSL 터미널**을 열고 아래 명령을 실행합니다.

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hand_tracking hand_tracking_node
```

정상 실행 시 다음과 비슷한 로그가 표시됩니다.

```text
UDP receiver started: 0.0.0.0:5005
Publishing topics: /hand_landmarks, /hand/finger_angles
```

**이 터미널을 종료하지 마세요.** 같은 노드를 중복 실행하면 UDP 포트 충돌(`Address already in use`)이 발생할 수 있습니다.

### 콘솔 ② — Windows PowerShell: 웹캠 실행

**새 Windows PowerShell**을 열고 Python 환경을 활성화합니다.

```powershell
conda activate rohand_mp
```

WSL에 있는 프로젝트의 `windows_sender` 폴더로 이동합니다.

```powershell
cd "\\wsl.localhost\Ubuntu-22.04\home\<사용자이름>\TactileData_Collection_Pipeline\windows_sender"
python Py_webCamReader.py
```

> `Ubuntu-22.04`와 `<사용자이름>`은 예시입니다. 각 팀원의 **WSL 배포판 이름과 Linux 사용자 이름**에 맞게 바꿔야 합니다. 배포판 이름은 PowerShell에서 `wsl -l -q`로 확인할 수 있습니다.
>
> WSL에서 `cd ~/TactileData_Collection_Pipeline/windows_sender && explorer.exe .`를 실행하면 Windows 파일 탐색기로 해당 폴더를 바로 열 수 있습니다.

웹캠 화면이 나타나면 손을 카메라에 보여주고 손가락을 움직입니다. **측정값은 자동 전송**되므로 별도로 전송 버튼을 누를 필요는 없습니다.

#### 웹캠 조작 키

| 키 | 동작 |
|---|---|
| `F` | 보정값 고정 / 보정 재개 (모든 채널 보정 후 고정 가능) |
| `C` | 보정값 초기화 |
| `R` | 현재 측정값을 기록 목록에 추가 |
| `S` | 기록 목록을 `measurements.csv`로 저장 |
| `Q` | 웹캠 프로그램 종료 |

처음 사용할 때는 손가락을 펴고 굽히는 동작으로 보정 범위를 충분히 학습시킨 다음 `F`로 보정을 고정하면 됩니다.

### 콘솔 ③ — WSL Ubuntu: 측정값 확인

**두 번째 WSL 터미널**을 열고 다음 명령을 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/TactileData_Collection_Pipeline/install/setup.bash
ros2 topic echo /hand/finger_angles
```

손이 인식되면 `ros2 topic hz /hand_landmarks`가 웹캠 프레임 속도(약 30 Hz)로 나와야 합니다. 보정 후에는 손가락을 움직일 때 다음과 같이 **6개 값이 계속 변경되면 정상 작동**입니다.

```yaml
data:
- 0.12
- 0.45
- 0.78
- 0.23
- 0.56
- 0.34
---
```

토픽 타입은 `std_msgs/msg/Float32MultiArray`이고, 값은 `0.0~1.0` 범위로 정규화됩니다.

> **방향 주의:** 이 값은 확인·기록용이며 로봇 핸드 제어에는 쓰지 않습니다 (제어는 `/hand_landmarks` → `pose_command`). 6개 값 모두 로봇 핸드와 방향이 반대이므로, 다른 곳에 쓸 때는 `1 - 값`으로 뒤집어야 합니다.
> - 굽힘 5개(순서 0~4): **1.0 = 펴짐, 0.0 = 굽힘** (`Py_webCamReader.py`의 `fg_bend = 1.0 - normalized`). 로봇 핸드(MuJoCo·실물)와 자세 명령 노드는 **0 = 펴짐, 1 = 굽힘**입니다.
> - 엄지 회전(순서 5): 웹캠 테스트에서 로봇 핸드의 엄지 뿌리 회전과 반대 방향으로 확인되었습니다.

| 순서 (0부터) | 의미 |
|---|---|
| 0 | 엄지 굽힘 |
| 1 | 검지 굽힘 |
| 2 | 중지 굽힘 |
| 3 | 약지 굽힘 |
| 4 | 소지 굽힘 |
| 5 | 엄지 회전 |

## 4. 종료 방법

1. **Windows 웹캠 창**에서 `Q`를 누릅니다.
2. **콘솔 ③**에서 `Ctrl+C`를 누릅니다.
3. **콘솔 ①**에서 `Ctrl+C`를 누릅니다.

## 5. 문제 해결

| 증상 | 확인할 사항 |
|---|---|
| `No module named 'network_config'` | `windows_sender/network_config.py` 파일명과 위치 확인 (`.py` 확장자 필수) |
| `mediapipe has no attribute 'solutions'` | Windows Python 버전 및 MediaPipe 호환 버전 확인. Python 3.11 + MediaPipe 0.10.21 구성 권장 |
| `Address already in use` | WSL에서 `sudo ss -lunp \| grep ':5005'`로 포트 점유 프로세스 확인 후 중복 노드 종료 |
| `/hand_landmarks`, `/hand/finger_angles`가 목록에 없음 | 콘솔 ①의 노드 실행 여부 및 각 터미널의 ROS 2 환경 `source` 여부 확인 |
| `/hand_landmarks`는 나오는데 `/hand/finger_angles`가 없음 | 아직 보정 전. 손가락을 끝까지 쥐었다 폈다 하고 엄지를 돌려 6개 모두 보정 |
| 토픽은 있지만 데이터가 없음 | Windows 웹캠에서 손이 인식되는지, Windows가 올바른 WSL IP/UDP 5005로 보내는지 확인 |

### ROS 2 수신 경로만 테스트하기

콘솔 ①과 ③이 실행 중인 상태에서 **추가 WSL 터미널**에 아래 명령을 입력합니다.

```bash
python3 -c 'import socket,json; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.sendto(json.dumps({"landmarks":[0.0]*63,"values":[0.1,0.2,0.3,0.4,0.5,0.6]}).encode(),("127.0.0.1",5005)); s.close()'
```

콘솔 ③에 `0.1`부터 `0.6`까지 출력되고 `ros2 topic echo --once /hand_landmarks`에 0이 63개 나온다면 **WSL UDP 수신 및 ROS 2 토픽 발행은 정상**입니다. 실제 웹캠 값이 안 나온다면 Windows 송신 프로그램과 네트워크 설정을 점검합니다.

---

**핵심 실행 순서:** `WSL 수신 노드` → `Windows 웹캠 프로그램` → `WSL 토픽 확인`
