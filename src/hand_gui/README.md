# hand_gui — 촉각 데이터 수집 GUI

로봇 손에 들어오는 힘(크기·방향)을 보면서 데이터를 녹화하고 Hugging Face에 올리는 프로그램이다.

```text
┌ [녹화|업로드|보정] ┬ 로봇 손 (MuJoCo, 힘 표시)           ┬ 웹캠 (사람 손) ──────┐
│ 작업 · 메모        │  · 원: 누르는 힘 (크기·초록→빨강)    ├ 로봇 핸드 카메라 ────┤
│ ● 녹화            │  · 파란 화살표: 미끄러지는 힘·방향   │                      │
│ 저장 폴더          ├ 액추에이터 힘 │ 촉각 누르는 힘 │ 촉각 미끄러지는 힘 (10초) ┤
│ 저장된 에피소드     │                                                          │
└──────────────────┴ 상태: 손가락·촉각·웹캠·카메라 수신 여부 ───────────────────┘
```

다크 모드. 아래 그래프는 화면 높이의 약 1/4 (경계선을 드래그해 조절). 왼쪽 탭: **녹화** (작업·메모·녹화·저장 폴더·에피소드 목록), **업로드** (Hugging Face), **보정** (웹캠 손 보정 초기화, 로봇 손 시점 초기화).

## 실행

```bash
pip install --user "pyqtgraph==0.13.7" "numpy<2"     # 최초 1회 (numpy 2는 ROS2 Humble과 충돌)

# 파이프라인과 함께 (GUI 창 하나만 뜸: MuJoCo 창·웹캠 창은 닫아 둠)
ros2 launch hand_bringup pipeline.launch.py gui:=true

# 따로 (파이프라인이 이미 실행 중일 때)
ros2 run hand_gui hand_gui
ros2 run hand_gui hand_gui --ros-args -p data_dir:=~/tactile_dataset -p test_object:=ball
```

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `data_dir` | `~/tactile_dataset` | 에피소드 저장 폴더 (화면에서도 변경 가능) |
| `test_object` | `none` | `ball`: 시뮬레이션의 테스트 공을 로봇 손 화면에도 표시 |
| `hand_xml` | `''` | AP002 MJCF 경로 (비우면 `external/rohand_mujoco/...`) |
| `robot_camera_topic` | `/gripper_camera/color/image_raw` | 로봇 핸드 카메라 영상 (`sensor_msgs/Image`, rgb8/bgr8/mono8) |
| `webcam_topic` | `/webcam/image_annotated/compressed` | 웹캠 영상 (`sensor_msgs/CompressedImage`) |
| `webcam_source` | `none` | 시작할 때 선택할 카메라: `none`, `network`, `/dev/videoN` (`hand_bringup`이 자동으로 지정) |

`gui:=true`이면 MuJoCo 창과 웹캠 창은 열리지 않는다. 웹캠 창의 보정 키(`F` 등)는 확인용 값에만 쓰이고, 로봇 제어용 보정은 자세 명령 노드가 자동으로 하므로 **보정** 탭의 초기화 버튼으로 충분하다.

WSL에서는 로봇 손 화면이 깨지지 않도록 소프트웨어 렌더링(`LIBGL_ALWAYS_SOFTWARE=1`)을 자동으로 켠다.

## 웹캠 선택

웹캠 칸 위의 목록에서 손 인식에 쓸 카메라를 고른다. 어떤 카메라를 골라도 영상은 같은 토픽으로, 손 좌표는 같은 UDP 포트로 들어오므로 나머지 파이프라인은 그대로다. 녹화 중에는 바꿀 수 없다.

| 항목 | 동작 | 환경 |
| --- | --- | --- |
| `USB: 이름 (/dev/videoN)` | GUI가 `linux_webcam_sender`를 창 없이 실행한다. 다른 카메라를 고르거나 GUI를 닫으면 끈다 | 네이티브 Ubuntu |
| `Windows 카메라 (PowerShell 연결)` | PowerShell에서 `windows_webcam_sender.py`를 실행해 연결한다. 영상은 `webcam_image_receiver`가 받는다 | WSL |
| `사용 안 함` | GUI는 아무것도 실행하지 않고, 토픽에 들어오는 영상만 보여 준다 | 공통 |

USB 카메라를 새로 꽂으면 **새로고침**을 누른다.

```bash
# 네이티브 Ubuntu: /dev/video0으로 시작, 다른 USB 카메라는 GUI에서 선택
ros2 launch hand_bringup pipeline.launch.py gui:=true
ros2 launch hand_bringup pipeline.launch.py gui:=true camera_device:=2   # 시작 카메라 지정

# WSL: 같은 명령. /dev/video0이 없으면 자동으로 Windows 카메라를 선택
ros2 launch hand_bringup pipeline.launch.py gui:=true
```

WSL에서는 PowerShell에서 카메라를 연결한다. 준비는 [hand_tracking README](../hand_tracking/README.md)의 Windows 환경 구성과 같다.

```powershell
conda activate rohand_mp
cd "\\wsl.localhost\Ubuntu-22.04\home\<사용자이름>\TactileData_Collection_Pipeline\windows_sender"
python windows_webcam_sender.py --camera 0     # 다른 카메라면 번호 변경, 종료는 Ctrl+C
```

웹캠 창과 `F`/`R`/`S`/`C`/`Q` 키가 필요하면 `--window`를 붙인다.

`ros2 run hand_gui hand_gui`로 따로 실행하면 기본값이 `none`이라 이미 실행 중인 웹캠과 충돌하지 않는다. 카메라를 바로 쓰려면 `-p webcam_source:=/dev/video2`처럼 지정한다. Windows 카메라를 쓰려면 `ros2 run hand_tracking webcam_image_receiver`도 실행되어 있어야 한다 (`hand_bringup`은 자동으로 켬).

## 화면

- **로봇 손:** 파이프라인의 손 자세를 따라 움직인다 (`/hand_sim/full_joint_states`, 없으면 `/hand_hw/state`). 기본 시점은 손바닥이 보이고 엄지가 위, 손가락이 왼쪽. 마우스 드래그로 회전, 휠로 확대, **보정** 탭에서 시점 초기화.
  - 원: 부위별 누르는 힘. 크기와 색(초록 → 빨강)은 지금까지의 최대값 기준이며 천천히 줄어든다. 옆에 부위 이름과 값.
  - 파란 화살표: 미끄러지는 힘. 길이 = 크기, 방향 = 센서가 보고한 각도를 손끝 좌표계에 그린 것. 실물 3D 힘 센서의 방향 기준은 아직 실물로 확인하지 않았다. 점 배열 센서는 방향 정보가 없어 원만 나온다.
  - 힘 표시는 3D 장면 위에 2D로 덧그려서 손가락이나 물체에 가려지지 않는다. 시뮬레이션의 테스트 공은 반투명으로 보인다.
- **그래프 (최근 10초):** 액추에이터 힘 (N) / 촉각 누르는 힘 / 촉각 미끄러지는 힘. 촉각 단위는 시뮬레이션은 N, 실물은 센서 원시값.

## 녹화

1. **녹화** 탭에서 작업(파지 종류)을 고르거나 입력하고, 메모를 적는다.
2. **● 녹화 시작** → 동작 → **■ 녹화 정지**. 한 번이 에피소드 하나이며, 아래 목록에 추가된다.

30 Hz로 저장한다. 0.5초 넘게 끊긴 값은 NaN으로 남겨 빈 구간이 보이게 한다.

```text
~/tactile_dataset/episode_000001/
  data.csv          프레임마다: 시간, 손가락별 명령·각도·속도·힘, 부위별 촉각(누름·미끄러짐·방향), 사람 손 관절 21개
  tactile_raw.npz   부위별 촉각 원본 배열 [프레임, 값]
  webcam.mp4        웹캠 영상 (받은 경우)
  robot_camera.mp4  로봇 핸드 카메라 영상 (받은 경우)
  meta.json         작업, 메모, fps, 단위, 손가락·부위 순서, 촉각 센서 종류
```

## Hugging Face 업로드

```bash
pip install --user "huggingface_hub<1"    # 최초 1회
huggingface-cli login                      # 토큰 입력 (또는 업로드 창에 직접 입력)
```

**업로드** 탭의 **Hugging Face에 업로드** 버튼 → 저장소(`아이디/데이터셋이름`), 비공개 여부 → 확인하면 저장 폴더 전체를 데이터셋 저장소로 올린다. 기본은 비공개다. 웹캠 영상에 얼굴이 나오면 공개 전에 확인한다.

## 알아 둘 점

- 저장 형식은 이 프로그램 자체 형식(`format_version: 1`)이다. LeRobot 형식이 필요하면 변환기를 따로 만든다.
- 녹화는 30 Hz 전용 타이머로 하고 화면은 15 Hz로 갱신한다. `meta.json`의 `measured_fps`가 실제 저장 속도다.
- 웹캠·로봇 카메라 영상은 녹화 시점의 최신 프레임을 30 Hz로 저장한다 (원래 영상이 15 Hz면 같은 프레임이 반복된다).
