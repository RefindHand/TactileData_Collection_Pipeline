# gripper_camera — Gemini 305 가상 RGB-D 카메라 (MuJoCo)

Orbbec **Gemini 305** CAD 모델과 가상 **Passive Stereo RGB-D 카메라**를 MuJoCo에 배치하고, 그 앞의 ROHand AP002를 찍은 영상을 만드는 패키지다. 두 가지로 실행할 수 있다.

| 실행 | 내용 |
| --- | --- |
| **ROS2 카메라 노드** `camera_node` | 선택한 영상을 `sensor_msgs/Image` + `CameraInfo` 토픽으로 발행 |
| **독립 테스트** `gemini305_sim.py` | ROS 없이 선택한 영상을 OpenCV 창으로 바로 확인 |

두 실행 방식은 같은 장면·렌더링·Depth 계산 코드(`gemini305_sim.py`)를 쓴다. ROS2 노드는 파이프라인이 실행 중이면 영상 속 손이 실제 손 자세를 따라간다 (아래 [손 자세 따라가기](#손-자세-따라가기)). `ros2 launch hand_bringup pipeline.launch.py`에 포함되어 있다 (기본 `hand_camera:=virtual`). 실물 핸드 카메라는 현재 Intel RealSense D405를 쓰며 (이전에는 D415), 이 패키지가 아니라 RealSense ROS 드라이버로 `hand_bringup`의 `hand_camera:=d405`에서 실행한다. 영상 저장은 `hand_gui` 녹화가 한다.

## 준비물

| 항목 | 내용 |
| --- | --- |
| MuJoCo | `pip install --user mujoco` |
| OpenCV | `pip install --user "opencv-contrib-python<4.12"` (hand_tracking과 같은 버전 범위) |
| AP002 모델 | `external/rohand_mujoco` (finger_node와 같음). 다른 위치면 `--hand-xml` 또는 `ROHAND_MODEL_XML` |
| 화면 | 독립 테스트의 영상 창 / rqt_image_view (WSL은 WSLg) |
| 영상 뷰어 (선택) | `sudo apt install ros-humble-rqt-image-view` |

## ROS2 카메라 노드

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gripper_camera
source install/setup.bash

ros2 run gripper_camera camera_node                                  # 기본 mode:=rgb
ros2 run gripper_camera camera_node --ros-args -p mode:=depth
```

출력 모드는 세 가지 방법으로 고른다.

- 실행 파라미터: `--ros-args -p mode:=rgb|left|right|disparity|depth`
- 노드를 실행한 터미널에서 번호 입력 (`1`~`5`, `0` = 노드 종료). launch처럼 터미널 입력이 없으면 메뉴는 생략된다.
- 실행 중 변경: `ros2 param set /gripper_camera mode disparity`

**선택한 모드의 토픽만 발행**하고, 그 토픽을 구독하는 곳이 없으면 렌더링·스테레오 매칭도 하지 않는다 (물리 시뮬레이션만 진행).

### 영상 확인

```bash
ros2 run rqt_image_view rqt_image_view /gripper_camera/color/image_raw
ros2 run rqt_image_view rqt_image_view /gripper_camera/depth/image_color   # Depth를 색으로 보기
ros2 topic hz /gripper_camera/depth/image_raw
```

rqt_image_view는 위의 apt 패키지가 필요하다. `depth/image_raw`(미터, 0.1~0.5 m 정도)나 `disparity/image_raw`(픽셀)를 rqt_image_view로 바로 보면 범위가 맞지 않아 어둡게 보일 수 있다. 이때는 `image_color` 토픽을 보거나, 창의 최대 거리 값을 0.5 정도로 줄이거나 dynamic range를 켠다.

### 토픽

모든 토픽은 노드 이름 아래(`/gripper_camera/...`)에 있다.

| mode | 영상 토픽 | 인코딩 | CameraInfo | frame_id |
| --- | --- | --- | --- | --- |
| `rgb` | `color/image_raw` | `rgb8` | `color/camera_info` | `gemini305_color_optical_frame` |
| `left` | `left/image_raw` | `rgb8` | `left/camera_info` | `gemini305_left_optical_frame` |
| `right` | `right/image_raw` | `rgb8` | `right/camera_info` | `gemini305_right_optical_frame` |
| `disparity` | `disparity/image_raw` | `32FC1`, **픽셀** | `disparity/camera_info` (왼쪽 카메라) | `gemini305_left_optical_frame` |
| `depth` | `depth/image_raw` | `32FC1`, **미터** | `depth/camera_info` (왼쪽 카메라) | `gemini305_left_optical_frame` |

- Disparity·Depth는 왼쪽 카메라 기준으로 정렬되어 있고, 매칭에 실패한 픽셀은 `NaN`이다 (REP 118).
- `disparity`/`depth` 모드는 보기용 컬러 영상 `disparity/image_color`, `depth/image_color`(`bgr8`, 독립 테스트 창과 같은 그림)도 발행한다. 구독자가 있을 때만 만든다.
- 영상과 CameraInfo는 같은 타임스탬프(렌더링 시점의 노드 시각)를 쓴다.
- CameraInfo는 왜곡 없는 이상적인 핀홀: `K = [fx 0 cx; 0 fx cy; 0 0 1]`, `cx = (W-1)/2`, `cy = (H-1)/2`. 오른쪽 카메라 `P[0,3] = -fx × baseline` (ROS 스테레오 규칙). fx는 848x530 기준 Stereo 410.7 px, RGB 392.9 px (컬러 FOV가 더 넓음).
- TF는 아직 발행하지 않는다.

### 파라미터

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `mode` | `rgb` | `rgb`, `left`, `right`, `disparity`, `depth` (실행 중 변경 가능) |
| `resolution` | `848x530` | `1280x800`, `848x530`, `640x400` |
| `rate` | `30.0` | 발행 주기 상한 (Hz). Disparity/Depth는 계산 시간 때문에 더 느림 |
| `static` | `false` | `true`면 손가락을 움직이지 않음 |
| `hand_xml` | `''` | AP002 MJCF 경로. 비우면 `ROHAND_MODEL_XML` 또는 프로젝트의 `external/rohand_mujoco/...` |
| `follow` | `true` | 파이프라인의 손 자세를 따라감 (아래 참고). `false`면 항상 혼자 움직임 |

`mode` 외의 파라미터는 시작할 때만 정할 수 있다. 카메라 위치·방향은 `gripper_camera/model/gemini305.xml`에서 바꾼다.

### 손 자세 따라가기

카메라 노드는 자기 MuJoCo 장면을 따로 갖고 있으므로, 파이프라인의 손 상태를 받아 영상 속 손을 맞춘다 (`finger_node` 패키지 필요).

| 받는 토픽 | 보내는 곳 | 방법 |
| --- | --- | --- |
| `/hand_sim/full_joint_states` | `hand_sim` (MuJoCo 백엔드) | 전체 관절 값을 그대로 복사 → 시뮬레이션과 똑같은 자세 |
| `/hand_hw/state` | `hand_sim` 또는 `hand_serial` (실물) | 액추에이터 6개 각도를 목표로 넣고 카메라 장면의 물리로 따라감 |

- 둘 다 오면 `/hand_sim/full_joint_states`를 쓰고, 0.5초 넘게 끊기면 `/hand_hw/state`를 쓴다.
- 아무것도 받기 전에는 지금처럼 손가락이 혼자 쥐었다 편다 (`static:=true`면 정지). 한 번 따라간 뒤 입력이 끊기면 마지막 자세를 유지한다.
- 따라가기를 켜면(`follow:=true`) 카메라 장면의 중력도 끈다. `hand_sim`과 같은 이유로, 약한 손가락 모터가 `/hand_hw/state` 각도에 정확히 도달하게 하기 위해서다.
- 카메라 장면에는 `hand_sim`의 테스트 공(`test_object:=ball`)이 없다. 시뮬레이션에서 공에 막힌 손가락은 영상에서 공중에 멈춘 것처럼 보인다.

## 독립 테스트 (OpenCV 창)

```bash
cd ~/TactileData_Collection_Pipeline
python3 src/gripper_camera/gripper_camera/gemini305_sim.py              # 기본 848x530 (빌드 불필요)
python3 src/gripper_camera/gripper_camera/gemini305_sim.py --viewer     # MuJoCo 3D 창도 함께 (카메라 모델 확인)
ros2 run gripper_camera gemini305_sim                                   # 빌드 후에는 이렇게도 실행 가능
```

터미널에 번호를 입력하면 해당 영상이 별도 창에 실시간으로 나온다. 창이 떠 있는 동안에도 다른 번호를 입력해 바꿀 수 있다.

```text
  1. RGB
  2. Stereo Left
  3. Stereo Right
  4. Disparity Map
  5. Depth Map
  0. 종료
```

- 선택한 영상만 렌더링·계산한다. Disparity/Depth를 고르면 좌·우 Stereo 영상만 내부에서 렌더링해 시차를 계산한다.
- 영상 창에서 `q`/`ESC`를 누르면 창만 닫힌다. 종료는 `0` 또는 `Ctrl+C`.
- 손가락은 4초 주기로 쥐었다 폈다 한다 (`--static`이면 정지).
- Disparity/Depth 창 왼쪽 위에 화면 중앙(+ 표시)의 값이 표시된다. Depth 색은 7 cm(빨강) ~ 50 cm(파랑), 계산 실패한 픽셀은 검정.

| 옵션 | 기본값 | 설명 |
| --- | --- | --- |
| `--resolution` | `848x530` | `1280x800`, `848x530`, `640x400` (데이터시트 16:10 스트림) |
| `--pos X Y Z` | XML 값 | 카메라 위치 (m, 월드 좌표) |
| `--euler RX RY RZ` | XML 값 | 카메라 방향 (deg, MuJoCo intrinsic xyz) |
| `--static` | 꺼짐 | 손가락을 움직이지 않음 |
| `--viewer` | 꺼짐 | MuJoCo 3D 창. 카메라를 더블클릭 후 `Ctrl+우클릭 드래그`(이동) / `Ctrl+좌클릭 드래그`(회전)하면 가상 카메라 영상도 따라 바뀐다 |
| `--hand-xml` | `external/rohand_mujoco/AP002/...` | AP002 MJCF 경로 |

예: `python3 src/gripper_camera/gripper_camera/gemini305_sim.py --pos -0.10 0.0 -0.15 --euler 0 10 -90`

## 카메라 모델 (`gripper_camera/model/gemini305.xml`)

카메라 3D 모델과 가상 카메라 3개가 모두 `gemini305` body 하나에 붙어 있다. **body의 `pos`/`quat`만 바꾸면 모델과 카메라가 함께 움직인다.**

```text
gemini305 (body, 원점 = 왼쪽 RGB 모듈 광학 중심, +X 오른쪽 / +Y 아래 / +Z 앞)
├── gemini305_housing  CAD 메시 (geom group 3, 가상 카메라에는 보이지 않음)
├── rgb                (0, 0, 0)      컬러 FOV  H 94° x V 68°
├── stereo_left        (0, 0, 0)      fx = fy = 410.7 px @ 848x530
└── stereo_right       (+18 mm, 0, 0) baseline 18 mm
```

| 항목 | 값 | 근거 (Gemini 305 Datasheet V1.0) |
| --- | --- | --- |
| 방식 | Stereo Vision (좌·우 컬러 글로벌 셔터) | 2. Specifications |
| Baseline | 18 mm | 2D 도면, 2. Specifications |
| 깊이·컬러 원점 | 왼쪽 RGB 모듈 광학 중심, 전면 유리 뒤 3.95 mm | 4.9, 4.10 |
| Stereo 내부 파라미터 | 1280x800: f = 620, 848x530: f = 410.7, 주점 = 영상 중심 | 4.7.2 |
| RGB FOV | H 94° x V 68° (왼쪽 모듈 영상) | 3.4.2 |
| 시차 탐색 범위 | 128 px → 최소 거리 58 mm @ 848x530 (데이터시트 60 mm) | 4.8 |

- 실제 장비의 RGB 스트림은 왼쪽 모듈 영상이므로 `rgb` 카메라는 `stereo_left`와 같은 위치에 있고 FOV만 컬러 FOV(조금 더 넓음)를 쓴다.
- 가상 카메라는 이미 정렬(rectify)된 이상적인 핀홀 카메라다. 렌즈 왜곡, 노이즈, IR 프로젝터는 구현하지 않았다.
- Depth는 MuJoCo 깊이 버퍼가 아니라 **좌·우 영상의 스테레오 매칭(OpenCV StereoSGBM)으로 얻은 시차**에서 계산한다: `depth = fx × baseline / disparity`.
- 패시브 스테레오는 무늬가 있어야 매칭되므로 손 뒤에 랜덤 점 무늬 배경판을 둔다. 실제 센서처럼 왼쪽 가장자리(시차 탐색 범위만큼)와 가려진 영역은 깊이가 비어 있다.

### 기본 배치

AP002 오른손은 월드 원점에 고정되어 있고 손가락은 -X, 손바닥은 -Z를 향한다. 카메라는 손바닥 앞 20 cm(`pos="-0.10 -0.02 -0.20"`)에서 +Z(손바닥)를 보며, 영상 위쪽이 손가락 끝이다.

## 파일

| 파일 | 내용 |
| --- | --- |
| `gripper_camera/gemini305_sim.py` | 장면 구성, 가상 카메라(렌더링·스테레오 Depth), 독립 테스트 프로그램 |
| `gripper_camera/camera_node.py` | ROS2 카메라 노드 (위 모듈을 그대로 사용) |
| `gripper_camera/model/gemini305.xml` | 카메라 body, 메시, 가상 카메라 3개 |
| `gripper_camera/model/meshes/gemini305_housing.stl` | Gemini 305 외형 메시 (CAD 좌표, mm) |
| `convert_cad.py` | STEP → STL 변환 스크립트 (1회용, 설치하지 않음) |

### CAD 변환 (다시 만들 때만)

공식 [Gemini 305 CAD_V1.1](https://doc.orbbec.com/documentation/Gemini%20305%20Documentation/Download%20CAD%20Files%20of%20Gemini%20305) ZIP을 받아 STEP 파일을 변환한다. OpenCascade 바인딩은 이 스크립트에만 필요하다 (용량이 커서 별도 폴더 설치 권장).

```bash
pip install --target /tmp/ocp cadquery-ocp
PYTHONPATH=/tmp/ocp python3 src/gripper_camera/convert_cad.py "<압축 해제 경로>/Gemini 305 CAD_V1.1/3D/3D__00_mb2502_asm(G305)_ V1.1.stp"
```

STEP 어셈블리 중 외형(하우징 + 전면 유리)만 내보낸다. 렌즈는 유리 뒤에 가려지고, 나머지 하나는 FOV 원뿔 등 면으로만 된 보조 형상이라 가상 카메라 시야를 가리므로 뺐다.

## 성능 (WSL2, 소프트웨어 렌더링, 848x530)

| 영상 | 독립 테스트 창 | ROS2 토픽 |
| --- | --- | --- |
| RGB / Stereo Left / Stereo Right | 약 20~30 fps | 약 29~30 Hz (`rate` 30) |
| Disparity / Depth | 약 7~10 fps | 약 9~11 Hz |

WSL2의 GPU 드라이버(Mesa d3d12)에서는 MuJoCo 오프스크린 렌더링 결과가 검게 나와서, WSL에서는 자동으로 소프트웨어 렌더링(`LIBGL_ALWAYS_SOFTWARE=1`)을 쓴다. 일반 Ubuntu에서는 GPU를 그대로 쓴다. 속도를 위해 가상 카메라 렌더링에서는 그림자와 안티에일리어싱을 끈다.
