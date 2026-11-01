# gripper_camera — Gemini 305 가상 RGB-D 카메라 (MuJoCo)

Orbbec **Gemini 305** CAD 모델과 가상 **Passive Stereo RGB-D 카메라**를 MuJoCo에 배치하고, 그 앞의 ROHand AP002를 찍은 영상을 확인하는 **독립 테스트 프로그램**이다. ROS2를 쓰지 않으며 `colcon build`도 필요 없다.

ROS2 이미지 토픽, 실물 카메라 연동, 데이터 저장은 아직 구현하지 않았다.

## 준비물

| 항목 | 내용 |
| --- | --- |
| MuJoCo | `pip install --user mujoco` |
| OpenCV | `pip install --user "opencv-contrib-python<4.12"` (hand_tracking과 같은 버전 범위) |
| AP002 모델 | `external/rohand_mujoco` (finger_node와 같음). 다른 위치면 `--hand-xml` 또는 `ROHAND_MODEL_XML` |
| 화면 | 영상 창 표시 (WSL은 WSLg) |

## 실행

```bash
cd ~/TactileData_Collection_Pipeline
python3 gripper_camera/gemini305_sim.py              # 기본 848x530
python3 gripper_camera/gemini305_sim.py --viewer     # MuJoCo 3D 창도 함께 (카메라 모델 확인)
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

예: `python3 gripper_camera/gemini305_sim.py --pos -0.10 0.0 -0.15 --euler 0 10 -90`

## 카메라 모델 (`model/gemini305.xml`)

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
| `gemini305_sim.py` | 테스트 프로그램 (장면 구성, 가상 카메라, 영상 선택·출력) |
| `model/gemini305.xml` | 카메라 body, 메시, 가상 카메라 3개 |
| `model/meshes/gemini305_housing.stl` | Gemini 305 외형 메시 (CAD 좌표, mm) |
| `convert_cad.py` | STEP → STL 변환 스크립트 (1회용) |

### CAD 변환 (다시 만들 때만)

공식 [Gemini 305 CAD_V1.1](https://doc.orbbec.com/documentation/Gemini%20305%20Documentation/Download%20CAD%20Files%20of%20Gemini%20305) ZIP을 받아 STEP 파일을 변환한다. OpenCascade 바인딩은 이 스크립트에만 필요하다 (용량이 커서 별도 폴더 설치 권장).

```bash
pip install --target /tmp/ocp cadquery-ocp
PYTHONPATH=/tmp/ocp python3 gripper_camera/convert_cad.py "<압축 해제 경로>/Gemini 305 CAD_V1.1/3D/3D__00_mb2502_asm(G305)_ V1.1.stp"
```

STEP 어셈블리 중 외형(하우징 + 전면 유리)만 내보낸다. 렌즈는 유리 뒤에 가려지고, 나머지 하나는 FOV 원뿔 등 면으로만 된 보조 형상이라 가상 카메라 시야를 가리므로 뺐다.

## 성능 (WSL2, 소프트웨어 렌더링, 848x530)

| 영상 | 속도 |
| --- | --- |
| RGB / Stereo Left / Stereo Right | 약 20~27 fps |
| Disparity / Depth | 약 7 fps (`640x400`이면 약 9 fps) |

WSL2의 GPU 드라이버(Mesa d3d12)에서는 MuJoCo 오프스크린 렌더링 결과가 검게 나와서, WSL에서는 자동으로 소프트웨어 렌더링(`LIBGL_ALWAYS_SOFTWARE=1`)을 쓴다. 일반 Ubuntu에서는 GPU를 그대로 쓴다. 속도를 위해 가상 카메라 렌더링에서는 그림자와 안티에일리어싱을 끈다.
