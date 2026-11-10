# finger_node — 손가락 노드 + 하드웨어 백엔드

ROHand AP002의 액추에이터 6개마다 하나씩 실행하는 **손가락 노드**와, 손가락 노드들이 함께 쓰는 **하드웨어 백엔드**(MuJoCo 시뮬레이션 / 실물)를 담은 패키지다.

```text
자세 명령 등 ──/finger/<f>/command──▶ finger_node ×6 ──/finger/<f>/state, tactile──▶ 피드백 노드
                                          │  ▲
                        /hand_hw/<f>/target ▼  │ /hand_hw/state, /hand_hw/tactile/<부위>
                                   hand_sim (MuJoCo)  또는  hand_serial (실물 AP002)
```

손가락 노드 6개가 각자 시리얼 포트를 열면 충돌하므로, 포트(또는 물리엔진)는 **백엔드 하나만** 열고 손가락 노드는 백엔드와 토픽으로만 통신한다. 백엔드는 `hand_sim`과 `hand_serial` 중 **하나만** 실행한다.

## 빌드·실행

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select finger_node
source install/setup.bash

# MuJoCo 시뮬레이션 + 손가락 노드 6개
ros2 launch finger_node fingers.launch.py

# 실물 AP002: 먼저 읽기 전용으로 값 확인 → 동작 허용
ros2 launch finger_node fingers.launch.py backend:=real
ros2 launch finger_node fingers.launch.py backend:=real enable_motion:=true

# 다른 터미널에서 명령·확인
ros2 topic pub --once /finger/index/command std_msgs/msg/Float64 "{data: 1.0}"
ros2 topic echo /finger/index/angle_force
```

launch 인자: `backend` (sim/real), `viewer`, `test_object` (ball/none), `port`, `enable_motion`, `real_mode`, `log_period`. 전체 목록은 `ros2 launch finger_node fingers.launch.py --show-args`.

### 준비물

| 대상 | 필요한 것 |
| --- | --- |
| 시뮬레이션 | `pip install --user mujoco` (numpy는 1.x 유지), MuJoCo 모델 `external/rohand_mujoco` (프로젝트 루트에서 실행하거나 `ROHAND_MODEL_XML` 환경변수로 지정) |
| 실물 | `pip install --user pymodbus==3.7.2 pyserial==3.5`, `dialout` 그룹, 24V 전원. USB 연결은 [hardware_connection_test/README.md](../../hardware_connection_test/README.md) 참고 |

**주의:** 실물을 쓸 때는 `hardware_connection_test/rohand_real_ros2.py` 등 같은 시리얼 포트를 쓰는 프로그램과 동시에 실행하지 않는다.

## 손가락 노드 (`finger_node`)

손가락 이름(`finger` 파라미터): `index, middle, ring, little, thumb_root, thumb`

| 토픽 | 타입 | 방향 | 내용 |
| --- | --- | --- | --- |
| `/finger/<f>/command` | `std_msgs/Float64` | 구독 | 목표 각도 (rad). 범위를 벗어나면 잘라서 전달 |
| `/finger/<f>/state` | `sensor_msgs/JointState` | 발행 | 관절 1개: `position` 각도 rad, `velocity` rad/s, `effort` 힘 N |
| `/finger/<f>/angle_force` | `std_msgs/Float64MultiArray` | 발행 | `[각도(deg), 힘(N), 목표 각도(deg)]` 확인용 |
| `/finger/<f>/tactile` | `std_msgs/Float32MultiArray` | 발행 | 손끝 3D 힘 (`thumb_root`는 없음), 백엔드 값을 그대로 전달 |
| `/finger/<f>/tactile_proximal` | `std_msgs/Float32MultiArray` | 발행 | 안쪽 마디 수직항력 (검지·중지·약지) |

- 각도는 **0 = 펴짐**, 값이 클수록 굽힘(엄지 뿌리는 회전). 범위는 MuJoCo AP002 모델 기준 (예: 검지 0~1.45 rad, 엄지 뿌리 0~1.57 rad).
- 실물 백엔드는 속도를 주지 않으므로 연속된 각도 차이로 속도를 계산한다.

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `finger` | `index` | 담당 손가락 |
| `min_angle`, `max_angle` | 손가락별 MuJoCo 범위 | 목표 각도 제한 (rad) |
| `log_period` | `1.0` | 터미널에 각도·힘을 출력하는 주기 (초, 0이면 끔) |
| `hw_state_topic` | `/hand_hw/state` | 백엔드 상태 토픽 |

## 촉각 센서 구성 (ROH-AP002 매뉴얼 7.4, 실물에서 확인)

| 위치 | 센서 | 손가락 | 토픽 | 형식 (`layout.dim[0].label`) |
| --- | --- | --- | --- | --- |
| 손끝 (원위 지골) | 3D 힘: 수직항력 + 접선력 | 5개 모두 | `/finger/<f>/tactile` | `force3d`: `[수직 N, 접선 N, 방향 deg]` |
| 안쪽 마디 (근위 지골) | 수직항력 | 검지·중지·약지 | `/finger/<f>/tactile_proximal` | `normal`: `[수직 N]` |
| 손바닥 | 11×5 점 배열 | — | `/hand_hw/tactile/palm` | 실물 `dot_matrix` 0~255 / 시뮬레이션 `normal` N, 칸마다 1값 |

- 손바닥 메시지는 차원이 2개(`[11 열, 5 행]`)이고, 값의 순서는 `열 × 5 + 행`입니다. 0열이 검지 쪽, 10열이 소지 쪽이고, 0행이 손목 쪽, 4행이 손가락 쪽입니다.
- 엄지 쪽 손목 모서리의 5칸(0, 1, 5, 6, 10번)은 센서가 없어서 NaN입니다.
- 값이 없거나 무효(0xFFFF)이면 NaN입니다.
- 실물 원시값의 단위는 mN이라 1000으로 나눠 N으로 냅니다. 수직항력은 25 N에서 포화되는 것을 실물로 확인했습니다.
- 손바닥은 레지스터 28개를 상위·하위 바이트 순으로 풀어 56개 값을 만들고, 그중 앞의 55개를 씁니다. 제조사 데모 코드는 상위 바이트만 읽지만, 실물에서는 하위 바이트도 변합니다.
- 실물 확인은 `ros2 run finger_node tactile_probe`로 합니다. 읽기 전용이며, 누른 곳의 원시 레지스터 값을 보여 줍니다.

## 하드웨어 백엔드

두 백엔드는 같은 토픽을 쓰므로 손가락 노드는 어느 쪽이든 그대로 동작한다.

| 토픽 | 타입 | 내용 |
| --- | --- | --- |
| `/hand_hw/<f>/target` (구독) | `std_msgs/Float64` | 손가락별 목표 각도 (rad) |
| `/hand_hw/state` (발행) | `sensor_msgs/JointState` | 6개 액추에이터: 각도 rad, 속도(시뮬레이션만), 힘 N |
| `/hand_hw/tactile/<부위>` (발행) | `std_msgs/Float32MultiArray` | 손끝 5개 + 손바닥 11×5 (위 표) |
| `/hand_hw/tactile_proximal/<f>` (발행) | `std_msgs/Float32MultiArray` | 안쪽 마디 (검지·중지·약지) |

### `hand_sim` — MuJoCo

- 공식 AP002 오른손 모델을 불러와 60 Hz로 시뮬레이션하고 상태를 발행한다. `/hand_sim/full_joint_states`(전체 관절)도 발행한다.
- 촉각은 센서가 있는 링크와 물체 사이의 접촉력으로 만든 근사값이며, 배치는 실물과 같습니다.
  - 손끝: `*_distal_link`, 방향은 손끝 링크 좌표계 기준
  - 안쪽 마디: 검지·중지·약지의 `*_proximal_link`
  - 손바닥: `base_link` 접촉 위치를 11×5 칸(`PalmGrid`, 매뉴얼 그림 기준 배치, 칸 간격 5 mm)에 나눠 담음
  - 연결 링크처럼 센서가 없는 곳의 접촉은 버립니다.
- 고정된 테스트 공은 손바닥(역시 고정)과 접촉이 계산되지 않습니다(MuJoCo는 고정 물체끼리의 접촉을 계산하지 않음). 그래서 시뮬레이션에서 손바닥 값은 움직이는 물체가 있을 때만 생깁니다.
- `test_object:=ball`: 손바닥 앞에 고정된 공(반지름 3 cm)을 넣어 쥐었을 때 접촉·힘이 생기게 한다.
- **중력은 기본으로 끈다** (`gravity` 파라미터, 기본 `false`). 공식 모델의 손가락 모터 게인(kp = 1)이 약해서 중력이 있으면 손가락이 처진다 (엄지 뿌리가 명령 0°에서 16.5°). 게인을 높이면 가벼운 링크 때문에 불안정해진다. 실물 손가락도 기어 구동이라 중력으로 처지지 않는다.

### `hand_serial` — 실물 AP002 (Modbus-RTU)

- 115200 bps, Slave ID 2. 시작 시 장치 식별값을 확인하고 다르면 실행을 거부한다.
- 기본은 **읽기 전용**. `enable_motion:=true`일 때만 목표값을 쓴다.
- `real_mode:=position` (기본): `hardware_connection_test`에서 확인한 위치 레지스터 (`1135` 목표 / `1145` 현재, 논리값 0~65535, 0 = 펴짐)를 쓴다. rad ↔ 0~65535는 MuJoCo 범위로 비례 변환하므로 0~1 비율이 시뮬레이션과 같다 (실제 관절 각도가 같다는 보장은 없음).
- `real_mode:=angle`: 펌웨어 각도 레지스터 (`1155`/`1165`)를 쓴다. **실물 미검증**이며 펌웨어 각도(첫 마디와 손바닥 사이)는 MuJoCo 각도와 뜻이 다르다.
- 목표값은 모아 두었다가 주기마다(기본 10 Hz) 6개를 한 번에 쓰고(Modbus `0x10`), 바뀌었을 때만 쓴다. 시작할 때 손에 들어 있는 목표값을 먼저 읽어 명령받지 않은 손가락이 움직이지 않게 한다.
- 힘은 펌웨어 `ROH_FINGER_FORCE`(mN)를 N으로 변환. 촉각 센서 종류(점 배열 / 3D 힘)는 자동 감지한다. 촉각 해석은 위 "촉각 센서 구성" 참고.
- 참고용으로 펌웨어 각도를 `/hand_hw/firmware_angle`(`JointState`, rad)로 따로 발행한다.
- 위치·힘·펌웨어 각도 레지스터는 그룹마다 따로 읽는다. 실물 펌웨어는 그 사이의 안 쓰는 손가락 7~10번 주소가 낀 연속 읽기를 `IllegalAddress`로 거부한다.
- **STOP / START** (두 백엔드 공통): `/hand_hw/motion_enable` (`std_msgs/Bool`)이 `false`이면 측정한 현재 자세를 목표로 한 번 쓰고(실물) 또는 현재 관절값으로 고정하고(시뮬레이션), `true`가 올 때까지 목표를 무시한다. 상태는 `/hand_hw/motion_state` (`running` / `stopped` / `read_only`).
- 레지스터 주소는 제조사 데모(`oymotion/roh_gen2_demos`) 기준이다. 촉각 배치(손끝·안쪽 마디·손바닥 바이트 순서·mN 단위)는 `tactile_probe`로 실물에서 확인했다. 관절 위치·힘 쓰기 동작은 아직 실물로 검증하지 않았다.
