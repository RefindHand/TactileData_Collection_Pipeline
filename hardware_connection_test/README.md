# Hardware Connection Test — ROHand AP002 / ROS2 / MuJoCo

> **프로젝트:** `TactileData_Collection_Pipeline`  
> **환경:** Windows 노트북 + WSL2 Ubuntu 22.04 + ROS2 Humble  
> **로봇:** OYMotion ROHand AP002 오른손 (6개 구동 축)  
> **문서 업데이트:** 2026-10-30

이 디렉터리는 **OYMotion ROHand AP002 로봇 핸드의 연결·통신·제어를 검증하는 예제**를 모아 놓은 곳이다. ROS2를 이용한 MuJoCo 시뮬레이터 제어와 실제 로봇 핸드의 Modbus-RTU 제어를 각각 시험한다.

## 1. 테스트 진행 단계

아래 순서대로 **시뮬레이션 제어 → 실물 통신 확인 → 실물 관절 제어**를 시험한다.

| 단계 | 테스트 | 확인 사항 |
| ---: | --- | --- |
| 1 | MuJoCo 모델 실행 | 제조사 공식 AP002 오른손 모델이 정상 실행되는지 확인 |
| 2 | ROS2 → MuJoCo 제어 | 키보드 명령으로 시뮬레이터의 6개 구동 축 제어 |
| 3 | Windows USB → WSL2 연결 | CH340 장치가 `/dev/ttyUSB0`으로 인식되는지 확인 |
| 4 | Modbus-RTU 읽기 | 프로토콜·하드웨어 식별값, 현재 위치·목표 위치·상태값 확인 |
| 5 | ROS2 → 실물 AP002 제어 | 제어 노드 활성화 후 키보드로 6개 구동 축의 목표 위치 전송 |
| 6 | ROS2 피드백 확인 | 위치·목표·상태 토픽에서 명령 결과 확인 |

상세 실행 방법은 **5장(USB 연결), 6장(실물 테스트), 7장(시뮬레이터 테스트)**에 정리했다.

## 2. 로봇 핸드 테스트 파일

프로젝트 루트는 `~/TactileData_Collection_Pipeline`이다. 현재 GitHub에 포함된 로봇 핸드 관련 파일은 다음과 같다.

| 파일 | 역할 |
| --- | --- |
| `README.md` | 이 문서: 연결·통신·제어 테스트 안내 |
| `rohand_sim_ros2.py` | ROS2와 AP002 MuJoCo 시뮬레이터를 연결하는 브리지 |
| `rohand_keyboard_ros2.py` | 시뮬레이터용 키보드 제어 노드 |
| `rohand_real_ros2.py` | 실제 AP002와 Modbus-RTU 통신하는 ROS2 브리지 |
| `rohand_real_keyboard_ros2.py` | 실제 로봇 핸드용 키보드 제어 노드 |

위 Python 파일은 직접 실행하는 독립 프로그램이므로 **이 예제 자체는 `colcon build`가 필요하지 않다.** 시뮬레이션과 실물용 코드는 서로 다른 ROS2 토픽을 사용한다.

GitHub에서 제외한 로컬 자료는 `build/`, `install/`, `log/`, `external/`, `src/rohand_ros2_pkg/` 및 ZIP·BAK·Windows `Zone.Identifier` 파일이다. 이들은 로컬 환경에 남아 있을 수 있지만 현재 저장소의 추적 대상은 아니다.

## 3. 실제 AP002 통신 방식

### 3.1 명령 및 피드백 경로

```text
[키보드 입력]
  rohand_real_keyboard_ros2.py
             │  ROS2 std_msgs/String
             ▼  /rohand_real/step
[ROS2 실물 브리지]
  rohand_real_ros2.py (rclpy + pymodbus)
             │  Modbus-RTU 요청/응답
             ▼
  /dev/ttyUSB0 (Linux CH341 드라이버)
             │  WSL2 USB 패스스루 (usbipd-win)
             ▼
  Windows USB 포트 ── CH340 USB-Serial 어댑터
             │  물리 직렬 연결 방식은 별도 확인 필요
             ▼
  ROHand AP002 제어기 ── 손가락 구동 모터 6개
             │
             └── 현재 위치/목표 위치/상태 레지스터 → ROS2 브리지 → ROS2 토픽
```

ROHand 자체가 ROS2를 직접 처리하는 방식이 아니라, **ROS2 브리지 노드가 ROS2 명령을 Modbus-RTU 레지스터 읽기·쓰기 요청으로 변환하는 구조**이다.

### 3.2 검증된 통신 설정

| 항목 | 설정 또는 확인 결과 |
| --- | --- |
| 로봇 | OYMotion ROHand AP002 오른손 |
| 하드웨어 장치 | USB-SERIAL CH340, VID:PID `1a86:7523` |
| WSL2 연결 도구 | `usbipd-win` |
| Linux USB 직렬 드라이버 | `ch341` |
| Linux 장치 경로 | `/dev/ttyUSB0` (재연결 시 변경될 수 있음) |
| 프로토콜 | **Modbus-RTU**, Holding Register 읽기/쓰기 |
| Baudrate | **115200 bps** |
| 직렬 포맷 | **8N1** (데이터 8비트, 패리티 없음, 정지비트 1개) |
| Slave/Node ID | **2** |
| Python | 3.10 |
| Python 패키지 | `pymodbus==3.7.2`, `pyserial==3.5` |
| ROS | ROS2 Humble |
| 프로토콜 버전 (레지스터 1000) | `0x0200` |
| 하드웨어 식별값 (레지스터 1003) | `0x2001` |

**Modbus-RTU와 RS485는 같은 개념이 아니다.** Modbus-RTU는 데이터 통신 프로토콜, RS485/RS232는 물리적 전기 신호 규격이다. 현재 CH340 장치와 Modbus-RTU 동작은 확인했으나, **CH340 어댑터의 로봇 측 물리 연결이 RS485인지 RS232/다른 직렬 방식인지까지는 검증하지 않았다.** 제조사 매뉴얼은 Modbus-RTU의 RS232·RS485 설정을 모두 설명한다.

### 3.3 손가락 번호 및 Modbus 레지스터

키보드 번호는 **시뮬레이터와 동일한 사용자 입력 순서**로 맞췄다. 반면 실제 펌웨어 내부 레지스터는 엄지 굽힘이 첫 번째다.

| 입력 번호 | 손가락 | 펌웨어 인덱스 | 목표 위치 쓰기 | 현재 위치 읽기 | 상태 읽기 |
| ---: | --- | ---: | ---: | ---: | ---: |
| `1` | 검지 (`index`) | 1 | 1136 | 1146 | 1086 |
| `2` | 중지 (`middle`) | 2 | 1137 | 1147 | 1087 |
| `3` | 약지 (`ring`) | 3 | 1138 | 1148 | 1088 |
| `4` | 소지 (`little`) | 4 | 1139 | 1149 | 1089 |
| `5` | 엄지 뿌리 회전 (`thumb_root`) | 5 | 1140 | 1150 | 1090 |
| `6` | 엄지 굽힘 (`thumb`) | 0 | 1135 | 1145 | 1085 |

위치는 펌웨어의 **0~65535 논리 위치값**을 사용자 친화적으로 **0~100%**로 변환해 사용한다. 이는 실제 모터 각도(rad)나 실물 관절의 절대 각도를 직접 뜻하지 않는다.

- 개별 손가락 목표 레지스터 쓰기: Modbus Function Code `0x06`
- 여섯 개 목표 레지스터 한 번에 쓰기 (`all`): Function Code `0x10`
- 현재 위치·목표값·상태 읽기: Function Code `0x03`
- 브리지의 실제 상태 폴링: 약 **1초 간격** (`rohand_real_ros2.py` 기준)

## 4. 준비 및 설치 확인

WSL Ubuntu 터미널에서 프로젝트 루트로 이동한다.

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash
python3 -m pip show pymodbus pyserial
python3 -c 'import rclpy, mujoco; print("ROS2 / MuJoCo import OK")'
```

`pymodbus` 또는 `pyserial`이 없으면:

```bash
python3 -m pip install --user pymodbus==3.7.2 pyserial==3.5
```

ROS2 Python 코드는 **각 터미널에서** `source /opt/ros/humble/setup.bash`를 실행해야 한다. `mujoco`가 없다면 해당 Python 환경에 `python3 -m pip install --user mujoco`로 설치한다.

**MuJoCo 모델 준비:** `external/rohand_mujoco`는 이 프로젝트의 GitHub 저장소에 포함되지 않는다. 새로 클론한 개발환경에서 시뮬레이터를 실행하려면 제조사의 공식 모델을 별도로 내려받는다(이미 존재하면 생략).

```bash
cd ~/TactileData_Collection_Pipeline
mkdir -p external
git clone https://github.com/oymotion/rohand_mujoco.git external/rohand_mujoco
```

실물 AP002 제어 예제는 MuJoCo 모델 없이 실행할 수 있다.


## 5. Windows USB → WSL2 연결 절차 (실물 핸드)

USB 케이블을 뺐다가 다시 연결한 경우 보통 `usbipd attach`를 다시 실행해야 한다. 아래 Windows 명령은 **Windows PowerShell**에서, Linux 명령은 **WSL Ubuntu**에서 실행한다.

### 5.1 Windows PowerShell

```powershell
usbipd list
```

출력에서 `USB-SERIAL CH340` (`1a86:7523`)의 **현재 BUSID**를 확인한다. 테스트 당시에는 `1-3`이었지만 고정값이라고 가정하지 않는다.

`Shared` 상태이고 WSL에 아직 연결되지 않았다면:

```powershell
usbipd attach --wsl --busid 1-3
```

장치가 `Not shared`라면 **관리자 PowerShell**에서 먼저:

```powershell
usbipd bind --busid 1-3
```

이후 `attach`를 실행한다. BUSID는 반드시 현재 출력에 맞게 바꾼다. `bind`는 공유 설정이 유지되면 매번 반복할 필요 없다.

### 5.2 WSL Ubuntu

```bash
lsusb
ls -l /dev/ttyUSB0
```

정상적인 인식 예:

```text
ID 1a86:7523 QinHeng Electronics CH340 serial converter
crw-rw---- 1 root dialout ... /dev/ttyUSB0
```

`lsusb`에 CH340은 보이나 `/dev/ttyUSB0`이 없다면:

```bash
sudo modprobe ch341
sudo dmesg | tail -30
ls -l /dev/ttyUSB0
```

장치 파일이 생성되었다가 사라지면 WSL의 USB/IP 연결이 해제되었을 수 있으므로 Windows의 `usbipd list`와 `attach` 상태를 다시 확인한다.

권한 오류(`Permission denied`)가 나면 현재 사용자 그룹을 확인한다.

```bash
groups
sudo usermod -aG dialout "$USER"
```

그룹 추가는 **WSL 셸 재로그인 후 적용**된다. 장치 파일에 대한 무분별한 권한 변경보다는 `dialout` 그룹 사용을 권장한다.

> **전원:** CH340 USB 연결은 통신용이며 로봇 모터 전원 공급을 대체하지 않는다. 제조사 규격에 맞는 **24V 외부 전원**, 극성, 배선, 고정 상태를 확인한다.

## 6. 실물 로봇 핸드 테스트 순서

**한 번에 `rohand_real_ros2.py` 브리지 프로세스를 하나만 실행한다.** 같은 `/dev/ttyUSB0`을 여러 프로그램이 동시에 사용하지 않도록 한다.

### 6.1 1단계 — 읽기 전용 통신 확인

**터미널 A (WSL):**

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash
python3 hardware_connection_test/rohand_real_ros2.py
```

정상 로그 예:

```text
Modbus 장치 연결: protocol=0x0200, hardware=0x2001
1 index      actual=  0.00% target=  0.00% status=2
2 middle     actual=  0.00% target=  0.00% status=2
...
5 thumb_root actual=  1.11% target=  1.11% status=2
```

이 모드에서는 모터 목표 위치를 쓰지 않는다. 관절 위치 및 목표 위치, 상태가 정상적으로 갱신되는지 확인한 다음 `Ctrl+C`로 종료한다. 위의 숫자는 당시 관찰된 **예시**이며 현재 값은 달라질 수 있다.

### 6.2 2단계 — 실물 제어 노드 실행

**터미널 A:**

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash
python3 hardware_connection_test/rohand_real_ros2.py --enable-motion
```

실물 동작을 허용하려면 실행 중 나타나는 확인 프롬프트에 **`ENABLE`** 을 정확하게 입력한다. 이 단계에서는 자동으로 손가락을 움직이지 않으며, 명령을 수신할 때만 목표값을 전송한다.

### 6.3 3단계 — 별도 터미널에서 손가락 제어

**터미널 B:**

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash
python3 hardware_connection_test/rohand_real_keyboard_ros2.py
```

키보드 입력 예:

```text
실물 명령 > 1 20     # 검지 절대 목표 20%
실물 명령 > 1 50     # 검지 절대 목표 50%
실물 명령 > 1 +10    # 현재 측정 위치에서 검지 +10%p
실물 명령 > 1 -10    # 현재 측정 위치에서 검지 -10%p
실물 명령 > 2 50     # 중지 50%
실물 명령 > 5 50     # 엄지 뿌리 회전 50%
실물 명령 > 6 50     # 엄지 굽힘 50%
실물 명령 > all 0   # 여섯 축 모두 절대 목표 0%
실물 명령 > q       # 키보드 프로그램 종료
```

- 절대 위치 명령: `1 50`처럼 **부호 없는 수** 사용 (`0~100`).
- 상대 위치 명령: `1 +10`, `1 -10`처럼 **`+`/`-` 부호 포함**; 기준은 브리지가 명령 직전에 다시 읽은 실제 위치다.
- `all`은 **절대 위치만** 지원하며 한 번에 6개 축을 움직일 수 있다.
- 명령을 보내면 **터미널 A**에서 레지스터 쓰기 결과와 실제 상태를 확인한다.
- 모터의 내부 가동 범위 보호와 외부 충돌 방지는 별개다. 주변에 손이나 물체가 끼지 않도록 한다.

### 6.4 ROS2 토픽 확인 (선택)

브리지 실행 중 별도 터미널에서:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /rohand_real/positions --once
ros2 topic echo /rohand_real/status --once
```

| 토픽 | ROS2 메시지 타입 | 역할 |
| --- | --- | --- |
| `/rohand_real/step` | `std_msgs/msg/String` | 키보드에서 전송한 실물 제어 명령 |
| `/rohand_real/positions` | `std_msgs/msg/Float64MultiArray` | 실제 위치 6개 (각각 0~1) |
| `/rohand_real/targets` | `std_msgs/msg/Float64MultiArray` | 목표 위치 6개 (각각 0~1) |
| `/rohand_real/status` | `std_msgs/msg/Int32MultiArray` | 상태 코드 6개 |

피드백 배열의 순서는 `[검지, 중지, 약지, 소지, 엄지 뿌리 회전, 엄지 굽힘]`이다. `status=2`는 이번 실측에서 목표 위치에 도달한 상태와 함께 관찰되었고, 제조사 FAQ에서 `status=5`는 모터 걸림/접촉 상태로 설명한다. 값의 엄밀한 해석은 펌웨어 버전에 맞는 제조사 문서를 참조한다.

**종료·안전:** `Ctrl+C`는 프로그램만 종료하며, 이미 움직이고 있는 손가락을 멈추지는 않는다. **별도의 비상정지 기능이 없으므로 주의한다.**

## 7. MuJoCo 시뮬레이터 제어 (실물 USB 불필요)

제조사 공식 AP002 모델을 수정하지 않고 ROS2 명령으로 구동한다.

**터미널 A (MuJoCo):**

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 python3 hardware_connection_test/rohand_sim_ros2.py
```

**터미널 B (키보드):**

```bash
cd ~/TactileData_Collection_Pipeline
source /opt/ros/humble/setup.bash
python3 hardware_connection_test/rohand_keyboard_ros2.py
```

예: `1 50`, `2 80`, `5 30`, `all 0`, `show`, `q`.

| 토픽 | 메시지 | 내용 |
| --- | --- | --- |
| `/rohand_sim/target` | `std_msgs/msg/Float64MultiArray` | actuator 목표 6개, 각각 0~1 |
| `/rohand_sim/joint_states` | `sensor_msgs/msg/JointState` | MuJoCo 구동 관절 상태 (기본 단위: rad 등) |

**중요:** 시뮬레이터의 0~100%는 MuJoCo XML의 actuator `ctrlrange`를 정규화한 값이고, 실물의 0~100%는 펌웨어 논리 위치값 `0~65535`를 정규화한 값이다. **같은 숫자라고 실물 관절 각도가 정확히 같다는 뜻은 아니다.**

## 8. 오류 해결

| 현상 | 점검 사항 |
| --- | --- |
| `lsusb`에 CH340이 없음 | Windows `usbipd list`에서 BUSID 확인 후 `usbipd attach --wsl --busid ...` |
| `lsusb`에는 CH340이 있으나 `/dev/ttyUSB0` 없음 | `sudo modprobe ch341`, `sudo dmesg | tail -30`; 연결이 끊어졌다면 USB 재전달 |
| `/dev/ttyUSB0: Permission denied` | `groups` 확인, `dialout` 그룹 추가 후 WSL 셸 재로그인 |
| 직렬 포트 열기 실패 | 장치 파일 존재 여부 및 다른 프로그램이 포트를 사용 중인지 확인 |
| Modbus timeout / 응답 실패 | 24V 전원, 통신 배선, Slave ID `2`, baudrate `115200`, USB 연결 확인 |
| `rclpy` import 실패 | 해당 터미널에서 `source /opt/ros/humble/setup.bash` |
| 명령 입력했으나 실물 움직임 없음 | 브리지에 `--enable-motion`과 `ENABLE`이 적용됐는지, ROS2 토픽 연결 및 터미널 A 오류 확인 |

## 9. 공식 참고 자료

- [OYMotion AP002 MuJoCo](https://github.com/oymotion/rohand_mujoco/tree/main/AP002)
- [OYMotion ROHand Gen2 Demo — 레지스터 정의](https://github.com/oymotion/roh_gen2_demos/blob/main/common/roh_registers_v2.py)
- [OYMotion Gen2 `loop_test.py`](https://github.com/oymotion/roh_gen2_demos/blob/main/loop_test/loop_test.py) — **실행 즉시 전체 손가락을 움직일 수 있으므로 연결 확인용으로 실행하지 않는다.**
- [OYMotion Modbus-RTU 프로토콜 문서](https://github.com/oymotion/roh_firmware/blob/main/protocol/OHandModBusRTUProtocol_EN.md) — 구형 프로토콜 문서의 내용은 Gen2 레지스터 정의 및 실측과 대조해서 사용

