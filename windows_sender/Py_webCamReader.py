import cv2
import math
import csv
import json
import socket
import numpy as np
import mediapipe as mp

from pathlib import Path
from datetime import datetime
from network_config import get_network_config

# ==========================================
# 설정
# ==========================================

EMA_ALPHA = 0.7

# hostname -I로 확인한 WSL IP로 변경
WSL_IP, UDP_PORT = get_network_config()
print(f"WSL 연결 대상: {WSL_IP}:{UDP_PORT}")

FG_NAMES = [
    "Thumb",
    "Index",
    "Middle",
    "Ring",
    "Pinky",
    "Thumb Rotation"
]

FG_KEYS = [
    "thumb",
    "index",
    "middle",
    "ring",
    "pinky",
    "thumb_rotation"
]

FG_COUNT = len(FG_NAMES)

# ==========================================
# UDP 초기화
# ==========================================

udp_socket = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

# ==========================================
# MediaPipe 및 웹캠 초기화
# ==========================================

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    model_complexity=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

if not cap.isOpened():
    hands.close()
    udp_socket.close()
    raise RuntimeError("웹캠을 열 수 없습니다.")

# ==========================================
# 보정 및 필터 변수
# ==========================================

fixed: bool = False

fg_max: list[float] = [float("-inf")] * FG_COUNT
fg_min: list[float] = [float("inf")] * FG_COUNT

fg_bend: list[float] = [0.0] * FG_COUNT
fg_raw: list[float] = [0.0] * FG_COUNT

fg_filtered: list[float | None] = [None] * FG_COUNT
fg_valid: list[bool] = [False] * FG_COUNT

hand_detected = False
latest_hand_data: list[float] | None = None

measurement_history: list[dict] = []

# ==========================================
# 3차원 벡터 계산
# ==========================================

def point3d(lm) -> np.ndarray:
    return np.array(
        [lm.x, lm.y, lm.z],
        dtype=np.float64
    )


def normalize_vector(
    v: np.ndarray
) -> np.ndarray | None:

    norm = np.linalg.norm(v)

    if norm < 1e-8:
        return None

    return v / norm


def joint_angle(a, b, c) -> float | None:

    a = point3d(a)
    b = point3d(b)
    c = point3d(c)

    v1 = normalize_vector(a - b)
    v2 = normalize_vector(c - b)

    if v1 is None or v2 is None:
        return None

    cosine = float(np.clip(
        np.dot(v1, v2),
        -1.0,
        1.0
    ))

    return math.degrees(math.acos(cosine))


def normalize(
    value: float,
    minimum: float,
    maximum: float
) -> float:

    if maximum <= minimum:
        return 0.0

    result = (
        (value - minimum)
        / (maximum - minimum)
    )

    return float(np.clip(result, 0.0, 1.0))


def ema(
    current: float,
    previous: float | None,
    alpha: float = EMA_ALPHA
) -> float:

    if previous is None:
        return current

    return (
        alpha * current
        + (1.0 - alpha) * previous
    )


# ==========================================
# 검지~소지 굽힘 계산
# ==========================================

def finger_bend_angle(
    landmarks,
    base: int
) -> float | None:

    mcp = joint_angle(
        landmarks[0],
        landmarks[base],
        landmarks[base + 1]
    )

    pip = joint_angle(
        landmarks[base],
        landmarks[base + 1],
        landmarks[base + 2]
    )

    dip = joint_angle(
        landmarks[base + 1],
        landmarks[base + 2],
        landmarks[base + 3]
    )

    if mcp is None or pip is None or dip is None:
        return None

    return (
        0.35 * (180.0 - mcp)
        + 0.45 * (180.0 - pip)
        + 0.20 * (180.0 - dip)
    )


# ==========================================
# 엄지 굽힘 계산
# ==========================================

def thumb_bend_angle(
    landmarks
) -> float | None:

    mcp = joint_angle(
        landmarks[1],
        landmarks[2],
        landmarks[3]
    )

    ip = joint_angle(
        landmarks[2],
        landmarks[3],
        landmarks[4]
    )

    if mcp is None or ip is None:
        return None

    return (
        0.5 * (180.0 - mcp)
        + 0.5 * (180.0 - ip)
    )


# ==========================================
# 손바닥 기준 좌표계
# ==========================================

def palm_coordinate_system(landmarks):

    wrist = point3d(landmarks[0])
    index_mcp = point3d(landmarks[5])
    pinky_mcp = point3d(landmarks[17])

    x_axis = normalize_vector(
        index_mcp - pinky_mcp
    )

    if x_axis is None:
        return None

    palm_center = (
        index_mcp + pinky_mcp
    ) / 2.0

    y_temp = palm_center - wrist

    y_temp = (
        y_temp
        - np.dot(y_temp, x_axis) * x_axis
    )

    y_axis = normalize_vector(y_temp)

    if y_axis is None:
        return None

    z_axis = normalize_vector(
        np.cross(x_axis, y_axis)
    )

    if z_axis is None:
        return None

    return x_axis, y_axis, z_axis


# ==========================================
# 엄지 회전 계산
# ==========================================

def thumb_rotation_angle(
    landmarks
) -> float | None:

    axes = palm_coordinate_system(landmarks)

    if axes is None:
        return None

    x_axis, y_axis, z_axis = axes

    thumb_cmc = point3d(landmarks[1])
    thumb_mcp = point3d(landmarks[2])

    thumb_vector = thumb_mcp - thumb_cmc

    thumb_projected = (
        thumb_vector
        - np.dot(thumb_vector, z_axis) * z_axis
    )

    thumb_projected = normalize_vector(
        thumb_projected
    )

    if thumb_projected is None:
        return None

    x_component = float(
        np.dot(thumb_projected, x_axis)
    )

    y_component = float(
        np.dot(thumb_projected, y_axis)
    )

    return math.degrees(
        math.atan2(x_component, y_component)
    )


# ==========================================
# 전체 손가락 측정
# ==========================================

def measure_hand(
    landmarks
) -> list[float | None]:

    values: list[float | None] = [None] * FG_COUNT

    values[0] = thumb_bend_angle(landmarks)
    values[1] = finger_bend_angle(landmarks, 5)
    values[2] = finger_bend_angle(landmarks, 9)
    values[3] = finger_bend_angle(landmarks, 13)
    values[4] = finger_bend_angle(landmarks, 17)
    values[5] = thumb_rotation_angle(landmarks)

    return values


# ==========================================
# 보정 초기화
# ==========================================

def reset_calibration() -> None:

    global fixed
    global fg_max, fg_min
    global fg_bend, fg_raw
    global fg_filtered, fg_valid
    global latest_hand_data

    fixed = False

    fg_max = [float("-inf")] * FG_COUNT
    fg_min = [float("inf")] * FG_COUNT

    fg_bend = [0.0] * FG_COUNT
    fg_raw = [0.0] * FG_COUNT

    fg_filtered = [None] * FG_COUNT
    fg_valid = [False] * FG_COUNT

    latest_hand_data = None


# ==========================================
# 측정값 기록
# ==========================================

def record_measurement(
    values: list[float]
) -> None:

    record = {
        "timestamp": datetime.now().isoformat(
            timespec="milliseconds"
        )
    }

    for i, name in enumerate(FG_KEYS):
        record[name] = round(values[i], 3)

    measurement_history.append(record)


# ==========================================
# CSV 파일 저장
# ==========================================

def save_measurements(
    filename: str = "measurements.csv"
) -> None:

    if not measurement_history:
        print("저장할 기록이 없습니다.")
        return

    path = Path(__file__).resolve().parent / filename

    with path.open(
        "w",
        newline="",
        encoding="utf-8-sig"
    ) as file:

        writer = csv.DictWriter(
            file,
            fieldnames=["timestamp"] + FG_KEYS
        )

        writer.writeheader()
        writer.writerows(measurement_history)

    print(f"CSV 저장 완료: {path}")
    print(f"기록 수: {len(measurement_history)}")


# ==========================================
# WSL로 실시간 UDP 전송
# ==========================================


def send_hand_data(
    landmarks: list[float],
    values: list[float] | None
) -> None:
    # landmarks: MediaPipe world landmarks 21 x (x, y, z) = 63 floats (m)
    #            -> /hand_landmarks -> pose_command_node
    # values: calibrated finger values (0~1), only after calibration
    #         -> /hand/finger_angles (monitoring / recording)
    payload = {"landmarks": landmarks}

    if values is not None:
        payload["values"] = values

    message = json.dumps(payload).encode("utf-8")

    udp_socket.sendto(
        message,
        (WSL_IP, UDP_PORT)
    )


# ==========================================
# 메인 루프
# ==========================================

try:
    while True:

        ret, frame = cap.read()

        if not ret:
            print("프레임 읽기 실패")
            break

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape

        imgRGB = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )

        imgRGB.flags.writeable = False
        result = hands.process(imgRGB)
        imgRGB.flags.writeable = True

        hand_detected = bool(
            result.multi_hand_landmarks
            and result.multi_hand_world_landmarks
        )

        latest_hand_data = None

        # ----------------------------------
        # 손가락 측정
        # ----------------------------------

        if hand_detected:

            world_hand = (
                result.multi_hand_world_landmarks[0]
            )

            image_hand = (
                result.multi_hand_landmarks[0]
            )

            measured = measure_hand(
                world_hand.landmark
            )

            for fg in range(FG_COUNT):

                current = measured[fg]

                if current is None:
                    fg_valid[fg] = False
                    fg_filtered[fg] = None
                    continue

                filtered = ema(
                    current,
                    fg_filtered[fg]
                )

                fg_filtered[fg] = filtered
                fg_raw[fg] = filtered

                if not fixed:

                    fg_max[fg] = max(
                        fg_max[fg],
                        filtered
                    )

                    fg_min[fg] = min(
                        fg_min[fg],
                        filtered
                    )

                fg_valid[fg] = (
                    fg_max[fg] > fg_min[fg] + 1e-6
                )

                if fg_valid[fg]:

                    normalized = normalize(
                        filtered,
                        fg_min[fg],
                        fg_max[fg]
                    )

                    if fg < 5:
                        fg_bend[fg] = 1.0 - normalized
                    else:
                        fg_bend[fg] = normalized

            # 관절 좌표는 보정과 관계없이 매 프레임 전송하고,
            # 손가락 값은 모든 측정값이 유효할 때만 함께 전송
            if all(fg_valid):
                latest_hand_data = fg_bend.copy()

            world_landmarks = [
                float(value)
                for lm in world_hand.landmark
                for value in (lm.x, lm.y, lm.z)
            ]

            try:
                send_hand_data(
                    world_landmarks,
                    latest_hand_data
                )

            except OSError as error:
                print(f"UDP 전송 오류: {error}")

            mp_draw.draw_landmarks(
                frame,
                image_hand,
                mp_hands.HAND_CONNECTIONS
            )

        else:
            fg_filtered = [None] * FG_COUNT
            fg_valid = [False] * FG_COUNT

        # ----------------------------------
        # 화면 표시
        # ----------------------------------

        status = (
            "FIXED" if fixed else "CALIBRATING"
        )

        status_color = (
            (0, 255, 0)
            if fixed else (0, 0, 255)
        )

        cv2.putText(
            frame,
            status,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            status_color,
            2
        )

        for fg in range(FG_COUNT):

            if not hand_detected:
                value_text = "NO HAND"

            elif not fg_valid[fg]:
                value_text = "NOT CALIBRATED"

            else:
                value_text = f"{fg_bend[fg]:.2f}"

            cv2.putText(
                frame,
                f"{FG_NAMES[fg]}: {value_text}",
                (20, 80 + fg * 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

        cv2.putText(
            frame,
            "F: Fix  R: Record  S: Save  C: Reset  Q: Quit",
            (20, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 0),
            1
        )

        cv2.imshow("Webcam", frame)

        # ----------------------------------
        # 키보드 입력
        # ----------------------------------

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        elif key == ord("f"):

            if not fixed:

                if all(
                    fg_max[i] > fg_min[i] + 1e-6
                    for i in range(FG_COUNT)
                ):
                    fixed = True
                    print("보정값 고정 완료")
                else:
                    print("보정값이 부족합니다.")

            else:
                fixed = False
                print("보정 재개")

        elif key == ord("c"):

            reset_calibration()

            print("보정값 초기화 완료")
            print(
                f"기록 {len(measurement_history)}개 유지"
            )

        elif key == ord("r"):

            if latest_hand_data is not None:

                record_measurement(latest_hand_data)

                print([
                    round(v, 3)
                    for v in latest_hand_data
                ])

                print(
                    f"누적 기록: {len(measurement_history)}개"
                )

            else:
                print(
                    "손이 인식되지 않았거나 "
                    "보정값이 부족합니다."
                )

        elif key == ord("s"):

            try:
                save_measurements()
            except OSError as error:
                print(f"CSV 저장 실패: {error}")

finally:
    cap.release()
    hands.close()
    udp_socket.close()
    cv2.destroyAllWindows()
