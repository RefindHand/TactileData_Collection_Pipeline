"""Per-person calibration profile panel (보정 tab).

Workflow for one person:
  1. Enter the person's name ("불러오기" if a profile already exists).
  2. Turn on "슬라이더로 로봇 움직이기": pose_command_node follows the sliders, not the webcam.
  3. Pick a pose (펼침 / 주먹 / 핀치 ...), move the sliders until the robot hand takes it.
  4. The person makes the same pose in front of the webcam -> "이 자세 기록" (1 s average).
  5. Repeat for the other poses, then "저장하고 적용".
The profile maps the person's joint angles to the recorded robot values
(see pose_command/calibration_profile.py).
"""
import math
import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QGridLayout, QHBoxLayout, QLabel, QLineEdit, QListWidget,
    QListWidgetItem, QPushButton, QSlider, QVBoxLayout, QWidget,
)

from finger_node.common import FINGER_LIMITS, FINGERS
from pose_command.calibration_profile import CalibrationProfile, profile_path

# (key stored in the profile, label shown). The combo box is editable: other poses
# such as the four grasp types can be typed in.
POSES = [('open', '펼침'), ('fist', '주먹'), ('pinch', '핀치')]
LABELS_KO = {'index': '검지', 'middle': '중지', 'ring': '약지', 'little': '소지',
             'thumb_root': '엄지 회전', 'thumb': '엄지'}
SLIDER_STEPS = 100
MANUAL_PERIOD_MS = 100    # well under pose_command's MANUAL_TIMEOUT (1 s)
RECORD_S = 1.0            # average the human angles over this long
MIN_SAMPLES = 5           # webcam runs ~10 Hz on the Jetson -> ~10 samples per second


def heading(text):
    label = QLabel(text)
    label.setObjectName('heading')
    return label


def muted(text):
    label = QLabel(text)
    label.setObjectName('muted')
    label.setWordWrap(True)
    return label


def average(samples):
    return [sum(values) / len(values) for values in zip(*samples)]


class CalibrationPanel(QWidget):
    def __init__(self, bridge, show_message):
        super().__init__()
        self.bridge = bridge
        self.show_message = show_message     # (text, ms) -> status bar
        self.profile = CalibrationProfile('')
        self.record_start = None

        self.name = QLineEdit()
        self.name.setPlaceholderText('사람 이름 (예: 조안정)')
        load = QPushButton('불러오기')
        load.clicked.connect(self.load)

        self.manual = QCheckBox('슬라이더로 로봇 움직이기 (웹캠 대신)')
        self.manual.toggled.connect(self.toggle_manual)
        self.manual_timer = QTimer(self)
        self.manual_timer.timeout.connect(self.send_manual)

        self.sliders = []
        self.values = []
        grid = QGridLayout()
        grid.setColumnStretch(1, 1)
        for row, finger in enumerate(FINGERS):
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, SLIDER_STEPS)
            value = QLabel('0.00')
            value.setMinimumWidth(34)
            slider.valueChanged.connect(
                lambda v, label=value: label.setText(f'{v / SLIDER_STEPS:.2f}'))
            grid.addWidget(QLabel(LABELS_KO[finger]), row, 0)
            grid.addWidget(slider, row, 1)
            grid.addWidget(value, row, 2)
            self.sliders.append(slider)
            self.values.append(value)

        self.pose = QComboBox()
        self.pose.setEditable(True)
        for key, label in POSES:
            self.pose.addItem(label, key)
        self.record_button = QPushButton('이 자세 기록 (1초)')
        self.record_button.clicked.connect(self.start_record)

        self.pose_list = QListWidget()
        self.pose_list.setMaximumHeight(110)
        self.pose_list.itemClicked.connect(self.pose_selected)
        remove = QPushButton('선택 삭제')
        remove.clicked.connect(self.remove_pose)
        save = QPushButton('저장하고 적용')
        save.clicked.connect(self.save)
        auto = QPushButton('자동 보정으로 되돌리기')
        auto.clicked.connect(self.use_auto)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(heading('사람별 보정 프로파일'))
        layout.addWidget(muted('① 이름 입력 ② 슬라이더 켜기 ③ 자세를 고르고 슬라이더로 로봇을 그 자세로 '
                               '④ 같은 자세를 웹캠 앞에서 하고 기록 ⑤ 자세마다 반복 후 저장'))
        row = QHBoxLayout()
        row.addWidget(self.name, 1)
        row.addWidget(load)
        layout.addLayout(row)
        layout.addWidget(self.manual)
        layout.addLayout(grid)
        row = QHBoxLayout()
        row.addWidget(self.pose, 1)
        row.addWidget(self.record_button)
        layout.addLayout(row)
        layout.addWidget(muted('기록한 자세 (누르면 그 로봇 값을 슬라이더로 불러옴)'))
        layout.addWidget(self.pose_list)
        row = QHBoxLayout()
        row.addWidget(remove)
        row.addWidget(save, 1)
        layout.addLayout(row)
        layout.addWidget(auto)

    # ---- sliders ------------------------------------------------------------------
    def slider_targets(self):
        return [slider.value() / SLIDER_STEPS for slider in self.sliders]

    def set_sliders(self, targets):
        for slider, value in zip(self.sliders, targets):
            slider.setValue(int(round(min(1.0, max(0.0, value)) * SLIDER_STEPS)))

    def current_robot_targets(self):
        """Last command sent to each finger (rad) as 0~1, or None if unknown."""
        fingers = self.bridge.snapshot()['fingers']
        targets = []
        for finger in FINGERS:
            command = fingers[finger]['command']
            if not math.isfinite(command):
                return None
            low, high = FINGER_LIMITS[finger]
            targets.append((command - low) / (high - low))
        return targets

    def toggle_manual(self, on):
        if on:
            # start the sliders where the robot already is, so turning this on does not jump
            current = self.current_robot_targets()
            if current is not None:
                self.set_sliders(current)
            self.send_manual()
            self.manual_timer.start(MANUAL_PERIOD_MS)
            self.show_message('슬라이더 모드: 로봇이 웹캠 대신 슬라이더를 따라갑니다')
        else:
            # pose_command goes back to the webcam ~1 s after the last slider message
            self.manual_timer.stop()
            self.show_message('슬라이더 모드 끔: 약 1초 뒤 웹캠을 다시 따라갑니다')

    def send_manual(self):
        self.bridge.set_manual_targets(self.slider_targets())

    # ---- poses ----------------------------------------------------------------------
    def pose_key(self):
        """Key of the selected pose ('pinch'), or the typed text for a new pose."""
        text = self.pose.currentText().strip()
        index = self.pose.findText(text)
        if index >= 0 and self.pose.itemData(index):
            return self.pose.itemData(index)
        return text

    def pose_label(self, key):
        return dict(POSES).get(key, key)

    def start_record(self):
        if not self.pose_key():
            self.show_message('기록할 자세 이름을 고르거나 입력하세요')
            return
        self.record_start = time.monotonic()
        self.record_button.setEnabled(False)
        self.record_button.setText('기록 중… 자세 유지')
        QTimer.singleShot(int(RECORD_S * 1000), self.finish_record)

    def finish_record(self):
        self.record_button.setEnabled(True)
        self.record_button.setText('이 자세 기록 (1초)')
        samples = self.bridge.human_angles_since(self.record_start)
        if len(samples) < MIN_SAMPLES:
            self.show_message(f'기록 실패: 손 각도가 {len(samples)}개만 들어왔습니다. '
                              '웹캠에 손이 보이는지 확인하세요', 10000)
            return
        key = self.pose_key()
        self.profile.set_pose(key, average(samples), self.slider_targets())
        self.refresh_list()
        self.show_message(f'{self.pose_label(key)} 기록 ({len(samples)}개 평균). '
                          '저장하고 적용을 눌러야 로봇에 반영됩니다')

    def remove_pose(self):
        item = self.pose_list.currentItem()
        if item is not None:
            self.profile.remove_pose(item.data(Qt.UserRole))
            self.refresh_list()

    def pose_selected(self, item):
        self.set_sliders(self.profile.poses[item.data(Qt.UserRole)]['robot'])

    def refresh_list(self):
        self.pose_list.clear()
        for key, values in self.profile.poses.items():
            robot = ' '.join(f'{v:.2f}' for v in values['robot'])
            item = QListWidgetItem(f'{self.pose_label(key)}  로봇 [{robot}]')
            item.setData(Qt.UserRole, key)
            self.pose_list.addItem(item)

    # ---- save / load ------------------------------------------------------------------
    def checked_path(self):
        try:
            return profile_path(self.name.text())
        except ValueError:
            self.show_message('이름을 입력하세요 (/ \\ : 와 .으로 시작하는 이름은 안 됨)')
            return None

    def load(self):
        path = self.checked_path()
        if path is None:
            return
        try:
            self.profile = CalibrationProfile.load(path)
        except (OSError, ValueError, KeyError) as error:
            self.show_message(f'불러오기 실패: {error}', 10000)
            return
        self.refresh_list()
        self.bridge.load_profile(self.profile.name)
        self.show_message(f'{self.profile.name} 프로파일 적용 ({len(self.profile.poses)}개 자세)')

    def save(self):
        path = self.checked_path()
        if path is None:
            return
        if len(self.profile.poses) < 2:
            self.show_message('자세를 2개 이상 기록해야 저장할 수 있습니다 (예: 펼침, 주먹)')
            return
        self.profile.name = self.name.text().strip()
        try:
            self.profile.save(path)
        except OSError as error:
            self.show_message(f'저장 실패: {error}', 10000)
            return
        self.bridge.load_profile(self.profile.name)
        self.show_message(f'저장하고 적용: {path}')

    def use_auto(self):
        self.bridge.load_profile('')
        self.show_message('프로파일 해제: 자동 보정으로 동작합니다')
