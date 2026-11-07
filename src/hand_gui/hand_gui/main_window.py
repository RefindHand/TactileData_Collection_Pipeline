"""Main window layout (dark theme, see theme.py).

  ┌ 녹화|업로드|보정 ┬ robot hand (MuJoCo + force overlay) ┬ webcam ──────────┐
  │ (tab content)   │                                     ├ robot hand camera ┤
  │                 ├ actuator force │ tactile normal │ tactile shear (10 s) ┤
  └─────────────────┴ status: data freshness ───────────────────────────────┘
"""
import json
import collections
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QPointF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap, QPolygonF
from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDialogButtonBox, QFileDialog, QFormLayout, QHBoxLayout,
    QLabel, QLineEdit, QListWidget, QMainWindow, QMessageBox, QPushButton, QSplitter, QTabWidget,
    QVBoxLayout, QWidget,
)

from finger_node.common import FINGERS, TACTILE_REGIONS
from hand_gui.hand_view import HandView
from hand_gui.recorder import Recorder
from hand_gui import theme
from hand_gui.uploader import INSTALL_HINT, UploadWorker, hub_available

RATE = 30.0              # recording and plot sampling rate (Hz)
UI_RATE = 15.0           # screen refresh: images, 3D hand, plots (Hz)
HISTORY_S = 10.0         # plot window (s)
STALE_S = 0.5
TASKS = ['MW', 'LP', 'TFP', 'PS', 'free']
COLORS = {'index': '#1f77b4', 'middle': '#ff7f0e', 'ring': '#2ca02c', 'little': '#d62728',
          'thumb': '#9467bd', 'thumb_root': '#8c564b', 'palm': '#7f7f7f'}
NAMES_KO = {'index': '검지', 'middle': '중지', 'ring': '약지', 'little': '소지',
            'thumb': '엄지', 'thumb_root': '엄지 회전', 'palm': '손바닥'}


def heat_color(fraction):
    """green -> yellow -> red."""
    f = min(max(fraction, 0.0), 1.0)
    return QColor(int(255 * min(1.0, 2 * f)), int(255 * min(1.0, 2 * (1 - f))), 30)


def draw_arrow(painter, start, end, head=8.0):
    painter.drawLine(start, end)
    dx, dy = end.x() - start.x(), end.y() - start.y()
    length = (dx * dx + dy * dy) ** 0.5
    if length < 1:
        return
    ux, uy = dx / length, dy / length
    left = QPointF(end.x() - head * (ux - 0.5 * uy), end.y() - head * (uy + 0.5 * ux))
    right = QPointF(end.x() - head * (ux + 0.5 * uy), end.y() - head * (uy - 0.5 * ux))
    painter.drawPolygon(QPolygonF([end, left, right]))


def to_pixmap(rgb):
    rgb = np.ascontiguousarray(rgb)
    height, width = rgb.shape[:2]
    image = QImage(rgb.data, width, height, 3 * width, QImage.Format_RGB888)
    return QPixmap.fromImage(image.copy())


class ImagePanel(QWidget):
    """Title + image scaled to fit (or, with fill, to cover and cropped at the centre)."""

    def __init__(self, title, placeholder, fill=False):
        super().__init__()
        self.placeholder = placeholder
        self.fill = fill
        self.title = QLabel(title)
        self.title.setObjectName('heading')
        self.view = QLabel(placeholder)
        self.view.setAlignment(Qt.AlignCenter)
        self.view.setMinimumSize(160, 100)
        self.view.setStyleSheet(f'background:#000000; color:{theme.MUTED}; border-radius:6px;')
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.addWidget(self.title)
        layout.addWidget(self.view, 1)
        self.pixmap = None

    def show_rgb(self, rgb):
        if rgb is None:
            if self.pixmap is not None:
                self.pixmap = None
                self.view.setPixmap(QPixmap())
                self.view.setText(self.placeholder)
            return
        self.set_pixmap(to_pixmap(rgb))

    def set_pixmap(self, pixmap):
        self.pixmap = pixmap
        size = self.view.size()
        if not self.fill:
            self.view.setPixmap(pixmap.scaled(size, Qt.KeepAspectRatio, Qt.SmoothTransformation))
            return
        scaled = pixmap.scaled(size, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation)
        x = (scaled.width() - size.width()) // 2
        y = (scaled.height() - size.height()) // 2
        self.view.setPixmap(scaled.copy(x, y, size.width(), size.height()))


class HandPanel(ImagePanel):
    """Robot hand image; drag to rotate, wheel to zoom."""

    def __init__(self, hand_view):
        super().__init__('로봇 손 (원 = 누르는 힘, 파란 화살표 = 미끄러지는 힘·방향)',
                         'MuJoCo 모델을 불러오는 중…')
        self.hand_view = hand_view
        self.last = None
        self.view.setToolTip('마우스 드래그: 회전 · 휠: 확대/축소')

    def mousePressEvent(self, event):
        self.last = event.pos()

    def mouseMoveEvent(self, event):
        if self.last is not None:
            delta = event.pos() - self.last
            self.hand_view.rotate(delta.x(), delta.y())
            self.last = event.pos()

    def mouseReleaseEvent(self, _event):
        self.last = None

    def wheelEvent(self, event):
        self.hand_view.zoom(event.angleDelta().y() / 120.0)

    def show_frame(self, rgb, items):
        """Hand image with forces drawn on top: circle = normal, arrow = shear."""
        pixmap = to_pixmap(rgb)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setFont(QFont(painter.font().family(), 9, QFont.Bold))
        for item in items:
            center = QPointF(item['u'], item['v'])
            color = heat_color(item['fraction'])
            radius = 5 + 16 * item['fraction']
            painter.setPen(QPen(QColor(20, 20, 20), 1.5))
            fill = QColor(color)
            fill.setAlpha(190)
            painter.setBrush(fill)
            painter.drawEllipse(center, radius, radius)
            if item['arrow'] is not None:
                painter.setPen(QPen(QColor(30, 120, 255), 3))
                painter.setBrush(QColor(30, 120, 255))
                draw_arrow(painter, center, QPointF(*item['arrow']))
            label = f"{NAMES_KO[item['region']]} {item['normal']:.2f}"
            painter.setPen(QColor(255, 255, 255))
            painter.drawText(QPointF(item['u'] + radius + 3, item['v'] - 3), label)
        painter.end()
        self.set_pixmap(pixmap)


class ForcePlot(pg.PlotWidget):
    def __init__(self, title, unit, names):
        super().__init__()
        self.setMinimumHeight(150)
        self.setBackground(theme.PANEL)
        self.setTitle(title, color=theme.TEXT, size='10pt')
        self.setLabel('left', unit)
        self.setLabel('bottom', '시간 (s)')
        self.showGrid(x=True, y=True, alpha=0.3)
        legend = self.addLegend(offset=(5, 2), labelTextSize='8pt', colCount=3)  # compact
        legend.setBrush(pg.mkBrush(27, 28, 31, 200))
        legend.setLabelTextColor(theme.TEXT)
        self.curves = {n: self.plot([], [], pen=pg.mkPen(COLORS[n], width=2), name=NAMES_KO[n])
                       for n in names}

    def update_curves(self, times, series):
        for name, curve in self.curves.items():
            curve.setData(times, series[name], connect='finite')


class UploadDialog(QDialog):
    def __init__(self, parent, folder, episodes):
        super().__init__(parent)
        self.setWindowTitle('Hugging Face 업로드')
        self.repo = QLineEdit()
        self.repo.setPlaceholderText('아이디/데이터셋이름 (예: RefindHand/tactile-ap002)')
        self.private = QCheckBox('비공개 (private)')
        self.private.setChecked(True)
        self.token = QLineEdit()
        self.token.setEchoMode(QLineEdit.Password)
        self.token.setPlaceholderText('비우면 huggingface-cli login 정보 사용')
        form = QFormLayout(self)
        form.addRow(QLabel(f'업로드할 폴더: {folder}\n에피소드 {episodes}개'))
        form.addRow('저장소', self.repo)
        form.addRow('', self.private)
        form.addRow('토큰', self.token)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.button(QDialogButtonBox.Ok).setText('업로드')
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)


class MainWindow(QMainWindow):
    def __init__(self, bridge, data_dir, test_object='none', hand_xml=''):
        super().__init__()
        self.bridge = bridge
        self.data_dir = Path(data_dir).expanduser()
        self.recorder = Recorder(RATE)
        self.upload_worker = None
        self.tick_count = 0
        self.setWindowTitle('촉각 데이터 수집 — ROHand AP002')
        self.resize(1400, 950)

        self.hand_view = HandView(640, 480, test_object=test_object, hand_xml=hand_xml)
        self.hand_panel = HandPanel(self.hand_view)
        self.webcam_panel = ImagePanel('웹캠 (사람 손)', '웹캠 영상 없음\n(/webcam/image_annotated/compressed)')
        # turned like the hand view, which makes it portrait: fill the panel, crop top/bottom
        self.robot_cam_panel = ImagePanel('로봇 핸드 카메라',
                                          f'로봇 카메라 영상 없음\n({bridge.robot_camera_topic})',
                                          fill=True)

        cameras = QSplitter(Qt.Vertical)
        cameras.addWidget(self.webcam_panel)
        cameras.addWidget(self.robot_cam_panel)
        top = QSplitter(Qt.Horizontal)
        top.addWidget(self.hand_panel)
        top.addWidget(cameras)
        top.setStretchFactor(0, 3)
        top.setStretchFactor(1, 2)

        self.plots = {
            'effort': ForcePlot('액추에이터 힘', 'N', FINGERS),
            'normal': ForcePlot('촉각 — 누르는 힘 (법선)', '센서 값', TACTILE_REGIONS),
            'tangential': ForcePlot('촉각 — 미끄러지는 힘 (전단)', '센서 값', TACTILE_REGIONS[:5]),
        }
        plots = QWidget()
        plot_row = QHBoxLayout(plots)
        plot_row.setContentsMargins(0, 0, 0, 0)
        for plot in self.plots.values():
            plot_row.addWidget(plot)

        body = QSplitter(Qt.Vertical)
        body.addWidget(top)
        body.addWidget(plots)
        body.setStretchFactor(0, 3)
        body.setStretchFactor(1, 1)
        body.setSizes([660, 230])  # plots ~1/4 of the height; drag the divider to change

        main = QSplitter(Qt.Horizontal)
        main.addWidget(self.build_sidebar())
        main.addWidget(body)
        main.setStretchFactor(0, 0)
        main.setStretchFactor(1, 1)
        main.setSizes([300, 1300])
        self.setCentralWidget(main)
        self.status = self.statusBar()
        self.refresh_episodes()

        length = int(HISTORY_S * RATE)
        self.history_t = collections.deque(maxlen=length)
        self.history = {key: {n: collections.deque(maxlen=length) for n in plot.curves}
                        for key, plot in self.plots.items()}

        self.last = None            # latest (snapshot, webcam_bgr, robot_rgb) for the screen
        self.webcam_cache = (None, None)  # (jpeg bytes, decoded) to decode each frame once
        # sampling + recording run in their own thread at a steady RATE, so a slow screen
        # refresh (3D render, plots) never delays recorded frames
        self.data_lock = threading.Lock()
        self.running = True
        self.sampler = threading.Thread(target=self.sample_loop, daemon=True)
        self.sampler.start()
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.refresh)
        self.ui_timer.start(int(round(1000 / UI_RATE)))

    # ---- left side: data saving tabs -------------------------------------------
    def build_sidebar(self):
        tabs = QTabWidget()
        tabs.setMinimumWidth(260)
        tabs.tabBar().setExpanding(True)
        tabs.addTab(self.build_record_tab(), '녹화')
        tabs.addTab(self.build_upload_tab(), '업로드')
        tabs.addTab(self.build_settings_tab(), '보정')
        return tabs

    @staticmethod
    def heading(text):
        label = QLabel(text)
        label.setObjectName('heading')
        return label

    @staticmethod
    def muted(text):
        label = QLabel(text)
        label.setObjectName('muted')
        label.setWordWrap(True)
        return label

    def build_record_tab(self):
        self.task = QComboBox()
        self.task.setEditable(True)
        self.task.addItems(TASKS)
        self.task.setToolTip('파지 종류 (직접 입력 가능)')
        self.notes = QLineEdit()
        self.notes.setPlaceholderText('물체, 조건 등')
        self.record_button = QPushButton('● 녹화 시작')
        self.record_button.setObjectName('record')
        self.record_button.setCheckable(True)
        self.record_button.toggled.connect(self.toggle_record)
        self.record_label = self.muted('대기 중')
        self.folder_button = QPushButton()
        self.folder_button.clicked.connect(self.choose_folder)
        self.show_folder()
        self.episode_list = QListWidget()

        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.addWidget(self.heading('작업 (파지 종류)'))
        layout.addWidget(self.task)
        layout.addWidget(self.heading('메모'))
        layout.addWidget(self.notes)
        layout.addSpacing(8)
        layout.addWidget(self.record_button)
        layout.addWidget(self.record_label)
        layout.addSpacing(8)
        layout.addWidget(self.heading('저장 폴더'))
        layout.addWidget(self.folder_button)
        layout.addWidget(self.heading('저장된 에피소드'))
        layout.addWidget(self.episode_list, 1)
        return tab

    def build_upload_tab(self):
        self.upload_info = self.muted('')
        self.upload_button = QPushButton('Hugging Face에 업로드')
        self.upload_button.clicked.connect(self.upload)
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.addWidget(self.heading('Hugging Face 데이터셋'))
        layout.addWidget(self.upload_info)
        layout.addWidget(self.upload_button)
        layout.addWidget(self.muted('저장 폴더 전체를 데이터셋 저장소로 올립니다. 기본은 비공개입니다. '
                                    '웹캠 영상에 얼굴이 나오면 공개 전에 확인하세요.'))
        layout.addWidget(self.muted(f'처음 한 번:\n{INSTALL_HINT}\nhuggingface-cli login'))
        layout.addStretch(1)
        return tab

    def build_settings_tab(self):
        reset = QPushButton('손 보정 초기화')
        reset.clicked.connect(self.reset_calibration)
        view = QPushButton('로봇 손 시점 초기화')
        view.clicked.connect(self.reset_view)
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.addWidget(self.heading('웹캠 손 보정'))
        layout.addWidget(self.muted('자세 명령 노드는 손을 끝까지 폈다 쥐면 자동으로 보정됩니다. '
                                    '로봇 손가락이 덜/더 굽으면 초기화한 뒤 다시 한 번 쥐었다 펴세요.'))
        layout.addWidget(reset)
        layout.addSpacing(12)
        layout.addWidget(self.heading('로봇 손 화면'))
        layout.addWidget(self.muted('드래그: 회전 · 휠: 확대/축소'))
        layout.addWidget(view)
        layout.addStretch(1)
        return tab

    def reset_calibration(self):
        self.bridge.reset_calibration()
        self.status.showMessage('손 보정 초기화: 손을 끝까지 폈다가 쥐었다 펴세요', 8000)

    def reset_view(self):
        self.hand_view.reset_camera()

    def refresh_episodes(self):
        self.episode_list.clear()
        folders = sorted(self.data_dir.glob('episode_*')) if self.data_dir.is_dir() else []
        for folder in reversed(folders):
            try:
                meta = json.loads((folder / 'meta.json').read_text())
                text = f"{folder.name} · {meta.get('task', '')} · {meta.get('frames', 0)}f · " \
                       f"{meta.get('duration_s', 0):.1f}s"
            except (OSError, ValueError):
                text = f'{folder.name} (meta 없음)'
            self.episode_list.addItem(text)
        self.upload_info.setText(f'폴더: {self.data_dir}\n에피소드 {len(folders)}개')

    def toggle_record(self, on):
        if on:
            folder = self.recorder.start(self.data_dir, self.task.currentText().strip() or 'free',
                                         self.notes.text().strip())
            self.record_button.setText('■ 녹화 정지')
            self.folder_button.setEnabled(False)
            self.upload_button.setEnabled(False)
            self.status.showMessage(f'녹화 중: {folder}')
        else:
            folder, frames = self.recorder.stop()
            self.record_button.setText('● 녹화 시작')
            self.folder_button.setEnabled(True)
            self.upload_button.setEnabled(True)
            self.record_label.setText(f'저장됨: {folder.name} ({frames} 프레임)' if folder else '대기 중')
            if folder:
                self.status.showMessage(f'저장 완료: {folder}', 10000)
            self.refresh_episodes()

    def choose_folder(self):
        folder = QFileDialog.getExistingDirectory(self, '저장 폴더 선택', str(self.data_dir))
        if folder:
            self.data_dir = Path(folder)
            self.show_folder()
            self.refresh_episodes()

    def show_folder(self):
        self.folder_button.setText(f'📁 {self.data_dir.name}')
        self.folder_button.setToolTip(f'저장 폴더: {self.data_dir}')

    def upload(self):
        if not hub_available():
            QMessageBox.information(self, 'Hugging Face 업로드',
                                    f'huggingface_hub가 설치되어 있지 않습니다.\n터미널에서 설치하세요:\n\n'
                                    f'{INSTALL_HINT}\nhuggingface-cli login')
            return
        episodes = sorted(self.data_dir.glob('episode_*')) if self.data_dir.is_dir() else []
        if not episodes:
            QMessageBox.information(self, 'Hugging Face 업로드', f'{self.data_dir}에 에피소드가 없습니다.')
            return
        dialog = UploadDialog(self, self.data_dir, len(episodes))
        if dialog.exec_() != QDialog.Accepted or not dialog.repo.text().strip():
            return
        repo = dialog.repo.text().strip()
        visibility = '비공개' if dialog.private.isChecked() else '공개'
        answer = QMessageBox.question(
            self, '업로드 확인',
            f'{self.data_dir}의 에피소드 {len(episodes)}개를\nhuggingface.co/datasets/{repo} '
            f'({visibility})에 올립니다.\n웹캠 영상에 얼굴이 있다면 공개 범위를 확인하세요. 계속할까요?')
        if answer != QMessageBox.Yes:
            return
        self.upload_button.setEnabled(False)
        self.upload_button.setText('업로드 중…')
        self.upload_worker = UploadWorker(self.data_dir, repo, dialog.private.isChecked(),
                                          dialog.token.text().strip())
        self.upload_worker.finished_ok.connect(self.upload_done)
        self.upload_worker.failed.connect(self.upload_failed)
        self.upload_worker.start()

    def upload_done(self, url):
        self.upload_button.setEnabled(True)
        self.upload_button.setText('Hugging Face에 업로드')
        QMessageBox.information(self, '업로드 완료', f'업로드했습니다:\n{url}')

    def upload_failed(self, message):
        self.upload_button.setEnabled(True)
        self.upload_button.setText('Hugging Face에 업로드')
        QMessageBox.warning(self, '업로드 실패', message)

    # ---- periodic update ---------------------------------------------------------
    def sample_loop(self):
        period = 1.0 / RATE
        deadline = time.monotonic()
        while self.running:
            self.sample()
            deadline += period
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif delay < -period:  # fell behind: resync instead of bursting
                deadline = time.monotonic()

    def sample(self):
        """RATE Hz (sampler thread): take the data, record it, extend the plot history."""
        snap = self.bridge.snapshot()
        now = snap['time']
        jpeg, stamp = snap['webcam_jpeg']
        webcam_bgr = None
        if jpeg is not None and now - stamp <= STALE_S:
            if jpeg is not self.webcam_cache[0]:
                self.webcam_cache = (jpeg, cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR))
            webcam_bgr = self.webcam_cache[1]
        robot_rgb, stamp = snap['robot_image']
        robot_rgb = robot_rgb if robot_rgb is not None and now - stamp <= STALE_S else None
        self.recorder.tick(snap, webcam_bgr, robot_rgb)
        with self.data_lock:
            self.append_history(snap)
            self.last = (snap, webcam_bgr, robot_rgb)

    def refresh(self):
        """15 Hz: draw what sample() collected."""
        with self.data_lock:
            if self.last is None:
                return
            snap, webcam_bgr, robot_rgb = self.last
            times = np.array(self.history_t) - snap['time']
            series = {key: {n: np.array(v, float) for n, v in hist.items()}
                      for key, hist in self.history.items()}
        self.tick_count += 1
        self.webcam_panel.show_rgb(None if webcam_bgr is None else webcam_bgr[:, :, ::-1])
        # shown turned 90 deg counter-clockwise like the hand view (thumb up); recorded as captured
        self.robot_cam_panel.show_rgb(None if robot_rgb is None else np.rot90(robot_rgb, 1))
        self.hand_view.update_pose(snap)
        self.hand_panel.show_frame(*self.hand_view.render(snap['tactile']))
        for key, plot in self.plots.items():
            plot.update_curves(times, series[key])
        self.update_status(snap, webcam_bgr is not None, robot_rgb is not None)

    def append_history(self, snap):
        now = snap['time']
        self.history_t.append(now)
        for f in FINGERS:
            e = snap['fingers'][f]
            self.history['effort'][f].append(e['effort'] if now - e['stamp'] <= STALE_S else np.nan)
        for key in ('normal', 'tangential'):
            for r in self.history[key]:
                e = snap['tactile'][r]
                self.history[key][r].append(e[key] if now - e['stamp'] <= STALE_S else np.nan)

    def update_status(self, snap, webcam_ok, robot_ok):
        now = snap['time']
        fingers = sum(now - e['stamp'] <= STALE_S for e in snap['fingers'].values())
        tactile = sum(now - e['stamp'] <= STALE_S for e in snap['tactile'].values())
        if self.recorder.active:
            elapsed = now - self.recorder.started
            self.record_label.setText(f'녹화 {elapsed:5.1f}s · {self.recorder.frames} 프레임')
        if not self.recorder.active and self.tick_count % 8 == 0:
            self.status.showMessage(
                f'손가락 {fingers}/{len(FINGERS)} · 촉각 {tactile}/{len(TACTILE_REGIONS)} · '
                f'웹캠 {"✓" if webcam_ok else "✗"} · 로봇 카메라 {"✓" if robot_ok else "✗"} · '
                f'{time.strftime("%H:%M:%S")}')

    def closeEvent(self, event):
        self.running = False
        self.sampler.join(timeout=1.0)
        if self.recorder.active:
            self.recorder.stop()
        self.ui_timer.stop()
        self.hand_view.close()
        super().closeEvent(event)
