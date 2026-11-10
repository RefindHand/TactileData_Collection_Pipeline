"""Dark theme for the whole window (Qt Fusion palette + pyqtgraph defaults)."""
import pyqtgraph as pg
from PyQt5.QtGui import QColor, QPalette
from PyQt5.QtWidgets import QStyleFactory

BACKGROUND = '#1b1c1f'
PANEL = '#232428'
INPUT = '#2b2d31'
TEXT = '#e3e5e8'
MUTED = '#9aa0a6'
ACCENT = '#4c8dff'
RECORD = '#e5484d'
BORDER = '#3a3c42'

STYLE = f"""
QWidget {{ font-size: 10pt; }}
QTabWidget::pane {{ border: 1px solid {BORDER}; background: {PANEL}; }}
QTabBar::tab {{ background: {BACKGROUND}; color: {MUTED}; padding: 8px 14px; border: 1px solid {BORDER}; }}
QTabBar::tab:selected {{ background: {PANEL}; color: {TEXT}; border-bottom: 3px solid {ACCENT}; }}
QPushButton {{ background: {INPUT}; color: {TEXT}; border: 1px solid {BORDER}; border-radius: 6px; padding: 7px 10px; }}
QPushButton:hover {{ border-color: {ACCENT}; }}
QPushButton:disabled {{ color: #5f6368; }}
QPushButton#record {{ font-size: 12pt; font-weight: bold; padding: 12px; }}
QPushButton#record:checked {{ background: {RECORD}; border-color: {RECORD}; color: white; }}
QPushButton#stop {{ background: {RECORD}; border-color: {RECORD}; color: white; font-size: 12pt; font-weight: bold; padding: 10px; }}
QPushButton#start {{ background: #1f8a5b; border-color: #1f8a5b; color: white; font-size: 12pt; font-weight: bold; padding: 10px; }}
QLineEdit, QComboBox, QPlainTextEdit, QListWidget {{ background: {INPUT}; color: {TEXT}; border: 1px solid {BORDER}; border-radius: 5px; padding: 4px; }}
QLabel#heading {{ color: {TEXT}; font-weight: bold; font-size: 11pt; }}
QLabel#muted {{ color: {MUTED}; }}
QSplitter::handle {{ background: {BACKGROUND}; }}
QStatusBar {{ color: {MUTED}; }}
"""


def apply_dark(app):
    app.setStyle(QStyleFactory.create('Fusion'))
    palette = QPalette()
    for role, color in [
        (QPalette.Window, BACKGROUND), (QPalette.WindowText, TEXT), (QPalette.Base, INPUT),
        (QPalette.AlternateBase, PANEL), (QPalette.ToolTipBase, PANEL), (QPalette.ToolTipText, TEXT),
        (QPalette.Text, TEXT), (QPalette.Button, INPUT), (QPalette.ButtonText, TEXT),
        (QPalette.Highlight, ACCENT), (QPalette.HighlightedText, '#ffffff'),
        (QPalette.PlaceholderText, MUTED), (QPalette.Link, ACCENT),
    ]:
        palette.setColor(role, QColor(color))
    for role in (QPalette.Text, QPalette.ButtonText, QPalette.WindowText):
        palette.setColor(QPalette.Disabled, role, QColor('#5f6368'))
    app.setPalette(palette)
    app.setStyleSheet(STYLE)
    pg.setConfigOption('background', PANEL)
    pg.setConfigOption('foreground', MUTED)
    pg.setConfigOption('antialias', True)
