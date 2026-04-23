#!/usr/bin/env python3
"""
RoboRooks — Desktop Launcher
Starts/stops cv_chess_play.py and chess_waypoint.py, streams their logs,
and keeps the OpenCV chess board window alive alongside this GUI.

Usage:
    python3 roborooks_app.py

Edit the SCRIPT_* paths below to match your workspace.
"""

import sys
import os
import subprocess
import shlex
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QTextEdit, QSplitter, QFrame, QSizePolicy,
    QGraphicsDropShadowEffect
)
from PyQt5.QtCore import (
    Qt, QThread, pyqtSignal, QTimer, QPropertyAnimation,
    QEasingCurve, QSize, QPoint
)
from PyQt5.QtGui import (
    QColor, QFont, QFontDatabase, QPalette, QTextCursor,
    QIcon, QPainter, QPen, QBrush, QLinearGradient, QPixmap
)

# =============================================================================
# === CONFIGURATION — edit these paths ========================================
# =============================================================================

SCRIPT_CV   = os.path.expanduser("~/roborooks/src/board_cv/scripts/cv_chess_play.py")
SCRIPT_WPT  = os.path.expanduser("~/roborooks/src/chess_play/scripts/chess_waypoint.py")

# ROS environment source command — adjust distro if needed
ROS_SETUP   = "source /opt/ros/noetic/setup.bash"
# Workspace overlay — adjust path if needed  
WS_SETUP    = os.path.expanduser("~/roborooks/devel/setup.bash")

# =============================================================================
# === THEME ===================================================================
# =============================================================================

DARK_BG      = "#0d0f14"
PANEL_BG     = "#13161d"
CARD_BG      = "#181c25"
BORDER       = "#252a38"
ACCENT_GREEN = "#00e5a0"
ACCENT_AMBER = "#f5a623"
ACCENT_RED   = "#ff4757"
ACCENT_BLUE  = "#4a9eff"
TEXT_PRIMARY = "#e8eaf0"
TEXT_DIM     = "#5a6070"
TEXT_MID     = "#8891a8"

FONT_MONO    = "JetBrains Mono,Fira Code,Consolas,Courier New"
FONT_UI      = "Inter,Segoe UI,Helvetica Neue,Arial"

STYLE_SHEET = f"""
QMainWindow, QWidget {{
    background-color: {DARK_BG};
    color: {TEXT_PRIMARY};
    font-family: {FONT_UI};
}}

QSplitter::handle {{
    background: {BORDER};
    width: 1px;
    height: 1px;
}}

QTextEdit {{
    background-color: {DARK_BG};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER};
    border-radius: 6px;
    font-family: {FONT_MONO};
    font-size: 11px;
    padding: 8px;
    selection-background-color: {ACCENT_BLUE};
}}

QScrollBar:vertical {{
    background: {PANEL_BG};
    width: 6px;
    border-radius: 3px;
}}
QScrollBar::handle:vertical {{
    background: {BORDER};
    border-radius: 3px;
    min-height: 20px;
}}
QScrollBar::handle:vertical:hover {{
    background: {TEXT_DIM};
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0px;
}}
QScrollBar:horizontal {{
    height: 0px;
}}

QPushButton {{
    border-radius: 6px;
    font-family: {FONT_UI};
    font-size: 12px;
    font-weight: 600;
    letter-spacing: 0.5px;
    padding: 8px 18px;
    border: none;
}}
QPushButton:disabled {{
    opacity: 0.4;
}}
"""

# =============================================================================
# === LOG READER THREAD =======================================================
# =============================================================================

class ProcessReader(QThread):
    """Reads stdout+stderr from a subprocess and emits each line."""
    line_ready = pyqtSignal(str)
    finished   = pyqtSignal()

    def __init__(self, proc):
        super().__init__()
        self._proc = proc

    def run(self):
        try:
            for raw in self._proc.stdout:
                line = raw.decode(errors="replace").rstrip("\n")
                self.line_ready.emit(line)
        except Exception:
            pass
        self.finished.emit()

# =============================================================================
# === PULSING STATUS DOT ======================================================
# =============================================================================

class StatusDot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(10, 10)
        self._state  = "off"   # "off" | "running" | "error"
        self._alpha  = 255
        self._timer  = QTimer(self)
        self._timer.timeout.connect(self._pulse)
        self._dir    = -1
        self._timer.start(30)

    def set_state(self, state):
        self._state = state
        self.update()

    def _pulse(self):
        if self._state != "running":
            self._alpha = 255
            self.update()
            return
        self._alpha += self._dir * 6
        if self._alpha <= 80:
            self._dir = 1
        if self._alpha >= 255:
            self._dir = -1
        self._alpha = max(80, min(255, self._alpha))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        colors = {
            "off":     QColor(TEXT_DIM),
            "running": QColor(ACCENT_GREEN),
            "error":   QColor(ACCENT_RED),
        }
        c = colors.get(self._state, QColor(TEXT_DIM))
        c.setAlpha(self._alpha)
        painter.setBrush(QBrush(c))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(0, 0, 10, 10)

# =============================================================================
# === PROCESS CARD ============================================================
# =============================================================================

class ProcessCard(QFrame):
    """
    A self-contained card that manages one script process:
    start/stop button, status indicator, and scrolling log output.
    """

    def __init__(self, title, script_path, color, parent=None):
        super().__init__(parent)
        self._title       = title
        self._script      = script_path
        self._color       = color
        self._proc        = None
        self._reader      = None
        self._line_count  = 0

        self.setObjectName("ProcessCard")
        self.setStyleSheet(f"""
            #ProcessCard {{
                background-color: {CARD_BG};
                border: 1px solid {BORDER};
                border-radius: 10px;
            }}
        """)

        self._build_ui()

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(10)

        # --- Header row ---
        header = QHBoxLayout()
        header.setSpacing(10)

        self._dot = StatusDot()
        header.addWidget(self._dot)

        title_lbl = QLabel(self._title)
        title_lbl.setFont(QFont(FONT_UI.split(",")[0], 13, QFont.Bold))
        title_lbl.setStyleSheet(f"color: {TEXT_PRIMARY}; background: transparent;")
        header.addWidget(title_lbl)

        header.addStretch()

        # Script path label
        short = os.path.basename(self._script)
        path_lbl = QLabel(short)
        path_lbl.setFont(QFont(FONT_MONO.split(",")[0], 9))
        path_lbl.setStyleSheet(f"color: {TEXT_DIM}; background: transparent;")
        header.addWidget(path_lbl)

        root.addLayout(header)

        # --- Divider ---
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet(f"background: {BORDER}; max-height: 1px; border: none;")
        root.addWidget(line)

        # --- Log area ---
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setLineWrapMode(QTextEdit.NoWrap)
        self._log.setFont(QFont(FONT_MONO.split(",")[0], 10))
        self._log.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._log.setPlaceholderText("No output yet — press Start.")
        root.addWidget(self._log)

        # --- Footer row ---
        footer = QHBoxLayout()
        footer.setSpacing(8)

        self._status_lbl = QLabel("Stopped")
        self._status_lbl.setFont(QFont(FONT_UI.split(",")[0], 10))
        self._status_lbl.setStyleSheet(f"color: {TEXT_DIM}; background: transparent;")
        footer.addWidget(self._status_lbl)

        footer.addStretch()

        self._clear_btn = QPushButton("Clear")
        self._clear_btn.setFixedHeight(30)
        self._clear_btn.setStyleSheet(f"""
            QPushButton {{
                background: transparent;
                color: {TEXT_DIM};
                border: 1px solid {BORDER};
                border-radius: 5px;
                font-size: 11px;
                padding: 0 12px;
            }}
            QPushButton:hover {{
                color: {TEXT_MID};
                border-color: {TEXT_DIM};
            }}
        """)
        self._clear_btn.clicked.connect(self._log.clear)
        footer.addWidget(self._clear_btn)

        self._btn = QPushButton("▶  Start")
        self._btn.setFixedHeight(32)
        self._btn.setFixedWidth(110)
        self._set_btn_start()
        self._btn.clicked.connect(self._toggle)
        footer.addWidget(self._btn)

        root.addLayout(footer)

    # --- Button styling helpers ---

    def _set_btn_start(self):
        self._btn.setText("▶  Start")
        self._btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {self._color};
                color: #000;
                border-radius: 6px;
                font-weight: 700;
                font-size: 12px;
            }}
            QPushButton:hover {{
                background-color: white;
            }}
        """)

    def _set_btn_stop(self):
        self._btn.setText("■  Stop")
        self._btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {CARD_BG};
                color: {ACCENT_RED};
                border: 1px solid {ACCENT_RED};
                border-radius: 6px;
                font-weight: 700;
                font-size: 12px;
            }}
            QPushButton:hover {{
                background-color: {ACCENT_RED};
                color: white;
            }}
        """)

    # --- Process control ---

    def _toggle(self):
        if self._proc is None or self._proc.poll() is not None:
            self._start()
        else:
            self._stop()

    def _build_cmd(self):
        """Build the shell command that sources ROS and runs the script."""
        cmds = []
        cmds.append(ROS_SETUP)
        if os.path.exists(WS_SETUP):
            cmds.append(f"source {WS_SETUP}")
        cmds.append(f"python3 {shlex.quote(self._script)}")
        return ["bash", "-c", " && ".join(cmds)]

    def _start(self):
        if not os.path.exists(self._script):
            self._append_line(
                f"[ERROR] Script not found: {self._script}", error=True
            )
            return

        self._log.clear()
        self._line_count = 0
        self._append_line(f"[{self._now()}] Starting {os.path.basename(self._script)}…")

        try:
            self._proc = subprocess.Popen(
                self._build_cmd(),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=os.environ.copy(),
            )
        except Exception as e:
            self._append_line(f"[ERROR] Failed to launch: {e}", error=True)
            self._dot.set_state("error")
            return

        self._dot.set_state("running")
        self._set_btn_stop()
        self._status_lbl.setText(f"PID {self._proc.pid}")
        self._status_lbl.setStyleSheet(f"color: {ACCENT_GREEN}; background: transparent;")

        self._reader = ProcessReader(self._proc)
        self._reader.line_ready.connect(self._append_line)
        self._reader.finished.connect(self._on_proc_finished)
        self._reader.start()

    def _stop(self):
        if self._proc and self._proc.poll() is None:
            self._append_line(f"[{self._now()}] Stopping…")
            self._proc.terminate()
            QTimer.singleShot(2000, self._force_kill)

    def _force_kill(self):
        if self._proc and self._proc.poll() is None:
            self._proc.kill()
            self._append_line(f"[{self._now()}] Force-killed.")

    def _on_proc_finished(self):
        code = self._proc.returncode if self._proc else "?"
        self._append_line(f"[{self._now()}] Process exited (code {code}).")
        if code == 0 or code == -15:  # clean exit or SIGTERM
            self._dot.set_state("off")
        else:
            self._dot.set_state("error")
        self._set_btn_start()
        self._status_lbl.setText("Stopped")
        self._status_lbl.setStyleSheet(f"color: {TEXT_DIM}; background: transparent;")

    def is_running(self):
        return self._proc is not None and self._proc.poll() is None

    def stop_process(self):
        """Called on app close."""
        if self.is_running():
            self._proc.terminate()

    # --- Log helpers ---

    def _append_line(self, text, error=False):
        cursor = self._log.textCursor()
        cursor.movePosition(QTextCursor.End)

        # Color-code ROS log levels and errors
        if error or "[ERROR]" in text or "Traceback" in text or "Error" in text:
            color = ACCENT_RED
        elif "[WARN]" in text:
            color = ACCENT_AMBER
        elif "[INFO]" in text or text.startswith("["):
            color = TEXT_MID
        elif "[AI]" in text or "[YOU]" in text or "[OVERRIDE]" in text:
            color = ACCENT_GREEN
        elif "[DEBUG]" in text:
            color = ACCENT_BLUE
        else:
            color = TEXT_PRIMARY

        fmt = cursor.charFormat()
        fmt.setForeground(QColor(color))
        cursor.setCharFormat(fmt)
        cursor.insertText(text + "\n")

        self._line_count += 1
        # Keep log from growing unbounded
        if self._line_count > 2000:
            cursor.movePosition(QTextCursor.Start)
            cursor.movePosition(QTextCursor.Down, QTextCursor.KeepAnchor, 200)
            cursor.removeSelectedText()
            self._line_count -= 200

        self._log.setTextCursor(cursor)
        self._log.ensureCursorVisible()

    @staticmethod
    def _now():
        return datetime.now().strftime("%H:%M:%S")

# =============================================================================
# === HEADER WIDGET ===========================================================
# =============================================================================

class Header(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(64)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(24, 0, 24, 0)

        # Chess knight unicode as logo
        logo = QLabel("♞")
        logo.setFont(QFont("serif", 28))
        logo.setStyleSheet(f"color: {ACCENT_GREEN}; background: transparent;")
        layout.addWidget(logo)

        name = QLabel("RoboRooks")
        name.setFont(QFont(FONT_UI.split(",")[0], 18, QFont.Bold))
        name.setStyleSheet(f"color: {TEXT_PRIMARY}; background: transparent; letter-spacing: 1px;")
        layout.addWidget(name)

        sub = QLabel("  Control Panel")
        sub.setFont(QFont(FONT_UI.split(",")[0], 12))
        sub.setStyleSheet(f"color: {TEXT_DIM}; background: transparent;")
        layout.addWidget(sub)

        layout.addStretch()

        self._clock = QLabel()
        self._clock.setFont(QFont(FONT_MONO.split(",")[0], 11))
        self._clock.setStyleSheet(f"color: {TEXT_DIM}; background: transparent;")
        layout.addWidget(self._clock)

        timer = QTimer(self)
        timer.timeout.connect(self._tick)
        timer.start(1000)
        self._tick()

    def _tick(self):
        self._clock.setText(datetime.now().strftime("%H:%M:%S"))

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(PANEL_BG))
        # bottom border line
        pen = QPen(QColor(BORDER))
        pen.setWidth(1)
        painter.setPen(pen)
        painter.drawLine(0, self.height() - 1, self.width(), self.height() - 1)

# =============================================================================
# === MAIN WINDOW =============================================================
# =============================================================================

class RoboRooksApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RoboRooks")
        self.setMinimumSize(980, 680)
        self.resize(1200, 750)
        self.setStyleSheet(STYLE_SHEET)

        # Remove default title bar frame for a cleaner look, keep resize
        self._drag_pos = None

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # Header
        self._header = Header()
        root.addWidget(self._header)

        # Body
        body = QWidget()
        body_layout = QVBoxLayout(body)
        body_layout.setContentsMargins(20, 16, 20, 16)
        body_layout.setSpacing(12)

        # Top control row
        ctrl = QHBoxLayout()
        ctrl.setSpacing(10)

        self._start_all_btn = self._make_global_btn(
            "▶  Start All", ACCENT_GREEN, "#000"
        )
        self._start_all_btn.clicked.connect(self._start_all)
        ctrl.addWidget(self._start_all_btn)

        self._stop_all_btn = self._make_global_btn(
            "■  Stop All", CARD_BG, ACCENT_RED, border=ACCENT_RED
        )
        self._stop_all_btn.clicked.connect(self._stop_all)
        ctrl.addWidget(self._stop_all_btn)

        ctrl.addStretch()

        hint = QLabel("OpenCV board window opens automatically when Vision is started.")
        hint.setFont(QFont(FONT_UI.split(",")[0], 10))
        hint.setStyleSheet(f"color: {TEXT_DIM}; background: transparent;")
        ctrl.addWidget(hint)

        body_layout.addLayout(ctrl)

        # Process cards in a horizontal splitter
        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(12)

        self._cv_card = ProcessCard(
            "Vision & Game Logic",
            SCRIPT_CV,
            ACCENT_BLUE,
        )
        splitter.addWidget(self._cv_card)

        self._wpt_card = ProcessCard(
            "Waypoint Controller",
            SCRIPT_WPT,
            ACCENT_AMBER,
        )
        splitter.addWidget(self._wpt_card)

        splitter.setSizes([580, 580])
        body_layout.addWidget(splitter)

        root.addWidget(body)

    # --- Global buttons ---

    def _make_global_btn(self, text, bg, fg, border=None):
        btn = QPushButton(text)
        btn.setFixedHeight(36)
        btn.setMinimumWidth(130)
        border_css = f"border: 1px solid {border};" if border else "border: none;"
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {bg};
                color: {fg};
                {border_css}
                border-radius: 7px;
                font-weight: 700;
                font-size: 12px;
                letter-spacing: 0.4px;
                padding: 0 16px;
            }}
            QPushButton:hover {{
                background-color: {"white" if bg == ACCENT_GREEN else ACCENT_RED};
                color: {"#000" if bg == ACCENT_GREEN else "white"};
                border: none;
            }}
        """)
        return btn

    def _start_all(self):
        self._cv_card._start()
        # Small delay so ROS master is ready before waypoint node
        QTimer.singleShot(1500, self._wpt_card._start)

    def _stop_all(self):
        self._cv_card._stop()
        self._wpt_card._stop()

    # --- Clean shutdown ---

    def closeEvent(self, event):
        self._cv_card.stop_process()
        self._wpt_card.stop_process()
        event.accept()

# =============================================================================
# === ENTRY POINT =============================================================
# =============================================================================

def main():
    app = QApplication(sys.argv)
    app.setApplicationName("RoboRooks")

    # Load a monospace font if available
    QFontDatabase.addApplicationFont("/usr/share/fonts/truetype/firacode/FiraCode-Regular.ttf")

    window = RoboRooksApp()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
