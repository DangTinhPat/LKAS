#!/usr/bin/env python3
"""
DVT Autonomous Robot — Simulation Control GUI
Dark-theme real-time monitor: launch, stop, waveform charts for e_y / e_psi.
"""

import sys
import os
import signal
import subprocess
import collections
import time

try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget,
        QVBoxLayout, QHBoxLayout, QGridLayout,
        QPushButton, QLabel, QFrame, QSizePolicy, QTextEdit,
    )
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QObject
    from PyQt5.QtGui import QFont, QFontDatabase
    import pyqtgraph as pg
    import numpy as np
except ImportError as err:
    print(f"[sim_gui] Missing: {err}")
    print("Fix: pip install PyQt5 pyqtgraph numpy --break-system-packages")
    sys.exit(1)

import rclpy
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Float64


# ── Design tokens ─────────────────────────────────────────────────────────────
BG      = "#0b0f1e"
SURF    = "#111827"
SURF2   = "#1a2235"
BORDER  = "#1e293b"
TEXT    = "#e2e8f0"
MUTED   = "#475569"
EY_C    = "#06d6a0"   # teal
EPSI_C  = "#fb923c"   # amber-orange
GO_C    = "#22c55e"   # emerald
STOP_C  = "#ef4444"   # red
WARN_C  = "#fbbf24"   # yellow

STATE_CLR = {
    "FOLLOW":   "#60a5fa",
    "PREPARE":  "#fbbf24",
    "OVERTAKE": "#f87171",
    "RETURN":   "#a78bfa",
}

HISTORY = 500   # samples shown (~50s at 10 Hz)


# ── Stylesheet helpers ─────────────────────────────────────────────────────────
GLOBAL_SS = f"""
QMainWindow, QWidget {{
    background: {BG};
    color: {TEXT};
    font-family: 'Inter', 'Segoe UI', 'Noto Sans', sans-serif;
}}
QScrollBar {{ width: 0px; height: 0px; }}
"""


def _lbl(text="", size=12, bold=False, color=None) -> QLabel:
    lb = QLabel(text)
    c  = color or TEXT
    w  = "600" if bold else "400"
    lb.setStyleSheet(
        f"color:{c};font-size:{size}px;font-weight:{w};"
        f"background:transparent;border:none;")
    return lb


def _btn(text: str, bg: str, hover: str) -> QPushButton:
    b = QPushButton(text)
    b.setCursor(Qt.PointingHandCursor)
    b.setFixedHeight(52)
    b.setStyleSheet(f"""
        QPushButton {{
            background:{bg}; color:#fff;
            border:none; border-radius:12px;
            font-size:14px; font-weight:700;
            letter-spacing:0.6px; padding:0 28px;
        }}
        QPushButton:hover   {{ background:{hover}; }}
        QPushButton:pressed {{ background:{hover}; opacity:0.85; }}
        QPushButton:disabled{{
            background:{BORDER}; color:{MUTED};
        }}
    """)
    return b


def _card() -> QFrame:
    f = QFrame()
    f.setStyleSheet(f"""
        QFrame {{
            background:{SURF};
            border:1px solid {BORDER};
            border-radius:14px;
        }}
    """)
    return f


# ── ROS2 bridge ───────────────────────────────────────────────────────────────
class RosBridge(QObject):
    sig_ey    = pyqtSignal(float)
    sig_epsi  = pyqtSignal(float)
    sig_state = pyqtSignal(str)
    sig_speed = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self._alive = True

    def run(self):
        try:
            rclpy.init()
        except Exception:
            pass
        node = rclpy.create_node("sim_gui_bridge")
        node.create_subscription(
            Vector3, "/status_err",
            lambda m: (self.sig_ey.emit(float(m.x)),
                       self.sig_epsi.emit(float(m.y))), 10)
        node.create_subscription(
            String, "/overtake/state",
            lambda m: self.sig_state.emit(m.data), 10)
        node.create_subscription(
            Float64, "/overtake/target_speed",
            lambda m: self.sig_speed.emit(float(m.data)), 10)

        while self._alive:
            try:
                rclpy.spin_once(node, timeout_sec=0.04)
            except Exception:
                break
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def stop(self):
        self._alive = False


# ── Waveform chart ─────────────────────────────────────────────────────────────
class WaveChart(QFrame):
    def __init__(self, title: str, color: str, unit: str,
                 y_min=-0.65, y_max=0.65, limit=0.267):
        super().__init__()
        self.setStyleSheet(f"""
            QFrame {{
                background:{SURF};
                border:1px solid {BORDER};
                border-radius:14px;
            }}
        """)
        self._buf   = collections.deque([0.0] * HISTORY, maxlen=HISTORY)
        self._x     = list(range(HISTORY))
        self._color = color
        self._unit  = unit

        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 14, 18, 14)
        layout.setSpacing(8)

        # Title row
        row = QHBoxLayout()
        dot = _lbl("⬤", 10, color=color)
        ttl = _lbl(title, 13, bold=True)
        self._val = _lbl("—", 14, bold=True, color=color)
        self._val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        row.addWidget(dot)
        row.addSpacing(8)
        row.addWidget(ttl)
        row.addStretch()
        row.addWidget(self._val)
        layout.addLayout(row)

        # pyqtgraph PlotWidget
        pg.setConfigOptions(antialias=True)
        self.pw = pg.PlotWidget()
        self.pw.setBackground(SURF2)
        self.pw.setMinimumHeight(140)
        self.pw.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.pw.showGrid(x=False, y=True, alpha=0.12)
        self.pw.setXRange(0, HISTORY, padding=0)
        self.pw.setYRange(y_min, y_max, padding=0)
        self.pw.getAxis("bottom").setStyle(showValues=False)
        self.pw.getAxis("bottom").setPen(None)
        self.pw.getAxis("left").setPen(pg.mkPen(MUTED, width=1))
        self.pw.getAxis("left").setTextPen(MUTED)

        # Zero line
        self.pw.addItem(pg.InfiniteLine(
            pos=0, angle=0,
            pen=pg.mkPen(MUTED, width=1, style=Qt.DashLine)))

        # ±limit lines (lane half-width)
        for sign in (+1, -1):
            self.pw.addItem(pg.InfiniteLine(
                pos=sign * limit, angle=0,
                pen=pg.mkPen(WARN_C + "55", width=1, style=Qt.DotLine)))

        # Main curve with transparent fill
        self._curve = self.pw.plot(
            self._x, list(self._buf),
            pen=pg.mkPen(color, width=2),
            fillLevel=0,
            brush=pg.mkBrush(color + "22"))

        # Style the embedded plot background
        vb = self.pw.getViewBox()
        vb.setBackgroundColor(SURF2)

        layout.addWidget(self.pw)

    def push(self, v: float):
        self._buf.append(v)
        self._curve.setData(self._x, list(self._buf))
        self._val.setText(f"{v:+.3f} {self._unit}")


# ── Main window ───────────────────────────────────────────────────────────────
class SimGui(QMainWindow):
    def __init__(self):
        super().__init__()
        self._proc          = None
        self._rqt_proc      = None
        self._last_rx       = 0.0
        self._log_lines     = collections.deque(maxlen=80)

        self._build_window()
        self._build_ui()
        self._build_ros()

    # ── Window setup ──────────────────────────────────────────────────────────
    def _build_window(self):
        self.setWindowTitle("DVT  ·  Sim Control")
        self.setMinimumSize(920, 680)
        self.resize(1080, 800)
        self.setStyleSheet(GLOBAL_SS)

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        vbox = QVBoxLayout(root)
        vbox.setContentsMargins(24, 20, 24, 20)
        vbox.setSpacing(14)

        vbox.addLayout(self._header())
        vbox.addWidget(self._status_card())
        vbox.addWidget(self._charts_block(), stretch=5)
        vbox.addWidget(self._log_block(),    stretch=2)
        vbox.addLayout(self._buttons())

    # ─ Header ─────────────────────────────────────────────────────────────────
    def _header(self) -> QHBoxLayout:
        row = QHBoxLayout()

        logo = _lbl("DVT", 28, bold=True, color=EY_C)
        sub  = _lbl("Autonomous Robot  ·  Simulation Control", 13, color=MUTED)
        left = QVBoxLayout()
        left.setSpacing(2)
        left.addWidget(logo)
        left.addWidget(sub)

        self._ros_dot = _lbl("⬤", 11, color=MUTED)
        self._ros_txt = _lbl("ROS offline", 11, color=MUTED)
        right = QHBoxLayout()
        right.setSpacing(6)
        right.addWidget(self._ros_dot)
        right.addWidget(self._ros_txt)

        row.addLayout(left)
        row.addStretch()
        row.addLayout(right)
        return row

    # ─ Status card ────────────────────────────────────────────────────────────
    def _status_card(self) -> QFrame:
        card = _card()
        lay  = QHBoxLayout(card)
        lay.setContentsMargins(24, 16, 24, 16)
        lay.setSpacing(0)

        def col(cap, init, color=TEXT):
            w = QWidget()
            w.setStyleSheet("background:transparent;")
            v = QVBoxLayout(w)
            v.setContentsMargins(0, 0, 0, 0)
            v.setSpacing(3)
            v.addWidget(_lbl(cap, 10, color=MUTED))
            val = _lbl(init, 20, bold=True, color=color)
            v.addWidget(val)
            return w, val

        w, self._state_lbl  = col("STATE",        "OFFLINE",   MUTED)
        lay.addWidget(w)
        lay.addWidget(self._vdiv())

        w, self._speed_lbl  = col("TARGET SPEED", "0.00 m/s")
        lay.addWidget(w)
        lay.addWidget(self._vdiv())

        w, self._ey_stat    = col("e_y  (lateral error)", "0.000 m",  EY_C)
        lay.addWidget(w)
        lay.addWidget(self._vdiv())

        w, self._epsi_stat  = col("e_ψ  (heading error)", "0.000 rad", EPSI_C)
        lay.addWidget(w)

        lay.addStretch()
        return card

    def _vdiv(self) -> QWidget:
        wrap = QWidget()
        wrap.setStyleSheet("background:transparent;")
        h = QHBoxLayout(wrap)
        h.setContentsMargins(20, 6, 20, 6)
        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        line.setFixedWidth(1)
        line.setStyleSheet(f"background:{BORDER}; border:none;")
        h.addWidget(line)
        return wrap

    # ─ Charts ─────────────────────────────────────────────────────────────────
    def _charts_block(self) -> QWidget:
        w = QWidget()
        w.setStyleSheet("background:transparent;")
        v = QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(10)

        self._chart_ey   = WaveChart(
            "Lateral Error  e_y",  EY_C,   "m",
            y_min=-0.65, y_max=0.65, limit=0.267)
        self._chart_epsi = WaveChart(
            "Heading Error  e_ψ",  EPSI_C, "rad",
            y_min=-0.65, y_max=0.65, limit=0.3)

        v.addWidget(self._chart_ey,   stretch=1)
        v.addWidget(self._chart_epsi, stretch=1)
        return w

    # ─ Log area ───────────────────────────────────────────────────────────────
    def _log_block(self) -> QFrame:
        card = _card()
        v = QVBoxLayout(card)
        v.setContentsMargins(18, 12, 18, 12)
        v.setSpacing(6)

        hdr = QHBoxLayout()
        hdr.addWidget(_lbl("State Log", 12, bold=True, color=MUTED))
        hdr.addStretch()
        clear_btn = QPushButton("clear")
        clear_btn.setFixedSize(52, 22)
        clear_btn.setCursor(Qt.PointingHandCursor)
        clear_btn.setStyleSheet(f"""
            QPushButton {{
                background:{BORDER}; color:{MUTED};
                border:none; border-radius:6px; font-size:10px;
            }}
            QPushButton:hover {{ background:{SURF2}; color:{TEXT}; }}
        """)
        clear_btn.clicked.connect(self._clear_log)
        hdr.addWidget(clear_btn)
        v.addLayout(hdr)

        self._log_view = QTextEdit()
        self._log_view.setReadOnly(True)
        self._log_view.setMaximumHeight(90)
        self._log_view.setStyleSheet(f"""
            QTextEdit {{
                background:{SURF2}; color:{MUTED};
                border:1px solid {BORDER}; border-radius:8px;
                font-family:'Consolas','Courier New',monospace;
                font-size:11px; padding:6px;
            }}
        """)
        v.addWidget(self._log_view)
        return card

    def _log(self, text: str):
        ts = time.strftime("%H:%M:%S")
        self._log_view.append(f"<span style='color:{MUTED}'>{ts}</span>"
                               f"  <span style='color:{TEXT}'>{text}</span>")
        sb = self._log_view.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _clear_log(self):
        self._log_view.clear()

    # ─ Buttons ────────────────────────────────────────────────────────────────
    def _buttons(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(12)

        self._launch_btn = _btn("▶   LAUNCH SIMULATION",   GO_C,     "#16a34a")
        self._rqt_btn    = _btn("🎥  Camera View",          "#7c3aed", "#6d28d9")
        self._stop_btn   = _btn("■   STOP  &  EXIT",       STOP_C,   "#dc2626")
        self._stop_btn.setEnabled(False)

        self._launch_btn.clicked.connect(self._on_launch)
        self._rqt_btn.clicked.connect(self._on_rqt)
        self._stop_btn.clicked.connect(self._on_stop)

        row.addWidget(self._launch_btn, stretch=3)
        row.addWidget(self._rqt_btn,    stretch=2)
        row.addWidget(self._stop_btn,   stretch=1)
        return row

    # ── ROS2 ──────────────────────────────────────────────────────────────────
    def _build_ros(self):
        self._bridge = RosBridge()
        self._ros_thread = QThread()
        self._bridge.moveToThread(self._ros_thread)
        self._ros_thread.started.connect(self._bridge.run)
        self._bridge.sig_ey.connect(self._on_ey)
        self._bridge.sig_epsi.connect(self._on_epsi)
        self._bridge.sig_state.connect(self._on_state)
        self._bridge.sig_speed.connect(self._on_speed)
        self._ros_thread.start()

        self._ros_timer = QTimer(self)
        self._ros_timer.timeout.connect(self._check_ros)
        self._ros_timer.start(2000)

        self._proc_timer = QTimer(self)
        self._proc_timer.timeout.connect(self._poll_proc)
        self._proc_timer.start(1000)

    # ── Slots ─────────────────────────────────────────────────────────────────
    def _on_ey(self, v: float):
        self._last_rx = time.monotonic()
        self._chart_ey.push(v)
        self._ey_stat.setText(f"{v:+.3f} m")

    def _on_epsi(self, v: float):
        self._chart_epsi.push(v)
        self._epsi_stat.setText(f"{v:+.3f} rad")

    _prev_state = ""
    def _on_state(self, s: str):
        color = STATE_CLR.get(s, MUTED)
        self._state_lbl.setText(s)
        self._state_lbl.setStyleSheet(
            f"color:{color};font-size:20px;font-weight:700;"
            f"background:transparent;border:none;")
        if s != self._prev_state:
            self._log(f"→ <span style='color:{color};font-weight:600'>{s}</span>")
            self._prev_state = s

    def _on_speed(self, v: float):
        self._speed_lbl.setText(f"{v:.2f} m/s")

    def _check_ros(self):
        alive = (time.monotonic() - self._last_rx) < 3.0
        c  = GO_C if alive else MUTED
        tx = "ROS online" if alive else "ROS offline"
        self._ros_dot.setStyleSheet(
            f"color:{c};font-size:11px;background:transparent;border:none;")
        self._ros_txt.setStyleSheet(
            f"color:{c};font-size:11px;background:transparent;border:none;")
        self._ros_txt.setText(tx)

    def _poll_proc(self):
        if self._proc and self._proc.poll() is not None:
            self._proc = None
            self._launch_btn.setText("▶   LAUNCH SIMULATION")
            self._launch_btn.setEnabled(True)
            self._stop_btn.setEnabled(False)
            self._state_lbl.setText("OFFLINE")
            self._state_lbl.setStyleSheet(
                f"color:{MUTED};font-size:20px;font-weight:700;"
                f"background:transparent;border:none;")
            self._log("Simulation exited")

    # ── Launch / Stop ─────────────────────────────────────────────────────────
    def _on_launch(self):
        if self._proc and self._proc.poll() is None:
            return
        env = os.environ.copy()
        try:
            self._proc = subprocess.Popen(
                ["ros2", "launch", "main_bot", "gazebo.launch.py"],
                env=env,
                preexec_fn=os.setsid)       # new process group → killpg kills all children
        except FileNotFoundError:
            self._log("<span style='color:#f87171'>ros2 not found — source your workspace first</span>")
            return
        self._launch_btn.setText("⬤   Running…")
        self._launch_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        self._log("Simulation launched  (PID %d)" % self._proc.pid)

    def _on_rqt(self):
        # Nếu đã mở rồi thì đóng (toggle)
        if self._rqt_proc and self._rqt_proc.poll() is None:
            try:
                self._rqt_proc.terminate()
            except Exception:
                pass
            self._rqt_proc = None
            self._rqt_btn.setText("🎥  Camera View")
            self._log("Camera view closed")
            return
        try:
            self._rqt_proc = subprocess.Popen(
                ["ros2", "run", "rqt_image_view", "rqt_image_view", "/processed_image"],
                env=os.environ.copy())
        except FileNotFoundError:
            self._log("<span style='color:#f87171'>rqt_image_view not found</span>")
            return
        self._rqt_btn.setText("🎥  Close Camera")
        self._log("Camera view opened  (/processed_image)")

    def _on_stop(self):
        if self._proc:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
                self._proc.wait(timeout=6)
            except Exception:
                pass
            self._proc = None
        self._launch_btn.setText("▶   LAUNCH SIMULATION")
        self._launch_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._state_lbl.setText("OFFLINE")
        self._state_lbl.setStyleSheet(
            f"color:{MUTED};font-size:20px;font-weight:700;"
            f"background:transparent;border:none;")
        self._log("Simulation stopped")

    # ── Close ─────────────────────────────────────────────────────────────────
    def closeEvent(self, event):
        if self._rqt_proc and self._rqt_proc.poll() is None:
            try:
                self._rqt_proc.terminate()
            except Exception:
                pass
        self._on_stop()
        self._bridge.stop()
        self._ros_thread.quit()
        self._ros_thread.wait(3000)
        event.accept()


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    app = QApplication(sys.argv)
    app.setApplicationName("DVT Sim Control")
    win = SimGui()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
