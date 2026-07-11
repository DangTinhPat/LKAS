#!/usr/bin/env python3
"""
DVT Autonomous Robot — Unified Control GUI
Single panel for both Simulation and the Real robot. Each has two submodes:
  - Autonomous — launches the full self-driving stack; state/target-speed readout here,
    e_y/e_psi error graphs and camera feed open in separate windows on demand.
  - Manual (Joystick) — launches only the drivetrain/MCU bring-up (no lane_follower_node/
    overtake_node), and drives it directly from an on-screen joystick. Used to bench-test
    motors/servo/IMU (real) or just drive around by hand (sim) without the autonomous pipeline.
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
        QDoubleSpinBox, QFormLayout,
    )
    from PyQt5.QtCore import Qt, QTimer, QPointF, pyqtSignal, QThread, QObject
    from PyQt5.QtGui import QFont, QPainter, QColor, QPen, QBrush
    import pyqtgraph as pg
except ImportError as err:
    print(f"[control_gui] Missing: {err}")
    print("Fix: pip install PyQt5 pyqtgraph numpy --break-system-packages")
    sys.exit(1)

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import String, Float64


# ── Design tokens ─────────────────────────────────────────────────────────────
BG, SURF, SURF2, BORDER = "#0b0f1e", "#111827", "#1a2235", "#1e293b"
TEXT, MUTED = "#e2e8f0", "#475569"
EY_C, EPSI_C = "#06d6a0", "#fb923c"
GO_C, STOP_C, WARN_C, ACCENT_C = "#22c55e", "#ef4444", "#fbbf24", "#60a5fa"

STATE_CLR = {
    "FOLLOW": "#60a5fa", "PREPARE": "#fbbf24",
    "OVERTAKE": "#f87171", "RETURN": "#a78bfa",
}

HISTORY = 500
PUBLISH_HZ = 20.0

LAUNCH_FILES = {
    ("sim",  "auto"):   "gazebo.launch.py",
    ("sim",  "manual"): "manual_test_sim.launch.py",
    ("real", "manual"): "manual_test_real.launch.py",
    ("real", "auto"):   "robot.launch.py",
}
SUBMODES = {
    "sim":  [("auto", "Autonomous"), ("manual", "Manual (Joystick)")],
    "real": [("manual", "Manual (Joystick)"), ("auto", "Autonomous")],
}


# ── Stylesheet helpers ─────────────────────────────────────────────────────────
GLOBAL_SS = f"""
QMainWindow, QWidget {{
    background: {BG}; color: {TEXT};
    font-family: 'Inter', 'Segoe UI', 'Noto Sans', sans-serif;
}}
QScrollBar {{ width: 0px; height: 0px; }}
"""


def _lbl(text="", size=12, bold=False, color=None) -> QLabel:
    lb = QLabel(text)
    lb.setStyleSheet(
        f"color:{color or TEXT};font-size:{size}px;"
        f"font-weight:{'600' if bold else '400'};"
        f"background:transparent;border:none;")
    return lb


def _btn(text: str, bg: str, hover: str) -> QPushButton:
    b = QPushButton(text)
    b.setCursor(Qt.PointingHandCursor)
    b.setFixedHeight(52)
    b.setStyleSheet(f"""
        QPushButton {{
            background:{bg}; color:#fff; border:none; border-radius:12px;
            font-size:14px; font-weight:700; letter-spacing:0.6px; padding:0 28px;
        }}
        QPushButton:hover    {{ background:{hover}; }}
        QPushButton:pressed  {{ background:{hover}; opacity:0.85; }}
        QPushButton:disabled {{ background:{BORDER}; color:{MUTED}; }}
    """)
    return b


def _toggle_btn(text: str) -> QPushButton:
    b = QPushButton(text)
    b.setCursor(Qt.PointingHandCursor)
    b.setCheckable(True)
    b.setFixedHeight(40)
    b.setStyleSheet(f"""
        QPushButton {{
            background:{SURF2}; color:{MUTED}; border:1px solid {BORDER};
            border-radius:10px; font-size:12px; font-weight:700;
            letter-spacing:0.4px; padding:0 20px;
        }}
        QPushButton:checked {{ background:{ACCENT_C}; color:#0b0f1e; border-color:{ACCENT_C}; }}
        QPushButton:disabled {{ color:{MUTED}; }}
    """)
    return b


def _card() -> QFrame:
    f = QFrame()
    f.setStyleSheet(f"QFrame {{ background:{SURF}; border:1px solid {BORDER}; border-radius:14px; }}")
    return f


# ── Joystick widget ─────────────────────────────────────────────────────────────
class JoystickWidget(QWidget):
    """Self-centering virtual joystick. Emits normalized (x, y) in [-1, 1],
    x = right(+)/left(-), y = forward(+)/back(-)."""

    moved = pyqtSignal(float, float)

    def __init__(self, diameter=200):
        super().__init__()
        self._d = diameter
        self.setFixedSize(diameter, diameter)
        self._knob = QPointF(0.0, 0.0)
        self._dragging = False

    def _center(self) -> QPointF:
        return QPointF(self._d / 2, self._d / 2)

    def _radius(self) -> float:
        return self._d / 2 - 22

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        c, r = self._center(), self._radius()

        p.setPen(QPen(QColor(BORDER), 2))
        p.setBrush(QBrush(QColor(SURF2)))
        p.drawEllipse(c, r + 14, r + 14)

        p.setPen(QPen(QColor(MUTED), 1, Qt.DashLine))
        p.drawLine(int(c.x() - r - 14), int(c.y()), int(c.x() + r + 14), int(c.y()))
        p.drawLine(int(c.x()), int(c.y() - r - 14), int(c.x()), int(c.y() + r + 14))

        knob_px = QPointF(c.x() + self._knob.x() * r, c.y() - self._knob.y() * r)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(GO_C if self._dragging else "#334155")))
        p.drawEllipse(knob_px, 18, 18)

    def _update_from_mouse(self, pos):
        c, r = self._center(), self._radius()
        dx = (pos.x() - c.x()) / r
        dy = -(pos.y() - c.y()) / r
        mag = (dx * dx + dy * dy) ** 0.5
        if mag > 1.0:
            dx, dy = dx / mag, dy / mag
        self._knob = QPointF(dx, dy)
        self.update()
        self.moved.emit(dx, dy)

    def _reset(self):
        self._dragging = False
        self._knob = QPointF(0.0, 0.0)
        self.update()
        self.moved.emit(0.0, 0.0)

    def mousePressEvent(self, event):
        self._dragging = True
        self._update_from_mouse(event.pos())

    def mouseMoveEvent(self, event):
        if self._dragging:
            self._update_from_mouse(event.pos())

    def mouseReleaseEvent(self, event):
        self._reset()

    def leaveEvent(self, event):
        if self._dragging:
            self._reset()


# ── Waveform chart ─────────────────────────────────────────────────────────────
class WaveChart(QFrame):
    def __init__(self, title: str, color: str, unit: str,
                 y_min=-0.65, y_max=0.65, limit=0.267):
        super().__init__()
        self.setStyleSheet(f"QFrame {{ background:{SURF}; border:1px solid {BORDER}; border-radius:14px; }}")
        self._buf = collections.deque([0.0] * HISTORY, maxlen=HISTORY)
        self._x = list(range(HISTORY))

        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 14, 18, 14)
        layout.setSpacing(8)

        row = QHBoxLayout()
        row.addWidget(_lbl("⬤", 10, color=color))
        row.addSpacing(8)
        row.addWidget(_lbl(title, 13, bold=True))
        row.addStretch()
        self._val = _lbl("—", 14, bold=True, color=color)
        self._val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        row.addWidget(self._val)
        layout.addLayout(row)

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

        self.pw.addItem(pg.InfiniteLine(pos=0, angle=0,
                         pen=pg.mkPen(MUTED, width=1, style=Qt.DashLine)))
        for sign in (+1, -1):
            self.pw.addItem(pg.InfiniteLine(pos=sign * limit, angle=0,
                             pen=pg.mkPen(WARN_C + "55", width=1, style=Qt.DotLine)))

        self._curve = self.pw.plot(
            self._x, list(self._buf), pen=pg.mkPen(color, width=2),
            fillLevel=0, brush=pg.mkBrush(color + "22"))
        self.pw.getViewBox().setBackgroundColor(SURF2)
        self._unit = unit
        layout.addWidget(self.pw)

    def push(self, v: float):
        self._buf.append(v)
        self._curve.setData(self._x, list(self._buf))
        self._val.setText(f"{v:+.3f} {self._unit}")


# ── Error graphs popup window ────────────────────────────────────────────────
class ErrorGraphsWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DVT · Error Graphs")
        self.setStyleSheet(GLOBAL_SS)
        self.resize(640, 560)
        v = QVBoxLayout(self)
        v.setContentsMargins(16, 16, 16, 16)
        v.setSpacing(10)
        self.chart_ey = WaveChart("Lateral Error  e_y", EY_C, "m", -0.65, 0.65, 0.267)
        self.chart_epsi = WaveChart("Heading Error  e_ψ", EPSI_C, "rad", -0.65, 0.65, 0.3)
        v.addWidget(self.chart_ey)
        v.addWidget(self.chart_epsi)


# ── ROS2 bridges ──────────────────────────────────────────────────────────────
class StatusBridge(QObject):
    """Subscribes to autonomous-stack status topics. Harmless to run in Manual mode too —
    it just won't receive anything since lane_control_node/overtake_node aren't launched."""

    sig_ready = pyqtSignal()
    sig_ey = pyqtSignal(float)
    sig_epsi = pyqtSignal(float)
    sig_state = pyqtSignal(str)
    sig_speed = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self._alive = True

    def run(self):
        context = rclpy.Context()
        rclpy.init(context=context)
        node = rclpy.create_node("control_gui_status_bridge", context=context)
        node.create_subscription(
            Vector3, "/status_err",
            lambda m: (self.sig_ey.emit(float(m.x)), self.sig_epsi.emit(float(m.y))), 10)
        node.create_subscription(
            String, "/overtake/state", lambda m: self.sig_state.emit(m.data), 10)
        node.create_subscription(
            Float64, "/overtake/target_speed", lambda m: self.sig_speed.emit(float(m.data)), 10)
        self.sig_ready.emit()

        # Explicit per-bridge executor — rclpy.spin_once(node) without one falls back to a
        # single shared *global* executor, and this app runs two bridges concurrently on two
        # threads, each spinning its own node; sharing that global executor between them is a
        # documented misuse (rclpy.spin_once explicitly warns against multi-threaded use) that
        # starves one bridge's callbacks in favor of the other's.
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        while self._alive:
            try:
                executor.spin_once(timeout_sec=0.04)
            except Exception:
                break
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown(context=context)

    def stop(self):
        self._alive = False


class DriveBridge(QObject):
    """Publishes joystick commands to /cmd_vel. Gated by `_enabled` so it only actually
    publishes while Manual submode is selected — in Autonomous submode lane_control_node is
    the sole /cmd_vel publisher, and this must stay silent to avoid interleaving stray zero
    commands with real driving commands on the same topic."""

    def __init__(self):
        super().__init__()
        self._alive = True
        self._enabled = False
        self._angular_z = 0.0
        self._linear_x = 0.0

    def set_enabled(self, enabled: bool):
        self._enabled = enabled
        if not enabled:
            self._angular_z = 0.0
            self._linear_x = 0.0

    def set_cmd(self, angular_z: float, linear_x: float):
        self._angular_z = angular_z
        self._linear_x = linear_x

    def run(self):
        context = rclpy.Context()
        rclpy.init(context=context)
        node = rclpy.create_node("control_gui_drive_bridge", context=context)
        pub = node.create_publisher(Twist, "/cmd_vel", 10)

        def publish():
            if not self._enabled:
                return
            msg = Twist()
            msg.linear.x = self._linear_x
            msg.angular.z = self._angular_z
            pub.publish(msg)

        node.create_timer(1.0 / PUBLISH_HZ, publish)

        # See the matching comment in StatusBridge.run() — must not share the global executor
        # with the other bridge's thread.
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        while self._alive:
            try:
                executor.spin_once(timeout_sec=0.05)
            except Exception:
                break

        # Safety: always leave the drivetrain commanded to zero on shutdown.
        self._enabled = True
        self._angular_z = 0.0
        self._linear_x = 0.0
        publish()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown(context=context)

    def stop(self):
        self._alive = False


# ── Main window ───────────────────────────────────────────────────────────────
class ControlGui(QMainWindow):
    def __init__(self):
        super().__init__()
        self._proc = None
        self._rqt_proc = None
        self._graphs_win = None
        self._mode = "sim"
        self._submode = "auto"
        self._max_linear = 0.3
        self._max_angular = 0.5
        self._log_lines = collections.deque(maxlen=80)

        self.setWindowTitle("DVT  ·  Control")
        self.setMinimumSize(720, 760)
        self.setStyleSheet(GLOBAL_SS)

        self._build_ui()
        self._build_ros()
        self._refresh_view()

    # ── UI construction ──────────────────────────────────────────────────────
    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        v = QVBoxLayout(root)
        v.setContentsMargins(24, 20, 24, 20)
        v.setSpacing(14)

        v.addLayout(self._header())

        mode_row = QHBoxLayout()
        mode_row.setSpacing(10)
        self._mode_btns = {}
        for key, label in (("sim", "SIMULATION"), ("real", "REAL ROBOT")):
            b = _toggle_btn(label)
            b.clicked.connect(lambda _=False, k=key: self._set_mode(k))
            self._mode_btns[key] = b
            mode_row.addWidget(b)
        v.addLayout(mode_row)

        self._submode_row = QHBoxLayout()
        self._submode_row.setSpacing(10)
        v.addLayout(self._submode_row)
        self._submode_btns = {}

        # Autonomous status card
        self._status_card = self._build_status_card()
        v.addWidget(self._status_card)

        # Manual joystick panel
        self._joystick_panel = self._build_joystick_panel()
        v.addWidget(self._joystick_panel)

        secondary_row = QHBoxLayout()
        secondary_row.setSpacing(12)
        self._graphs_btn = _btn("📈  Error Graphs", "#7c3aed", "#6d28d9")
        self._camera_btn = _btn("🎥  Camera View", "#7c3aed", "#6d28d9")
        self._graphs_btn.clicked.connect(self._on_toggle_graphs)
        self._camera_btn.clicked.connect(self._on_toggle_camera)
        secondary_row.addWidget(self._graphs_btn)
        secondary_row.addWidget(self._camera_btn)
        v.addLayout(secondary_row)

        v.addWidget(self._log_block(), stretch=1)
        v.addLayout(self._buttons())

        self._rebuild_submode_row()

    def _header(self) -> QHBoxLayout:
        row = QHBoxLayout()
        left = QVBoxLayout()
        left.setSpacing(2)
        left.addWidget(_lbl("DVT", 28, bold=True, color=EY_C))
        left.addWidget(_lbl("Autonomous Robot  ·  Control", 13, color=MUTED))
        self._ros_dot = _lbl("⬤", 11, color=MUTED)
        self._ros_txt = _lbl("ROS starting…", 11, color=MUTED)
        right = QHBoxLayout()
        right.setSpacing(6)
        right.addWidget(self._ros_dot)
        right.addWidget(self._ros_txt)
        row.addLayout(left)
        row.addStretch()
        row.addLayout(right)
        return row

    def _build_status_card(self) -> QFrame:
        card = _card()
        lay = QHBoxLayout(card)
        lay.setContentsMargins(24, 16, 24, 16)

        def col(cap, init, color=TEXT):
            w = QWidget()
            w.setStyleSheet("background:transparent;")
            cv = QVBoxLayout(w)
            cv.setContentsMargins(0, 0, 0, 0)
            cv.setSpacing(3)
            cv.addWidget(_lbl(cap, 10, color=MUTED))
            val = _lbl(init, 20, bold=True, color=color)
            cv.addWidget(val)
            return w, val

        w, self._state_lbl = col("STATE", "OFFLINE", MUTED)
        lay.addWidget(w)
        w, self._speed_lbl = col("TARGET SPEED", "0.00 m/s")
        lay.addWidget(w)
        lay.addStretch()
        return card

    def _build_joystick_panel(self) -> QFrame:
        card = _card()
        v = QVBoxLayout(card)
        v.setContentsMargins(20, 16, 20, 16)
        v.setSpacing(12)

        v.addWidget(_lbl("Manual drive — this joystick controls /cmd_vel directly", 12, color=WARN_C))
        self._readout = _lbl("linear: +0.00 m/s   angular: +0.00 rad/s", 13, bold=True)
        v.addWidget(self._readout)

        joy_row = QHBoxLayout()
        joy_row.addStretch()
        self._joystick = JoystickWidget()
        self._joystick.moved.connect(self._on_joystick)
        joy_row.addWidget(self._joystick)
        joy_row.addStretch()
        v.addLayout(joy_row)

        form = QFormLayout()
        self._lin_spin = QDoubleSpinBox()
        self._lin_spin.setRange(0.05, 2.0)
        self._lin_spin.setSingleStep(0.05)
        self._lin_spin.setValue(self._max_linear)
        self._lin_spin.valueChanged.connect(self._on_limits_changed)
        self._ang_spin = QDoubleSpinBox()
        self._ang_spin.setRange(0.05, 2.0)
        self._ang_spin.setSingleStep(0.05)
        self._ang_spin.setValue(self._max_angular)
        self._ang_spin.valueChanged.connect(self._on_limits_changed)
        form.addRow(_lbl("Max linear speed (m/s)", 11, color=MUTED), self._lin_spin)
        form.addRow(_lbl("Max angular speed (rad/s)", 11, color=MUTED), self._ang_spin)
        v.addLayout(form)

        estop = _btn("■   E-STOP", STOP_C, "#dc2626")
        estop.clicked.connect(self._on_estop)
        v.addWidget(estop)
        return card

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
            QPushButton {{ background:{BORDER}; color:{MUTED}; border:none; border-radius:6px; font-size:10px; }}
            QPushButton:hover {{ background:{SURF2}; color:{TEXT}; }}
        """)
        clear_btn.clicked.connect(lambda: self._log_view.clear())
        hdr.addWidget(clear_btn)
        v.addLayout(hdr)
        self._log_view = QTextEdit()
        self._log_view.setReadOnly(True)
        self._log_view.setMaximumHeight(110)
        self._log_view.setStyleSheet(f"""
            QTextEdit {{
                background:{SURF2}; color:{MUTED}; border:1px solid {BORDER}; border-radius:8px;
                font-family:'Consolas','Courier New',monospace; font-size:11px; padding:6px;
            }}
        """)
        v.addWidget(self._log_view)
        return card

    def _log(self, text: str):
        ts = time.strftime("%H:%M:%S")
        self._log_view.append(f"<span style='color:{MUTED}'>{ts}</span>  <span style='color:{TEXT}'>{text}</span>")
        sb = self._log_view.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _buttons(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(12)
        self._launch_btn = _btn("▶   LAUNCH", GO_C, "#16a34a")
        self._stop_btn = _btn("■   STOP", STOP_C, "#dc2626")
        self._stop_btn.setEnabled(False)
        self._launch_btn.clicked.connect(self._on_launch)
        self._stop_btn.clicked.connect(self._on_stop)
        row.addWidget(self._launch_btn, stretch=3)
        row.addWidget(self._stop_btn, stretch=1)
        return row

    # ── Mode / submode switching ─────────────────────────────────────────────
    def _set_mode(self, mode: str):
        if self._proc and self._proc.poll() is None:
            self._mode_btns[self._mode].setChecked(True)
            self._mode_btns["real" if mode == "sim" else "sim"].setChecked(False)
            return
        self._mode = mode
        self._submode = SUBMODES[mode][0][0]
        self._rebuild_submode_row()
        self._refresh_view()

    def _rebuild_submode_row(self):
        while self._submode_row.count():
            item = self._submode_row.takeAt(0)
            w = item.widget()
            if w:
                w.setParent(None)
        self._submode_btns = {}
        for key, label in SUBMODES[self._mode]:
            b = _toggle_btn(label)
            b.clicked.connect(lambda _=False, k=key: self._set_submode(k))
            self._submode_btns[key] = b
            self._submode_row.addWidget(b)

    def _set_submode(self, submode: str):
        if self._proc and self._proc.poll() is None:
            for k, b in self._submode_btns.items():
                b.setChecked(k == self._submode)
            return
        self._submode = submode
        self._refresh_view()

    def _refresh_view(self):
        for key, b in self._mode_btns.items():
            b.setChecked(key == self._mode)
        for key, b in self._submode_btns.items():
            b.setChecked(key == self._submode)
        is_manual = self._submode == "manual"
        self._joystick_panel.setVisible(is_manual)
        self._status_card.setVisible(not is_manual)
        self._drive_bridge.set_enabled(is_manual)
        if not is_manual:
            self._joystick._reset()

    # ── ROS ───────────────────────────────────────────────────────────────────
    def _build_ros(self):
        self._status_bridge = StatusBridge()
        self._status_thread = QThread()
        self._status_bridge.moveToThread(self._status_thread)
        self._status_thread.started.connect(self._status_bridge.run)
        self._status_bridge.sig_ready.connect(self._on_ros_ready)
        self._status_bridge.sig_ey.connect(self._on_ey)
        self._status_bridge.sig_epsi.connect(self._on_epsi)
        self._status_bridge.sig_state.connect(self._on_state)
        self._status_bridge.sig_speed.connect(self._on_speed)
        self._status_thread.start()

        self._drive_bridge = DriveBridge()
        self._drive_thread = QThread()
        self._drive_bridge.moveToThread(self._drive_thread)
        self._drive_thread.started.connect(self._drive_bridge.run)
        self._drive_thread.start()

        self._proc_timer = QTimer(self)
        self._proc_timer.timeout.connect(self._poll_proc)
        self._proc_timer.start(1000)

    def _on_ros_ready(self):
        self._ros_dot.setStyleSheet(f"color:{GO_C};font-size:11px;background:transparent;border:none;")
        self._ros_txt.setStyleSheet(f"color:{GO_C};font-size:11px;background:transparent;border:none;")
        self._ros_txt.setText("ROS ready")

    def _on_ey(self, v: float):
        if self._graphs_win:
            self._graphs_win.chart_ey.push(v)

    def _on_epsi(self, v: float):
        if self._graphs_win:
            self._graphs_win.chart_epsi.push(v)

    _prev_state = ""

    def _on_state(self, s: str):
        color = STATE_CLR.get(s, MUTED)
        self._state_lbl.setText(s)
        self._state_lbl.setStyleSheet(f"color:{color};font-size:20px;font-weight:700;background:transparent;border:none;")
        if s != self._prev_state:
            self._log(f"→ <span style='color:{color};font-weight:600'>{s}</span>")
            self._prev_state = s

    def _on_speed(self, v: float):
        self._speed_lbl.setText(f"{v:.2f} m/s")

    # ── Joystick ──────────────────────────────────────────────────────────────
    def _on_limits_changed(self):
        self._max_linear = self._lin_spin.value()
        self._max_angular = self._ang_spin.value()

    def _on_joystick(self, x: float, y: float):
        angular_z = -x * self._max_angular
        linear_x = y * self._max_linear
        self._drive_bridge.set_cmd(angular_z, linear_x)
        self._readout.setText(f"linear: {linear_x:+.2f} m/s   angular: {angular_z:+.2f} rad/s")

    def _on_estop(self):
        self._joystick._reset()

    # ── Secondary windows ────────────────────────────────────────────────────
    def _on_toggle_graphs(self):
        if self._graphs_win is None:
            self._graphs_win = ErrorGraphsWindow()
            self._graphs_win.destroyed.connect(lambda: setattr(self, "_graphs_win", None))
        if self._graphs_win.isVisible():
            self._graphs_win.hide()
        else:
            self._graphs_win.show()
            self._graphs_win.raise_()

    def _on_toggle_camera(self):
        if self._rqt_proc and self._rqt_proc.poll() is None:
            try:
                self._rqt_proc.terminate()
            except Exception:
                pass
            self._rqt_proc = None
            self._log("Camera view closed")
            return
        try:
            self._rqt_proc = subprocess.Popen(
                ["ros2", "run", "rqt_image_view", "rqt_image_view", "/processed_image"],
                env=os.environ.copy())
        except FileNotFoundError:
            self._log("<span style='color:#f87171'>rqt_image_view not found</span>")
            return
        self._log("Camera view opened  (/processed_image)")

    # ── Launch / Stop ─────────────────────────────────────────────────────────
    def _on_launch(self):
        if self._proc and self._proc.poll() is None:
            return
        launch_file = LAUNCH_FILES[(self._mode, self._submode)]
        env = os.environ.copy()
        try:
            self._proc = subprocess.Popen(
                ["ros2", "launch", "main_bot", launch_file],
                env=env, preexec_fn=os.setsid)
        except FileNotFoundError:
            self._log("<span style='color:#f87171'>ros2 not found — source your workspace first</span>")
            return
        self._launch_btn.setText("⬤   Running…")
        self._launch_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        for b in list(self._mode_btns.values()) + list(self._submode_btns.values()):
            b.setEnabled(False)
        self._log(f"Launched {launch_file}  (PID {self._proc.pid})")

    def _on_stop(self):
        self._joystick._reset()
        if self._proc:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
                self._proc.wait(timeout=6)
            except Exception:
                pass
            self._proc = None
        self._launch_btn.setText("▶   LAUNCH")
        self._launch_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        for b in list(self._mode_btns.values()) + list(self._submode_btns.values()):
            b.setEnabled(True)
        self._state_lbl.setText("OFFLINE")
        self._state_lbl.setStyleSheet(f"color:{MUTED};font-size:20px;font-weight:700;background:transparent;border:none;")
        self._log("Stopped")

    def _poll_proc(self):
        if self._proc and self._proc.poll() is not None:
            self._proc = None
            self._launch_btn.setText("▶   LAUNCH")
            self._launch_btn.setEnabled(True)
            self._stop_btn.setEnabled(False)
            for b in list(self._mode_btns.values()) + list(self._submode_btns.values()):
                b.setEnabled(True)
            self._state_lbl.setText("OFFLINE")
            self._state_lbl.setStyleSheet(f"color:{MUTED};font-size:20px;font-weight:700;background:transparent;border:none;")
            self._log("Process exited")

    # ── Close ─────────────────────────────────────────────────────────────────
    def closeEvent(self, event):
        if self._rqt_proc and self._rqt_proc.poll() is None:
            try:
                self._rqt_proc.terminate()
            except Exception:
                pass
        if self._graphs_win:
            self._graphs_win.close()
        self._on_stop()
        self._status_bridge.stop()
        self._status_thread.quit()
        self._status_thread.wait(3000)
        self._drive_bridge.stop()
        self._drive_thread.quit()
        self._drive_thread.wait(3000)
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("DVT Control")
    win = ControlGui()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
