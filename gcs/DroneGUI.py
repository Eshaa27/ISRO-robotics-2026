#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quadcopter GCS — Ground Control Station
Runs on  : Laptop (Python 3)
Framework: PySide6
Transport: TCP (persistent connection, auto-reconnect)
"""

import sys
import time
import socket
import threading
import datetime
import os

import cv2
from PySide6.QtGui import (QFont, QColor, QTextCursor, QPalette, QImage,
                           QPixmap, QShortcut, QKeySequence)
from PySide6.QtCore import Qt, QTimer, Signal, QObject, QThread

try:
    from VIOdrift_correction import VIOEngine, _open_camera, CAMERA_DEVICE
except ImportError:
    VIOEngine = None
    _open_camera = None
    CAMERA_DEVICE = 0

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QTextEdit, QFrame, QSizePolicy
)

# ─── CONFIG ───────────────────────────────────────────────────────────────────
JETSON_IP        = "192.168.70.104"
JETSON_PORT      = 5005
CONNECT_TIMEOUT  = 5.0      # seconds to wait for ACK_START
KILL_RETRY_MS    = 500      # retry KILL every 500ms until ACK
DISARM_RETRY_MS  = 300      # retry DISARM every 300ms until ACK
WATCHDOG_MS      = 2000     # reconnect attempt interval
PING_INTERVAL_MS = 5000     # CHANGE 4: send PING every 5 seconds
CAMERA_INDEX     = CAMERA_DEVICE
# ──────────────────────────────────────────────────────────────────────────────


# ── SIGNAL BRIDGE ─────────────────────────────────────────────────────────────
class Signals(QObject):
    log         = Signal(str, str)   # message, level
    status      = Signal(str, str)   # text, color
    state       = Signal(str)        # flight state string
    connected   = Signal(bool)       # TCP connection status
    ack_kill    = Signal()
    ack_disarm  = Signal()
    ack_start   = Signal()


class CameraWorker(QObject):
    """Captures laptop frames and runs the shared standalone VIO engine."""
    frame = Signal(QImage)
    drift = Signal(float, float, float, bool)
    log = Signal(str, str)
    finished = Signal()

    def __init__(self, camera_index):
        super(CameraWorker, self).__init__()
        self.camera_index = camera_index
        self.running = True
        self.show_vio = False
        self._display_lock = threading.Lock()

    def set_show_vio(self, enabled):
        with self._display_lock:
            self.show_vio = enabled

    def run(self):
        if VIOEngine is None or _open_camera is None:
            self.log.emit("VIO module could not be loaded.", "ERROR")
            self.finished.emit()
            return

        cap = _open_camera(self.camera_index)
        if cap is None:
            self.log.emit("Could not open laptop camera {}.".format(
                self.camera_index), "ERROR")
            self.finished.emit()
            return

        vio = VIOEngine()
        self.log.emit("Live video started on camera {}.".format(
            self.camera_index), "OK")
        try:
            while self.running:
                ret, frame = cap.read()
                if not ret or frame is None or frame.size == 0:
                    self.log.emit("Camera returned an empty frame.", "WARN")
                    continue

                annotated = vio.process_frame(frame)
                with self._display_lock:
                    display_frame = annotated if self.show_vio else frame
                height, width, channels = display_frame.shape
                image = QImage(display_frame.data, width, height,
                               channels * width, QImage.Format_BGR888).copy()
                dx, dy, dm = vio.get_drift()
                self.frame.emit(image)
                self.drift.emit(float(dx), float(dy), float(dm),
                                vio.is_locked())
        except Exception as exc:
            self.log.emit("Video processing error: {}".format(exc), "ERROR")
        finally:
            cap.release()
            self.finished.emit()

    def stop(self):
        self.running = False


# ── TCP WORKER ────────────────────────────────────────────────────────────────
class TCPWorker(QObject):
    """
    Runs in a QThread. Owns the TCP socket.
    Emits signals to update the GUI thread safely.
    """
    def __init__(self, signals):
        super(TCPWorker, self).__init__()
        self.signals     = signals
        self.sock        = None
        self.running     = True
        self.connected   = False
        self._send_queue = []
        self._lock       = threading.Lock()

    def run(self):
        while self.running:
            if not self.connected:
                self._try_connect()
            else:
                self._recv_loop()
            time.sleep(0.5)

    def _try_connect(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1.0)
            s.connect((JETSON_IP, JETSON_PORT))
            s.settimeout(1.0)
            self.sock      = s
            self.connected = True
            self.signals.connected.emit(True)
            self.signals.log.emit("TCP connected to {}:{}.".format(
                JETSON_IP, JETSON_PORT), "OK")
        except Exception as e:
            self.connected = False
            self.signals.connected.emit(False)

    def _recv_loop(self):
        try:
            data = self.sock.recv(1024)
            if not data:
                raise Exception("Connection closed by Jetson.")
            msgs = data.decode("utf-8", errors="ignore").strip().split("\n")
            for msg in msgs:
                msg = msg.strip()
                if not msg:
                    continue
                self._dispatch(msg)
        except socket.timeout:
            # Send anything queued
            with self._lock:
                queue = list(self._send_queue)
                self._send_queue = []
            for m in queue:
                self._raw_send(m)
        except Exception as e:
            self.signals.log.emit("Connection lost: {}".format(e), "ERROR")
            self._disconnect()

    def _dispatch(self, msg):
        self.signals.log.emit("[Jetson] {}".format(msg), "DRONE")
        if msg == "ACK_START":
            self.signals.ack_start.emit()
        elif msg == "ACK_KILL":
            self.signals.ack_kill.emit()
        elif msg == "ACK_DISARM":
            self.signals.ack_disarm.emit()
        elif msg.startswith("STATE:"):
            self.signals.state.emit(msg.split(":", 1)[1])
        elif msg.startswith("LOG:"):
            self.signals.log.emit(msg.split(":", 1)[1], "DRONE")
        elif msg == "CONN_FAIL":
            self.signals.log.emit("Command rejected by Jetson.", "ERROR")

    def send(self, message):
        with self._lock:
            self._send_queue.append(message)
        if self.connected:
            with self._lock:
                queue = list(self._send_queue)
                self._send_queue = []
            for m in queue:
                self._raw_send(m)

    def _raw_send(self, message):
        try:
            self.sock.sendall((message + "\n").encode("utf-8"))
        except Exception as e:
            self.signals.log.emit("Send failed: {}".format(e), "ERROR")
            self._disconnect()

    def _disconnect(self):
        self.connected = False
        self.signals.connected.emit(False)
        try:
            self.sock.close()
        except Exception:
            pass
        self.sock = None

    def stop(self):
        self.running = False
        self._disconnect()


# ── MAIN WINDOW ───────────────────────────────────────────────────────────────
class GCSWindow(QMainWindow):

    def __init__(self):
        super(GCSWindow, self).__init__()
        self.setWindowTitle("Quadcopter GCS")
        self.setMinimumSize(960, 680)
        self.setStyleSheet(STYLESHEET)

        self.signals          = Signals()
        self._kill_timer      = QTimer()
        self._disarm_timer    = QTimer()
        self._start_timeout   = QTimer()
        self._ping_timer      = QTimer()          # CHANGE 4: ping timer
        self._pending_start   = False
        self._kill_acked      = False
        self._disarm_acked    = False
        self._tcp_connected   = False
        self._camera_thread   = None
        self._camera_worker   = None
        self._video_active    = False
        self._drift_visible   = False

        self._build_ui()
        self._connect_signals()
        self._start_worker()
        self._quit_shortcut = QShortcut(QKeySequence("Q"), self)
        self._quit_shortcut.setContext(Qt.WindowShortcut)
        self._quit_shortcut.activated.connect(self._close_from_keyboard)
        self.showFullScreen()

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(12)

        # ── Header ────────────────────────────────────────────────────────────
        header = QVBoxLayout()
        header.setSpacing(2)

        title = QLabel("QUADCOPTER GCS")
        title.setObjectName("title")
        title.setAlignment(Qt.AlignCenter)
        header.addWidget(title)

        sub = QLabel("GPS-Denied Autonomous Flight Controller")
        sub.setObjectName("subtitle")
        sub.setAlignment(Qt.AlignCenter)
        header.addWidget(sub)
        root.addLayout(header)

        # ── Live video and VIO panel ─────────────────────────────────────────
        video_card = QFrame()
        video_card.setObjectName("video_card")
        video_layout = QVBoxLayout(video_card)
        video_layout.setContentsMargins(12, 12, 12, 12)
        video_header = QHBoxLayout()
        video_title = QLabel("LIVE CAMERA / VIO")
        video_title.setObjectName("section_title")
        video_header.addWidget(video_title)
        video_header.addStretch()
        self.video_status = QLabel("OFFLINE")
        self.video_status.setObjectName("video_off")
        video_header.addWidget(self.video_status)
        video_layout.addLayout(video_header)

        self.video_label = QLabel("Live video is stopped")
        self.video_label.setObjectName("video_view")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(480, 270)
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        video_layout.addWidget(self.video_label, 1)

        self.drift_label = QLabel("VIO drift correction: hidden")
        self.drift_label.setObjectName("drift_label")
        self.drift_label.setAlignment(Qt.AlignCenter)
        self.drift_label.setVisible(False)
        video_layout.addWidget(self.drift_label)

        video_buttons = QHBoxLayout()
        self.video_btn = QPushButton("●   START LIVE VIDEO")
        self.video_btn.setObjectName("btn_video")
        self.video_btn.setFixedHeight(42)
        self.video_btn.clicked.connect(self._toggle_video)
        video_buttons.addWidget(self.video_btn)

        self.drift_btn = QPushButton("＋   SHOW VIO DRIFT")
        self.drift_btn.setObjectName("btn_drift")
        self.drift_btn.setFixedHeight(42)
        self.drift_btn.clicked.connect(self._toggle_drift)
        video_buttons.addWidget(self.drift_btn)
        video_layout.addLayout(video_buttons)
        root.addWidget(video_card, 2)

        # ── Status bar ────────────────────────────────────────────────────────
        status_card = QFrame()
        status_card.setObjectName("card")
        status_layout = QHBoxLayout(status_card)
        status_layout.setContentsMargins(16, 10, 16, 10)

        # Connection dot + label
        self.conn_dot = QLabel("●")
        self.conn_dot.setObjectName("dot_red")
        status_layout.addWidget(self.conn_dot)

        self.conn_label = QLabel("Not connected")
        self.conn_label.setObjectName("status_red")
        status_layout.addWidget(self.conn_label)

        status_layout.addStretch()

        state_lbl = QLabel("State:")
        state_lbl.setObjectName("muted")
        status_layout.addWidget(state_lbl)

        self.state_label = QLabel("IDLE")
        self.state_label.setObjectName("state_idle")
        status_layout.addWidget(self.state_label)

        root.addWidget(status_card)

        # ── TCP status ────────────────────────────────────────────────────────
        tcp_card = QFrame()
        tcp_card.setObjectName("card")
        tcp_layout = QHBoxLayout(tcp_card)
        tcp_layout.setContentsMargins(16, 8, 16, 8)

        self.tcp_dot = QLabel("●")
        self.tcp_dot.setObjectName("dot_red")
        tcp_layout.addWidget(self.tcp_dot)

        self.tcp_label = QLabel("TCP: Connecting to {}:{}...".format(
            JETSON_IP, JETSON_PORT))
        self.tcp_label.setObjectName("muted")
        tcp_layout.addWidget(self.tcp_label)
        tcp_layout.addStretch()
        root.addWidget(tcp_card)

        # ── Buttons row 1: START + KILL ───────────────────────────────────────
        btn_row1 = QHBoxLayout()
        btn_row1.setSpacing(12)

        self.start_btn = QPushButton("▶   START")
        self.start_btn.setObjectName("btn_start")
        self.start_btn.setFixedHeight(56)
        self.start_btn.clicked.connect(self._on_start)
        btn_row1.addWidget(self.start_btn)

        self.kill_btn = QPushButton("✕   FORCE KILL / FAILSAFE")
        self.kill_btn.setObjectName("btn_kill")
        self.kill_btn.setFixedHeight(56)
        self.kill_btn.clicked.connect(self._on_kill)
        btn_row1.addWidget(self.kill_btn)

        root.addLayout(btn_row1)

        # ── Button row 2: DISARM ──────────────────────────────────────────────
        self.disarm_btn = QPushButton("⚡   DISARM  (Hardware Emergency Stop)")
        self.disarm_btn.setObjectName("btn_disarm")
        self.disarm_btn.setFixedHeight(44)
        self.disarm_btn.clicked.connect(self._on_disarm)
        root.addWidget(self.disarm_btn)

        # ── Log panel ─────────────────────────────────────────────────────────
        log_header = QHBoxLayout()
        log_title = QLabel("Flight Log")
        log_title.setObjectName("log_title")
        log_header.addWidget(log_title)
        log_header.addStretch()

        clear_btn = QPushButton("Clear")
        clear_btn.setObjectName("btn_clear")
        clear_btn.setFixedHeight(24)
        clear_btn.clicked.connect(self._clear_log)
        log_header.addWidget(clear_btn)
        root.addLayout(log_header)

        self.log_panel = QTextEdit()
        self.log_panel.setObjectName("log_panel")
        self.log_panel.setReadOnly(True)
        self.log_panel.setFont(QFont("Courier New", 9))
        root.addWidget(self.log_panel)

        self._log("GCS initialised. Establishing TCP connection...", "INFO")

    # ── SIGNALS ───────────────────────────────────────────────────────────────

    def _connect_signals(self):
        self.signals.log.connect(self._log)
        self.signals.status.connect(self._set_status)
        self.signals.state.connect(self._update_state)
        self.signals.connected.connect(self._on_tcp_status)
        self.signals.ack_start.connect(self._on_ack_start)
        self.signals.ack_kill.connect(self._on_ack_kill)
        self.signals.ack_disarm.connect(self._on_ack_disarm)

        self._kill_timer.timeout.connect(self._retry_kill)
        self._disarm_timer.timeout.connect(self._retry_disarm)
        self._start_timeout.setSingleShot(True)
        self._start_timeout.timeout.connect(self._on_start_timeout)
        self._ping_timer.setInterval(PING_INTERVAL_MS)          # CHANGE 4: configure ping timer
        self._ping_timer.timeout.connect(self._send_ping)        # CHANGE 4: connect ping slot

    # ── LOCAL VIDEO / VIO ────────────────────────────────────────────────────

    def _toggle_video(self):
        if self._video_active:
            self._stop_video()
        else:
            self._start_video()

    def _start_video(self):
        if self._video_active:
            return
        self._camera_thread = QThread()
        self._camera_worker = CameraWorker(CAMERA_INDEX)
        self._camera_worker.moveToThread(self._camera_thread)
        self._camera_thread.started.connect(self._camera_worker.run)
        self._camera_worker.frame.connect(self._show_video_frame)
        self._camera_worker.drift.connect(self._show_drift)
        self._camera_worker.log.connect(self._log)
        self._camera_worker.finished.connect(self._camera_thread.quit)
        self._camera_worker.finished.connect(self._camera_worker.deleteLater)
        self._camera_thread.finished.connect(self._camera_thread.deleteLater)
        self._camera_thread.finished.connect(self._video_thread_finished)
        self._video_active = True
        self.video_btn.setText("■   STOP LIVE VIDEO")
        self.video_status.setText("STARTING")
        self.video_status.setObjectName("video_starting")
        self.video_status.setStyleSheet("color: #fab387; font-weight: bold;")
        self._log("Starting laptop camera and VIO engine.", "INFO")
        self._camera_thread.start()

    def _stop_video(self):
        if self._camera_worker is not None:
            self._camera_worker.stop()
        self._log("Stopping live video.", "INFO")

    def _video_thread_finished(self):
        self._video_active = False
        self._camera_worker = None
        self._camera_thread = None
        self.video_btn.setText("●   START LIVE VIDEO")
        self.video_status.setText("OFFLINE")
        self.video_status.setStyleSheet("color: #f38ba8; font-weight: bold;")
        self.video_label.setText("Live video is stopped")

    def _show_video_frame(self, image):
        self.video_label.setPixmap(QPixmap.fromImage(image).scaled(
            self.video_label.size(), Qt.KeepAspectRatio,
            Qt.SmoothTransformation))
        self.video_status.setText("LIVE")
        self.video_status.setStyleSheet("color: #a6e3a1; font-weight: bold;")

    def _toggle_drift(self):
        self._drift_visible = not self._drift_visible
        self.drift_label.setVisible(self._drift_visible)
        if self._camera_worker is not None:
            self._camera_worker.set_show_vio(self._drift_visible)
        if self._drift_visible:
            self.drift_btn.setText("−   HIDE VIO DRIFT")
            self._log("VIO drift correction display enabled.", "INFO")
        else:
            self.drift_btn.setText("＋   SHOW VIO DRIFT")
            self._log("VIO drift correction display hidden.", "INFO")

    def _show_drift(self, dx, dy, distance, locked):
        if not self._drift_visible:
            return
        state = "LOCKED" if locked else "SEARCHING"
        self.drift_label.setText(
            "VIO DRIFT  |  {}  |  X {:7.1f} px   Y {:7.1f} px   {:0.4f} m".format(
                state, dx, dy, distance))

    # ── TCP WORKER ────────────────────────────────────────────────────────────

    def _start_worker(self):
        self.worker_thread = QThread()
        self.worker        = TCPWorker(self.signals)
        self.worker.moveToThread(self.worker_thread)
        self.worker_thread.started.connect(self.worker.run)
        self.worker_thread.start()

    # ── PING ──────────────────────────────────────────────────────────────────

    def _send_ping(self):                          # CHANGE 4: ping method
        self.worker.send("PING")

    # ── COMMAND HANDLERS ──────────────────────────────────────────────────────

    def _on_start(self):
        if not self._tcp_connected:
            self._log("Not connected to Jetson. Cannot send START.", "ERROR")
            return
        self._log("Sending START command...", "INFO")
        self._set_status("Waiting for ACK...", "#fab387")
        self.worker.send("START")
        self._pending_start = True
        self._start_timeout.start(int(CONNECT_TIMEOUT * 1000))

    def _on_ack_start(self):
        if not self._pending_start:
            return
        self._pending_start = False
        self._start_timeout.stop()
        self._log("ACK_START received. Mission started.", "OK")
        self._set_status("Connected", "#a6e3a1")
        self._update_state("TAKEOFF")
        self.start_btn.setEnabled(False)

    def _on_start_timeout(self):
        if self._pending_start:
            self._pending_start = False
            self._log("No ACK from Jetson within {}s. Connection failed.".format(
                int(CONNECT_TIMEOUT)), "ERROR")
            self._set_status("Not connected", "#f38ba8")

    def _on_kill(self):
        self._log("FORCE KILL pressed: stopping local video and entering failsafe.", "ERROR")
        self._stop_video()
        self._kill_acked = False
        self._disarm_acked = False
        self._disarm_timer.stop()
        self._update_state("KILL / FAILSAFE")
        self._set_status("FAILSAFE ACTIVE", "#f38ba8")
        self.start_btn.setEnabled(False)
        self.video_btn.setEnabled(False)
        self.drift_btn.setEnabled(False)
        self.kill_btn.setEnabled(False)
        self.disarm_btn.setEnabled(False)
        self._log("Sending KILL and DISARM to Jetson immediately.", "WARN")
        self.worker.send("KILL")
        self.worker.send("DISARM")
        self._kill_timer.start(KILL_RETRY_MS)
        self._disarm_timer.start(DISARM_RETRY_MS)

    def _retry_kill(self):
        if not self._kill_acked:
            self._log("Retrying remote KILL failsafe.", "WARN")
            self.worker.send("KILL")

    def _on_ack_kill(self):
        self._kill_acked = True
        self._kill_timer.stop()
        self._log("KILL acknowledged by Jetson; flight processes should be stopped.", "OK")
        self._update_state("KILL / FAILSAFE")
        self._set_status("KILLED", "#f38ba8")

    def _on_disarm(self):
        self._log("DISARM command issued. Highest priority. Retrying every {}ms.".format(
            DISARM_RETRY_MS), "WARN")
        self._disarm_acked = False
        self._kill_timer.stop()       # Stop kill retries — disarm takes over
        self._update_state("DISARMED")
        self._set_status("DISARM sent", "#f38ba8")
        self.start_btn.setEnabled(False)
        self.kill_btn.setEnabled(False)
        self.worker.send("DISARM")
        self._disarm_timer.start(DISARM_RETRY_MS)

    def _retry_disarm(self):
        if not self._disarm_acked:
            self._log("Retrying hardware DISARM.", "WARN")
            self.worker.send("DISARM")

    def _on_ack_disarm(self):
        self._disarm_acked = True
        self._disarm_timer.stop()
        self._log("DISARM acknowledged. Motors stopped.", "OK")
        self._update_state("DISARMED")
        self._set_status("DISARMED", "#f38ba8")

    # ── TCP STATUS ────────────────────────────────────────────────────────────

    def _on_tcp_status(self, connected):
        self._tcp_connected = connected
        if connected:
            self.tcp_dot.setObjectName("dot_green")
            self.tcp_dot.setStyleSheet("color: #a6e3a1; font-size: 14px;")
            self.tcp_label.setText("TCP: Connected to {}:{}".format(
                JETSON_IP, JETSON_PORT))
            self.tcp_label.setStyleSheet("color: #a6e3a1;")
            self._ping_timer.start()                             # CHANGE 5: start ping on connect
        else:
            self.tcp_dot.setObjectName("dot_red")
            self.tcp_dot.setStyleSheet("color: #f38ba8; font-size: 14px;")
            self.tcp_label.setText("TCP: Reconnecting to {}:{}...".format(
                JETSON_IP, JETSON_PORT))
            self.tcp_label.setStyleSheet("color: #6c7086;")
            self._ping_timer.stop()                              # CHANGE 6: stop ping on disconnect
            # Trigger failsafe if was connected
            if self._pending_start is False and self.start_btn.isEnabled() is False:
                self._log("TCP connection lost. Failsafe triggered.", "ERROR")
                self._update_state("KILL / FAILSAFE")

    # ── UI HELPERS ────────────────────────────────────────────────────────────

    def _log(self, text, level="INFO"):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        colors = {
            "INFO":  "#89b4fa",
            "OK":    "#a6e3a1",
            "WARN":  "#fab387",
            "ERROR": "#f38ba8",
            "DRONE": "#cba6f7",
        }
        color = colors.get(level, "#cdd6f4")
        time_html  = '<span style="color:#585b70;">[{}]</span> '.format(ts)
        level_html = '<span style="color:#444459;">[{}]</span> '.format(level)
        msg_html   = '<span style="color:{};">{}</span>'.format(color, text)
        self.log_panel.append(time_html + level_html + msg_html)
        self.log_panel.moveCursor(QTextCursor.End)

    def _clear_log(self):
        self.log_panel.clear()
        self._log("Log cleared.", "INFO")

    def _set_status(self, text, color):
        self.conn_label.setText(text)
        self.conn_label.setStyleSheet("color: {}; font-weight: bold;".format(color))
        self.conn_dot.setStyleSheet("color: {}; font-size: 14px;".format(color))

    def _update_state(self, state):
        colors = {
            "IDLE":              "#6c7086",
            "TAKEOFF":           "#89dceb",
            "HOVER":             "#a6e3a1",
            "LANDING":           "#fab387",
            "LANDED":            "#a6e3a1",
            "MOTORS RUNNING":    "#89dceb",
            "KILL / FAILSAFE":   "#f38ba8",
            "DISARMED":          "#f38ba8",
            "DISARMED / LANDED": "#6c7086",
            "ARM_FAILED":        "#f38ba8",
        }
        color = colors.get(state, "#cdd6f4")
        self.state_label.setText(state)
        self.state_label.setStyleSheet(
            "color: {}; font-weight: bold; font-size: 11px;".format(color))

    def closeEvent(self, event):
        self._log("Closing GCS: stopping video and requesting failsafe.", "WARN")
        self._stop_video()
        if self._tcp_connected:
            self.worker.send("KILL")
            self.worker.send("DISARM")
        self.worker.stop()
        self.worker_thread.quit()
        self.worker_thread.wait(3000)
        if self._camera_thread is not None:
            self._camera_thread.quit()
            self._camera_thread.wait(2000)
        event.accept()

    def _close_from_keyboard(self):
        self._log("Q pressed. Closing GCS safely.", "WARN")
        self.close()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Q:
            self._close_from_keyboard()
            return
        super(GCSWindow, self).keyPressEvent(event)


# ── STYLESHEET ────────────────────────────────────────────────────────────────
STYLESHEET = """
QMainWindow, QWidget {
    background-color: #0f0f1a;
    color: #cdd6f4;
}

QLabel#title {
    font-family: 'Courier New', monospace;
    font-size: 22px;
    font-weight: bold;
    color: #cdd6f4;
    letter-spacing: 6px;
}

QLabel#subtitle {
    font-size: 10px;
    color: #45475a;
    letter-spacing: 2px;
}

QFrame#card {
    background-color: #1e1e2e;
    border: 1px solid #313244;
    border-radius: 8px;
}

QFrame#video_card {
    background-color: #141421;
    border: 1px solid #313244;
    border-radius: 10px;
}

QLabel#section_title {
    color: #cdd6f4;
    font-size: 12px;
    font-weight: bold;
    letter-spacing: 2px;
}

QLabel#video_view {
    background-color: #08080e;
    border: 1px solid #313244;
    border-radius: 6px;
    color: #585b70;
    font-size: 16px;
}

QLabel#video_off { color: #f38ba8; font-weight: bold; }
QLabel#video_starting { color: #fab387; font-weight: bold; }
QLabel#drift_label {
    background-color: #1e1e2e;
    color: #89dceb;
    border: 1px solid #313244;
    border-radius: 5px;
    padding: 7px;
    font-family: 'Courier New', monospace;
    font-size: 11px;
}

QLabel#dot_red   { color: #f38ba8; font-size: 14px; }
QLabel#dot_green { color: #a6e3a1; font-size: 14px; }
QLabel#status_red   { color: #f38ba8; font-weight: bold; font-size: 11px; }
QLabel#status_green { color: #a6e3a1; font-weight: bold; font-size: 11px; }
QLabel#state_idle { color: #6c7086; font-weight: bold; font-size: 11px; }
QLabel#muted { color: #45475a; font-size: 10px; }
QLabel#log_title { color: #cdd6f4; font-weight: bold; font-size: 11px; }

QPushButton#btn_start {
    background-color: #1e3a2e;
    color: #a6e3a1;
    border: 1px solid #a6e3a1;
    border-radius: 8px;
    font-size: 15px;
    font-weight: bold;
    letter-spacing: 2px;
}
QPushButton#btn_start:hover {
    background-color: #2a4a3e;
}
QPushButton#btn_start:pressed {
    background-color: #a6e3a1;
    color: #0f0f1a;
}
QPushButton#btn_start:disabled {
    background-color: #1a1a2e;
    color: #313244;
    border-color: #313244;
}

QPushButton#btn_kill {
    background-color: #8f1d35;
    color: #ffffff;
    border: 2px solid #ff6b87;
    border-radius: 8px;
    font-size: 13px;
    font-weight: bold;
    letter-spacing: 2px;
}
QPushButton#btn_kill:hover {
    background-color: #b32645;
}
QPushButton#btn_kill:pressed {
    background-color: #ff6b87;
    color: #0f0f1a;
}
QPushButton#btn_kill:disabled {
    background-color: #451522;
    color: #9b5264;
    border-color: #6d3041;
}

QPushButton#btn_video, QPushButton#btn_drift {
    background-color: #20243a;
    color: #89dceb;
    border: 1px solid #89dceb;
    border-radius: 6px;
    font-size: 11px;
    font-weight: bold;
    letter-spacing: 1px;
}
QPushButton#btn_video:hover, QPushButton#btn_drift:hover {
    background-color: #29304b;
}
QPushButton#btn_video:disabled, QPushButton#btn_drift:disabled {
    background-color: #1a1a2e;
    color: #45475a;
    border-color: #313244;
}

QPushButton#btn_disarm {
    background-color: #2e1e0a;
    color: #fab387;
    border: 1px solid #fab387;
    border-radius: 6px;
    font-size: 12px;
    font-weight: bold;
    letter-spacing: 1px;
}
QPushButton#btn_disarm:hover {
    background-color: #3e2e1a;
}
QPushButton#btn_disarm:pressed {
    background-color: #fab387;
    color: #0f0f1a;
}
QPushButton#btn_disarm:disabled {
    background-color: #1a1a2e;
    color: #313244;
    border-color: #313244;
}

QPushButton#btn_clear {
    background-color: transparent;
    color: #45475a;
    border: none;
    font-size: 10px;
}
QPushButton#btn_clear:hover { color: #cdd6f4; }

QTextEdit#log_panel {
    background-color: #090912;
    color: #cdd6f4;
    border: 1px solid #1e1e2e;
    border-radius: 8px;
    padding: 8px;
    font-family: 'Courier New', monospace;
    font-size: 10px;
}

QScrollBar:vertical {
    background: #1e1e2e;
    width: 6px;
    border-radius: 3px;
}
QScrollBar::handle:vertical {
    background: #313244;
    border-radius: 3px;
}
"""

# ── MAIN ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = GCSWindow()
    window.show()
    sys.exit(app.exec())