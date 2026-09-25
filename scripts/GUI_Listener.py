#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
GCS Command Listener — Production Grade
Runs on  : Jetson Nano (ROS1 Melodic)
Python   : 2.7
Transport: TCP (persistent connection)

Safety features:
  1. Watchdog — triggers KILL if no command received for > 2 seconds
  2. Dual kill — software KILL (ROS + processes) + hardware DISARM (MAVLink direct)
  3. KILL retried by GUI until ACK — guaranteed delivery
  4. DISARM bypasses ROS entirely — direct MAVLink to Pixhawk
  5. TCP disconnect triggers automatic failsafe
"""

import socket
import threading
import subprocess
import signal
import sys
import os
import time
import datetime

# ── ROS ───────────────────────────────────────────────────────────────────────
try:
    import rospy
    from std_msgs.msg import String, Bool
    ROS_AVAILABLE = True
except ImportError:
    print("[WARN] ROS not found. Running standalone.")
    ROS_AVAILABLE = False

# ── pymavlink (for direct DISARM) ─────────────────────────────────────────────
try:
    from pymavlink import mavutil
    MAV_AVAILABLE = True
except ImportError:
    print("[WARN] pymavlink not found. Hardware DISARM unavailable.")
    MAV_AVAILABLE = False

# ─── CONFIG ───────────────────────────────────────────────────────────────────
LISTEN_HOST          = "0.0.0.0"
LISTEN_PORT          = 5005
ALLOWED_CMDS         = {"START", "KILL", "DISARM", "PING"}   # CHANGE 1: added PING
WATCHDOG_TIMEOUT     = 12.0      # CHANGE 2: was 2.0 — now 2× the 5s ping interval

PIXHAWK_PORT         = "/dev/ttyACM0"
PIXHAWK_BAUD         = 115200

TOPIC_FLIGHT_CMD     = "/gcs/flight_command"
TOPIC_KILL_FLAG      = "/gcs/kill_flag"

FLIGHT_MANAGER_CMD   = ["python2", "/home/jetson/motor.py"]
# ──────────────────────────────────────────────────────────────────────────────


def timestamp():
    return datetime.datetime.now().strftime("%H:%M:%S")


def log(msg, level="INFO"):
    print("[{}] [{}] {}".format(timestamp(), level, msg))
    sys.stdout.flush()


# ── STATE MACHINE ─────────────────────────────────────────────────────────────
STATES = ["IDLE", "TAKEOFF", "HOVER", "LANDING", "LANDED", "FAILSAFE", "DISARMED"]


class GCSCommandListener(object):

    def __init__(self):
        self.client_sock      = None
        self.client_addr      = None
        self.server_sock      = None
        self.flight_proc      = None
        self.mission_active   = False
        self.kill_in_progress = False
        self.current_state    = "IDLE"
        self.shutdown_event   = threading.Event()
        self.last_cmd_time    = time.time()
        self.client_connected = False
        self._client_lock     = threading.Lock()
        self._safety_lock     = threading.Lock()
        self._flight_lock     = threading.Lock()

        # ── ROS init ──────────────────────────────────────────────────────────
        if ROS_AVAILABLE:
            rospy.init_node("gcs_command_listener", anonymous=False,
                            log_level=rospy.INFO)
            self.flight_pub = rospy.Publisher(TOPIC_FLIGHT_CMD, String,
                                              queue_size=1, latch=True)
            self.kill_pub   = rospy.Publisher(TOPIC_KILL_FLAG, Bool,
                                              queue_size=1, latch=True)
            log("ROS node ready. Topics: {} {}".format(
                TOPIC_FLIGHT_CMD, TOPIC_KILL_FLAG))
        else:
            self.flight_pub = None
            self.kill_pub   = None

        signal.signal(signal.SIGINT,  self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    # ── TCP SERVER ────────────────────────────────────────────────────────────

    def start(self):
        try:
            self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_sock.bind((LISTEN_HOST, LISTEN_PORT))
            self.server_sock.listen(1)
            self.server_sock.settimeout(1.0)
            log("TCP server listening on port {}.".format(LISTEN_PORT))
        except Exception as e:
            log("Server bind failed: {}".format(e), "ERROR")
            sys.exit(1)

        # Start watchdog thread
        wdog = threading.Thread(target=self._watchdog_loop)
        wdog.daemon = True
        wdog.start()

        self._accept_loop()

    def _accept_loop(self):
        log("Waiting for GUI connection...")
        while not self.shutdown_event.is_set():
            try:
                conn, addr = self.server_sock.accept()
                self.client_sock = conn
                self.client_addr = addr
                self.client_connected = True
                self.client_sock.settimeout(1.0)
                self.last_cmd_time = time.time()
                log("GUI connected from {}:{}.".format(addr[0], addr[1]))
                self._client_loop()
                self.client_connected = False
                log("GUI disconnected. Triggering failsafe.")
                self._trigger_failsafe_on_disconnect()
            except socket.timeout:
                continue
            except Exception as e:
                if not self.shutdown_event.is_set():
                    log("Accept error: {}".format(e), "ERROR")

        self._cleanup()

    def _client_loop(self):
        buf = ""
        while not self.shutdown_event.is_set():
            try:
                data = self.client_sock.recv(256)
                if not data:
                    break
                buf += data.decode("utf-8", errors="ignore")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip().upper()
                    if not line:
                        continue
                    self.last_cmd_time = time.time()
                    self._dispatch(line)
            except socket.timeout:
                continue
            except Exception as e:
                log("Client recv error: {}".format(e), "ERROR")
                break

    # ── DISPATCH ──────────────────────────────────────────────────────────────

    def _dispatch(self, cmd):
        log("Received: '{}'".format(cmd))
        if cmd not in ALLOWED_CMDS:
            log("Rejected unknown command: '{}'.".format(cmd), "WARN")
            self._send("CONN_FAIL")
            return

        if cmd == "START":
            self._handle_start()
        elif cmd == "KILL":
            t = threading.Thread(target=self._handle_kill)
            t.daemon = True
            t.start()
        elif cmd == "DISARM":
            t = threading.Thread(target=self._handle_disarm)
            t.daemon = True
            t.start()
        elif cmd == "PING":                        # CHANGE 3: handle PING
            pass                                   # keepalive only — last_cmd_time already reset above

    # ── COMMAND: START ────────────────────────────────────────────────────────

    def _handle_start(self):
        if self.mission_active or self.kill_in_progress:
            log("Mission already active. Ignoring START.", "WARN")
            self._send("LOG:Mission already running")
            return

        log("START validated. Launching flight manager...")
        if not self._launch_flight_manager():
            self._send("CONN_FAIL")
            self._send("LOG:Flight manager failed to launch.")
            self._set_state("FAILSAFE")
            return

        if ROS_AVAILABLE:
            try:
                self.flight_pub.publish(String(data="START"))
            except Exception as e:
                log("START ROS publish failed: {}".format(e), "ERROR")
                self._terminate_flight_manager()
                self._send("CONN_FAIL")
                self._send("LOG:START rejected because ROS publish failed.")
                self._set_state("FAILSAFE")
                return

        self.mission_active = True
        self._send("ACK_START")
        self._set_state("TAKEOFF")

    # ── COMMAND: KILL (Software) ───────────────────────────────────────────────

    def _handle_kill(self):
        with self._safety_lock:
            if self.kill_in_progress:
                self._send("ACK_KILL")
                return

            self.kill_in_progress = True
            self.mission_active   = False

            log("KILL sequence started.", "WARN")
            self._send("ACK_KILL")
            self._set_state("KILL / FAILSAFE")

            if ROS_AVAILABLE:
                try:
                    self.kill_pub.publish(Bool(data=True))
                    self.flight_pub.publish(String(data="KILL"))
                except Exception as e:
                    log("ROS publish error: {}".format(e), "WARN")

            self._terminate_flight_manager()

            for node in ["flight_manager", "vio_node", "lidar_node", "mavros"]:
                self._kill_ros_node(node)

            self._send("LOG:All flight processes and ROS nodes terminated. Failsafe active.")
            log("KILL sequence complete.")

    # ── COMMAND: DISARM (Hardware — highest priority) ─────────────────────────

    def _handle_disarm(self):
        """
        Bypasses ROS entirely.
        Sends MAVLink MAV_CMD_COMPONENT_ARM_DISARM directly to Pixhawk.
        This is the last line of defence — must work even if ROS is dead.
        """
        with self._safety_lock:
            self._handle_disarm_locked()

    def _handle_disarm_locked(self):
        log("DISARM command received. Sending direct MAVLink disarm.", "WARN")

        # Kill everything first
        self.mission_active = False
        self._terminate_flight_manager()

        if ROS_AVAILABLE:
            try:
                self.kill_pub.publish(Bool(data=True))
            except Exception:
                pass

        # Direct MAVLink disarm
        disarm_success = False
        if MAV_AVAILABLE:
            disarm_success = self._mavlink_disarm()
        else:
            log("pymavlink not available. Cannot send hardware disarm.", "ERROR")

        self._send("ACK_DISARM")
        self._set_state("DISARMED")

        if disarm_success:
            self._send("LOG:Hardware DISARM sent to Pixhawk successfully.")
            log("Hardware DISARM complete.")
        else:
            self._send("LOG:WARNING — MAVLink disarm may have failed. Check Pixhawk.")
            log("MAVLink disarm failed or unavailable.", "ERROR")

    def _mavlink_disarm(self):
        """Direct MAVLink connection to Pixhawk — independent of ROS/MAVROS."""
        mav = None
        try:
            log("Opening MAVLink connection for DISARM...")
            mav = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
            hb  = mav.wait_heartbeat(timeout=5)
            if hb is None:
                log("No heartbeat for DISARM. Pixhawk unreachable.", "ERROR")
                return False

            log("Heartbeat received. Sending DISARM...")
            # Send disarm command 3 times to be sure
            for _ in range(3):
                mav.mav.command_long_send(
                    mav.target_system,
                    mav.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    0,   # 0 = disarm
                    0, 0, 0, 0, 0, 0
                )
                time.sleep(0.1)

            # Wait for ACK
            ack = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)

            if ack and ack.result == 0:
                log("MAVLink DISARM ACK received.")
                return True
            else:
                log("MAVLink DISARM command sent, but no success ACK received.", "WARN")
                return False

        except Exception as e:
            log("MAVLink disarm error: {}".format(e), "ERROR")
            return False
        finally:
            if mav is not None:
                try:
                    mav.close()
                except Exception:
                    pass

    # ── WATCHDOG ──────────────────────────────────────────────────────────────

    def _watchdog_loop(self):
        """
        If GUI is connected but no command received for WATCHDOG_TIMEOUT seconds,
        trigger automatic kill. Protects against frozen GUI or broken pipe.
        """
        log("Watchdog started (timeout={}s).".format(WATCHDOG_TIMEOUT))
        while not self.shutdown_event.is_set():
            time.sleep(0.5)
            if self.client_sock is None:
                continue
            if not self.client_connected:
                continue
            if not self.mission_active:
                continue
            elapsed = time.time() - self.last_cmd_time
            if elapsed > WATCHDOG_TIMEOUT:
                log("Watchdog timeout! No command for {:.1f}s. "
                    "Triggering auto-kill.".format(elapsed), "WARN")
                self._send("LOG:Watchdog triggered. Auto-kill initiated.")
                t = threading.Thread(target=self._handle_kill)
                t.daemon = True
                t.start()
                # Reset timer so we don't fire again immediately
                self.last_cmd_time = time.time()

    def _trigger_failsafe_on_disconnect(self):
        """Called when TCP connection drops."""
        flight_running = (self.flight_proc is not None and
                          self.flight_proc.poll() is None)
        if not (self.mission_active or flight_running or self.kill_in_progress):
            return
        if self.kill_in_progress:
            return
        log("TCP disconnect detected. Triggering failsafe.", "WARN")
        t = threading.Thread(target=self._handle_kill)
        t.daemon = True
        t.start()

    # ── FLIGHT MANAGER ────────────────────────────────────────────────────────

    def _launch_flight_manager(self):
        with self._flight_lock:
            if self.flight_proc and self.flight_proc.poll() is None:
                log("Flight manager already running.", "WARN")
                return True
            try:
                env = os.environ.copy()
                popen_args = {
                    "env": env,
                    "stdout": subprocess.PIPE,
                    "stderr": subprocess.STDOUT,
                }
                if os.name == "posix":
                    popen_args["preexec_fn"] = os.setsid
                self.flight_proc = subprocess.Popen(FLIGHT_MANAGER_CMD,
                                                    **popen_args)
                log("Flight manager launched (PID {}).".format(
                    self.flight_proc.pid))
                t = threading.Thread(target=self._pipe_stdout,
                                      args=(self.flight_proc,))
                t.daemon = True
                t.start()
                return True
            except Exception as e:
                self.flight_proc = None
                log("Failed to launch flight manager: {}".format(e), "ERROR")
                self._send("LOG:ERROR — could not launch flight manager.")
                return False

    def _terminate_flight_manager(self):
        with self._flight_lock:
            proc = self.flight_proc
            if proc is None:
                return
            if proc.poll() is not None:
                self.flight_proc = None
                return
            pid = proc.pid
            log("Force killing flight manager PID {}...".format(pid))
            try:
                if os.name == "posix":
                    os.killpg(os.getpgid(pid), signal.SIGKILL)
                else:
                    proc.kill()
            except Exception as e:
                log("Kill error: {}".format(e), "WARN")
                try:
                    proc.kill()
                except Exception:
                    pass
            for _ in range(20):
                if proc.poll() is not None:
                    self.flight_proc = None
                    log("Flight manager terminated.")
                    return
                time.sleep(0.05)
            log("Flight manager did not exit within the shutdown window.", "ERROR")

    def _pipe_stdout(self, proc):
        for line in iter(proc.stdout.readline, b""):
            text = line.decode("utf-8", errors="ignore").rstrip()
            if text:
                log("[flight_manager] {}".format(text))
                self._send("LOG:{}".format(text))

    # ── ROS NODE KILL ─────────────────────────────────────────────────────────

    def _kill_ros_node(self, node_name):
        try:
            proc = subprocess.Popen(
                ["rosnode", "kill", "/{}".format(node_name)],
                stdout=open(os.devnull, "w"),
                stderr=subprocess.STDOUT
            )
            for _ in range(30):   # 3 second timeout
                if proc.poll() is not None:
                    break
                time.sleep(0.1)
            else:
                proc.kill()
                return
            if proc.returncode == 0:
                log("ROS node '{}' killed.".format(node_name))
        except Exception as e:
            log("rosnode kill '{}' error: {}".format(node_name, e), "WARN")

    # ── HELPERS ───────────────────────────────────────────────────────────────

    def _send(self, message):
        with self._client_lock:
            sock = self.client_sock
            if sock is None or not self.client_connected:
                return
            try:
                sock.sendall((message + "\n").encode("utf-8"))
            except Exception as e:
                log("Send to GUI failed: {}".format(e), "ERROR")

    def _set_state(self, state):
        self.current_state = state
        self._send("STATE:{}".format(state))
        log("State → {}".format(state))

    def _signal_handler(self, signum, frame):
        log("Signal {}. Shutting down.".format(signum), "WARN")
        self.shutdown_event.set()

    def _cleanup(self):
        self._terminate_flight_manager()
        self.client_connected = False
        if self.client_sock:
            try:
                self.client_sock.close()
            except Exception:
                pass
        if self.server_sock:
            try:
                self.server_sock.close()
            except Exception:
                pass
        log("Cleanup complete.")


# ── MAIN ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    listener = GCSCommandListener()
    listener.start()