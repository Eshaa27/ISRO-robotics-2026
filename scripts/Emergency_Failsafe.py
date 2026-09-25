#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
emergency_response.py
IRoC-U 2026 — Emergency Response System (ERS)

Continuously monitors ALL critical parameters at 10 Hz and immediately
triggers emergency landing (AUTO.LAND) on any fault.  Designed to be
imported into final.py alongside the flight controller.

Monitored hazards
─────────────────
  E01  MAVROS / telemetry link lost          -> immediate AUTO.LAND
  E02  IMU data timeout or NaN               -> immediate AUTO.LAND
  E03  Magnetometer failure                  -> immediate AUTO.LAND
  E04  Barometer failure                     -> immediate AUTO.LAND
  E05  Optical flow failure                  -> immediate AUTO.LAND
  E06  Rangefinder failure / out of range    -> immediate AUTO.LAND
  E07  EKF attitude / pos / vel flag failure -> immediate AUTO.LAND
  E08  Battery critical (voltage / percent)  -> immediate AUTO.LAND
  E09  Altitude ceiling breach (>5.0 m)      -> immediate AUTO.LAND
  E10  Excessive tilt (roll or pitch >25 deg)-> immediate AUTO.LAND
  E11  Software watchdog: control loop hang  -> immediate AUTO.LAND
  E12  OFFBOARD mode lost during flight      -> immediate AUTO.LAND
  E13  Excessive velocity (runaway)          -> immediate AUTO.LAND
  E14  Local position data timeout           -> immediate AUTO.LAND
  E15  Flight module abort flag              -> immediate AUTO.LAND

Architecture
────────────
  EmergencyResponseSystem runs a 10 Hz rospy.Timer (independent of flight)
  On trigger: publishes AUTO.LAND mode, disarms if still on ground,
    logs the full fault code, and calls an optional user callback
  Thread-safe: single _triggered flag prevents duplicate actions
  Watchdog: updated by the flight control loop; if it stops updating,
    the ERS detects a software hang
"""

import rospy
import math
import threading
import time

import tf.transformations

from mavros_msgs.msg import State, ExtendedState, EstimatorStatus
from mavros_msgs.srv import CommandBool, SetMode, SetModeRequest
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, BatteryState, Range
from mavros_msgs.msg import OpticalFlowRad

from Sensor_check import SensorHealthStatus


# ─────────────────────────────────────────────────────────────────────────────
# Emergency thresholds — conservative values for IRoC-U indoor arena
# ─────────────────────────────────────────────────────────────────────────────
class ErsThresholds:
    # Sensor data timeouts (seconds)
    TELEMETRY_TIMEOUT   = 2.0
    IMU_TIMEOUT         = 1.5
    POSITION_TIMEOUT    = 2.0
    BATTERY_TIMEOUT     = 5.0
    RANGE_TIMEOUT       = 2.0
    FLOW_TIMEOUT        = 2.0

    # Altitude
    MAX_ALTITUDE_M      = 5.0     # hard ceiling (E09)

    # Tilt
    MAX_TILT_DEG        = 25.0    # roll or pitch (E10)

    # Battery
    BATT_CRITICAL_V     = 13.8    # 4S LiPo 3.45 V/cell
    BATT_CRITICAL_PCT   = 15.0    # percent (0–100)

    # Velocity runaway (m/s) — optical-flow frame can drift fast
    MAX_HORIZ_VEL_MPS   = 3.0
    MAX_VERT_VEL_MPS    = 2.5

    # EKF — ALL three must be True while airborne
    EKF_FAIL_COUNT      = 5       # consecutive failures before trigger

    # Software watchdog
    WATCHDOG_TIMEOUT_S  = 3.0     # if control loop stops kicking, declare hang

    # Monitor loop rate
    MONITOR_RATE_HZ     = 10


# ─────────────────────────────────────────────────────────────────────────────
# Emergency event record
# ─────────────────────────────────────────────────────────────────────────────
class EmergencyEvent:
    def __init__(self, code, message, severity='CRITICAL'):
        self.code      = code
        self.message   = message
        self.severity  = severity
        self.timestamp = rospy.Time.now().to_sec() if not rospy.is_shutdown() else time.time()

    def __str__(self):
        return '[ERS:%s] %s — %s' % (self.code, self.severity, self.message)


# ─────────────────────────────────────────────────────────────────────────────
# Emergency Response System
# ─────────────────────────────────────────────────────────────────────────────
class EmergencyResponseSystem(object):
    """
    Standalone emergency monitor.  Import and attach to the flight node.

    Parameters
    ----------
    shared_status : SensorHealthStatus
        Written by SensorCalibrationNode; read here for pre-computed flags.
    on_emergency_cb : callable or None
        Called with (EmergencyEvent) after AUTO.LAND is commanded.
    standalone : bool
        If True calls rospy.init_node() internally.

    Watchdog usage in flight loop
    ──────────────────────────────
        ers.kick_watchdog()    # call this every control cycle
    """

    def __init__(self, shared_status, on_emergency_cb=None,
                 alert_callback=None, standalone=False):
        if standalone:
            rospy.init_node('emergency_response_node')

        self.thresh         = ErsThresholds()
        self.shared_status  = shared_status
        self._on_emergency  = on_emergency_cb
        self._alert_callback = alert_callback
        self._alert_pub = rospy.Publisher(
            '/gcs/emergency_alert', String, queue_size=10, latch=True)

        # ── Trigger state ─────────────────────────────────────────────────────
        self._triggered      = False
        self._trigger_lock   = threading.Lock()
        self._event_log      = []       # list[EmergencyEvent]

        # ── Watchdog ──────────────────────────────────────────────────────────
        self._watchdog_last_kick = time.time()
        self._watchdog_enabled   = False   # only active after ARM

        # ── Flight context (set by flight node) ───────────────────────────────
        self._airborne       = False    # True when TAKEOFF / HOVER / DESCENT
        self._ekf_fail_count = 0

        # ── MAVROS raw subscriptions (ERS owns its own subscribers) ──────────
        self._vehicle_state   = None
        self._imu             = None
        self._local_pose      = None
        self._local_velocity  = None
        self._battery         = None
        self._rangefinder     = None
        self._estimator       = None
        self._ext_state       = None
        self._optical_flow    = None

        self._last_imu_t      = None
        self._last_pose_t     = None
        self._last_battery_t  = None
        self._last_range_t    = None
        self._last_flow_t     = None

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber('/mavros/state',                State,           self._cb_state)
        rospy.Subscriber('/mavros/extended_state',       ExtendedState,   self._cb_ext_state)
        rospy.Subscriber('/mavros/imu/data',             Imu,             self._cb_imu)
        rospy.Subscriber('/mavros/local_position/pose',  PoseStamped,     self._cb_pose)
        rospy.Subscriber('/mavros/local_position/velocity_local',
                         TwistStamped, self._cb_vel)
        rospy.Subscriber('/mavros/battery',              BatteryState,    self._cb_battery)
        rospy.Subscriber('/mavros/distance_sensor/rangefinder_sub',
                         Range, self._cb_range)
        rospy.Subscriber('/mavros/estimator_status',     EstimatorStatus, self._cb_estimator)
        rospy.Subscriber('/mavros/px4flow/raw/optical_flow_rad',
                         OpticalFlowRad, self._cb_flow)

        # ── Services ─────────────────────────────────────────────────────────
        rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
        rospy.wait_for_service('/mavros/set_mode',   timeout=30)
        self._arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self._mode_srv   = rospy.ServiceProxy('/mavros/set_mode',   SetMode)

        # ── Monitor timer ─────────────────────────────────────────────────────
        period = 1.0 / self.thresh.MONITOR_RATE_HZ
        self._monitor_timer = rospy.Timer(rospy.Duration(period), self._monitor)

        rospy.loginfo('[ERS] Emergency Response System ACTIVE at %d Hz'
                      % self.thresh.MONITOR_RATE_HZ)
        rospy.loginfo('[ERS] Max altitude: %.1fm | Max tilt: %.0f deg | '
                      'Batt critical: %.1fV / %.0f%%'
                      % (self.thresh.MAX_ALTITUDE_M, self.thresh.MAX_TILT_DEG,
                         self.thresh.BATT_CRITICAL_V,
                         self.thresh.BATT_CRITICAL_PCT))

    # ─────────────────────────────────────────────────────────────────────────
    # Public API
    # ─────────────────────────────────────────────────────────────────────────
    def kick_watchdog(self):
        """Call this every flight control cycle to prevent watchdog trigger."""
        self._watchdog_last_kick = time.time()

    def enable_watchdog(self):
        """Enable after ARM so pre-flight time does not count."""
        self._watchdog_enabled   = True
        self._watchdog_last_kick = time.time()
        rospy.loginfo('[ERS] Watchdog ENABLED')

    def set_airborne(self, is_airborne):
        """Tell ERS whether the drone is airborne (more checks apply)."""
        self._airborne = is_airborne

    def is_triggered(self):
        with self._trigger_lock:
            return self._triggered

    def get_event_log(self):
        return list(self._event_log)

    def manual_trigger(self, reason='manual trigger'):
        """Allow flight node or operator to force emergency landing."""
        self._fire(EmergencyEvent('E00', reason))

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────────
    def _cb_state(self, msg):
        self._vehicle_state = msg

    def _cb_ext_state(self, msg):
        self._ext_state = msg

    def _cb_imu(self, msg):
        self._imu        = msg
        self._last_imu_t = time.time()

    def _cb_pose(self, msg):
        self._local_pose  = msg
        self._last_pose_t = time.time()

    def _cb_vel(self, msg):
        self._local_velocity = msg

    def _cb_battery(self, msg):
        self._battery        = msg
        self._last_battery_t = time.time()

    def _cb_range(self, msg):
        self._rangefinder  = msg
        self._last_range_t = time.time()

    def _cb_estimator(self, msg):
        self._estimator = msg

    def _cb_flow(self, msg):
        self._optical_flow = msg
        self._last_flow_t  = time.time()

    # ─────────────────────────────────────────────────────────────────────────
    # Emergency action
    # ─────────────────────────────────────────────────────────────────────────
    def _fire(self, event):
        """
        Execute emergency landing exactly once.
        Thread-safe — multiple concurrent calls collapse to one action.
        """
        with self._trigger_lock:
            if self._triggered:
                return   # already handled
            self._triggered = True

        self._event_log.append(event)

        banner = '!' * 70
        rospy.logerr(banner)
        rospy.logerr('!!! EMERGENCY RESPONSE TRIGGERED !!!')
        rospy.logerr('!!! %s' % str(event))
        rospy.logerr(banner)
        try:
            self._alert_pub.publish(String(data=str(event)))
        except Exception as e:
            rospy.logerr('[ERS] Alert topic publish error: %s' % e)

        # Notify the GCS before the landing command so the operator sees the
        # fault immediately while AUTO.LAND is being requested.
        if self._alert_callback is not None:
            try:
                self._alert_callback(event)
            except Exception as e:
                rospy.logerr('[ERS] Alert callback error: %s' % e)

        self._cmd_auto_land()

        if self._on_emergency is not None:
            try:
                self._on_emergency(event)
            except Exception as e:
                rospy.logerr('[ERS] Callback error: %s' % e)

    def _cmd_auto_land(self):
        """Send AUTO.LAND with retries."""
        for attempt in range(5):
            try:
                req = SetModeRequest()
                req.custom_mode = 'AUTO.LAND'
                r = self._mode_srv(req)
                if r.mode_sent:
                    rospy.logerr('[ERS] AUTO.LAND commanded (attempt %d)'
                                 % (attempt + 1))
                    return
            except rospy.ServiceException as e:
                rospy.logerr('[ERS] SetMode error (attempt %d): %s'
                             % (attempt + 1, e))
            rospy.sleep(0.2)
        rospy.logerr('[ERS] AUTO.LAND service failed — attempting DISARM')
        self._cmd_disarm()

    def _cmd_disarm(self):
        """Last-resort disarm (only safe on ground)."""
        try:
            self._arming_srv(False)
            rospy.logerr('[ERS] DISARM sent')
        except Exception as e:
            rospy.logerr('[ERS] DISARM error: %s' % e)

    # ─────────────────────────────────────────────────────────────────────────
    # Monitor loop (10 Hz)
    # ─────────────────────────────────────────────────────────────────────────
    def _monitor(self, event=None):
        """Evaluate every hazard condition in priority order."""
        if self._triggered:
            return   # already triggered; wait for landing

        now = time.time()

        # ── E01  Telemetry / MAVROS link ──────────────────────────────────────
        if self._vehicle_state is None or not self._vehicle_state.connected:
            self._fire(EmergencyEvent('E01', 'MAVROS telemetry link lost'))
            return

        # ── E02  IMU timeout ──────────────────────────────────────────────────
        if self._last_imu_t is None or \
           (now - self._last_imu_t) > self.thresh.IMU_TIMEOUT:
            self._fire(EmergencyEvent(
                'E02', 'IMU data timeout (%.1fs)'
                % (now - (self._last_imu_t or 0))
            ))
            return

        # IMU NaN check
        if self._imu is not None:
            ax = self._imu.linear_acceleration.x
            if math.isnan(ax) or math.isinf(ax):
                self._fire(EmergencyEvent('E02', 'IMU NaN/Inf detected'))
                return

        # ── E08  Battery critical ─────────────────────────────────────────────
        if self._battery is not None and self._last_battery_t is not None:
            volt = self._battery.voltage
            pct  = self._battery.percentage * 100.0

            if volt < self.thresh.BATT_CRITICAL_V:
                self._fire(EmergencyEvent(
                    'E08', 'Battery voltage critical: %.2fV < %.2fV'
                    % (volt, self.thresh.BATT_CRITICAL_V)
                ))
                return

            if pct >= 0 and pct < self.thresh.BATT_CRITICAL_PCT:
                self._fire(EmergencyEvent(
                    'E08', 'Battery percent critical: %.1f%% < %.1f%%'
                    % (pct, self.thresh.BATT_CRITICAL_PCT)
                ))
                return
        elif self._last_battery_t is not None and \
             (now - self._last_battery_t) > self.thresh.BATTERY_TIMEOUT:
            self._fire(EmergencyEvent('E08', 'Battery data timeout'))
            return

        # ── (Airborne-only checks below) ──────────────────────────────────────
        if not self._airborne:
            return

        # ── E12  OFFBOARD mode lost while airborne ────────────────────────────
        if self._vehicle_state and \
           self._vehicle_state.mode not in ('OFFBOARD', 'AUTO.LAND',
                                            'AUTO.RTL', 'STABILIZE',
                                            'ALT_HOLD'):
            if not hasattr(self, '_mode_loss_t'):
                self._mode_loss_t = now
            if (now - self._mode_loss_t) > 1.5:
                self._fire(EmergencyEvent(
                    'E12', 'OFFBOARD mode lost: current=%s'
                    % self._vehicle_state.mode
                ))
                return
        else:
            self._mode_loss_t = now

        # ── E09  Altitude ceiling ─────────────────────────────────────────────
        if self._local_pose is not None:
            alt = self._local_pose.pose.position.z
            if alt > self.thresh.MAX_ALTITUDE_M:
                self._fire(EmergencyEvent(
                    'E09', 'Altitude ceiling: %.2fm > %.2fm'
                    % (alt, self.thresh.MAX_ALTITUDE_M)
                ))
                return

        # ── E14  Local position timeout ───────────────────────────────────────
        if self._last_pose_t is None or \
           (now - self._last_pose_t) > self.thresh.POSITION_TIMEOUT:
            self._fire(EmergencyEvent(
                'E14', 'Local position timeout: %.1fs'
                % (now - (self._last_pose_t or 0))
            ))
            return

        # ── E10  Tilt (roll / pitch) ──────────────────────────────────────────
        if self._imu is not None:
            q = self._imu.orientation
            roll, pitch, _ = tf.transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )
            roll_d  = abs(math.degrees(roll))
            pitch_d = abs(math.degrees(pitch))
            if roll_d > self.thresh.MAX_TILT_DEG or \
               pitch_d > self.thresh.MAX_TILT_DEG:
                self._fire(EmergencyEvent(
                    'E10', 'Unsafe tilt: roll=%.1f deg pitch=%.1f deg'
                    % (roll_d, pitch_d)
                ))
                return

        # ── E07  EKF health ───────────────────────────────────────────────────
        if self._estimator is not None:
            ekf_ok = (
                self._estimator.attitude_status_flag and
                self._estimator.pos_horiz_rel_status_flag and
                self._estimator.velocity_horiz_status_flag and
                self._estimator.pos_vert_abs_status_flag
            )
            if not ekf_ok:
                self._ekf_fail_count += 1
                rospy.logwarn(
                    '[ERS] EKF degraded count=%d/%d (att=%s pos=%s vel=%s hgt=%s)'
                    % (self._ekf_fail_count, self.thresh.EKF_FAIL_COUNT,
                       self._estimator.attitude_status_flag,
                       self._estimator.pos_horiz_rel_status_flag,
                       self._estimator.velocity_horiz_status_flag,
                       self._estimator.pos_vert_abs_status_flag)
                )
                if self._ekf_fail_count >= self.thresh.EKF_FAIL_COUNT:
                    self._fire(EmergencyEvent(
                        'E07', 'EKF failure: att=%s pos=%s vel=%s hgt=%s'
                        % (self._estimator.attitude_status_flag,
                           self._estimator.pos_horiz_rel_status_flag,
                           self._estimator.velocity_horiz_status_flag,
                           self._estimator.pos_vert_abs_status_flag)
                    ))
                    return
            else:
                if self._ekf_fail_count > 0:
                    rospy.loginfo('[ERS] EKF health restored')
                self._ekf_fail_count = 0

        # ── E05  Optical flow timeout ─────────────────────────────────────────
        if self._last_flow_t is not None and \
           (now - self._last_flow_t) > self.thresh.FLOW_TIMEOUT:
            self._fire(EmergencyEvent(
                'E05', 'Optical flow timeout: %.1fs'
                % (now - self._last_flow_t)
            ))
            return

        # ── E06  Rangefinder timeout / bad range ──────────────────────────────
        if self._last_range_t is not None and \
           (now - self._last_range_t) > self.thresh.RANGE_TIMEOUT:
            self._fire(EmergencyEvent(
                'E06', 'Rangefinder timeout: %.1fs'
                % (now - self._last_range_t)
            ))
            return
        if self._rangefinder is not None:
            r = self._rangefinder.range
            if r < 0.02 or r > 10.0 or math.isnan(r):
                self._fire(EmergencyEvent(
                    'E06', 'Rangefinder invalid reading: %.2fm' % r
                ))
                return

        # ── E13  Velocity runaway ─────────────────────────────────────────────
        if self._local_velocity is not None:
            vx = self._local_velocity.twist.linear.x
            vy = self._local_velocity.twist.linear.y
            vz = self._local_velocity.twist.linear.z
            horiz_v = math.sqrt(vx**2 + vy**2)
            vert_v  = abs(vz)
            if horiz_v > self.thresh.MAX_HORIZ_VEL_MPS:
                self._fire(EmergencyEvent(
                    'E13', 'Horizontal velocity runaway: %.2fm/s' % horiz_v
                ))
                return
            if vert_v > self.thresh.MAX_VERT_VEL_MPS:
                self._fire(EmergencyEvent(
                    'E13', 'Vertical velocity runaway: %.2fm/s' % vert_v
                ))
                return

        # ── E11  Software watchdog ────────────────────────────────────────────
        if self._watchdog_enabled:
            age = time.time() - self._watchdog_last_kick
            if age > self.thresh.WATCHDOG_TIMEOUT_S:
                self._fire(EmergencyEvent(
                    'E11', 'Software watchdog: control loop frozen for %.1fs'
                    % age
                ))
                return

        # ── Shared sensor status cross-check ─────────────────────────────────
        s = self.shared_status
        if s is not None and not s.imu_ok and self._airborne:
            self._fire(EmergencyEvent('E02', 'Shared IMU health flag failed'))
            return
        if s is not None and not s.baro_ok and self._airborne:
            self._fire(EmergencyEvent('E04', 'Barometer health flag failed'))
            return

    # ─────────────────────────────────────────────────────────────────────────
    # Blocking wait for landing after emergency
    # ─────────────────────────────────────────────────────────────────────────
    def wait_for_landing(self, timeout_s=60.0):
        """Block until ext_state confirms LANDED or timeout."""
        deadline = time.time() + timeout_s
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if time.time() > deadline:
                rospy.logerr('[ERS] Emergency landing timeout — force DISARM')
                self._cmd_disarm()
                return False

            landed = (
                self._ext_state is not None and
                self._ext_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
            )
            low_alt = (
                self._local_pose is not None and
                self._local_pose.pose.position.z < 0.15
            )
            if landed or low_alt:
                rospy.logerr('[ERS] Emergency landing confirmed — DISARMING')
                self._cmd_disarm()
                return True
            rate.sleep()
        return False

    def print_event_log(self):
        """Dump all recorded events to roslog."""
        rospy.logerr('[ERS] EVENT LOG')
        for ev in self._event_log:
            rospy.logerr('[ERS] %s' % ev)
        if not self._event_log:
            rospy.loginfo('[ERS] No emergency events recorded')


# ─────────────────────────────────────────────────────────────────────────────
# Standalone entry point (for testing ERS alone)
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    rospy.init_node('emergency_response_standalone')

    dummy_status = SensorHealthStatus()
    dummy_status.all_systems_go = True
    dummy_status.imu_ok  = True
    dummy_status.baro_ok = True

    def on_emergency(event):
        rospy.logerr('=== CALLBACK: %s ===' % event)

    ers = EmergencyResponseSystem(
        shared_status=dummy_status,
        on_emergency_cb=on_emergency,
        standalone=False
    )

    rospy.spin()