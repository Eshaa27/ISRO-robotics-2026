#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
sensor_calibration.py
IRoC-U 2026 — Sensor Calibration & Health Monitor

Covers:
  • MPU6000 IMU (accel + gyro)
  • ST Micro 16-bit gyroscope
  • ST Micro 14-bit accel/compass (magnetometer)
  • MEAS barometer
  • Optical Flow sensor (PX4FLOW or equivalent)
  • Battery voltage / current
  • MAVROS telemetry link

Exports:
  SensorCalibrationNode  — start as a standalone node or import into final.py
  SensorHealthStatus     — dataclass shared across modules
"""

import rospy
import math
import time

from mavros_msgs.msg import (
    State,
    EstimatorStatus,
    OpticalFlowRad,
)
from mavros_msgs.srv import (
    CommandBool,
    SetMode,
    ParamGet,
    ParamGetRequest,
    StreamRate,
    StreamRateRequest,
)
from sensor_msgs.msg import (
    Imu,
    MagneticField,
    BatteryState,
    FluidPressure,
    Range,
)
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64


# ─────────────────────────────────────────────────────────────────────────────
# Shared health status — imported by other modules
# ─────────────────────────────────────────────────────────────────────────────
class SensorHealthStatus:
    """Thread-safe status object updated by SensorCalibrationNode."""

    def __init__(self):
        self.failure_reasons = []
        self.all_systems_go = False
        self.calibration_complete = False
        self.calibration_ok = False
        self.calibration_message = 'NOT_CALIBRATED'

        self.mavros_connected = False
        self.telemetry_ok = False
        self.imu_ok = False
        self.imu_accel_variance = 0.0
        self.imu_gyro_variance = 0.0
        self.mag_ok = False
        self.mag_field_magnitude = 0.0
        self.baro_ok = False
        self.baro_altitude_m = 0.0
        self.optical_flow_ok = False
        self.optical_flow_quality = 0.0
        self.rangefinder_ok = False
        self.rangefinder_distance_m = 0.0
        self.ekf_pos_ok = False
        self.ekf_vel_ok = False
        self.ekf_att_ok = False
        self.ekf_hgt_ok = False
        self.battery_ok = False
        self.battery_voltage = 0.0
        self.battery_percent = 100.0
        self.local_position_ok = False
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.last_imu_time = None
        self.last_mag_time = None
        self.last_baro_time = None
        self.last_flow_time = None
        self.last_range_time = None
        self.last_position_time = None
        self.last_battery_time = None


# ─────────────────────────────────────────────────────────────────────────────
# Calibration thresholds
# ─────────────────────────────────────────────────────────────────────────────
class CalibThresholds:
    # Sensor timeouts (seconds) — real hardware is slower than Gazebo
    IMU_TIMEOUT        = 2.0
    MAG_TIMEOUT        = 3.0
    BARO_TIMEOUT       = 3.0
    FLOW_TIMEOUT       = 3.0
    RANGE_TIMEOUT      = 3.0
    POSITION_TIMEOUT   = 3.0
    BATTERY_TIMEOUT    = 5.0
    TELEMETRY_TIMEOUT  = 3.0

    # IMU sanity — static on ground, accel magnitude should be ~9.81 m/s²
    ACCEL_STATIC_MIN   = 9.0    # m/s²
    ACCEL_STATIC_MAX   = 10.6   # m/s²
    GYRO_STATIC_MAX    = 0.05   # rad/s  (stationary bias)

    # Magnetometer — Earth field 25–65 µT = 0.00025–0.00065 T
    # BatteryState.percentage is 0.0–1.0 in ROS
    MAG_MIN_GAUSS      = 0.0025   # Gauss  (0.25 Gauss = 25 µT)
    MAG_MAX_GAUSS      = 0.006    # Gauss

    # Battery
    BATTERY_MIN_PCT    = 20.0    # percent (0–100)
    BATTERY_MIN_VOLT   = 14.0   # V for 4S LiPo
    BATTERY_WARN_PCT   = 30.0

    # Optical flow quality  (0 = worst, 255 = best)
    FLOW_MIN_QUALITY   = 50

    # Rangefinder
    RANGE_MIN_M        = 0.05
    RANGE_MAX_M        = 8.0

    # EKF — all three flags must be True before arming
    EKF_REQUIRED_FLAGS = ['attitude', 'pos_horiz_rel', 'velocity_horiz', 'pos_vert_abs']


# ─────────────────────────────────────────────────────────────────────────────
# Rolling window statistics helper
# ─────────────────────────────────────────────────────────────────────────────
class RollingStats:
    def __init__(self, window=50):
        self._buf = []
        self._window = window

    def push(self, value):
        self._buf.append(value)
        if len(self._buf) > self._window:
            self._buf.pop(0)

    def variance(self):
        if len(self._buf) < 5:
            return 0.0
        mean = sum(self._buf) / len(self._buf)
        return sum((x - mean) ** 2 for x in self._buf) / len(self._buf)

    def mean(self):
        if not self._buf:
            return 0.0
        return sum(self._buf) / len(self._buf)

    def count(self):
        return len(self._buf)


# ─────────────────────────────────────────────────────────────────────────────
# Main calibration node
# ─────────────────────────────────────────────────────────────────────────────
class SensorCalibrationNode(object):
    """
    Subscribes to every sensor topic published by MAVROS and performs
    continuous health checking.  Results are written into a shared
    SensorHealthStatus object that other modules can read directly.

    Usage (standalone):
        node = SensorCalibrationNode(standalone=True)
        rospy.spin()

    Usage (imported into final.py):
        node = SensorCalibrationNode(standalone=False)
        status = node.status          # SensorHealthStatus
    """

    def __init__(self, standalone=False, log_callback=None):
        if standalone:
            rospy.init_node('sensor_calibration_node')

        self.status = SensorHealthStatus()
        self.thresh = CalibThresholds()
        self._log_callback = log_callback

        # Rolling stats for noise estimation
        self._accel_z_stats  = RollingStats(100)
        self._gyro_z_stats   = RollingStats(100)
        self._mag_x_stats    = RollingStats(50)
        self._flow_q_stats   = RollingStats(30)

        # ── MAVROS stream rate request (ensure data flows) ────────────────────
        self._request_stream_rates()

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber('/mavros/state',                State,           self._cb_state)
        rospy.Subscriber('/mavros/imu/data',             Imu,             self._cb_imu)
        rospy.Subscriber('/mavros/imu/mag',              MagneticField,   self._cb_mag)
        rospy.Subscriber('/mavros/imu/static_pressure',  FluidPressure,   self._cb_baro)
        rospy.Subscriber('/mavros/px4flow/raw/optical_flow_rad', OpticalFlowRad, self._cb_flow)
        rospy.Subscriber('/mavros/distance_sensor/rangefinder_sub', Range, self._cb_range)
        rospy.Subscriber('/mavros/battery',              BatteryState,    self._cb_battery)
        rospy.Subscriber('/mavros/local_position/pose',  PoseStamped,     self._cb_local_pos)
        rospy.Subscriber('/mavros/estimator_status',     EstimatorStatus, self._cb_ekf)

        # ── Periodic health-check timer ───────────────────────────────────────
        self._check_timer = rospy.Timer(rospy.Duration(1.0), self._run_health_check)

        rospy.loginfo('[CAL] Sensor Calibration Node initialised')

    def _report(self, level, message):
        """Write to ROS logs and optionally forward the same message to GCS."""
        if level == 'ERROR':
            rospy.logerr(message)
        elif level == 'WARN':
            rospy.logwarn(message)
        else:
            rospy.loginfo(message)
        if self._log_callback is not None:
            self._log_callback(message, level)

    # ─── Stream rate setup ────────────────────────────────────────────────────
    def _request_stream_rates(self):
        """Ask MAVROS to increase telemetry stream rates."""
        try:
            rospy.wait_for_service('/mavros/set_stream_rate', timeout=5.0)
            srv = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
            req = StreamRateRequest()
            req.stream_id = 0       # ALL
            req.message_rate = 50   # Hz
            req.on_off = True
            srv(req)
            self._report('INFO', '[CAL] Stream rates set to 50 Hz')
        except Exception as e:
            self._report('WARN', '[CAL] Could not set stream rates: %s' % e)

    # ─── Callbacks ────────────────────────────────────────────────────────────
    def _cb_state(self, msg):
        self.status.mavros_connected = msg.connected
        self.status.telemetry_ok     = msg.connected

    def _cb_imu(self, msg):
        now = rospy.Time.now().to_sec()
        self.status.last_imu_time = now

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        accel_mag = math.sqrt(ax**2 + ay**2 + az**2)
        gyro_mag  = math.sqrt(gx**2 + gy**2 + gz**2)

        self._accel_z_stats.push(accel_mag)
        self._gyro_z_stats.push(gyro_mag)

        # Sanity: on the ground, |a| ≈ g
        accel_ok = (self.thresh.ACCEL_STATIC_MIN
                    <= self._accel_z_stats.mean()
                    <= self.thresh.ACCEL_STATIC_MAX)
        gyro_ok  = self._gyro_z_stats.mean() < self.thresh.GYRO_STATIC_MAX

        self.status.imu_ok = accel_ok and gyro_ok
        self.status.imu_accel_variance = self._accel_z_stats.variance()
        self.status.imu_gyro_variance  = self._gyro_z_stats.variance()

    def _cb_mag(self, msg):
        self.status.last_mag_time = rospy.Time.now().to_sec()
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        mz = msg.magnetic_field.z
        # ROS MagneticField is in Tesla; convert to Gauss (1 T = 10000 G)
        mag_gauss = math.sqrt(mx**2 + my**2 + mz**2) * 10000.0
        self.status.mag_field_magnitude = mag_gauss
        self._mag_x_stats.push(mag_gauss)

        self.status.mag_ok = (
            self.thresh.MAG_MIN_GAUSS
            <= self._mag_x_stats.mean()
            <= self.thresh.MAG_MAX_GAUSS
        )

    def _cb_baro(self, msg):
        self.status.last_baro_time = rospy.Time.now().to_sec()
        # Convert pressure (Pa) to rough altitude via barometric formula
        pressure_pa = msg.fluid_pressure
        if pressure_pa > 0:
            # ISA sea-level pressure = 101325 Pa
            alt = 44330.0 * (1.0 - (pressure_pa / 101325.0) ** 0.1903)
            self.status.baro_altitude_m = alt
            self.status.baro_ok = (pressure_pa > 50000.0)   # sanity: > 50 kPa
        else:
            self.status.baro_ok = False

    def _cb_flow(self, msg):
        self.status.last_flow_time = rospy.Time.now().to_sec()
        quality = msg.quality          # 0–255
        self._flow_q_stats.push(quality)
        self.status.optical_flow_quality = self._flow_q_stats.mean()
        self.status.optical_flow_ok = (
            self._flow_q_stats.mean() >= self.thresh.FLOW_MIN_QUALITY
        )

    def _cb_range(self, msg):
        self.status.last_range_time = rospy.Time.now().to_sec()
        d = msg.range
        self.status.rangefinder_distance_m = d
        self.status.rangefinder_ok = (
            self.thresh.RANGE_MIN_M <= d <= self.thresh.RANGE_MAX_M
        )

    def _cb_battery(self, msg):
        self.status.last_battery_time = rospy.Time.now().to_sec()
        # BatteryState.percentage is 0.0–1.0
        pct   = msg.percentage * 100.0 if msg.percentage >= 0 else -1.0
        volt  = msg.voltage

        self.status.battery_voltage = volt
        self.status.battery_percent = pct

        volt_ok = volt >= self.thresh.BATTERY_MIN_VOLT
        pct_ok  = pct  >= self.thresh.BATTERY_MIN_PCT or pct < 0   # < 0 = unknown

        self.status.battery_ok = volt_ok and pct_ok

        if pct > 0 and pct < self.thresh.BATTERY_WARN_PCT:
            rospy.logwarn_throttle(
                10, '[CAL] Battery low: %.1f%% / %.2fV' % (pct, volt)
            )

    def _cb_local_pos(self, msg):
        self.status.last_position_time = rospy.Time.now().to_sec()
        self.status.position_x = msg.pose.position.x
        self.status.position_y = msg.pose.position.y
        self.status.position_z = msg.pose.position.z
        self.status.local_position_ok = True

    def _cb_ekf(self, msg):
        self.status.ekf_att_ok     = msg.attitude_status_flag
        self.status.ekf_pos_ok     = msg.pos_horiz_rel_status_flag
        self.status.ekf_vel_ok     = msg.velocity_horiz_status_flag
        self.status.ekf_hgt_ok     = msg.pos_vert_abs_status_flag

    # ─── Periodic health evaluator ────────────────────────────────────────────
    def _run_health_check(self, event=None):
        now = rospy.Time.now().to_sec()
        failures = []

        # ── 1. Telemetry link ─────────────────────────────────────────────────
        if not self.status.mavros_connected:
            failures.append('MAVROS_NOT_CONNECTED')

        # ── 2. IMU ───────────────────────────────────────────────────────────
        if self.status.last_imu_time is None or \
           (now - self.status.last_imu_time) > self.thresh.IMU_TIMEOUT:
            failures.append('IMU_TIMEOUT')
            self.status.imu_ok = False
        elif not self.status.imu_ok:
            failures.append('IMU_BAD_READINGS')

        # ── 3. Magnetometer ──────────────────────────────────────────────────
        if self.status.last_mag_time is None or \
           (now - self.status.last_mag_time) > self.thresh.MAG_TIMEOUT:
            failures.append('MAG_TIMEOUT')
            self.status.mag_ok = False
        elif not self.status.mag_ok:
            failures.append('MAG_BAD_FIELD: %.4f Gauss'
                            % self.status.mag_field_magnitude)

        # ── 4. Barometer ─────────────────────────────────────────────────────
        if self.status.last_baro_time is None or \
           (now - self.status.last_baro_time) > self.thresh.BARO_TIMEOUT:
            failures.append('BARO_TIMEOUT')
            self.status.baro_ok = False
        elif not self.status.baro_ok:
            failures.append('BARO_BAD_PRESSURE')

        # ── 5. Optical flow ──────────────────────────────────────────────────
        if self.status.last_flow_time is None or \
           (now - self.status.last_flow_time) > self.thresh.FLOW_TIMEOUT:
            failures.append('OPTICAL_FLOW_TIMEOUT')
            self.status.optical_flow_ok = False
        elif not self.status.optical_flow_ok:
            failures.append('OPTICAL_FLOW_LOW_QUALITY: %.0f/255'
                            % self.status.optical_flow_quality)

        # ── 6. Rangefinder ───────────────────────────────────────────────────
        if self.status.last_range_time is None or \
           (now - self.status.last_range_time) > self.thresh.RANGE_TIMEOUT:
            failures.append('RANGEFINDER_TIMEOUT')
            self.status.rangefinder_ok = False
        elif not self.status.rangefinder_ok:
            failures.append('RANGEFINDER_BAD_RANGE: %.2fm'
                            % self.status.rangefinder_distance_m)

        # ── 7. EKF ───────────────────────────────────────────────────────────
        if not self.status.ekf_att_ok:
            failures.append('EKF_ATTITUDE_FAIL')
        if not self.status.ekf_pos_ok:
            failures.append('EKF_HORIZ_POS_FAIL')
        if not self.status.ekf_vel_ok:
            failures.append('EKF_HORIZ_VEL_FAIL')
        if not self.status.ekf_hgt_ok:
            failures.append('EKF_VERT_POS_FAIL')

        # ── 8. Battery ───────────────────────────────────────────────────────
        if self.status.last_battery_time is None or \
           (now - self.status.last_battery_time) > self.thresh.BATTERY_TIMEOUT:
            failures.append('BATTERY_TIMEOUT')
            self.status.battery_ok = False
        elif not self.status.battery_ok:
            failures.append('BATTERY_LOW: %.1f%% / %.2fV'
                            % (self.status.battery_percent,
                               self.status.battery_voltage))

        # ── 9. Local position ────────────────────────────────────────────────
        if self.status.last_position_time is None or \
           (now - self.status.last_position_time) > self.thresh.POSITION_TIMEOUT:
            failures.append('LOCAL_POSITION_TIMEOUT')
            self.status.local_position_ok = False

        # ── Compile result ────────────────────────────────────────────────────
        self.status.failure_reasons = failures
        self.status.all_systems_go  = (len(failures) == 0)

        if self.status.all_systems_go:
            rospy.loginfo_throttle(
                10, '[CAL] All sensors healthy | Bat: %.1f%% %.2fV | '
                    'Flow Q: %.0f | Range: %.2fm'
                    % (self.status.battery_percent,
                       self.status.battery_voltage,
                       self.status.optical_flow_quality,
                       self.status.rangefinder_distance_m)
            )
        else:
            rospy.logwarn_throttle(
                2, '[CAL] Health failures: %s' % ', '.join(failures)
            )

    def calibrate_and_check(self, timeout_s=90.0):
        """Activate telemetry, validate every sensor, and return GO/NO-GO."""
        self.status.calibration_complete = False
        self.status.calibration_ok = False
        self.status.calibration_message = 'CALIBRATING'
        self._report('INFO', '[CAL] Activating MAVROS sensor streams...')
        self._report('INFO', '[CAL] Keeping vehicle stationary while sensors settle.')

        deadline = rospy.Time.now().to_sec() + timeout_s
        rate = rospy.Rate(2)
        last_failure_report = None
        while not rospy.is_shutdown():
            self._run_health_check()
            if self.status.all_systems_go:
                break
            failures = ', '.join(self.status.failure_reasons)
            if failures != last_failure_report:
                self._report('WARN', '[CAL] Waiting for: %s' % failures)
                last_failure_report = failures
            if rospy.Time.now().to_sec() >= deadline:
                self.status.calibration_message = 'TIMEOUT'
                self._report('ERROR', '[CAL] Calibration timeout: %s' % failures)
                self._report_sensor_summary()
                return False
            rate.sleep()

        self.status.calibration_complete = True
        self.status.calibration_ok = self.status.all_systems_go
        self.status.calibration_message = (
            'GOOD_TO_GO' if self.status.all_systems_go else 'NOT_READY')
        self._report_sensor_summary()
        if self.status.all_systems_go:
            self._report('INFO', '[CAL] GOOD_TO_GO: all sensors healthy.')
        else:
            self._report('ERROR', '[CAL] NOT_READY: %s' %
                         ', '.join(self.status.failure_reasons))
        return self.status.all_systems_go

    def _report_sensor_summary(self):
        """Forward a complete, human-readable sensor report to the GCS."""
        s = self.status
        checks = [
            ('MAVROS telemetry', s.telemetry_ok),
            ('IMU', s.imu_ok),
            ('Magnetometer', s.mag_ok),
            ('Barometer', s.baro_ok),
            ('Optical flow', s.optical_flow_ok),
            ('Rangefinder', s.rangefinder_ok),
            ('EKF attitude', s.ekf_att_ok),
            ('EKF horizontal position', s.ekf_pos_ok),
            ('EKF horizontal velocity', s.ekf_vel_ok),
            ('EKF vertical position', s.ekf_hgt_ok),
            ('Battery', s.battery_ok),
            ('Local position', s.local_position_ok),
        ]
        for name, healthy in checks:
            self._report('INFO' if healthy else 'ERROR',
                         '[CAL] %-26s %s' % (name, 'OK' if healthy else 'FAIL'))

    # ─── Blocking pre-flight check ────────────────────────────────────────────
    def wait_for_all_systems_go(self, timeout_s=60.0):
        """
        Block until all sensors pass or timeout.
        Returns True if ready, False if timeout.
        """
        rospy.loginfo('[CAL] Waiting for all systems ready (timeout %.0fs)...'
                      % timeout_s)
        deadline = rospy.Time.now().to_sec() + timeout_s
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            if rospy.Time.now().to_sec() > deadline:
                rospy.logerr('[CAL] Preflight timeout! Failures: %s'
                             % self.status.failure_reasons)
                return False

            if self.status.all_systems_go:
                rospy.loginfo('[CAL] All systems GO — proceeding to flight')
                return True

            if self.status.failure_reasons:
                rospy.logwarn('[CAL] Waiting: %s'
                              % ', '.join(self.status.failure_reasons))
            rate.sleep()

        return False

    def print_full_report(self):
        """Print a one-time human-readable report."""
        s = self.status
        sep = '-' * 60
        rospy.loginfo(sep)
        rospy.loginfo('[CAL] PRE-FLIGHT SENSOR REPORT')
        rospy.loginfo(sep)
        rospy.loginfo('[CAL] Telemetry  : %s' % ('OK' if s.telemetry_ok  else 'FAIL'))
        rospy.loginfo('[CAL] IMU        : %s  accel_var=%.4f gyro_var=%.6f'
                      % ('OK' if s.imu_ok else 'FAIL',
                         s.imu_accel_variance, s.imu_gyro_variance))
        rospy.loginfo('[CAL] Magnetometer: %s  %.4f Gauss'
                      % ('OK' if s.mag_ok else 'FAIL', s.mag_field_magnitude))
        rospy.loginfo('[CAL] Barometer  : %s  %.1fm'
                      % ('OK' if s.baro_ok else 'FAIL', s.baro_altitude_m))
        rospy.loginfo('[CAL] Optical Flo: %s  quality=%.0f/255'
                      % ('OK' if s.optical_flow_ok else 'FAIL',
                         s.optical_flow_quality))
        rospy.loginfo('[CAL] Rangefinder: %s  %.2fm'
                      % ('OK' if s.rangefinder_ok else 'FAIL',
                         s.rangefinder_distance_m))
        rospy.loginfo('[CAL] EKF att    : %s' % ('OK' if s.ekf_att_ok else 'FAIL'))
        rospy.loginfo('[CAL] EKF pos    : %s' % ('OK' if s.ekf_pos_ok else 'FAIL'))
        rospy.loginfo('[CAL] EKF vel    : %s' % ('OK' if s.ekf_vel_ok else 'FAIL'))
        rospy.loginfo('[CAL] EKF hgt    : %s' % ('OK' if s.ekf_hgt_ok else 'FAIL'))
        rospy.loginfo('[CAL] Battery    : %s  %.1f%% / %.2fV'
                      % ('OK' if s.battery_ok else 'FAIL',
                         s.battery_percent, s.battery_voltage))
        rospy.loginfo('[CAL] Local Pos  : %s  x=%.2f y=%.2f z=%.2f'
                      % ('OK' if s.local_position_ok else 'FAIL',
                         s.position_x, s.position_y, s.position_z))
        rospy.loginfo(sep)
        if s.all_systems_go:
            rospy.loginfo('[CAL] RESULT: ALL SYSTEMS GO')
        else:
            rospy.logerr('[CAL] RESULT: NOT READY — %s'
                         % ', '.join(s.failure_reasons))
        rospy.loginfo(sep)


# ─────────────────────────────────────────────────────────────────────────────
# Standalone entry point
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    node = SensorCalibrationNode(standalone=True)
    rospy.sleep(3.0)          # let callbacks populate
    node.print_full_report()
    rospy.spin()