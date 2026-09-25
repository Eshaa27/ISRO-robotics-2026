#!/usr/bin/env python
"""
autonomous_flight.py
IRoC-U 2026 Qualification  |  ROS 1 Melodic / Python 2.7
=========================================================
Hardware : Pixhawk (PX4) + Jetson Nano + USB Webcam
Sensors  : MPU6000 IMU, Barometer, Magnetometer, Optical-Flow, EKF3
Stack    : MAVROS  (fmu communication via /mavros/*)

Mission
-------
1. Establish webcam + optical-flow lock, capture takeoff origin (x, y)
2. Vertical take-off to 4.0 m (barometer / EKF3 altitude)
3. Stable 30-second hover  -  optical-flow drift correction to origin
4. Autonomous landing at origin

Exported entry point (called by final.py)
-----------------------------------------
    run(shared_status, ers)
"""

import rospy
import math
import threading
import time

import cv2

from geometry_msgs.msg  import PoseStamped, TwistStamped
from mavros_msgs.msg    import (
    State,
    OpticalFlowRad,
    PositionTarget,
)
from mavros_msgs.srv    import (
    CommandBool,
    CommandBoolRequest,
    SetMode,
    SetModeRequest,
)
from sensor_msgs.msg    import Range
from std_msgs.msg       import Float64


# ─────────────────────────────────────────────────────────────────────────────
# MISSION CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────

TARGET_ALTITUDE_M   = 4.0      # metres above takeoff
HOVER_DURATION_S    = 30.0     # seconds
HOVER_TOLERANCE_M   = 0.20     # ±0.20 m  (IRoC-U rule)
MAX_DRIFT_M         = 0.30     # optical-flow drift threshold before correction
LAND_DETECT_ALT_M   = 0.12     # altitude below which we declare landed
SETPOINT_RATE_HZ    = 20.0     # offboard setpoint publish rate
WEBCAM_DEVICE       = 0        # /dev/video0

# PID gains for horizontal drift correction (optical flow)
KP_XY               = 0.55
KI_XY               = 0.02
KD_XY               = 0.18

# Optical-flow quality threshold (0-255)
FLOW_QUALITY_MIN    = 80
FLOW_TIMEOUT_S      = 0.5
LIDAR_TIMEOUT_S     = 0.5
LIDAR_MIN_M         = 0.05
LIDAR_MAX_M         = 8.0
ALTITUDE_STABLE_S   = 1.0
RETURN_TOLERANCE_M  = 0.15
RETURN_TIMEOUT_S    = 15.0
TOUCHDOWN_SAMPLES   = 5


def _is_finite(value):
    return not math.isnan(value) and not math.isinf(value)


# ─────────────────────────────────────────────────────────────────────────────
# STATE MACHINE
# ─────────────────────────────────────────────────────────────────────────────

class FlightState(object):
    INIT            = "INIT"
    WAIT_READY      = "WAIT_READY"
    OFFBOARD_REQ    = "OFFBOARD_REQ"
    ARM_REQ         = "ARM_REQ"
    TAKEOFF         = "TAKEOFF"
    HOVER           = "HOVER"
    RETURN          = "RETURN"
    LANDING         = "LANDING"
    DONE            = "DONE"


# ─────────────────────────────────────────────────────────────────────────────
# PID CONTROLLER  (simple 1-axis)
# ─────────────────────────────────────────────────────────────────────────────

class PID(object):
    def __init__(self, kp, ki, kd, limit=1.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit     = limit
        self._integral = 0.0
        self._prev_err = 0.0
        self._prev_t   = None

    def reset(self):
        self._integral = 0.0
        self._prev_err = 0.0
        self._prev_t   = None

    def compute(self, error, now):
        if self._prev_t is None:
            self._prev_t   = now
            self._prev_err = error
            return 0.0
        dt = now - self._prev_t
        if dt <= 0.0:
            return 0.0
        self._integral += error * dt
        derivative      = (error - self._prev_err) / dt
        self._prev_err  = error
        self._prev_t    = now
        out = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.limit, min(self.limit, out))


# ─────────────────────────────────────────────────────────────────────────────
# MAIN NODE
# ─────────────────────────────────────────────────────────────────────────────

class FinalMissionNode(object):
    """
    When imported by final.py:
        node = FinalMissionNode(shared_status=cal.status, ers=ers)
    When run standalone:
        node = FinalMissionNode()
        rospy.spin()
    """

    def __init__(self, shared_status=None, ers=None):
        # rospy.init_node is NOT called here when imported — final.py owns it.
        # When running standalone (if __name__ == '__main__'), caller must
        # call rospy.init_node before constructing this class.

        self._shared_status = shared_status   # SensorHealthStatus or None
        self._ers           = ers             # EmergencyResponseSystem or None

        # ── state ─────────────────────────────────────────────────────────────
        self.state          = FlightState.INIT
        self._lock          = threading.Lock()

        # ── telemetry ─────────────────────────────────────────────────────────
        self.mav_state          = None          # mavros_msgs/State
        self.local_pose         = None          # geometry_msgs/PoseStamped
        self.flow_msg           = None          # mavros_msgs/OpticalFlowRad
        self.rangefinder_alt    = None          # sensor_msgs/Range  (optional)

        # ── origin captured at takeoff ────────────────────────────────────────
        self.origin_x   = 0.0
        self.origin_y   = 0.0
        self.origin_z   = 0.0          # barometer / EKF z at ground

        # ── hover book-keeping ────────────────────────────────────────────────
        self.hover_start    = None
        self.return_start   = None
        self.altitude_band_start = None
        self.touchdown_count = 0
        self.offboard_count = 0
        self.arm_sent       = False
        self.offboard_sent  = False
        self.land_sent      = False

        # ── PIDs (x-axis, y-axis position correction) ─────────────────────────
        self.pid_x = PID(KP_XY, KI_XY, KD_XY)
        self.pid_y = PID(KP_XY, KI_XY, KD_XY)

        # ── webcam ────────────────────────────────────────────────────────────
        self.cap            = None
        self.webcam_ok      = False
        self.center_frame   = None          # (cx, cy) pixel centre

        # ── optical flow state ────────────────────────────────────────────────
        self.flow_ok        = False
        self.flow_quality   = 0.0
        self.flow_vx        = 0.0           # integrated body-frame velocity x
        self.flow_vy        = 0.0           # integrated body-frame velocity y
        self._last_flow_time = None
        self._last_range_time = None
        self.origin_range_m = None

        # ── setpoint being published ──────────────────────────────────────────
        self.sp_x   = 0.0
        self.sp_y   = 0.0
        self.sp_z   = 0.0
        self.sp_yaw = 0.0

        # ─────────────────────────────────────────────────────────────────────
        # SERVICES
        # ─────────────────────────────────────────────────────────────────────
        rospy.loginfo('[Mission] Waiting for MAVROS services ...')
        rospy.wait_for_service('/mavros/set_mode',   timeout=30)
        rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
        self._set_mode_srv = rospy.ServiceProxy('/mavros/set_mode',   SetMode)
        self._arming_srv   = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        rospy.loginfo('[Mission] MAVROS services ready')

        # ─────────────────────────────────────────────────────────────────────
        # PUBLISHERS
        # ─────────────────────────────────────────────────────────────────────
        self._sp_pub = rospy.Publisher(
            '/mavros/setpoint_position/local',
            PoseStamped,
            queue_size=10
        )

        # ─────────────────────────────────────────────────────────────────────
        # SUBSCRIBERS
        # ─────────────────────────────────────────────────────────────────────
        rospy.Subscriber('/mavros/state',                  State,          self._state_cb)
        rospy.Subscriber('/mavros/local_position/pose',    PoseStamped,    self._pose_cb)
        rospy.Subscriber('/mavros/optical_flow/raw/send',  OpticalFlowRad, self._flow_cb)
        rospy.Subscriber('/mavros/px4flow/raw/optical_flow_rad',
                 OpticalFlowRad, self._flow_cb)
        rospy.Subscriber('/mavros/distance_sensor/rangefinder_pub',
                         Range, self._range_cb)
        rospy.Subscriber('/mavros/distance_sensor/rangefinder_sub',
                 Range, self._range_cb)

        # ─────────────────────────────────────────────────────────────────────
        # INITIALISE WEBCAM
        # ─────────────────────────────────────────────────────────────────────
        self._init_webcam()

        # ─────────────────────────────────────────────────────────────────────
        # TIMERS
        # ─────────────────────────────────────────────────────────────────────
        dt = 1.0 / SETPOINT_RATE_HZ
        rospy.Timer(rospy.Duration(dt),   self._control_loop)
        rospy.Timer(rospy.Duration(0.2),  self._state_machine)   # 5 Hz
        rospy.Timer(rospy.Duration(0.1),  self._webcam_loop)     # 10 Hz

        rospy.loginfo('[Mission] Node initialised  |  %.0f Hz setpoints', SETPOINT_RATE_HZ)

    # =========================================================================
    # WEBCAM
    # =========================================================================

    def _init_webcam(self):
        rospy.loginfo('[Webcam] Opening /dev/video%d ...', WEBCAM_DEVICE)
        self.cap = cv2.VideoCapture(WEBCAM_DEVICE)
        if not self.cap.isOpened():
            rospy.logerr('[Webcam] FAILED to open camera - check /dev/video%d', WEBCAM_DEVICE)
            self.webcam_ok = False
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Warm-up: grab a few frames
        for _ in range(10):
            self.cap.read()
            time.sleep(0.05)

        ret, frame = self.cap.read()
        if not ret or frame is None:
            rospy.logerr('[Webcam] Camera opened but no frame received')
            self.webcam_ok = False
            return

        h, w = frame.shape[:2]
        self.center_frame = (w // 2, h // 2)
        self.webcam_ok    = True
        rospy.loginfo('[Webcam] OK  |  %dx%d  |  centre=%s', w, h, str(self.center_frame))

    def _webcam_loop(self, _event):
        """Grab frame, annotate centre cross-hair, display (non-blocking)."""
        if not self.webcam_ok or self.cap is None:
            return
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        cx, cy = self.center_frame
        cv2.drawMarker(frame, (cx, cy), (0, 255, 0),
                       cv2.MARKER_CROSS, 40, 2)
        cv2.putText(frame, 'STATE: %s' % self.state, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2)
        cv2.putText(frame, 'FLOW: %s' % ('OK' if self.flow_ok else 'NO'),
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 0) if self.flow_ok else (0, 0, 255), 2)
        if self.local_pose:
            alt = self.local_pose.pose.position.z - self.origin_z
            cv2.putText(frame, 'ALT: %.2fm' % alt, (10, 85),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)

        cv2.imshow('Mission Camera', frame)
        cv2.waitKey(1)

    # =========================================================================
    # CALLBACKS
    # =========================================================================

    def _state_cb(self, msg):
        self.mav_state = msg

    def _pose_cb(self, msg):
        self.local_pose = msg

    def _range_cb(self, msg):
        if not _is_finite(msg.range):
            self.rangefinder_alt = None
            self._last_range_time = None
            return
        self.rangefinder_alt = msg
        self._last_range_time = time.time()

    def _flow_cb(self, msg):
        """
        OpticalFlowRad carries integrated pixel-flow in rad.
        Quality 0-255:  255 = best.
        We use it to confirm sensor health; position hold is done
        via EKF3 local_position (which fuses optical flow internally).
        """
        self.flow_msg = msg
        self.flow_quality = msg.quality
        self.flow_ok  = (msg.quality >= FLOW_QUALITY_MIN and
                         _is_finite(msg.integrated_x) and
                         _is_finite(msg.integrated_y))
        self._last_flow_time = time.time()

    # =========================================================================
    # HELPERS
    # =========================================================================

    def _current_altitude(self):
        """Return fresh LiDAR height above ground, or None if invalid."""
        if not self._lidar_is_fresh():
            return 0.0
        return self.rangefinder_alt.range

    def _lidar_is_fresh(self):
        if self.rangefinder_alt is None or self._last_range_time is None:
            return False
        age = time.time() - self._last_range_time
        reading = self.rangefinder_alt.range
        return (age <= LIDAR_TIMEOUT_S and _is_finite(reading) and
                LIDAR_MIN_M <= reading <= LIDAR_MAX_M)

    def _flow_is_fresh(self):
        return (self.flow_ok and self._last_flow_time is not None and
                time.time() - self._last_flow_time <= FLOW_TIMEOUT_S)

    def _sensor_guard(self):
        """Abort airborne flight immediately on stale LiDAR or optical flow."""
        if self._airborne_checks_active() and not self._lidar_is_fresh():
            if self._ers is not None:
                self._ers.manual_trigger('LiDAR reading stale or invalid')
            return False
        if self._airborne_checks_active() and not self._flow_is_fresh():
            if self._ers is not None:
                self._ers.manual_trigger('Optical flow reading stale or invalid')
            return False
        return True

    def _airborne_checks_active(self):
        return self.state in (FlightState.TAKEOFF, FlightState.HOVER,
                              FlightState.RETURN, FlightState.LANDING)

    def _publish_sp(self, x, y, z, yaw=0.0):
        msg = PoseStamped()
        msg.header.stamp    = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._sp_pub.publish(msg)

    def _set_mode(self, mode_str):
        try:
            req = SetModeRequest()
            req.custom_mode = mode_str
            self._set_mode_srv(req)
            rospy.loginfo('[Mission] Mode -> %s', mode_str)
        except rospy.ServiceException as exc:
            rospy.logerr('[Mission] set_mode error: %s', str(exc))

    def _arm(self, arm_val):
        try:
            req = CommandBoolRequest()
            req.value = arm_val
            self._arming_srv(req)
            rospy.loginfo('[Mission] Arming -> %s', str(arm_val))
        except rospy.ServiceException as exc:
            rospy.logerr('[Mission] arming error: %s', str(exc))

    def _is_armed(self):
        return self.mav_state is not None and self.mav_state.armed

    def _is_offboard(self):
        return self.mav_state is not None and self.mav_state.mode == 'OFFBOARD'

    def _check_flow_and_cam(self):
        """Require fresh camera, optical-flow, and LiDAR data before arming."""
        if not self.webcam_ok:
            rospy.logwarn_throttle(3, '[Mission] Webcam not ready')
            return False
        if not self._flow_is_fresh():
            rospy.logwarn_throttle(3,
                '[Mission] Optical flow stale/low quality (quality=%.0f)',
                self.flow_quality)
            return False
        if not self._lidar_is_fresh():
            rospy.logwarn_throttle(3, '[Mission] LiDAR not ready or out of range')
            return False
        if self._shared_status is not None:
            if not self._shared_status.optical_flow_ok:
                rospy.logwarn_throttle(3, '[Mission] Calibration rejected optical flow')
                return False
            if not self._shared_status.rangefinder_ok:
                rospy.logwarn_throttle(3, '[Mission] Calibration rejected LiDAR')
                return False
        return True

    def _at_target_altitude(self):
        altitude = self._current_altitude()
        if altitude <= 0.0:
            self.altitude_band_start = None
            return False
        in_band = abs(altitude - TARGET_ALTITUDE_M) <= HOVER_TOLERANCE_M
        if not in_band:
            self.altitude_band_start = None
            return False
        if self.altitude_band_start is None:
            self.altitude_band_start = time.time()
        return time.time() - self.altitude_band_start >= ALTITUDE_STABLE_S

    # =========================================================================
    # STATE MACHINE  (5 Hz)
    # =========================================================================

    def _state_machine(self, _event):

        # ── INIT ──────────────────────────────────────────────────────────────
        if self.state == FlightState.INIT:
            if (self.mav_state is not None and
                    self.local_pose is not None):
                rospy.loginfo('[SM] Telemetry OK  ->  WAIT_READY')
                self.state = FlightState.WAIT_READY

        # ── WAIT_READY ────────────────────────────────────────────────────────
        elif self.state == FlightState.WAIT_READY:
            if not self._check_flow_and_cam():
                return

            if (self.mav_state is not None and
                    not self._is_armed()):
                # Capture ground origin
                self.origin_x = self.local_pose.pose.position.x
                self.origin_y = self.local_pose.pose.position.y
                self.origin_z = self.local_pose.pose.position.z
                self.origin_range_m = self.rangefinder_alt.range

                rospy.loginfo('[SM] Origin locked  X=%.3f  Y=%.3f  Z=%.3f',
                              self.origin_x, self.origin_y, self.origin_z)
                rospy.loginfo('[SM] Webcam center: %s', str(self.center_frame))
                rospy.loginfo('[SM] Optical flow : OK (quality=%.0f)',
                              self.flow_quality)
                rospy.loginfo('[SM] LiDAR height : %.2fm', self.origin_range_m)
                rospy.loginfo('[SM] -> OFFBOARD_REQ')

                self.sp_x   = self.origin_x
                self.sp_y   = self.origin_y
                self.sp_z   = self.origin_z
                self.pid_x.reset()
                self.pid_y.reset()
                self.state  = FlightState.OFFBOARD_REQ

        # ── OFFBOARD_REQ ──────────────────────────────────────────────────────
        elif self.state == FlightState.OFFBOARD_REQ:
            # Need >= 20 setpoints streamed before switching mode
            if self.offboard_count >= int(SETPOINT_RATE_HZ):
                if not self.offboard_sent:
                    self._set_mode('OFFBOARD')
                    self.offboard_sent = True

                if self._is_offboard():
                    rospy.loginfo('[SM] OFFBOARD active  ->  ARM_REQ')
                    self.state = FlightState.ARM_REQ

        # ── ARM_REQ ───────────────────────────────────────────────────────────
        elif self.state == FlightState.ARM_REQ:
            if not self.arm_sent:
                self._arm(True)
                self.arm_sent = True
                # Notify ERS that arming has been requested
                if self._ers is not None:
                    self._ers.set_airborne(False)
                    self._ers.enable_watchdog()

            if self._is_armed():
                rospy.loginfo('[SM] ARMED  ->  TAKEOFF')
                if self._ers is not None:
                    self._ers.set_airborne(True)
                self.state = FlightState.TAKEOFF

        # ── TAKEOFF ───────────────────────────────────────────────────────────
        elif self.state == FlightState.TAKEOFF:
            if not self._sensor_guard():
                return
            alt = self._current_altitude()
            rospy.loginfo_throttle(1,
                '[SM] Takeoff %.2f / %.2fm', alt, TARGET_ALTITUDE_M)

            if self._at_target_altitude():
                self.hover_start = rospy.Time.now()
                self.pid_x.reset()
                self.pid_y.reset()
                rospy.loginfo('[SM] Altitude reached  ->  HOVER')
                self.state = FlightState.HOVER

        # ── HOVER ─────────────────────────────────────────────────────────────
        elif self.state == FlightState.HOVER:
            if not self._sensor_guard():
                return
            elapsed = (rospy.Time.now() - self.hover_start).to_sec()
            rospy.loginfo_throttle(2,
                '[SM] Hover %.1f / %.1fs  |  flow=%s',
                elapsed, HOVER_DURATION_S,
                'OK' if self.flow_ok else 'WARN')

            if elapsed >= HOVER_DURATION_S:
                self.return_start = time.time()
                rospy.loginfo('[SM] Hover complete  ->  RETURN')
                self.state = FlightState.RETURN

        # ── RETURN ────────────────────────────────────────────────────────────
        elif self.state == FlightState.RETURN:
            if not self._sensor_guard():
                return
            dx = self.origin_x - self.local_pose.pose.position.x
            dy = self.origin_y - self.local_pose.pose.position.y
            drift = math.sqrt(dx**2 + dy**2)
            rospy.loginfo_throttle(1, '[SM] Returning to origin  drift=%.2fm', drift)
            if drift <= RETURN_TOLERANCE_M:
                rospy.loginfo('[SM] Origin reacquired  ->  LANDING')
                self.state = FlightState.LANDING
            elif time.time() - self.return_start > RETURN_TIMEOUT_S:
                if self._ers is not None:
                    self._ers.manual_trigger(
                        'Could not return to takeoff origin within %.0fs'
                        % RETURN_TIMEOUT_S)

        # ── LANDING ───────────────────────────────────────────────────────────
        elif self.state == FlightState.LANDING:
            if not self._sensor_guard():
                return
            if not self.land_sent:
                self._set_mode('AUTO.LAND')
                self.land_sent = True
                rospy.loginfo('[SM] AUTO.LAND commanded')

            alt = self._current_altitude()
            rospy.loginfo_throttle(1, '[SM] Landing  alt=%.2fm', alt)

            landed = self._lidar_is_fresh() and alt < LAND_DETECT_ALT_M
            self.touchdown_count = (self.touchdown_count + 1 if landed else 0)

            if self.touchdown_count >= TOUCHDOWN_SAMPLES:
                rospy.loginfo('[SM] Touchdown  ->  DONE')
                if self._ers is not None:
                    self._ers.set_airborne(False)
                self._arm(False)
                self.state = FlightState.DONE

        # ── DONE ──────────────────────────────────────────────────────────────
        elif self.state == FlightState.DONE:
            rospy.loginfo('[SM] *** MISSION COMPLETE ***')
            if self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()
            rospy.signal_shutdown('Mission complete')

    # =========================================================================
    # CONTROL LOOP  (SETPOINT_RATE_HZ)
    # =========================================================================

    def _control_loop(self, _event):
        """
        Publish offboard position setpoints every tick.
        Kicks ERS watchdog on every cycle when ERS is attached.
        During HOVER: apply optical-flow-backed PID correction so the
        drone returns to (origin_x, origin_y) if it drifts.
        """
        # Kick ERS watchdog every cycle — E11 fires if this stops
        if self._ers is not None:
            self._ers.kick_watchdog()

        if self.local_pose is None:
            return

        now = rospy.Time.now().to_sec()

        # ── Pre-offboard: hold ground ─────────────────────────────────────────
        if self.state in (FlightState.WAIT_READY,
                          FlightState.OFFBOARD_REQ,
                          FlightState.ARM_REQ):
            self._publish_sp(self.origin_x, self.origin_y, self.origin_z)
            self.offboard_count += 1
            return

        # ── Takeoff: climb to TARGET_ALTITUDE, hold origin XY ─────────────────
        if self.state == FlightState.TAKEOFF:
            target_z = self.origin_z + TARGET_ALTITUDE_M
            self._publish_sp(self.origin_x, self.origin_y, target_z)
            return

        # ── Return: hold target altitude while moving back to origin ──────────
        if self.state == FlightState.RETURN:
            target_z = self.origin_z + TARGET_ALTITUDE_M
            self._publish_sp(self.origin_x, self.origin_y, target_z)
            return

        # ── Hover: hold altitude + optical-flow drift correction ──────────────
        if self.state == FlightState.HOVER:
            target_z = self.origin_z + TARGET_ALTITUDE_M

            cur_x = self.local_pose.pose.position.x
            cur_y = self.local_pose.pose.position.y

            err_x = self.origin_x - cur_x
            err_y = self.origin_y - cur_y

            drift_h = math.sqrt(err_x**2 + err_y**2)

            if drift_h > MAX_DRIFT_M:
                vx_corr = self.pid_x.compute(err_x, now)
                vy_corr = self.pid_y.compute(err_y, now)
                corrected_x = cur_x + vx_corr * (1.0 / SETPOINT_RATE_HZ)
                corrected_y = cur_y + vy_corr * (1.0 / SETPOINT_RATE_HZ)
                rospy.logwarn_throttle(1,
                    '[Hover] Drift %.2fm  ->  correcting', drift_h)
                self._publish_sp(corrected_x, corrected_y, target_z)
            else:
                self.pid_x.reset()
                self.pid_y.reset()
                self._publish_sp(self.origin_x, self.origin_y, target_z)
            return

        # ── All other states: hold last known position ─────────────────────────
        self._publish_sp(
            self.local_pose.pose.position.x,
            self.local_pose.pose.position.y,
            self.local_pose.pose.position.z
        )


# =============================================================================
# MODULE ENTRY POINT  —  called by final.py
# =============================================================================

def run(shared_status=None, ers=None):
    """
    Instantiate and start the flight node.
    Called by final.py after calibration and ERS are already running.
    rospy.init_node() must already have been called by final.py.
    """
    rospy.loginfo('[Flight] Starting autonomous flight node ...')
    node = FinalMissionNode(shared_status=shared_status, ers=ers)
    rospy.loginfo('[Flight] Autonomous flight node ready')
    return node


# =============================================================================
# STANDALONE ENTRY POINT  —  run this file directly for testing
# =============================================================================

if __name__ == '__main__':
    rospy.init_node('autonomous_flight_standalone')
    rospy.loginfo('=' * 65)
    rospy.loginfo('autonomous_flight.py  |  standalone mode')
    rospy.loginfo('Take-off %.1fm  |  Hover %ds  |  Land',
                  TARGET_ALTITUDE_M, int(HOVER_DURATION_S))
    rospy.loginfo('=' * 65)
    try:
        node = FinalMissionNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('[Mission] Interrupted')
    finally:
        cv2.destroyAllWindows()