#!/usr/bin/env python
"""
final.py
========
IRoC-U 2026  |  ROS 1 Melodic  |  Python 2.7
Entry point launched by comm2.py on START command.

Execution order
---------------
  1. sensor_calibration  — blocking preflight check (all sensors must pass)
  2. emergency_response  — background 10 Hz safety monitor (runs for entire mission)
  3. autonomous_flight   — takeoff 4 m  ->  hover 30 s  ->  AUTO.LAND
"""

import rospy
import sys

import Sensor_check as sensor_calibration
import Emergency_Failsafe as emergency_response
import Autonomous_Flight as autonomous_flight

PREFLIGHT_TIMEOUT_S = 90.0

# Module-level reference so the emergency callback can reach ers
ers = None


def _notify_gcs(event):
    """Write an alert marker for comm2.py to forward over TCP."""
    message = str(event).replace('\n', ' ').replace('\r', ' ')
    print('EMERGENCY_ALERT:{}'.format(message))
    sys.stdout.flush()


def _notify_calibration(message, level='INFO'):
    """Forward calibration logs; emit a machine-readable approval marker."""
    print('CALIBRATION:{}:{}'.format(level, message))
    sys.stdout.flush()


def _on_emergency(event):
    """
    Called by EmergencyResponseSystem immediately on any fault (E01-E15).
    AUTO.LAND has already been commanded by ERS before this fires.
    Block until touchdown is confirmed, then shut down cleanly.
    """
    rospy.logerr('[final.py] EMERGENCY: %s', str(event))
    ers.print_event_log()
    ers.wait_for_landing(timeout_s=60.0)
    rospy.signal_shutdown('Emergency landing complete')


def main():
    rospy.init_node('final_mission_node')

    rospy.loginfo('=' * 60)
    rospy.loginfo('IRoC-U 2026  |  ROS 1 Melodic  |  final.py')
    rospy.loginfo('=' * 60)

    # ── STEP 1: Sensor Calibration ─────────────────────────────────────────────
    rospy.loginfo('[final.py] STEP 1 — Sensor Calibration')

    cal = sensor_calibration.SensorCalibrationNode(
        standalone=False,
        log_callback=_notify_calibration)

    ready = cal.calibrate_and_check(timeout_s=PREFLIGHT_TIMEOUT_S)
    cal.print_full_report()

    if not ready:
        rospy.logerr('[final.py] STEP 1 FAILED — preflight not passed, aborting')
        print('NOT_READY')
        sys.stdout.flush()
        sys.exit(1)

    rospy.loginfo('[final.py] STEP 1 PASSED — all sensors healthy')
    print('GOOD_TO_GO')
    sys.stdout.flush()

    # ── STEP 2: Emergency Response System ──────────────────────────────────────
    rospy.loginfo('[final.py] STEP 2 — Emergency Response System')

    global ers
    ers = emergency_response.EmergencyResponseSystem(
        shared_status   = cal.status,
        on_emergency_cb = _on_emergency,
        alert_callback  = _notify_gcs,
        standalone      = False
    )

    rospy.loginfo('[final.py] STEP 2 OK — ERS active at 10 Hz')

    # ── STEP 3: Autonomous Flight ───────────────────────────────────────────────
    rospy.loginfo('[final.py] STEP 3 — Autonomous Flight')

    autonomous_flight.run(shared_status=cal.status, ers=ers)

    rospy.loginfo('[final.py] All systems live — spinning')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo('[final.py] Interrupted')
    except rospy.ROSInterruptException:
        pass