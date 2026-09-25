# -*- coding: utf-8 -*-
#!/usr/bin/env python
"""
Simple webcam visual odometry test.

Run from a laptop with:
    python vio.py

The annotated camera view is shown in an OpenCV window. Press q or Esc to
quit, or use --camera to select another camera index.
"""

import argparse
import cv2
import numpy as np
import threading
import time
import math
import logging
import sys

# =============================================================================
# CONFIG
# =============================================================================

CAMERA_DEVICE         = 0        # Laptop's default camera
FRAME_W               = 640
FRAME_H               = 480
FRAME_FPS             = 30

MAX_FEATURES          = 300
FEATURE_QUALITY       = 0.01
FEATURE_MIN_DIST      = 10
LK_WIN_SIZE           = (21, 21)
LK_MAX_LEVEL          = 3
LK_CRITERIA           = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
REDETECT_INTERVAL     = 30
MIN_FEATURES_FOR_LOCK = 20

# Approximate focal length for a generic 640x480 USB webcam
FOCAL_PX              = 600.0

LOGGER                = logging.getLogger('vio')


# =============================================================================
# OPEN CAMERA
# =============================================================================

def _open_camera(camera_device):
    """Open a local webcam using OpenCV's platform camera backend."""
    backend = cv2.CAP_ANY
    if sys.platform.startswith('win') and hasattr(cv2, 'CAP_DSHOW'):
        backend = cv2.CAP_DSHOW

    LOGGER.info('Opening camera %d ...', camera_device)
    cap = cv2.VideoCapture(camera_device, backend)
    if not cap.isOpened():
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, FRAME_FPS)
    return cap


# =============================================================================
# VIO ENGINE
# =============================================================================

class VIOEngine(object):

    def __init__(self):
        self._lock          = threading.Lock()
        self._prev_gray     = None
        self._prev_pts      = None
        self._frame_count   = 0
        self._origin_locked = False
        self._drift_px_x    = 0.0
        self._drift_px_y    = 0.0
        self.altitude_m     = 1.0

    def set_altitude(self, alt_m):
        with self._lock:
            self.altitude_m = max(0.1, alt_m)

    def is_locked(self):
        with self._lock:
            return self._origin_locked

    def get_drift(self):
        with self._lock:
            dx    = self._drift_px_x
            dy    = self._drift_px_y
            scale = self.altitude_m / FOCAL_PX
            dm    = math.sqrt(dx**2 + dy**2) * scale
        return dx, dy, dm

    def reset_drift(self):
        with self._lock:
            self._drift_px_x = 0.0
            self._drift_px_y = 0.0

    def process_frame(self, frame_bgr):
        gray      = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        annotated = frame_bgr.copy()
        h, w      = gray.shape
        cx, cy    = w // 2, h // 2

        with self._lock:
            self._frame_count += 1

            need_detect = (
                self._prev_pts is None or
                self._prev_gray is None or
                self._frame_count % REDETECT_INTERVAL == 0 or
                (self._prev_pts is not None and
                 len(self._prev_pts) < MIN_FEATURES_FOR_LOCK // 2)
            )

            if need_detect:
                pts = cv2.goodFeaturesToTrack(
                    gray,
                    maxCorners   = MAX_FEATURES,
                    qualityLevel = FEATURE_QUALITY,
                    minDistance  = FEATURE_MIN_DIST,
                    blockSize    = 7
                )
                self._prev_pts  = pts
                self._prev_gray = gray.copy()
                cv2.drawMarker(annotated, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 40, 2)
                cv2.putText(annotated, 'VIO: SEARCHING',
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2)
                return annotated

            next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                self._prev_gray, gray,
                self._prev_pts, None,
                winSize  = LK_WIN_SIZE,
                maxLevel = LK_MAX_LEVEL,
                criteria = LK_CRITERIA
            )

            good_prev = self._prev_pts[status == 1]
            good_next = next_pts[status == 1]

            if len(good_next) >= 4:
                mean_flow = (good_next - good_prev).mean(axis=0)
            else:
                mean_flow = np.array([0.0, 0.0])

            if not self._origin_locked:
                if len(good_next) >= MIN_FEATURES_FOR_LOCK:
                    self._origin_locked = True
                    self._drift_px_x    = 0.0
                    self._drift_px_y    = 0.0
                    LOGGER.info('Origin locked at centre (%d, %d)', cx, cy)

            if self._origin_locked:
                self._drift_px_x += mean_flow[0]
                self._drift_px_y += mean_flow[1]

            self._prev_pts  = good_next.reshape(-1, 1, 2)
            self._prev_gray = gray.copy()

            dx     = self._drift_px_x
            dy     = self._drift_px_y
            locked = self._origin_locked
            alt    = self.altitude_m

        # -- annotate outside lock --
        for pt in good_next:
            cv2.circle(annotated, tuple(pt.astype(int)), 3, (0, 255, 100), -1)
        for p, n in zip(good_prev, good_next):
            cv2.arrowedLine(annotated,
                            tuple(p.astype(int)), tuple(n.astype(int)),
                            (200, 200, 0), 1, tipLength=0.4)

        # centre crosshair
        cv2.drawMarker(annotated, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 40, 2)

        # drift arrow
        if locked and (abs(dx) > 1 or abs(dy) > 1):
            cur_x = max(5, min(w - 5, int(cx + dx)))
            cur_y = max(5, min(h - 5, int(cy + dy)))
            cv2.arrowedLine(annotated, (cx, cy), (cur_x, cur_y),
                            (0, 0, 255), 2, tipLength=0.15)
            cv2.circle(annotated, (cur_x, cur_y), 6, (0, 0, 255), 2)

        status_str = 'VIO: LOCKED' if locked else 'VIO: SEARCHING'
        cv2.putText(annotated, status_str,
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2)
        cv2.putText(annotated, 'FEATURES: %d' % len(good_next),
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (100, 255, 100), 2)

        if locked:
            dm     = math.sqrt(dx**2 + dy**2) * (alt / FOCAL_PX)
            colour = (0, 80, 255) if dm > 0.05 else (0, 220, 0)
            cv2.putText(annotated,
                        'DRIFT px=(%.1f,%.1f) m=%.3f' % (dx, dy, dm),
                        (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)

        return annotated


# =============================================================================
# STANDALONE CAMERA LOOP
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description='Webcam visual odometry demo')
    parser.add_argument('--camera', type=int, default=CAMERA_DEVICE,
                        help='camera index (default: %(default)s)')
    parser.add_argument('--altitude', type=float, default=1.0,
                        help='camera height in metres (default: %(default)s)')
    parser.add_argument('--no-display', action='store_true',
                        help='process frames without opening a camera window')
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    cap = _open_camera(args.camera)
    if cap is None:
        LOGGER.error('Could not open camera %d. Try --camera 1 or another index.',
                     args.camera)
        return 1

    vio = VIOEngine()
    vio.set_altitude(args.altitude)
    last_report = 0.0
    LOGGER.info('Running. Press q or Esc in the camera window to quit.')

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None or frame.size == 0:
                LOGGER.warning('Empty frame skipped')
                continue

            annotated = vio.process_frame(frame)
            now = time.time()
            if vio.is_locked() and now - last_report >= 2.0:
                dx, dy, dm = vio.get_drift()
                LOGGER.info('Drift: x=%.1f px, y=%.1f px, distance=%.4f m',
                            dx, dy, dm)
                last_report = now

            if not args.no_display:
                cv2.imshow('VIO - Laptop Camera', annotated)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break
    except KeyboardInterrupt:
        LOGGER.info('Stopping')
    finally:
        cap.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())