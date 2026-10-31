"""Camera-based yaw reference: stop the compass heading from creeping.

Problem: in Loiter the autopilot holds whatever heading the EKF believes. If the
compass reading creeps (needle settling, magnetic field from the power wires
changing as the battery sags and current rises, temperature), the EKF heading
creeps too and the drone slowly yaws to "hold" a heading that is moving.

Fix: a second camera looking at the world (ideally straight down at the ground)
measures how much the drone ACTUALLY rotates. Magnetism can't touch that. Then:

  heading = previous heading + rotation the camera saw          (short/medium term)
            + slow pull toward the compass, time constant tau_s   (long term, absolute)

In hold_modes (Loiter, PosHold, Brake) the compass pull is switched off entirely
while the camera is tracking, so the heading is held as solid as the camera can
see. The gap between the compass and this heading is the compass DRIFT; it is
logged and reported to the ground station.

If the camera loses tracking (night, water, featureless ground), the heading is
carried by the FC's gyro rate and blended back to the compass over tau_lost_s.
No step, no lag.

VisualYawTracker
  KLT feature tracking against a keyframe (not frame-to-frame integration), so a
  hovering drone can hold one keyframe for minutes with sub-degree error. RANSAC
  rejects moving things. Each rotation is cross-checked against the FC gyro.
"""

import logging
import math

import cv2
import numpy as np

from .heading import wrap180, wrap360

log = logging.getLogger(__name__)


class VisualYawSample(object):
    __slots__ = ("valid", "total_deg", "segment", "inliers", "reason", "t")

    def __init__(self, valid=False, total_deg=None, segment=0, inliers=0, reason="", t=0.0):
        self.valid, self.total_deg, self.segment = valid, total_deg, segment
        self.inliers, self.reason, self.t = inliers, reason, t


class VisualYawTracker(object):
    """Cumulative clockwise yaw rotation (deg) of the drone, seen by a world-facing camera.

    `segment` increases whenever tracking was lost: rotation is only comparable
    between samples of the same segment.
    """

    def __init__(self, orientation="down", hfov_deg=62.0, invert=False, roi=(0.0, 0.0, 1.0, 1.0),
                 process_width=640, max_features=250, min_features=60, min_inliers=25,
                 min_inlier_ratio=0.6, max_gyro_mismatch_dps=15.0):
        if orientation not in ("down", "forward"):
            raise ValueError("visual_yaw.orientation must be 'down' or 'forward'")
        self.orientation = orientation
        self.hfov = math.radians(float(hfov_deg))
        self.sign = -1.0 if invert else 1.0
        self.roi = [float(v) for v in roi]
        self.process_width = int(process_width)
        self.max_features = int(max_features)
        self.min_features = int(min_features)
        self.min_inliers = int(min_inliers)
        self.min_inlier_ratio = float(min_inlier_ratio)
        self.max_gyro_mismatch = float(max_gyro_mismatch_dps)

        self.total = 0.0
        self.segment = 0
        self._prev_gray = None
        self._key_pts = None   # feature positions in the keyframe (centred coords)
        self._cur_pts = None   # same features, tracked into the latest frame (pixel coords)
        self._key_total = 0.0
        self._last_t = None
        self._mask = None
        self._center = None
        self._f = None
        self._sign_votes = 0
        self.debug_points = None

    @classmethod
    def from_config(cls, vcfg):
        return cls(orientation=vcfg.get("orientation", "down"), hfov_deg=vcfg.get("hfov_deg", 62.0),
                   invert=vcfg.get("invert", False), roi=vcfg.get("roi", [0, 0, 1, 1]),
                   min_features=vcfg.get("min_features", 60), min_inliers=vcfg.get("min_inliers", 25),
                   max_gyro_mismatch_dps=vcfg.get("max_gyro_mismatch_dps", 15.0))

    # ------------------------------------------------------------------ helpers
    def _prep(self, frame):
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if gray.shape[1] > self.process_width:
            s = self.process_width / float(gray.shape[1])
            gray = cv2.resize(gray, (self.process_width, int(round(gray.shape[0] * s))), interpolation=cv2.INTER_AREA)
        if self._mask is None or self._mask.shape != gray.shape:
            h, w = gray.shape
            x0, y0, x1, y1 = self.roi
            self._mask = np.zeros_like(gray)
            self._mask[int(y0 * h):int(y1 * h), int(x0 * w):int(x1 * w)] = 255
            self._center = np.float32([(w - 1) / 2.0, (h - 1) / 2.0])
            self._f = (w / 2.0) / math.tan(self.hfov / 2.0)
        return gray

    def _new_keyframe(self, gray):
        pts = cv2.goodFeaturesToTrack(gray, maxCorners=self.max_features, qualityLevel=0.01,
                                      minDistance=8, mask=self._mask, blockSize=7)
        if pts is None or len(pts) < self.min_inliers:
            self._key_pts = self._cur_pts = None
            self.segment += 1  # continuity broken: rotation until the next keyframe is unknown
            return False
        self._cur_pts = pts.reshape(-1, 2).astype(np.float32)
        self._key_pts = self._cur_pts - self._center
        self._key_total = self.total
        return True

    def _lose(self, gray, reason, t):
        self._prev_gray = gray
        self._key_pts = self._cur_pts = None
        self.segment += 1
        return VisualYawSample(False, reason=reason, segment=self.segment, t=t)

    def _yaw_from(self, M):
        if self.orientation == "down":
            # image rotation; the ground turns the opposite way to the drone
            return -math.degrees(math.atan2(M[1, 0], M[0, 0]))
        # forward camera: yaw shows up as sideways shift of the scene
        return -math.degrees(math.atan2(M[0, 2], self._f))

    # ------------------------------------------------------------------- update
    def update(self, frame, t, gyro_dps=None):
        gray = self._prep(frame)
        if self._prev_gray is None or self._key_pts is None:
            self._prev_gray = gray
            ok = self._new_keyframe(gray)
            self._last_t = t
            if not ok:
                return VisualYawSample(False, reason="too_few_features", segment=self.segment, t=t)
            return VisualYawSample(True, self.total, self.segment, len(self._key_pts), "keyframe", t)

        lk = dict(winSize=(21, 21), maxLevel=3,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))
        p0 = self._cur_pts.reshape(-1, 1, 2)
        p1, st1, _ = cv2.calcOpticalFlowPyrLK(self._prev_gray, gray, p0, None, **lk)
        p0r, st2, _ = cv2.calcOpticalFlowPyrLK(gray, self._prev_gray, p1, None, **lk)
        fb_err = np.linalg.norm((p0r - p0).reshape(-1, 2), axis=1)
        p1 = p1.reshape(-1, 2)
        h, w = gray.shape
        xi = np.clip(p1[:, 0], 0, w - 1).astype(int)
        yi = np.clip(p1[:, 1], 0, h - 1).astype(int)
        good = (st1.ravel() == 1) & (st2.ravel() == 1) & (fb_err < 1.0) & (self._mask[yi, xi] > 0)
        self._key_pts, self._cur_pts = self._key_pts[good], p1[good]
        if len(self._cur_pts) < self.min_inliers:
            return self._lose(gray, "lost_tracking", t)

        M, inl = cv2.estimateAffinePartial2D(self._key_pts, self._cur_pts - self._center,
                                             method=cv2.RANSAC, ransacReprojThreshold=1.5,
                                             maxIters=500, confidence=0.995)
        n_in = int(inl.sum()) if inl is not None else 0
        if M is None or n_in < self.min_inliers or n_in < self.min_inlier_ratio * len(self._cur_pts):
            return self._lose(gray, "inconsistent_motion", t)

        total = self._key_total + self.sign * self._yaw_from(M)
        dt = max(1e-3, t - self._last_t)
        rate = wrap180(total - self.total) / dt
        if gyro_dps is not None and abs(rate - gyro_dps) > max(self.max_gyro_mismatch, 0.5 * abs(gyro_dps)):
            if abs(gyro_dps) > 10.0 and abs(rate + gyro_dps) < self.max_gyro_mismatch:
                self._sign_votes += 1
                if self._sign_votes == 15:
                    log.warning("visual yaw turns the opposite way to the gyro - set visual_yaw.invert: true")
            return self._lose(gray, "gyro_mismatch", t)

        self.total = total
        self._last_t = t
        self._prev_gray = gray
        keep = inl.ravel().astype(bool)
        self._key_pts, self._cur_pts = self._key_pts[keep], self._cur_pts[keep]
        self.debug_points = self._cur_pts
        if len(self._cur_pts) < self.min_features:
            self._new_keyframe(gray)  # refresh before tracking runs out; total carries over
        return VisualYawSample(True, self.total, self.segment, n_in, "", t)

    def debug_image(self, frame, height=400):
        img = frame if frame.ndim == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        s = float(height) / img.shape[0]
        img = cv2.resize(img, (int(img.shape[1] * s), height))
        if self.debug_points is not None and self._prev_gray is not None:
            k = img.shape[1] / float(self._prev_gray.shape[1])
            for x, y in self.debug_points:
                cv2.circle(img, (int(x * k), int(y * k)), 2, (0, 255, 0), -1)
        return img


class StabilizedHeading(object):
    __slots__ = ("valid", "heading_deg", "drift_deg", "holding", "source")

    def __init__(self, valid=False, heading_deg=None, drift_deg=None, holding=False, source=""):
        self.valid, self.heading_deg, self.drift_deg = valid, heading_deg, drift_deg
        self.holding, self.source = holding, source


class HeadingStabilizer(object):
    """Complementary filter: camera rotation (or gyro) for change, compass for absolute."""

    def __init__(self, tau_s=60.0, tau_lost_s=3.0, hold_modes=("LOITER", "POSHOLD", "BRAKE"), max_vis_age_s=0.2):
        self.tau = float(tau_s)
        self.tau_lost = float(tau_lost_s)
        self.hold_modes = set(str(m).upper() for m in (hold_modes or []))
        self.max_vis_age = float(max_vis_age_s)
        self.h = None
        self._t = None
        self._last_vis = None  # (segment, total_deg)

    @classmethod
    def from_config(cls, vcfg):
        return cls(tau_s=vcfg.get("tau_s", 60.0), tau_lost_s=vcfg.get("tau_lost_s", 3.0),
                   hold_modes=vcfg.get("hold_modes", ["LOITER", "POSHOLD", "BRAKE"]))

    def update(self, t, compass_deg, visual=None, mode=None, gyro_dps=None):
        vis_ok = visual is not None and visual.valid and (t - visual.t) <= self.max_vis_age
        dt = 0.0 if self._t is None else max(0.0, t - self._t)
        self._t = t

        if self.h is None:
            self._last_vis = (visual.segment, visual.total_deg) if vis_ok else None
            if compass_deg is None:
                return StabilizedHeading()
            self.h = compass_deg
            return StabilizedHeading(True, self.h, 0.0, False, "compass")

        # 1. propagate: how much did the drone turn since last time?
        if vis_ok:
            if self._last_vis is not None and self._last_vis[0] == visual.segment:
                self.h = wrap360(self.h + wrap180(visual.total_deg - self._last_vis[1]))
            self._last_vis = (visual.segment, visual.total_deg)
            source = "camera"
        else:
            self._last_vis = None
            if gyro_dps is not None:
                self.h = wrap360(self.h + gyro_dps * dt)
                source = "gyro"
            else:
                source = "compass"

        # 2. correct toward the compass (absolute reference), unless holding
        holding = vis_ok and mode is not None and str(mode).upper() in self.hold_modes
        drift = None
        if compass_deg is not None:
            drift = wrap180(compass_deg - self.h)
            if source == "compass":
                self.h = compass_deg  # nothing else to go on
            elif not holding:
                tau = self.tau if vis_ok else self.tau_lost
                self.h = wrap360(self.h + (1.0 - math.exp(-dt / tau)) * drift)
        valid = vis_ok or compass_deg is not None
        return StabilizedHeading(valid, self.h if valid else None, drift, holding, source)
