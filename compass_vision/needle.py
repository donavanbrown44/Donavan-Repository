"""Find the compass needle in a camera frame and measure its angle.

Pipeline per frame:
  1. Warp the frame to a top-down, nose-up view of the dial using the 4 rim points
     from calibration (fixes an oblique camera, rotation and mirroring).
  2. Threshold the colour of the needle's NORTH end (red by default). Using the
     coloured end is what tells north from south - a plain line would have a
     180 degree ambiguity.
  3. Keep only pixels inside a ring (drops the pivot hub and bezel markings), keep
     the biggest blob.
  4. The blob's long axis gives the needle direction; which end of the axis is
     "north" comes from where the blob sits relative to the centre.

Angle convention: degrees clockwise from the drone's nose, as seen from above.
That is the bearing of magnetic north relative to the nose.
"""

import math

import cv2
import numpy as np

CANVAS = 400            # rectified view is CANVAS x CANVAS pixels
RIM_FRACTION = 0.45     # dial rim radius as a fraction of CANVAS


class NeedleReading(object):
    __slots__ = ("valid", "angle_deg", "confidence", "area_frac", "concentration",
                 "radial_extent", "reason", "rectified", "mask")

    def __init__(self, valid=False, angle_deg=None, confidence=0.0, area_frac=0.0,
                 concentration=0.0, radial_extent=0.0, reason="", rectified=None, mask=None):
        self.valid = valid
        self.angle_deg = angle_deg
        self.confidence = confidence
        self.area_frac = area_frac
        self.concentration = concentration
        self.radial_extent = radial_extent
        self.reason = reason
        self.rectified = rectified
        self.mask = mask


def rim_targets(canvas=CANVAS):
    """Where the nose/right/tail/left rim points land in the rectified view."""
    c = canvas / 2.0
    r = RIM_FRACTION * canvas
    return np.array([[c, c - r], [c + r, c], [c, c + r], [c - r, c]], dtype=np.float32)


class NeedleDetector(object):
    def __init__(self, dial_points, hsv_ranges, annulus=(0.15, 0.85),
                 min_area_frac=0.002, max_area_frac=0.25, min_radial_extent=0.25,
                 min_concentration=0.90, canvas=CANVAS):
        if dial_points is None or len(dial_points) != 4:
            raise ValueError("dial.points must hold 4 [x, y] image points (nose, right, "
                             "tail, left). Run: python3 -m compass_vision.calibrate setup")
        self.canvas = int(canvas)
        self.center = self.canvas / 2.0
        self.radius = RIM_FRACTION * self.canvas
        src = np.array(dial_points, dtype=np.float32).reshape(4, 2)
        self.H = cv2.getPerspectiveTransform(src, rim_targets(self.canvas))

        ys, xs = np.mgrid[0:self.canvas, 0:self.canvas].astype(np.float32)
        self._dx = xs - self.center
        self._dy = ys - self.center
        self._rr = np.hypot(self._dx, self._dy)
        self.annulus = (float(annulus[0]), float(annulus[1]))
        ring = (self._rr >= self.annulus[0] * self.radius) & (self._rr <= self.annulus[1] * self.radius)
        self._ring = ring.astype(np.uint8) * 255
        self._ring_area = float(np.count_nonzero(self._ring))
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

        self.set_hsv_ranges(hsv_ranges)
        self.min_area_frac = float(min_area_frac)
        self.max_area_frac = float(max_area_frac)
        self.min_radial_extent = float(min_radial_extent)
        self.min_concentration = float(min_concentration)

    @classmethod
    def from_config(cls, cfg, dial_points=None, hsv_ranges=None):
        n = cfg["needle"]
        return cls(dial_points if dial_points is not None else cfg["dial"]["points"],
                   hsv_ranges if hsv_ranges is not None else n["hsv_ranges"],
                   annulus=cfg["dial"]["annulus"],
                   min_area_frac=n["min_area_frac"],
                   max_area_frac=n["max_area_frac"],
                   min_radial_extent=n["min_radial_extent"],
                   min_concentration=n["min_concentration"])

    def set_hsv_ranges(self, hsv_ranges):
        self.hsv_ranges = []
        for r in hsv_ranges:
            if len(r) != 6:
                raise ValueError("each needle.hsv_ranges entry needs 6 numbers: %r" % (r,))
            self.hsv_ranges.append((np.array(r[:3], dtype=np.uint8), np.array(r[3:], dtype=np.uint8)))

    def rectify(self, frame):
        return cv2.warpPerspective(frame, self.H, (self.canvas, self.canvas), flags=cv2.INTER_LINEAR)

    def color_mask(self, rectified):
        hsv = cv2.cvtColor(rectified, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in self.hsv_ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
        mask = cv2.bitwise_and(mask, self._ring)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)

    def detect(self, frame):
        rect = self.rectify(frame)
        mask = self.color_mask(rect)
        out = NeedleReading(rectified=rect, mask=mask)

        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if n <= 1:
            out.reason = "no_needle_pixels"
            return out
        k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        sel = labels == k
        dx = self._dx[sel]
        dy = self._dy[sel]
        rr = self._rr[sel]

        out.area_frac = float(stats[k, cv2.CC_STAT_AREA]) / self._ring_area
        out.radial_extent = float(rr.max() - rr.min()) / self.radius
        mx, my = float(dx.sum()), float(dy.sum())
        out.concentration = math.hypot(mx, my) / max(float(rr.sum()), 1e-6)

        if out.area_frac < self.min_area_frac:
            out.reason = "blob_too_small"
            return out
        if out.area_frac > self.max_area_frac:
            out.reason = "blob_too_big"
            return out
        if out.radial_extent < self.min_radial_extent:
            out.reason = "blob_not_needle_shaped"
            return out
        if out.concentration < self.min_concentration:
            out.reason = "blob_spread_out"
            return out

        # Direction from the dial centre to the blob (depends on a good centre).
        centroid_angle = math.degrees(math.atan2(mx, -my)) % 360.0
        angle = centroid_angle
        # Long axis of the blob (independent of the centre estimate). Use it when the
        # blob is clearly elongated, picking the end that points toward the blob.
        cx, cy = float(dx.mean()), float(dy.mean())
        ex, ey = dx - cx, dy - cy
        sxx, syy, sxy = float((ex * ex).mean()), float((ey * ey).mean()), float((ex * ey).mean())
        tr, det = sxx + syy, sxx * syy - sxy * sxy
        disc = math.sqrt(max(tr * tr / 4.0 - det, 0.0))
        l1, l2 = tr / 2.0 + disc, max(tr / 2.0 - disc, 1e-9)
        if l1 / l2 >= 9.0:  # long axis at least 3x the short axis
            theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)  # image-frame axis angle
            axis_angle = math.degrees(math.atan2(math.cos(theta), -math.sin(theta))) % 360.0
            if abs((axis_angle - centroid_angle + 180.0) % 360.0 - 180.0) > 90.0:
                axis_angle = (axis_angle + 180.0) % 360.0
            angle = axis_angle

        out.valid = True
        out.angle_deg = angle
        span = 1.0 - self.min_concentration
        out.confidence = 1.0 if span <= 0 else min(1.0, max(0.05, (out.concentration - self.min_concentration) / span))
        return out

    def annotate(self, reading, lines=()):
        """Rectified view with the mask, rings and detected direction drawn on it."""
        img = reading.rectified.copy()
        if reading.mask is not None:
            img[reading.mask > 0] = (0.4 * img[reading.mask > 0] + (0, 153, 0)).astype(np.uint8)
        c = (int(round(self.center)), int(round(self.center)))
        for frac, color in ((self.annulus[0], (255, 200, 0)), (self.annulus[1], (255, 200, 0)), (1.0, (200, 200, 200))):
            cv2.circle(img, c, int(round(frac * self.radius)), color, 1)
        cv2.putText(img, "NOSE", (c[0] - 22, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        if reading.valid:
            a = math.radians(reading.angle_deg)
            tip = (int(c[0] + self.radius * math.sin(a)), int(c[1] - self.radius * math.cos(a)))
            cv2.line(img, c, tip, (0, 0, 255), 2)
            text = "needle %.1f deg  conf %.2f" % (reading.angle_deg, reading.confidence)
            color = (0, 255, 0)
        else:
            text = "NO READING: %s" % reading.reason
            color = (0, 0, 255)
        all_lines = [text] + list(lines)
        top = self.canvas - 8 - 18 * len(all_lines)
        cv2.rectangle(img, (0, top), (self.canvas, self.canvas), (0, 0, 0), -1)
        for i, line in enumerate(all_lines):
            cv2.putText(img, line, (6, top + 16 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        color if i == 0 else (255, 255, 255), 1, cv2.LINE_AA)
        return img


def hsv_ranges_from_sample(h, s, v, hue_tol=12):
    """Build HSV ranges around a sampled needle colour (handles red's hue wrap)."""
    h, s, v, hue_tol = int(h), int(s), int(v), int(hue_tol)
    s_lo = max(50, int(s * 0.5))
    v_lo = max(40, int(v * 0.4))
    lo, hi = h - hue_tol, h + hue_tol
    if lo < 0:
        return [[0, s_lo, v_lo, hi, 255, 255], [180 + lo, s_lo, v_lo, 179, 255, 255]]
    if hi > 179:
        return [[lo, s_lo, v_lo, 179, 255, 255], [0, s_lo, v_lo, hi - 180, 255, 255]]
    return [[lo, s_lo, v_lo, hi, 255, 255]]


def sample_hsv(bgr_image, x, y, half=3):
    """Median-ish HSV of a small patch, with a circular mean for hue."""
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    h_img, w_img = hsv.shape[:2]
    x0, x1 = max(0, x - half), min(w_img, x + half + 1)
    y0, y1 = max(0, y - half), min(h_img, y + half + 1)
    patch = hsv[y0:y1, x0:x1].reshape(-1, 3).astype(np.float64)
    ang = patch[:, 0] * (2.0 * math.pi / 180.0)
    hue = (math.degrees(math.atan2(np.sin(ang).mean(), np.cos(ang).mean())) / 2.0) % 180.0
    return int(round(hue)) % 180, int(np.median(patch[:, 1])), int(np.median(patch[:, 2]))
