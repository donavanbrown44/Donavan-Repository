"""Turn a needle angle into a heading the flight controller can use.

  needle angle (clockwise from nose to magnetic north)
    -> raw magnetic heading      = -needle
    -> corrected magnetic heading = raw + deviation_table(raw)   (mount offset + airframe effects)
    -> true heading               = magnetic + declination        (the EKF works in TRUE north)

Then the reading is gated:
  - tilt   : a needle compass reads wrong when tilted (it is a horizontal instrument)
  - yaw rate: a damped needle lags/overshoots while the drone is spinning
  - jumps  : a single bad frame (glare, prop shadow) must not yank the heading around
If a reading fails a gate it is simply not sent. The FC then coasts on its gyros
instead of fusing a bad number.
"""

import bisect
import math


def wrap360(a):
    return a % 360.0


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def circular_mean_deg(angles):
    """Return (mean, std) in degrees of a list of angles."""
    if not angles:
        return None, None
    s = sum(math.sin(math.radians(a)) for a in angles) / len(angles)
    c = sum(math.cos(math.radians(a)) for a in angles) / len(angles)
    r = min(1.0, math.hypot(s, c))
    std = math.degrees(math.sqrt(max(0.0, -2.0 * math.log(r)))) if r > 1e-12 else 180.0
    return wrap360(math.degrees(math.atan2(s, c))), std


class DeviationTable(object):
    """Correction vs. measured magnetic heading, linearly interpolated around 360.

    Built by the compass swing (calibrate swing). One entry = a constant offset.
    """

    def __init__(self, pairs):
        merged = {}
        for measured, corr in pairs or []:
            merged.setdefault(round(wrap360(float(measured)), 3), []).append(float(corr))
        self.measured = sorted(merged)
        self.corrections = [sum(merged[m]) / len(merged[m]) for m in self.measured]

    def correction(self, heading_deg):
        n = len(self.measured)
        if n == 0:
            return 0.0
        if n == 1:
            return self.corrections[0]
        x = wrap360(heading_deg)
        j = bisect.bisect_right(self.measured, x)
        i0, i1 = (j - 1) % n, j % n
        m0, m1 = self.measured[i0], self.measured[i1]
        span = wrap360(m1 - m0) or 360.0
        frac = wrap360(x - m0) / span
        c0, c1 = self.corrections[i0], self.corrections[i1]
        return c0 + frac * wrap180(c1 - c0)


class Attitude(object):
    """Latest attitude from the flight controller (radians, rad/s, monotonic time)."""
    __slots__ = ("roll", "pitch", "yaw", "yawspeed", "t")

    def __init__(self, roll, pitch, yaw, yawspeed, t):
        self.roll, self.pitch, self.yaw, self.yawspeed, self.t = roll, pitch, yaw, yawspeed, t

    @property
    def tilt_deg(self):
        c = math.cos(self.roll) * math.cos(self.pitch)
        return math.degrees(math.acos(max(-1.0, min(1.0, c))))


class HeadingSample(object):
    __slots__ = ("valid", "reason", "mag_raw_deg", "mag_deg", "true_deg", "std_deg")

    def __init__(self, valid=False, reason="", mag_raw_deg=None, mag_deg=None, true_deg=None, std_deg=None):
        self.valid, self.reason = valid, reason
        self.mag_raw_deg, self.mag_deg, self.true_deg, self.std_deg = mag_raw_deg, mag_deg, true_deg, std_deg


def declination_is_auto(value):
    return isinstance(value, str) and value.strip().lower() == "auto"


def needle_to_magnetic(needle_deg):
    """Needle points to magnetic north `needle_deg` clockwise of the nose -> heading."""
    return wrap360(-needle_deg)


class HeadingEstimator(object):
    def __init__(self, hcfg):
        dec = hcfg.get("declination_deg", 0.0)
        # None = "auto": unknown until the FC's GPS gives a fix (see declination.py)
        self.declination = None if declination_is_auto(dec) else float(dec)
        self.table = DeviationTable(hcfg.get("deviation_table") or [])
        self.alpha = min(1.0, max(0.01, float(hcfg.get("smoothing_alpha", 1.0))))
        self.max_jump = float(hcfg.get("max_jump_deg", 35.0))
        self.reacquire_frames = int(hcfg.get("reacquire_frames", 6))
        self.reset_after_s = float(hcfg.get("reset_after_s", 0.5))
        self.base_std = float(hcfg.get("base_std_deg", 3.0))
        self.max_tilt = float(hcfg.get("max_tilt_deg", 20.0))
        self.max_yaw_rate = float(hcfg.get("max_yaw_rate_dps", 90.0))
        self._filt = None
        self._t_last = None
        self._jumps = 0

    def set_declination(self, deg):
        self.declination = float(deg)
        self._filt = None  # don't compare against headings computed without it

    def correct(self, needle_deg):
        """Return (raw magnetic, corrected magnetic, true) headings for a needle angle.

        true is None while the declination is still unknown (auto, no GPS fix yet).
        """
        raw = needle_to_magnetic(needle_deg)
        mag = wrap360(raw + self.table.correction(raw))
        true = None if self.declination is None else wrap360(mag + self.declination)
        return raw, mag, true

    def update(self, t, needle_deg, confidence=1.0, attitude=None):
        if needle_deg is None:
            return HeadingSample(reason="no_needle")
        raw, mag, true = self.correct(needle_deg)
        s = HeadingSample(mag_raw_deg=raw, mag_deg=mag, true_deg=true)
        if true is None:
            s.reason = "waiting_for_gps_declination"
            return s

        if attitude is not None:
            if attitude.tilt_deg > self.max_tilt:
                s.reason = "tilt"
                return s
            if abs(math.degrees(attitude.yawspeed)) > self.max_yaw_rate:
                s.reason = "yaw_rate"
                return s

        if self._filt is None or self._t_last is None or (t - self._t_last) > self.reset_after_s:
            self._filt = true
        else:
            d = wrap180(true - self._filt)
            if abs(d) > self.max_jump:
                self._jumps += 1
                if self._jumps < self.reacquire_frames:
                    s.reason = "jump"
                    return s
                self._filt = true  # consistently different: it's real, follow it
            else:
                self._filt = wrap360(self._filt + self.alpha * d)
        self._jumps = 0
        self._t_last = t

        s.valid = True
        s.true_deg = self._filt
        s.std_deg = self.base_std / max(float(confidence), 0.25)
        return s
