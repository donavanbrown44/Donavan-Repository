"""Turn a needle angle into a heading the flight controller can use.

  needle angle (clockwise from nose to magnetic north)
    -> raw magnetic heading      = -needle
    -> corrected magnetic heading = raw + deviation_table(raw)   (mount offset + airframe effects)
    -> true heading               = magnetic + declination        (the EKF works in TRUE north)

Then the reading is gated:
  - tilt    : a needle compass reads wrong when tilted (it is a horizontal instrument)
  - yaw rate: a damped needle lags/overshoots while the drone is spinning
  - settling: after a turn the needle keeps swinging for a few seconds before it
              finds north again; those readings are skipped (see SettleDetector)
  - jumps   : a single bad frame (glare, prop shadow) must not yank the heading around
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


class SettleDetector(object):
    """Has the needle finished swinging after a turn?

    A perfect compass's heading changes exactly as fast as the gyro says the drone
    turns. So: compass heading minus gyro-integrated rotation = the needle's own
    motion (lag, swing, overshoot). While that residual is still trending, the needle
    hasn't settled. "Trending" = linear-fit slope over `window_s` is above
    `max_rate_dps` AND clearly above what the reading noise alone could produce, so
    vibration jitter doesn't keep it unsettled forever. Must stay calm for `hold_s`.

    While the gyro says the drone is turning faster than `turn_dps`, the needle is
    trailing the turn by definition, so it isn't trusted at all.

    Adapts to whatever compass you fit; no fixed timer to tune. Also records how long
    the needle took to settle after each turn, as a measure of your compass.
    """

    def __init__(self, max_rate_dps=3.0, turn_dps=15.0, window_s=1.0, hold_s=0.3, give_up_s=10.0):
        self.max_rate = float(max_rate_dps)
        self.turn_dps = float(turn_dps)  # turning faster than this: the needle is lagging, full stop
        self.window = float(window_s)
        self.hold = float(hold_s)
        self.give_up = float(give_up_s)
        self.enabled = self.max_rate > 0
        self.settled = True
        self.settle_time = None     # set once each time the needle settles after a turn
        self.gave_up = False
        self._buf = []              # (t, residual_deg), residual unwrapped
        self._g = 0.0               # gyro-integrated rotation, deg
        self._t = None
        self._calm_since = None
        self._unsettled_since = None
        self._last_turn_t = None

    def update(self, t, heading_deg, gyro_dps):
        """Feed every compass reading with the current gyro yaw rate. Returns settled?"""
        self.settle_time = None
        if not self.enabled or gyro_dps is None:
            return True
        if self._t is not None:
            dt = t - self._t
            if dt > self.window:  # gap in readings: start over
                self._buf = []
                self._calm_since = None
            else:
                self._g += gyro_dps * dt
        self._t = t
        turning = abs(gyro_dps) > self.turn_dps
        if turning:
            self._last_turn_t = t

        r = heading_deg - self._g
        if self._buf:
            r = self._buf[-1][1] + wrap180(r - self._buf[-1][1])
        self._buf.append((t, r))
        while self._buf and t - self._buf[0][0] > self.window:
            self._buf.pop(0)

        calm = False
        n = len(self._buf)
        if n >= 5 and t - self._buf[0][0] >= 0.6 * self.window:
            ts = [p[0] for p in self._buf]
            rs = [p[1] for p in self._buf]
            tm, rm = sum(ts) / n, sum(rs) / n
            stt = sum((x - tm) ** 2 for x in ts)
            slope = sum((x - tm) * (y - rm) for x, y in zip(ts, rs)) / stt
            resid = math.sqrt(sum((y - rm - slope * (x - tm)) ** 2 for x, y in zip(ts, rs)) / max(1, n - 2))
            noise_slope = resid / math.sqrt(stt)  # 1-sigma slope from noise alone
            # allow for jitter, but cap it so a slow swing can't pass as "noise"
            calm = abs(slope) < min(max(self.max_rate, 3.0 * noise_slope), 2.0 * self.max_rate)

        was = self.settled
        if turning:  # a damped needle always trails a turn; don't wait for the fit to notice
            self._calm_since = None
            self.settled = False
            self._unsettled_since = None  # the give-up timer is for a needle that won't settle when still
            return False
        if calm:
            self._calm_since = self._calm_since if self._calm_since is not None else t
            self.settled = (t - self._calm_since) >= self.hold
        else:
            self._calm_since = None
            self.settled = False

        if self.settled:
            self._unsettled_since = None
            self.gave_up = False
            if not was and self._last_turn_t is not None and t - self._last_turn_t < 30.0:
                self.settle_time = t - self._last_turn_t
        else:
            self._unsettled_since = self._unsettled_since if self._unsettled_since is not None else t
            if t - self._unsettled_since > self.give_up:
                self.gave_up = True  # never block the compass for good (e.g. a loose mount)
                return True
        return self.settled


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
        self.settle = SettleDetector(max_rate_dps=hcfg.get("settle_max_rate_dps", 3.0),
                                     turn_dps=hcfg.get("settle_turn_dps", 15.0),
                                     hold_s=hcfg.get("settle_hold_s", 0.3),
                                     give_up_s=hcfg.get("settle_give_up_s", 10.0))
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
        # runs on every reading (even ones gated below) so it can follow the needle
        settled = self.settle.update(t, mag, math.degrees(attitude.yawspeed) if attitude is not None else None)
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
        if not settled:
            s.reason = "needle_settling"
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
