import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from compass_vision.config import DEFAULTS  # noqa: E402
from compass_vision.heading import (Attitude, DeviationTable, HeadingEstimator,  # noqa: E402
                                    circular_mean_deg, needle_to_magnetic, wrap180)


def est(**over):
    h = dict(DEFAULTS["heading"])
    h["declination_deg"] = 0.0
    h.update(over)
    return HeadingEstimator(h)


def close(a, b, tol=1e-6):
    return abs(wrap180(a - b)) < tol


def test_needle_to_heading_sign():
    # North is 30 deg to the right of the nose -> the nose points 30 deg west of north.
    assert close(needle_to_magnetic(30.0), 330.0)
    assert close(needle_to_magnetic(0.0), 0.0)
    assert close(needle_to_magnetic(270.0), 90.0)  # north off the left side -> facing east


def test_declination_is_added():
    raw, mag, true = est(declination_deg=-12.5).correct(30.0)
    assert close(raw, 330.0) and close(mag, 330.0) and close(true, 317.5)


def test_deviation_table_interpolates_and_wraps():
    t = DeviationTable([[0, 2.0], [90, 4.0], [180, -2.0], [270, 0.0]])
    assert close(t.correction(45), 3.0)
    assert close(t.correction(315), 1.0)      # between 270 (0) and 360/0 (2)
    assert close(t.correction(359.9), 2.0, 0.01)
    assert DeviationTable([[123, -7.0]]).correction(10) == -7.0  # single point = offset
    assert DeviationTable([]).correction(10) == 0.0


def test_jump_rejected_then_reacquired():
    e = est(reacquire_frames=3)
    t = 0.0
    assert e.update(t, 10.0).valid
    t += 0.033
    s = e.update(t, 190.0)  # one-frame 180 flip (e.g. glare)
    assert not s.valid and s.reason == "jump"
    t += 0.033
    assert e.update(t, 11.0).valid  # back to normal
    # a real, persistent change gets followed after 3 frames
    results = []
    for _ in range(3):
        t += 0.033
        results.append(e.update(t, 100.0).valid)
    assert results == [False, False, True]


def test_gap_resets_filter():
    e = est()
    assert e.update(0.0, 10.0).valid
    assert e.update(2.0, 150.0).valid  # long gap: trust the new reading


def test_tilt_and_yaw_rate_gates():
    e = est(max_tilt_deg=20, max_yaw_rate_dps=90)
    tilted = Attitude(math.radians(25), 0.0, 0.0, 0.0, 0.0)
    spinning = Attitude(0.0, 0.0, 0.0, math.radians(120), 0.0)
    level = Attitude(math.radians(5), math.radians(5), 0.0, math.radians(10), 0.0)
    assert e.update(0.0, 10.0, attitude=tilted).reason == "tilt"
    assert e.update(0.0, 10.0, attitude=spinning).reason == "yaw_rate"
    assert e.update(0.0, 10.0, attitude=level).valid


def test_low_confidence_inflates_std():
    e = est(base_std_deg=3.0)
    assert abs(e.update(0.0, 10.0, confidence=1.0).std_deg - 3.0) < 1e-9
    assert e.update(0.01, 10.0, confidence=0.5).std_deg > 3.0


def test_circular_mean_across_north():
    m, s = circular_mean_deg([358.0, 2.0, 0.0])
    assert close(m, 0.0, 1e-6) and s < 3.0


def test_auto_declination_waits_for_gps_then_applies():
    e = est(declination_deg="auto")
    s = e.update(0.0, 30.0)
    assert not s.valid and s.reason == "waiting_for_gps_declination"
    assert close(s.mag_deg, 330.0) and s.true_deg is None  # magnetic still available for display
    e.set_declination(7.5)
    s = e.update(0.03, 30.0)
    assert s.valid and close(s.true_deg, 337.5)
