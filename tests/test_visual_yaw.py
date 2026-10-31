import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from compass_vision.heading import wrap180  # noqa: E402
from compass_vision.visual_yaw import HeadingStabilizer, VisualYawSample, VisualYawTracker  # noqa: E402
from synth import ground_view, make_ground  # noqa: E402


@pytest.fixture(scope="module")
def ground():
    return make_ground()


def fly(ground, yaw_fn, n, off_fn=lambda t: (0, 0), fps=30.0, gyro=True, tracker=None):
    tr = tracker or VisualYawTracker()
    out = []
    for k in range(n):
        t = k / fps
        gy = (yaw_fn(t + 0.01) - yaw_fn(t - 0.01)) / 0.02 if gyro else None
        out.append((t, yaw_fn(t), tr.update(ground_view(ground, yaw_fn(t), off_fn(t), seed=k), t, gy)))
    return tr, out


def test_slow_creep_while_hovering_is_measured(ground):
    # 1 deg/min creep over 40 s, drone wobbling around in the wind
    _, out = fly(ground, lambda t: t / 60.0, 1200, off_fn=lambda t: (8 * math.sin(t), 5 * math.cos(0.7 * t)))
    assert all(s.valid for _, _, s in out)
    assert max(abs(s.total_deg - y) for _, y, s in out) < 0.05


def test_big_rotation_with_keyframe_refresh(ground):
    _, out = fly(ground, lambda t: 45.0 * t, 240)  # 360 deg at 45 deg/s
    assert all(s.valid for _, _, s in out)
    assert len(set(s.segment for _, _, s in out)) == 1  # never lost continuity
    assert abs(out[-1][2].total_deg - out[-1][1]) < 1.5


def test_blank_view_loses_tracking_and_starts_new_segment(ground):
    tr, out = fly(ground, lambda t: 0.0, 10)
    seg = out[-1][2].segment
    s = tr.update(np.full((480, 640, 3), 128, np.uint8), 1.0, 0.0)  # lens covered
    assert not s.valid
    s2 = tr.update(ground_view(ground, 0.0), 1.1, 0.0)
    assert s2.segment != seg


def test_disagreeing_with_gyro_is_rejected(ground):
    tr, _ = fly(ground, lambda t: 0.0, 10)
    s = tr.update(ground_view(ground, 3.0), 10 / 30.0, gyro_dps=0.0)  # 3 deg in one frame = 90 deg/s vs gyro 0
    assert not s.valid and s.reason == "gyro_mismatch"


def test_invert_flips_sign(ground):
    _, out = fly(ground, lambda t: 10.0 * t, 30, tracker=VisualYawTracker(invert=True), gyro=False)
    assert out[-1][2].total_deg < 0


# ------------------------------------------------------------------- stabilizer
def vis(total, t, seg=0):
    return VisualYawSample(True, total, seg, 100, "", t)


def test_loiter_hold_ignores_creeping_compass():
    st = HeadingStabilizer(tau_s=60.0)
    out = None
    for k in range(30 * 120):  # 2 minutes in LOITER, compass creeps 6 deg/min, drone actually still
        t = k / 30.0
        out = st.update(t, 100.0 + 0.1 * t, vis(0.0, t), mode="LOITER", gyro_dps=0.0)
    assert out.holding and abs(wrap180(out.heading_deg - 100.0)) < 1e-6
    assert abs(out.drift_deg - 12.0) < 0.1  # the drift is still measured and reported


def test_outside_hold_compass_pulls_slowly():
    st = HeadingStabilizer(tau_s=60.0)
    st.update(0.0, 100.0, vis(0.0, 0.0), mode="ALT_HOLD")
    out = None
    for k in range(1, 30 * 60 + 1):  # compass jumps to 110 and stays; camera says no rotation
        t = k / 30.0
        out = st.update(t, 110.0, vis(0.0, t), mode="ALT_HOLD")
    assert 105.5 < out.heading_deg < 107.0  # ~63% of the way after one time constant


def test_real_rotation_followed_instantly_via_camera():
    st = HeadingStabilizer(tau_s=60.0)
    st.update(0.0, 0.0, vis(0.0, 0.0), mode="LOITER")
    # pilot yaws 90 deg in 2 s; compass is gated (None) during the turn
    for k in range(1, 61):
        t = k / 30.0
        out = st.update(t, None, vis(1.5 * k, t), mode="LOITER")
    assert out.valid and abs(wrap180(out.heading_deg - 90.0)) < 1e-6


def test_camera_lost_blends_back_to_compass_without_step():
    st = HeadingStabilizer(tau_s=60.0, tau_lost_s=3.0)
    for k in range(30):
        st.update(k / 30.0, 100.0 + k * 0.3, vis(0.0, k / 30.0), mode="LOITER")  # held at 100, compass 108.7
    prev = st.h
    steps = []
    for k in range(30, 30 + 30 * 15):  # camera gone; gyro says still
        out = st.update(k / 30.0, 108.7, None, mode="LOITER", gyro_dps=0.0)
        steps.append(abs(wrap180(out.heading_deg - prev)))
        prev = out.heading_deg
    assert max(steps) < 0.5                       # smooth, no jump
    assert abs(wrap180(out.heading_deg - 108.7)) < 0.1  # converged to compass


def test_segment_change_does_not_apply_bogus_delta():
    st = HeadingStabilizer()
    st.update(0.0, 50.0, vis(0.0, 0.0, seg=0), mode="LOITER")
    out = st.update(0.033, 50.0, vis(200.0, 0.033, seg=1), mode="LOITER")  # new segment, unrelated total
    assert abs(wrap180(out.heading_deg - 50.0)) < 1e-6
