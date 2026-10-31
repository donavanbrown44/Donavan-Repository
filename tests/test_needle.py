import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from compass_vision.config import DEFAULTS  # noqa: E402
from compass_vision.heading import wrap180  # noqa: E402
from compass_vision.needle import NeedleDetector, hsv_ranges_from_sample, sample_hsv  # noqa: E402
from synth import camera_view, draw_dial  # noqa: E402

RED = DEFAULTS["needle"]["hsv_ranges"]


def detector(points):
    n = DEFAULTS["needle"]
    return NeedleDetector(points, RED, annulus=DEFAULTS["dial"]["annulus"],
                          min_area_frac=n["min_area_frac"], max_area_frac=n["max_area_frac"],
                          min_radial_extent=n["min_radial_extent"],
                          min_concentration=n["min_concentration"])


@pytest.mark.parametrize("mirror", [False, True])
def test_angle_all_around_oblique_camera(mirror):
    errs = []
    for i, ang in enumerate(range(0, 360, 7)):
        img, pts = camera_view(draw_dial(ang), mirror=mirror, seed=i)
        r = detector(pts).detect(img)
        assert r.valid, (ang, r.reason)
        errs.append(abs(wrap180(r.angle_deg - ang)))
    assert max(errs) < 1.0, max(errs)


def test_north_south_not_confused():
    # Needle pointing at the tail: a line detector would also accept 0 deg.
    img, pts = camera_view(draw_dial(180.0))
    r = detector(pts).detect(img)
    assert r.valid and abs(wrap180(r.angle_deg - 180.0)) < 1.0


def test_sloppy_calibration_clicks_still_accurate():
    # Clicks off by a few pixels: the long-axis estimate keeps the error small.
    rng = np.random.RandomState(3)
    errs = []
    for ang in range(0, 360, 15):
        img, pts = camera_view(draw_dial(ang))
        noisy = (np.array(pts) + rng.uniform(-4, 4, (4, 2))).tolist()
        r = detector(noisy).detect(img)
        assert r.valid
        errs.append(abs(wrap180(r.angle_deg - ang)))
    assert max(errs) < 2.0, max(errs)


def test_printed_red_mark_outside_annulus_is_ignored():
    img, pts = camera_view(draw_dial(90.0, red_n_marking=True))
    r = detector(pts).detect(img)
    assert r.valid and abs(wrap180(r.angle_deg - 90.0)) < 1.0


def test_no_needle_is_rejected():
    img, pts = camera_view(draw_dial(45.0))
    img[:] = (90, 90, 90)  # lens covered / lights off
    r = detector(pts).detect(img)
    assert not r.valid and r.reason == "no_needle_pixels"


def test_round_glare_blob_is_rejected():
    import cv2
    img, pts = camera_view(draw_dial(0.0, needle_len=0.0, needle_halfwidth=0.0))
    det = detector(pts)
    # Paint a round red blob where the needle would be (not needle shaped).
    c = cv2.perspectiveTransform(np.float32([[[200, 120]]]), np.linalg.inv(det.H))[0][0]
    cv2.circle(img, (int(c[0]), int(c[1])), 14, (30, 30, 210), -1)
    r = det.detect(img)
    assert not r.valid, r.angle_deg


def test_colour_sampling_round_trip():
    img, pts = camera_view(draw_dial(30.0))
    det = detector(pts)
    rect = det.rectify(img)
    # sample the needle a bit out from the hub along 30 deg
    import math
    x = int(det.center + 0.45 * det.radius * math.sin(math.radians(30)))
    y = int(det.center - 0.45 * det.radius * math.cos(math.radians(30)))
    h, s, v = sample_hsv(rect, x, y)
    det.set_hsv_ranges(hsv_ranges_from_sample(h, s, v))
    r = det.detect(img)
    assert r.valid and abs(wrap180(r.angle_deg - 30.0)) < 1.0


def test_hsv_ranges_wrap_for_red():
    assert len(hsv_ranges_from_sample(3, 200, 200)) == 2
    assert len(hsv_ranges_from_sample(176, 200, 200)) == 2
    assert len(hsv_ranges_from_sample(60, 200, 200)) == 1
