import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from compass_vision.calibrate import settled_average  # noqa: E402


def stream(fn, seconds=30.0, fps=30.0, noise=0.2, seed=0):
    rnd = random.Random(seed)
    for k in range(int(seconds * fps)):
        t = k / fps
        yield t, fn(t) + rnd.gauss(0, noise)


def test_swing_waits_for_needle_then_averages():
    # drone just turned to 90: needle swings 30 deg and settles over ~3 s
    needle = lambda t: 90.0 + 30.0 * math.exp(-t / 0.8) * math.cos(2 * math.pi * 0.6 * t)
    readings, waited, settled = settled_average(stream(needle), seconds=2.0)
    assert settled and 2.0 < waited < 6.0
    mean = sum(readings) / len(readings)
    assert abs(mean - 90.0) < 0.5 and len(readings) >= 55


def test_swing_times_out_if_never_steady():
    readings, waited, settled = settled_average(stream(lambda t: 5.0 * t, seconds=40), seconds=1.0, timeout_s=5.0)
    assert not settled and readings
