import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from estimate_latency import estimate_lag  # noqa: E402


def test_recovers_known_lag():
    rng = np.random.RandomState(0)
    t_f = np.arange(0, 20, 0.05)                     # FC attitude at 20 Hz
    truth = lambda t: 40 * np.sin(2 * np.pi * 0.7 * t) + 15 * np.sin(2 * np.pi * 1.9 * t) + 350
    h_f = truth(t_f) % 360
    t_v = np.arange(0.01, 20, 1 / 30.0)              # camera at 30 Hz, 85 ms late, offset +7 deg
    h_v = (truth(t_v - 0.085) + 7 + rng.normal(0, 1.0, t_v.shape)) % 360
    lag, r = estimate_lag(t_v, h_v, t_f, h_f)
    assert abs(lag - 0.085) <= 0.006, lag
    assert r > 0.95
