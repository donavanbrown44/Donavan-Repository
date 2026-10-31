#!/usr/bin/env python3
"""Estimate how far the camera heading lags reality, to set the EKF delay parameter.

1. Run in shadow mode (FC keeps its own heading, nothing is sent):
       python3 -m compass_vision --config config.yaml --shadow
2. Props OFF. Pick the drone up, keep it level, and twist it left/right by
   ~30-60 deg a few times per second for ~20 s (a few faster, a few slower).
3. Stop, then:
       python3 scripts/estimate_latency.py logs/compass_YYYYmmdd_HHMMSS.csv

The FC's yaw is driven by its gyros over short time scales, so it is a good
"truth" for timing even if its absolute heading is off. The time shift that best
lines up the two curves is camera latency + needle lag.
"""

import csv
import sys

import numpy as np


def estimate_lag(t_v, h_v, t_f, h_f, max_lag_s=0.4, step_s=0.005):
    """Return (lag_s, correlation). Positive lag = vision is behind the FC."""
    t_v, t_f = np.asarray(t_v, float), np.asarray(t_f, float)
    v = np.degrees(np.unwrap(np.radians(np.asarray(h_v, float))))
    f = np.degrees(np.unwrap(np.radians(np.asarray(h_f, float))))
    t0, t1 = max(t_v[0], t_f[0]) + max_lag_s, min(t_v[-1], t_f[-1])
    if t1 - t0 < 3.0:
        raise ValueError("need at least a few seconds of overlapping data")
    grid = np.arange(t0, t1, step_s)
    vg = np.interp(grid, t_v, v)
    vg -= vg.mean()
    best = (0.0, -2.0)
    for lag in np.arange(0.0, max_lag_s + 1e-9, step_s):
        fg = np.interp(grid - lag, t_f, f)
        fg -= fg.mean()
        denom = np.sqrt((vg * vg).sum() * (fg * fg).sum())
        r = float((vg * fg).sum() / denom) if denom > 0 else 0.0
        if r > best[1]:
            best = (float(lag), r)
    return best


def load(path):
    vis, fc, proc = [], {}, []
    with open(path) as f:
        for row in csv.DictReader(f):
            if row["mag_raw_deg"]:
                vis.append((float(row["t_capture"]), float(row["mag_raw_deg"])))
                proc.append(float(row["latency_ms"]))
            if row.get("fc_att_time") and row["fc_yaw_deg"]:
                fc[float(row["fc_att_time"])] = float(row["fc_yaw_deg"])
    fc_sorted = sorted(fc.items())
    return vis, fc_sorted, proc


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    vis, fc, proc = load(sys.argv[1])
    if len(vis) < 50 or len(fc) < 50:
        sys.exit("not enough data (need camera readings AND FC attitude - was this run with --shadow?)")
    t_v, h_v = zip(*vis)
    t_f, h_f = zip(*fc)
    swing = np.ptp(np.degrees(np.unwrap(np.radians(h_f))))
    if swing < 20:
        sys.exit("the drone barely rotated (%.0f deg of FC yaw). Twist it back and forth more." % swing)
    lag, r = estimate_lag(t_v, h_v, t_f, h_f)
    if r < 0.8:
        sys.exit("could not line up the camera and FC headings (correlation %.2f). Check the needle "
                 "detection is solid (valid %% in the status line) and repeat." % r)
    proc_med, proc_min = float(np.median(proc)), float(np.min(proc))
    uart_ms = 3.0  # ~250-byte message at 921600 baud
    print("camera heading lags the FC by %.0f ms (correlation %.3f)" % (lag * 1000, r))
    if r < 0.9:
        print("WARNING: weak correlation - treat this number with suspicion.")
    print("processing time on the Jetson: median %.0f ms, min %.0f ms" % (proc_med, proc_min))
    print("suggested starting values:")
    # ArduPilot maps our capture timestamps onto its clock using the fastest packets seen.
    print("  ArduPilot  VISO_DELAY_MS = %.0f" % (lag * 1000 + proc_min + uart_ms))
    # PX4 (without TIMESYNC replies) timestamps the message when it arrives.
    print("  PX4        EKF2_EV_DELAY = %.0f" % (lag * 1000 + proc_med + uart_ms))


if __name__ == "__main__":
    main()
