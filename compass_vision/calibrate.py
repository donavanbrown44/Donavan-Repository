"""Calibration tool.

  setup     GUI (monitor on the Jetson, or `ssh -X`). Click 4 points on the dial
            rim - NOSE, RIGHT, TAIL, LEFT - then click the needle's north tip to
            teach its colour. Writes dial.points and needle.hsv_ranges.

  swing     Terminal only, works over plain SSH. "Compass swing": point the drone
            at known headings, the tool records what the camera reads and builds
            the deviation table (fixes mounting offset and the airframe's own
            magnetic effect). 1 point = offset only; 8 points (every 45 deg) is good.

  snapshot  Save one camera frame to a file (to calibrate from an image on a laptop:
            `setup --image snap.jpg`).

Everything is written to calibration.yaml next to config.yaml.
"""

import argparse
import sys
import time

import cv2
import numpy as np

from .camera import Camera
from .config import load_config, save_calibration
from .heading import circular_mean_deg, declination_is_auto, needle_to_magnetic, wrap180, wrap360
from .needle import NeedleDetector, hsv_ranges_from_sample, sample_hsv

POINT_NAMES = ["NOSE", "RIGHT", "TAIL", "LEFT"]


def _frame_source(cfg, image_path):
    if image_path:
        img = cv2.imread(image_path)
        if img is None:
            sys.exit("could not read %s" % image_path)
        return None, lambda: img
    cam = Camera(cfg["camera"])
    state = {"seq": 0, "frame": None}

    def get():
        got = cam.read(state["seq"], timeout=2.0)
        if got is not None:
            state["frame"], _, state["seq"] = got
        return state["frame"]
    return cam, get


def cmd_setup(cfg, args):
    cam, get_frame = _frame_source(cfg, args.image)
    first = get_frame()
    if first is None:
        sys.exit("no frames from the camera")
    disp_scale = min(1.0, 1280.0 / first.shape[1])
    st = {"points": [], "hsv_ranges": cfg["needle"]["hsv_ranges"], "sample": None,
          "tol": 12, "rect": None}

    def on_camera(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(st["points"]) < 4:
            st["points"].append([round(x / disp_scale, 1), round(y / disp_scale, 1)])

    def on_rect(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and st["rect"] is not None:
            st["sample"] = sample_hsv(st["rect"], x, y)
            st["hsv_ranges"] = hsv_ranges_from_sample(*st["sample"], hue_tol=st["tol"])
            print("needle colour HSV=%s -> ranges %s" % (st["sample"], st["hsv_ranges"]))

    cv2.namedWindow("camera")
    cv2.setMouseCallback("camera", on_camera)
    print(__doc__)
    print("Keys: r = redo points, [ / ] = narrower/wider colour, s = save, q = quit\n")
    detector = None
    try:
        while True:
            frame = get_frame()
            disp = cv2.resize(frame, None, fx=disp_scale, fy=disp_scale) if disp_scale < 1 else frame.copy()
            for i, (px, py) in enumerate(st["points"]):
                p = (int(px * disp_scale), int(py * disp_scale))
                cv2.circle(disp, p, 5, (0, 255, 255), -1)
                cv2.putText(disp, POINT_NAMES[i], (p[0] + 6, p[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            if len(st["points"]) < 4:
                msg = "Click the dial rim at the drone's %s" % POINT_NAMES[len(st["points"])]
            else:
                msg = "Now click the needle's NORTH tip in the 'rectified' window"
            cv2.putText(disp, msg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
            cv2.putText(disp, msg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow("camera", disp)

            if len(st["points"]) == 4:
                if detector is None:
                    detector = NeedleDetector.from_config(cfg, dial_points=st["points"], hsv_ranges=st["hsv_ranges"])
                    cv2.namedWindow("rectified")
                    cv2.setMouseCallback("rectified", on_rect)
                detector.set_hsv_ranges(st["hsv_ranges"])
                reading = detector.detect(frame)
                st["rect"] = reading.rectified
                extra = []
                if reading.valid:
                    raw = needle_to_magnetic(reading.angle_deg)
                    extra.append("raw magnetic heading %.1f (uncorrected)" % raw)
                extra.append("mask px %.1f%%  conc %.3f  extent %.2f" % (
                    100 * reading.area_frac, reading.concentration, reading.radial_extent))
                cv2.imshow("rectified", detector.annotate(reading, extra))

            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                print("quit without saving")
                return
            if key == ord("r"):
                if detector is not None:
                    cv2.destroyWindow("rectified")
                st["points"] = []
                st["rect"] = None
                detector = None
            if key in (ord("["), ord("]")) and st["sample"] is not None:
                st["tol"] = max(3, min(40, st["tol"] + (2 if key == ord("]") else -2)))
                st["hsv_ranges"] = hsv_ranges_from_sample(*st["sample"], hue_tol=st["tol"])
                print("hue tolerance %d -> %s" % (st["tol"], st["hsv_ranges"]))
            if key == ord("s"):
                if len(st["points"]) != 4:
                    print("click all 4 rim points first")
                    continue
                path = save_calibration(cfg, {"dial": {"points": st["points"]},
                                              "needle": {"hsv_ranges": st["hsv_ranges"]}})
                print("saved %s" % path)
                print("Next: `python3 -m compass_vision.calibrate swing` to fix offset/deviation.")
                return
    finally:
        cv2.destroyAllWindows()
        if cam:
            cam.close()


def raw_heading_stream(cam, detector):
    """Yield (t, raw magnetic heading or None) for each new camera frame."""
    seq = 0
    while True:
        got = cam.read(seq, timeout=1.0)
        if got is None:
            yield time.monotonic(), None
            continue
        frame, t, seq = got
        r = detector.detect(frame)
        yield t, (needle_to_magnetic(r.angle_deg) if r.valid else None)


def settled_average(samples, seconds, window_s=1.0, tol_deg=0.7, timeout_s=20.0):
    """Wait for the needle to stop moving, then average `seconds` of readings.

    After you turn the drone the needle swings for a while before it finds north;
    averaging during that would bake the swing into the calibration. Steady = the
    last window_s of readings vary less than tol_deg and aren't trending.
    samples: iterator of (t, heading or None).
    Returns (readings, waited_s, settled) - settled is False if it timed out.
    """
    recent, readings = [], []
    t0 = t_settled = None
    for t, h in samples:
        t0 = t if t0 is None else t0
        if t_settled is None:
            if h is not None:
                recent.append((t, h))
            recent = [p for p in recent if t - p[0] <= window_s]
            steady = False
            if len(recent) >= 5 and t - recent[0][0] >= 0.8 * window_s:
                hs = [p[1] for p in recent]
                half = len(hs) // 2
                _, std = circular_mean_deg(hs)
                m1, _ = circular_mean_deg(hs[:half])
                m2, _ = circular_mean_deg(hs[half:])
                steady = std < tol_deg and abs(wrap180(m2 - m1)) < tol_deg / 2.0
            if steady or t - t0 > timeout_s:
                t_settled = t
            continue
        if h is not None:
            readings.append(h)
        if t - t_settled >= seconds:
            break
    if t_settled is None:  # ran out of readings before it ever settled
        return readings, 0.0, False
    waited = t_settled - t0
    return readings, waited, waited <= timeout_s


def cmd_swing(cfg, args):
    if not cfg["dial"]["points"]:
        sys.exit("run `calibrate setup` first")
    detector = NeedleDetector.from_config(cfg)
    decl_cfg = cfg["heading"]["declination_deg"]
    if args.true and declination_is_auto(decl_cfg):
        sys.exit("--true needs a numeric declination, but declination_deg is 'auto'. Enter MAGNETIC "
                 "references instead (handheld compass), or look the declination up and pass true "
                 "references minus it.")
    decl = 0.0 if declination_is_auto(decl_cfg) else float(decl_cfg)
    table = [list(p) for p in cfg["heading"]["deviation_table"]] if args.keep else []
    cam = Camera(cfg["camera"])
    ref_kind = "TRUE" if args.true else "MAGNETIC"
    print("""
Compass swing
-------------
Point the drone's nose at a known %s heading, hold it still, type that heading
and press Enter. The tool waits for the needle to stop swinging, then averages
%.0f s of camera readings. Repeat at several
headings (every 45 deg is ideal). Empty line = finish and save.

Reference ideas: a handheld compass held a few metres away from the drone, or
lining the drone up with a road/runway/building edge whose heading you know.
Do it outdoors, away from cars and rebar, with the drone powered as it flies.
""" % (ref_kind, args.seconds))
    try:
        while True:
            try:
                line = input("%s reference heading (deg, Enter to finish): " % ref_kind).strip()
            except EOFError:
                break
            if not line:
                break
            try:
                ref = wrap360(float(line))
            except ValueError:
                print("not a number")
                continue
            if args.true:
                ref = wrap360(ref - decl)
            print("  waiting for the needle to settle...")
            readings, waited, settled = settled_average(raw_heading_stream(cam, detector), args.seconds)
            if settled:
                print("  needle settled after %.1f s" % waited)
            else:
                print("  WARNING: needle never settled (is the drone moving or vibrating?) - using it anyway")
            if len(readings) < 10:
                print("  only %d valid readings - check lighting / `calibrate setup`. Skipped." % len(readings))
                continue
            mean, std = circular_mean_deg(readings)
            corr = wrap180(ref - mean)
            table.append([round(mean, 1), round(corr, 1)])
            print("  camera reads %.1f (+/- %.1f, n=%d) -> correction %+.1f deg" % (mean, std, len(readings), corr))
    finally:
        cam.close()
    if not table:
        print("nothing recorded, calibration unchanged")
        return
    table.sort()
    path = save_calibration(cfg, {"heading": {"deviation_table": table}})
    corrs = [c for _, c in table]
    print("\nsaved %d points to %s" % (len(table), path))
    print("corrections range %+.1f .. %+.1f deg" % (min(corrs), max(corrs)))
    if len(table) >= 4 and (max(corrs) - min(corrs)) > 20:
        print("WARNING: corrections vary by >20 deg around the circle. Something magnetic near the "
              "compass is distorting it (steel screws, battery, power wires, motors).")


def cmd_snapshot(cfg, args):
    cam = Camera(cfg["camera"])
    try:
        got = cam.read(0, timeout=5.0)
        if got is None:
            sys.exit("no frames from the camera")
        for _ in range(10):  # let auto-exposure settle
            got = cam.read(got[2], timeout=2.0) or got
        cv2.imwrite(args.out, got[0])
        print("saved %s (%dx%d)" % (args.out, got[0].shape[1], got[0].shape[0]))
    finally:
        cam.close()


def main(argv=None):
    p = argparse.ArgumentParser(description="Calibrate the compass camera")
    p.add_argument("--config", default="config.yaml")
    sub = p.add_subparsers(dest="cmd")
    s = sub.add_parser("setup", help="GUI: dial points + needle colour")
    s.add_argument("--image", help="calibrate from a saved image instead of the live camera")
    w = sub.add_parser("swing", help="terminal: build the deviation table")
    w.add_argument("--seconds", type=float, default=2.0, help="averaging time per heading")
    w.add_argument("--true", action="store_true", help="references are TRUE headings (declination is removed)")
    w.add_argument("--keep", action="store_true", help="add to the existing table instead of replacing it")
    n = sub.add_parser("snapshot", help="save one frame")
    n.add_argument("--out", default="snapshot.jpg")
    args = p.parse_args(argv)
    if not args.cmd:
        p.print_help()
        return
    cfg = load_config(args.config)
    {"setup": cmd_setup, "swing": cmd_swing, "snapshot": cmd_snapshot}[args.cmd](cfg, args)


if __name__ == "__main__":
    main()
