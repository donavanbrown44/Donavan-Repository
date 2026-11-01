"""Main loop: camera -> needle angle -> heading -> Pixhawk.

Usage:
  python3 -m compass_vision --config config.yaml            # normal: send heading to FC
  python3 -m compass_vision --config config.yaml --shadow   # read FC, log, DON'T send
  python3 -m compass_vision --config config.yaml --dry-run  # no FC at all, just print

With visual_yaw enabled, a second (ground-facing) camera measures the drone's real
rotation; the heading sent is the compass heading held steady by that camera (see
visual_yaw.py), so it can't creep in Loiter.
"""

import argparse
import csv
import logging
import math
import os
import signal
import time

import cv2
import numpy as np

from . import declination
from .camera import Camera
from .config import load_config, resolve_path
from .heading import HeadingEstimator, wrap180
from .needle import NeedleDetector
from .visual_yaw import HeadingStabilizer, VisualYawTracker

log = logging.getLogger("compass_vision")

CSV_FIELDS = ["unix_time", "t_capture", "latency_ms", "needle_deg", "confidence",
              "mag_raw_deg", "mag_deg", "declination_deg", "true_deg", "std_deg", "reason",
              "vis_valid", "vis_total_deg", "vis_inliers", "vis_reason",
              "out_deg", "out_source", "holding", "compass_drift_deg", "sent",
              "fc_mode", "fc_att_time", "fc_roll_deg", "fc_pitch_deg", "fc_yaw_deg", "fc_yawspeed_dps",
              "fc_minus_vision_deg"]


class CsvLog(object):
    def __init__(self, directory):
        if not os.path.isdir(directory):
            os.makedirs(directory)
        self.path = os.path.join(directory, time.strftime("compass_%Y%m%d_%H%M%S.csv"))
        self._f = open(self.path, "w")
        self._w = csv.writer(self._f)
        self._w.writerow(CSV_FIELDS)
        self._last_flush = time.monotonic()

    def write(self, row):
        self._w.writerow(["" if v is None else (round(v, 3) if isinstance(v, float) else v) for v in row])
        if time.monotonic() - self._last_flush > 1.0:
            self._f.flush()
            self._last_flush = time.monotonic()

    def close(self):
        self._f.close()


def debug_view(frame, detector, reading, lines, extra=None):
    """Raw frame (with the dial centre) | annotated rectified dial | [ground camera]."""
    rect = detector.annotate(reading, lines)
    h = rect.shape[0]
    scale = float(h) / frame.shape[0]
    raw = cv2.resize(frame, (int(frame.shape[1] * scale), h))
    pts = cv2.perspectiveTransform(np.float32([[[detector.center, detector.center]]]),
                                   np.linalg.inv(detector.H))[0][0] * scale
    cv2.circle(raw, (int(pts[0]), int(pts[1])), 4, (0, 255, 255), -1)
    panels = [raw, rect] + ([extra] if extra is not None else [])
    return np.hstack(panels)


def _start_visual_yaw(vcfg):
    """Second camera + tracker + stabilizer, or (None, None, None) if it can't start."""
    if not vcfg.get("enabled"):
        return None, None, None
    try:
        cam = Camera(vcfg["camera"])
    except Exception as e:  # never let the helper camera take the compass down with it
        log.error("visual_yaw camera failed to start (%s) - running on the compass alone", e)
        return None, None, None
    log.info("visual yaw: %s-facing %s camera, hold heading in %s, compass pull tau %.0f s",
             vcfg["orientation"], vcfg["camera"]["source"], ", ".join(vcfg["hold_modes"]), vcfg["tau_s"])
    return cam, VisualYawTracker.from_config(vcfg), HeadingStabilizer.from_config(vcfg)


def run(cfg, mode, debug_port_override=None):
    detector = NeedleDetector.from_config(cfg)
    estimator = HeadingEstimator(cfg["heading"])
    mcfg, lcfg, vcfg = cfg["mavlink"], cfg["logging"], cfg["visual_yaw"]

    link = None
    if mode != "dry-run":
        from .fc_link import FcLink  # imported here so --dry-run works without pymavlink
        link = FcLink(mcfg["device"], baud=mcfg["baud"], firmware=mcfg["firmware"],
                      source_system=mcfg["source_system"], source_component=mcfg["source_component"],
                      attitude_rate_hz=mcfg["attitude_rate_hz"])
        log.info("MAVLink on %s @ %s baud, firmware=%s, mode=%s",
                 mcfg["device"], mcfg["baud"], mcfg["firmware"], mode)

    auto_decl = estimator.declination is None
    if auto_decl:
        declination.require_model()  # fail now if pygeomag is missing, not mid-session
        if link:
            link.want_gps()
            log.info("declination: auto - will compute from the FC's GPS at the first 3D fix; "
                     "no heading is sent until then")
        else:
            log.warning("declination: auto needs the FC's GPS, so --dry-run shows magnetic heading only")
    else:
        log.info("declination: %+.2f deg (fixed, from config)", estimator.declination)

    camera = Camera(cfg["camera"])
    vis_cam, tracker, stabilizer = _start_visual_yaw(vcfg)
    csv_log = CsvLog(resolve_path(cfg, lcfg["csv_dir"])) if lcfg.get("csv_dir") else None
    if csv_log:
        log.info("logging to %s", csv_log.path)
    port = debug_port_override if debug_port_override is not None else int(lcfg.get("debug_port") or 0)
    stream = None
    if port > 0:
        from .debug_stream import DebugStream
        stream = DebugStream(port)

    stop = {"flag": False}

    def _stop(*_):
        stop["flag"] = True
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    min_send_period = 1.0 / max(1.0, float(mcfg["max_send_hz"]))
    base_std = float(cfg["heading"]["base_std_deg"])
    drift_warn = float(vcfg.get("drift_warn_deg", 8.0))
    last_send = 0.0
    seq = vis_seq = 0
    vis = vis_frame = None
    drift_since = None
    last_drift_msg = 0.0
    stats = {"frames": 0, "valid": 0, "sent": 0, "reasons": {}}
    last_status = time.monotonic()
    last = None
    warned_fc = False
    warned_settle = False

    try:
        while not stop["flag"]:
            got = camera.read(seq, timeout=1.0)
            if link:
                link.poll()
                link.tick()
                if estimator.declination is None:
                    fix = link.gps_fix()
                    if fix is not None:
                        try:
                            d = declination.declination_deg(*fix)
                        except Exception as e:  # keep running; retried on the next frame
                            log.error("declination calculation failed: %s", e)
                        else:
                            estimator.set_declination(d)
                            log.info("declination %+.2f deg from GPS fix %.5f, %.5f (%s)",
                                     d, fix[0], fix[1], declination.MODEL_NAME)
            if got is None:
                log.warning("no camera frames")
                continue
            frame, t_cap, seq = got

            # --- compass ---------------------------------------------------------
            reading = detector.detect(frame)
            att = link.attitude() if link else None
            gyro_dps = math.degrees(att.yawspeed) if att else None
            sample = estimator.update(t_cap, reading.angle_deg if reading.valid else None,
                                      reading.confidence, att)
            reason = reading.reason if not reading.valid else sample.reason
            fc_mode = link.flight_mode if link else None
            if estimator.settle.settle_time is not None:
                log.info("needle settled %.1f s after the turn", estimator.settle.settle_time)
            if estimator.settle.gave_up and not warned_settle:
                log.warning("needle never settled for %.0f s - using the compass anyway. Check the compass "
                            "mount for vibration, or raise heading.settle_max_rate_dps",
                            estimator.settle.give_up)
            warned_settle = estimator.settle.gave_up

            # --- ground camera: real rotation, holds the heading steady ------------
            st = None
            if tracker:
                vg = vis_cam.read(vis_seq, timeout=0.0)
                if vg is not None:
                    vis_frame, vt, vis_seq = vg
                    vis = tracker.update(vis_frame, vt, gyro_dps)
                st = stabilizer.update(t_cap, sample.true_deg if sample.valid else None, vis, fc_mode, gyro_dps)
                out_valid, out_deg = st.valid, st.heading_deg
                out_std = sample.std_deg if sample.valid else base_std
                if st.drift_deg is not None and abs(st.drift_deg) > drift_warn:
                    drift_since = drift_since or t_cap
                    if t_cap - drift_since > 3.0 and t_cap - last_drift_msg > 10.0:
                        last_drift_msg = t_cap
                        msg = "Compass drift %+.0fdeg, cam %s" % (st.drift_deg, "holding" if st.holding else "correcting")
                        log.warning(msg)
                        if link:
                            link.send_statustext(msg)
                else:
                    drift_since = None
            else:
                out_valid, out_deg, out_std = sample.valid, sample.true_deg, sample.std_deg

            sent = False
            if out_valid and link and mode == "send" and link.fc_connected():
                if t_cap - last_send >= min_send_period:
                    link.send_heading(t_cap, math.radians(wrap180(out_deg)), math.radians(out_std),
                                      quality=int(round(reading.confidence * 100)) if reading.valid else 50)
                    last_send = t_cap
                    sent = True
            latency_ms = (time.monotonic() - t_cap) * 1000.0

            fc_yaw = fc_diff = None
            if att is not None:
                fc_yaw = math.degrees(att.yaw) % 360.0
                if out_deg is not None:
                    fc_diff = wrap180(fc_yaw - out_deg)

            stats["frames"] += 1
            stats["valid"] += int(out_valid)
            stats["sent"] += int(sent)
            if reason:
                stats["reasons"][reason] = stats["reasons"].get(reason, 0) + 1
            last = (sample, out_deg, st, fc_yaw, latency_ms)

            if csv_log:
                vis_ok = vis is not None and vis.valid
                csv_log.write([time.time(), t_cap, latency_ms, reading.angle_deg, reading.confidence,
                               sample.mag_raw_deg, sample.mag_deg, estimator.declination,
                               sample.true_deg, sample.std_deg, reason,
                               int(vis_ok) if vis is not None else None,
                               vis.total_deg if vis_ok else None,
                               vis.inliers if vis is not None else None,
                               vis.reason if vis is not None else None,
                               out_deg, st.source if st else "compass",
                               int(st.holding) if st else None,
                               st.drift_deg if st else None, int(sent), fc_mode,
                               att.t if att else None,
                               math.degrees(att.roll) if att else None,
                               math.degrees(att.pitch) if att else None,
                               fc_yaw, gyro_dps, fc_diff])

            if stream is not None and stream.wants_frame():
                lines = ["out %s  compass %s  mag %s" % (_fmt(out_deg), _fmt(sample.true_deg), _fmt(sample.mag_deg))]
                if st is not None:
                    lines.append("held by %s  compass drift %s%s" % (st.source, _fmt(st.drift_deg, "%+.1f"),
                                                                    "  HOLD" if st.holding else ""))
                if fc_yaw is not None:
                    lines.append("FC %s yaw %.1f" % (fc_mode or "", fc_yaw))
                if sample.reason:
                    lines.append("compass gated: %s" % sample.reason)
                extra = tracker.debug_image(vis_frame) if (tracker and vis_frame is not None) else None
                stream.update(debug_view(frame, detector, reading, lines, extra))

            now = time.monotonic()
            if now - last_status >= float(lcfg.get("status_period_s", 2.0)):
                dt = now - last_status
                s, od, sst, fy, lat = last
                fc_txt = "dry-run"
                if link:
                    fc_txt = ("FC %s %s yaw %s" % (link.autopilot_name(), link.flight_mode or "", _fmt(fy))
                              if link.fc_connected() else "FC NOT CONNECTED")
                    if not link.fc_connected() and not warned_fc:
                        log.warning("no heartbeat from the flight controller on %s - check wiring, "
                                    "baud and that the port is set to MAVLink", mcfg["device"])
                        warned_fc = True
                    if link.fc_connected():
                        warned_fc = False
                if estimator.declination is not None:
                    decl_txt = "decl %+.1f%s" % (estimator.declination, " gps" if auto_decl else "")
                else:
                    decl_txt = "decl waiting for GPS fix"
                vis_txt = ""
                if tracker:
                    if vis is not None and vis.valid:
                        vis_txt = " | cam %d pts drift %s%s" % (vis.inliers, _fmt(sst.drift_deg, "%+.1f"),
                                                                " HOLD" if sst.holding else "")
                    else:
                        vis_txt = " | cam lost (%s)" % (vis.reason if vis is not None else "no frames")
                top = sorted(stats["reasons"].items(), key=lambda kv: -kv[1])[:3]
                log.info("%.0f fps | valid %d%% | sent %.0f Hz | out %s (compass %s, mag %s, %s)%s | %s | "
                         "latency %.0f ms%s",
                         stats["frames"] / dt, 100.0 * stats["valid"] / max(1, stats["frames"]),
                         stats["sent"] / dt, _fmt(od), _fmt(s.true_deg), _fmt(s.mag_deg), decl_txt, vis_txt,
                         fc_txt, lat, (" | rejects " + ", ".join("%s=%d" % kv for kv in top)) if top else "")
                stats = {"frames": 0, "valid": 0, "sent": 0, "reasons": {}}
                last_status = now
    finally:
        camera.close()
        if vis_cam:
            vis_cam.close()
        if link:
            link.close()
        if csv_log:
            csv_log.close()
        if stream:
            stream.close()


def _fmt(v, f="%.1f"):
    return "---" if v is None else f % v


def main(argv=None):
    p = argparse.ArgumentParser(prog="compass_vision",
                                description="Physical compass -> camera -> heading -> Pixhawk")
    p.add_argument("--config", default="config.yaml")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--shadow", action="store_true",
                   help="connect to the FC and log its heading next to ours, but send nothing")
    g.add_argument("--dry-run", action="store_true", help="no flight controller, just print/log")
    p.add_argument("--source", help="override compass camera: path to an image/video file")
    p.add_argument("--ground-source", help="override ground (visual_yaw) camera: image/video file")
    p.add_argument("--debug-port", type=int, help="serve MJPEG debug video on this port")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args(argv)

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    cfg = load_config(args.config)
    if args.source:
        cfg["camera"]["source"] = "file"
        cfg["camera"]["file_path"] = args.source
    if args.ground_source:
        cfg["visual_yaw"]["enabled"] = True
        cfg["visual_yaw"]["camera"]["source"] = "file"
        cfg["visual_yaw"]["camera"]["file_path"] = args.ground_source
    if not cfg["dial"]["points"]:
        p.error("not calibrated yet: run `python3 -m compass_vision.calibrate setup --config %s`" % args.config)
    mode = "shadow" if args.shadow else "dry-run" if args.dry_run else "send"
    run(cfg, mode, args.debug_port)


if __name__ == "__main__":
    main()
