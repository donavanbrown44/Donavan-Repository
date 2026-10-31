"""Main loop: camera -> needle angle -> heading -> Pixhawk.

Usage:
  python3 -m compass_vision --config config.yaml            # normal: send heading to FC
  python3 -m compass_vision --config config.yaml --shadow   # read FC, log, DON'T send
  python3 -m compass_vision --config config.yaml --dry-run  # no FC at all, just print
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

from .camera import Camera
from .config import load_config, resolve_path
from .heading import HeadingEstimator, wrap180
from .needle import NeedleDetector

log = logging.getLogger("compass_vision")

CSV_FIELDS = ["unix_time", "t_capture", "latency_ms", "needle_deg", "confidence",
              "mag_raw_deg", "mag_deg", "true_deg", "std_deg", "sent", "reason",
              "fc_att_time", "fc_roll_deg", "fc_pitch_deg", "fc_yaw_deg", "fc_yawspeed_dps",
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


def debug_view(frame, detector, reading, lines):
    """Raw frame (with the calibration points) next to the annotated rectified dial."""
    rect = detector.annotate(reading, lines)
    h = rect.shape[0]
    scale = float(h) / frame.shape[0]
    raw = cv2.resize(frame, (int(frame.shape[1] * scale), h))
    pts = cv2.perspectiveTransform(np.float32([[[detector.center, detector.center]]]),
                                   np.linalg.inv(detector.H))[0][0] * scale
    cv2.circle(raw, (int(pts[0]), int(pts[1])), 4, (0, 255, 255), -1)
    return np.hstack([raw, rect])


def run(cfg, mode, debug_port_override=None):
    detector = NeedleDetector.from_config(cfg)
    estimator = HeadingEstimator(cfg["heading"])
    mcfg = cfg["mavlink"]
    lcfg = cfg["logging"]

    link = None
    if mode != "dry-run":
        from .fc_link import FcLink  # imported here so --dry-run works without pymavlink
        link = FcLink(mcfg["device"], baud=mcfg["baud"], firmware=mcfg["firmware"],
                      source_system=mcfg["source_system"], source_component=mcfg["source_component"],
                      attitude_rate_hz=mcfg["attitude_rate_hz"])
        log.info("MAVLink on %s @ %s baud, firmware=%s, mode=%s",
                 mcfg["device"], mcfg["baud"], mcfg["firmware"], mode)

    camera = Camera(cfg["camera"])
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
    last_send = 0.0
    seq = 0
    stats = {"frames": 0, "valid": 0, "sent": 0, "reasons": {}}
    last_status = time.monotonic()
    last = None
    warned_fc = False

    try:
        while not stop["flag"]:
            got = camera.read(seq, timeout=1.0)
            if link:
                link.poll()
                link.tick()
            if got is None:
                log.warning("no camera frames")
                continue
            frame, t_cap, seq = got

            reading = detector.detect(frame)
            att = link.attitude() if link else None
            sample = estimator.update(t_cap, reading.angle_deg if reading.valid else None,
                                      reading.confidence, att)
            reason = reading.reason if not reading.valid else sample.reason

            sent = False
            if sample.valid and link and mode == "send" and link.fc_connected():
                if t_cap - last_send >= min_send_period:
                    yaw = math.radians(wrap180(sample.true_deg))
                    link.send_heading(t_cap, yaw, math.radians(sample.std_deg),
                                      quality=int(round(reading.confidence * 100)))
                    last_send = t_cap
                    sent = True
            latency_ms = (time.monotonic() - t_cap) * 1000.0

            fc_yaw = fc_diff = None
            if att is not None:
                fc_yaw = math.degrees(att.yaw) % 360.0
                if sample.true_deg is not None:
                    fc_diff = wrap180(fc_yaw - sample.true_deg)

            stats["frames"] += 1
            stats["valid"] += int(sample.valid)
            stats["sent"] += int(sent)
            if reason:
                stats["reasons"][reason] = stats["reasons"].get(reason, 0) + 1
            last = (sample, reading, fc_yaw, latency_ms)

            if csv_log:
                csv_log.write([time.time(), t_cap, latency_ms, reading.angle_deg, reading.confidence,
                               sample.mag_raw_deg, sample.mag_deg, sample.true_deg, sample.std_deg,
                               int(sent), reason,
                               att.t if att else None,
                               math.degrees(att.roll) if att else None,
                               math.degrees(att.pitch) if att else None,
                               fc_yaw, math.degrees(att.yawspeed) if att else None, fc_diff])

            if stream is not None and stream.wants_frame():
                lines = ["true hdg %s  mag %s" % (_fmt(sample.true_deg), _fmt(sample.mag_deg))]
                if fc_yaw is not None:
                    lines.append("FC yaw %.1f" % fc_yaw)
                if sample.reason:
                    lines.append("gated: %s" % sample.reason)
                stream.update(debug_view(frame, detector, reading, lines))

            now = time.monotonic()
            if now - last_status >= float(lcfg.get("status_period_s", 2.0)):
                dt = now - last_status
                s, r, fy, lat = last
                fc_txt = "dry-run"
                if link:
                    fc_txt = ("FC %s yaw %s" % (link.autopilot_name(), _fmt(fy))
                              if link.fc_connected() else "FC NOT CONNECTED")
                    if not link.fc_connected() and not warned_fc:
                        log.warning("no heartbeat from the flight controller on %s - check wiring, "
                                    "baud and that the port is set to MAVLink", mcfg["device"])
                        warned_fc = True
                    if link.fc_connected():
                        warned_fc = False
                top = sorted(stats["reasons"].items(), key=lambda kv: -kv[1])[:3]
                log.info("%.0f fps | valid %d%% | sent %.0f Hz | true hdg %s | %s | latency %.0f ms%s",
                         stats["frames"] / dt, 100.0 * stats["valid"] / max(1, stats["frames"]),
                         stats["sent"] / dt, _fmt(s.true_deg), fc_txt, lat,
                         (" | rejects " + ", ".join("%s=%d" % kv for kv in top)) if top else "")
                stats = {"frames": 0, "valid": 0, "sent": 0, "reasons": {}}
                last_status = now
    finally:
        camera.close()
        if link:
            link.close()
        if csv_log:
            csv_log.close()
        if stream:
            stream.close()


def _fmt(v):
    return "---" if v is None else "%.1f" % v


def main(argv=None):
    p = argparse.ArgumentParser(prog="compass_vision",
                                description="Physical compass -> camera -> heading -> Pixhawk")
    p.add_argument("--config", default="config.yaml")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--shadow", action="store_true",
                   help="connect to the FC and log its heading next to ours, but send nothing")
    g.add_argument("--dry-run", action="store_true", help="no flight controller, just print/log")
    p.add_argument("--source", help="override camera: path to an image/video file")
    p.add_argument("--debug-port", type=int, help="serve MJPEG debug video on this port")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args(argv)

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    cfg = load_config(args.config)
    if args.source:
        cfg["camera"]["source"] = "file"
        cfg["camera"]["file_path"] = args.source
    if not cfg["dial"]["points"]:
        p.error("not calibrated yet: run `python3 -m compass_vision.calibrate setup --config %s`" % args.config)
    mode = "shadow" if args.shadow else "dry-run" if args.dry_run else "send"
    run(cfg, mode, args.debug_port)


if __name__ == "__main__":
    main()
