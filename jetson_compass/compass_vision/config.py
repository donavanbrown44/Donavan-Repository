"""Configuration loading.

Two files:
  config.yaml       - hand-edited settings (camera, UART, declination, gates...)
  calibration.yaml  - written by the calibrate tool (dial points, needle colour,
                      deviation table). Values here override config.yaml.

Keeping machine-written values in their own file means the calibrate tool never
rewrites (and strips the comments out of) the file you edit by hand.
"""

import copy
import os

import yaml

DEFAULTS = {
    "calibration_file": "calibration.yaml",
    "camera": {
        "source": "csi",            # csi | usb | gstreamer | file
        "csi_sensor_id": 0,
        "csi_exposure_range_ns": "",  # e.g. "100000 2000000" to cap exposure (less blur)
        "usb_index": 0,
        "gstreamer_pipeline": "",   # used when source == gstreamer
        "file_path": "",            # image or video, used when source == file
        "width": 1280,
        "height": 720,
        "fps": 30,
    },
    "dial": {
        # Four image points on the dial rim, in this order:
        # [nose, right, tail, left]  (right = the drone's right side)
        "points": None,
        # Only pixels between these fractions of the dial radius count as needle.
        # Inner cuts out the pivot hub, outer cuts out bezel / printed markings.
        "annulus": [0.15, 0.85],
    },
    "needle": {
        # HSV ranges (OpenCV: H 0-179, S/V 0-255) for the NORTH end of the needle.
        # Each entry: [h_lo, s_lo, v_lo, h_hi, s_hi, v_hi]. Default = red.
        "hsv_ranges": [[0, 90, 60, 10, 255, 255], [170, 90, 60, 179, 255, 255]],
        "min_area_frac": 0.002,     # of the annulus area
        "max_area_frac": 0.25,
        "min_radial_extent": 0.25,  # blob must span this much of the radius
        "min_concentration": 0.90,  # 1.0 = every pixel points the same way
    },
    "heading": {
        "declination_deg": "auto",  # "auto" (from the FC's GPS) or degrees, EAST positive
        "deviation_table": [],      # [[measured_mag_deg, correction_deg], ...]
        "smoothing_alpha": 1.0,     # 1.0 = no smoothing (let the EKF filter)
        "max_jump_deg": 35.0,       # reject frame-to-frame jumps bigger than this
        "reacquire_frames": 6,      # ...unless this many in a row agree
        "reset_after_s": 0.5,       # gap after which the next reading is trusted
        "base_std_deg": 3.0,        # 1-sigma heading noise reported to the FC
        "max_tilt_deg": 20.0,       # don't send while tilted more than this
        "max_yaw_rate_dps": 90.0,   # don't send while yawing faster than this
    },
    "mavlink": {
        "firmware": "ardupilot",    # ardupilot | px4
        "device": "/dev/ttyTHS1",   # or e.g. udpin:0.0.0.0:14540 for SITL
        "baud": 921600,
        "source_system": 1,
        "source_component": 197,    # MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
        "max_send_hz": 30.0,
        "attitude_rate_hz": 20.0,
    },
    "logging": {
        "csv_dir": "logs",          # "" disables CSV logging
        "debug_port": 0,            # >0 serves MJPEG debug video on this port
        "status_period_s": 2.0,
    },
}


def deep_merge(base, override):
    """Return a copy of base with override merged in. Lists are replaced, not merged."""
    out = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def _read_yaml(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    return data or {}


def calibration_path(cfg, config_path):
    cal = cfg.get("calibration_file") or "calibration.yaml"
    if os.path.isabs(cal):
        return cal
    return os.path.join(os.path.dirname(os.path.abspath(config_path)), cal)


def load_config(config_path):
    cfg = deep_merge(DEFAULTS, _read_yaml(config_path))
    cal_path = calibration_path(cfg, config_path)
    if os.path.exists(cal_path):
        cfg = deep_merge(cfg, _read_yaml(cal_path))
    cfg["_config_path"] = os.path.abspath(config_path)
    cfg["_calibration_path"] = cal_path
    return cfg


def resolve_path(cfg, path):
    """Resolve a path from the config relative to the config file's folder."""
    if not path or os.path.isabs(path):
        return path
    return os.path.join(os.path.dirname(cfg["_config_path"]), path)


def save_calibration(cfg, updates):
    """Merge `updates` into calibration.yaml and write it back."""
    path = cfg["_calibration_path"]
    current = _read_yaml(path) if os.path.exists(path) else {}
    merged = deep_merge(current, updates)
    text = yaml.safe_dump(merged, default_flow_style=None)
    with open(path, "w") as f:
        f.write("# Written by `python3 -m compass_vision.calibrate`.\n")
        f.write("# Values here override config.yaml. Safe to hand-edit.\n")
        f.write(text)
    return path
