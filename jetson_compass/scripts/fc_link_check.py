#!/usr/bin/env python3
"""Check the Jetson <-> Pixhawk MAVLink link without the camera.

    python3 scripts/fc_link_check.py --config config.yaml

Prints the autopilot it finds and the FC's own heading for 10 seconds.
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from compass_vision.config import load_config  # noqa: E402
from compass_vision.fc_link import FcLink  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument("--config", default="config.yaml")
p.add_argument("--seconds", type=float, default=10.0)
args = p.parse_args()
m = load_config(args.config)["mavlink"]

print("opening %s @ %s baud ..." % (m["device"], m["baud"]))
link = FcLink(m["device"], baud=m["baud"], firmware=m["firmware"],
              source_system=m["source_system"], source_component=m["source_component"],
              attitude_rate_hz=m["attitude_rate_hz"])
end = time.monotonic() + args.seconds
last_print = 0.0
seen = False
while time.monotonic() < end:
    link.poll()
    link.tick()
    now = time.monotonic()
    if link.fc_connected() and not seen:
        seen = True
        print("heartbeat from %s (system %s, component %s)" % (
            link.autopilot_name(), link.target_system, link.target_component))
        if link.autopilot_name().lower() != m["firmware"].lower():
            print("WARNING: config says firmware=%s but the FC is %s" % (m["firmware"], link.autopilot_name()))
    if now - last_print >= 0.5:
        last_print = now
        a = link.attitude()
        if a:
            print("FC heading %6.1f  roll %5.1f  pitch %5.1f" % (
                math.degrees(a.yaw) % 360, math.degrees(a.roll), math.degrees(a.pitch)))
    time.sleep(0.01)
link.close()
if not seen:
    print("No heartbeat. Check: TX->RX crossed, GND connected, baud matches, FC port set to MAVLink.")
    sys.exit(1)
