#!/usr/bin/env python3
"""Check that a Jetson UART works before blaming the flight controller.

Put a jumper wire between pin 8 (TX) and pin 10 (RX) on the 40-pin header
(nothing else connected), then:

    python3 scripts/uart_loopback_test.py /dev/ttyTHS1 921600

If you get your bytes back, that device name is the right port and it is free.
"""

import sys
import time

import serial

dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyTHS1"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 921600
payload = b"compass-loopback-0123456789"

try:
    port = serial.Serial(dev, baud, timeout=0.5)
except serial.SerialException as e:
    sys.exit("cannot open %s: %s\n(try: sudo usermod -aG dialout $USER, then log out/in)" % (dev, e))

port.reset_input_buffer()
port.write(payload)
time.sleep(0.1)
got = port.read(len(payload) * 4)
port.close()

if payload in got:
    print("OK: %s at %d baud loops back. This is your port." % (dev, baud))
elif got:
    print("Got data back but not what was sent: %r" % got)
    print("Something else (a serial console?) is using %s. Try: sudo systemctl disable --now nvgetty" % dev)
else:
    print("Nothing came back on %s. Wrong device, or pins 8/10 not jumpered." % dev)
    sys.exit(1)
