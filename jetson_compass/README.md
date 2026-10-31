# Jetson compass vision → Pixhawk heading

A camera on a Jetson watches a **physical needle compass** on the drone, works out
the heading, and sends it to a **Pixhawk 6C Mini** over UART in place of the
magnetometer.

- What it does and why it's built this way, plus a test plan: [docs/PROJECT_BRIEF.md](docs/PROJECT_BRIEF.md)
- Short version of the data path: it does **not** fake magnetometer data. It sends the heading as
  an external-vision yaw measurement over MAVLink, which PX4 and ArduPilot both accept natively.

```
camera ──► find red north tip ──► needle angle ──► heading (+deviation, +declination)
                                                         │  skip if tilted / spinning / glitch
                                                         ▼
                              Pixhawk EKF ◄── MAVLink over UART (TELEM2)
                              (PX4: ODOMETRY yaw  |  ArduPilot: VISION_POSITION_ESTIMATE yaw)
```

---

## 1. Hardware

**Mounting.** Put the compass and camera on one rigid bracket so they can't move
relative to each other. Vibration-isolate the bracket as a whole. Keep the compass
as far as you can from power wires, ESCs, motors, the battery and steel screws: it
is just as sensitive to them as the digital mag was. Use a liquid-damped compass
with a red (or otherwise distinct) north end. If the needle isn't colour-coded,
put a dot of bright paint on the north tip.

**Buy a compass balanced for North America.** In San Marcos the magnetic field dips
about 58° into the ground, and a compass needle is weighted to cancel the dip for
the region it's sold for. Compass makers split the world into "zones"; North America
is usually zone 1 / "MN". A compass balanced for another zone tilts and drags on its
pivot, which is worst when the drone is moving. A "global needle" compass also works.

**Camera tips.** Refocus the lens for the short distance: Pi-style lenses come
focused at infinity, so twist the lens. Keep the dial near the centre of the image.
Add a small LED so lighting is consistent, then cap exposure (`csi_exposure_range_ns`)
to kill motion blur. The camera doesn't have to be straight above the compass;
calibration corrects for the angle.

**Wiring (3 wires, both sides 3.3 V logic):**

| Jetson 40-pin header | | Pixhawk 6C Mini TELEM2 (JST-GH 6-pin) |
|---|---|---|
| Pin 8 (UART TX) | → | Pin 3 (RX) |
| Pin 10 (UART RX) | ← | Pin 2 (TX) |
| Pin 6 (GND) | — | Pin 6 (GND) |

Do **not** connect TELEM pin 1 (5 V) to the Jetson. Leave CTS/RTS unconnected and turn flow control off on the FC.

## 2. Jetson setup

```bash
git clone <this repo> ~/Donavan-Repository
cd ~/Donavan-Repository/jetson_compass
sudo apt install python3-pip python3-yaml python3-numpy
pip3 install --user pymavlink pyserial pygeomag
sudo usermod -aG dialout $USER          # serial port access, then log out/in
```

Use the OpenCV that ships with JetPack: **don't** `pip install opencv-python`, because it has
no GStreamer and the CSI camera won't open. Check with
`python3 -c "import cv2; print(cv2.getBuildInformation())" | grep GStreamer`. It should say YES.

**Find the UART.** Jumper pin 8 to pin 10, then run
`python3 scripts/uart_loopback_test.py /dev/ttyTHS1`. Try `/dev/ttyTHS0` if that fails.
Usually it's `ttyTHS1` on a Nano, and `ttyTHS0` or `ttyTHS1` on Xavier NX and Orin depending on JetPack.
If a login prompt or other stray data shows up, run `sudo systemctl disable --now nvgetty`.
Put the working device in `config.yaml → mavlink.device`.

**Set `config.yaml`:** `camera.source` (csi/usb). The defaults are already
`mavlink.firmware: ardupilot` and `heading.declination_deg: auto`.

**Declination is automatic.** The compass reads magnetic north and ArduPilot's EKF
works in true north. With `auto`, the Jetson asks the Pixhawk for its GPS position
(`GPS_RAW_INT`) and computes the declination once at the first 3D fix, using the World
Magnetic Model 2025 (`pygeomag`, offline, no internet needed). It sends **no heading until
then**, so the drone won't arm before GPS lock. The value shows in the status line and
in each CSV row. For flying without GPS (indoors), put a fixed number there instead
(east positive, from the [NOAA calculator](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml)).

**Our flying site, San Marcos** (WMM2025, late 2026, drifts about −0.1°/year):

| Site | Declination | Fixed value for no-GPS flights |
|---|---|---|
| San Marcos, TX | 3.2° E | `declination_deg: 3.2` |
| San Marcos, CA | 10.9° E | `declination_deg: 10.9` |

With `auto` you'll see the matching value in the status line (`decl +3.2 gps` or
`decl +10.9 gps`). If it shows something very different, the GPS position is wrong.

## 3. Flight controller parameters

Leave the GPS/baro position settings alone. Only the yaw source changes.

### ArduPilot (4.1+), in Mission Planner

| Parameter | Value | Why |
|---|---|---|
| `SERIAL2_PROTOCOL` | 2 | MAVLink2 on TELEM2 |
| `SERIAL2_BAUD` | 921 | 921600, must match `mavlink.baud` |
| `BRD_SER2_RTSCTS` | 0 | no flow control wires |
| `VISO_TYPE` | 1 | MAVLink vision input (reboot after setting) |
| `EK3_SRC1_YAW` | 6 | yaw from ExternalNav (us) |
| `EK3_SRC1_POSXY` / `VELXY` / `POSZ` | unchanged (GPS/GPS/Baro) | **never 6**, we don't send position |
| `COMPASS_ENABLE` | 0 | magnetometers off |
| `VISO_YAW_M_NSE` | 0.05 | yaw noise in rad (~3°). ArduPilot uses this, not our per-frame value. |
| `VISO_DELAY_MS` | 80 to start | then use the value from `estimate_latency.py` |

If data stops, pre-arm reports "VisOdom: not healthy", which is what you want.
You don't need to change the `SR2_*` stream rates. The Jetson asks for the
`ATTITUDE` and `GPS_RAW_INT` messages it needs.

### PX4 (v1.14+), only if you ever switch (set `mavlink.firmware: px4`)

| Parameter | Value | Why |
|---|---|---|
| `MAV_1_CONFIG` | TELEM 2 | MAVLink on the port the Jetson is on (reboot, then the next ones appear) |
| `MAV_1_MODE` | Onboard | companion-computer profile |
| `SER_TEL2_BAUD` | 921600 | must match `mavlink.baud` |
| `MAV_1_FLOW_CTRL` | Force off | only TX/RX/GND wired (if your version has this param) |
| `EKF2_EV_CTRL` | 8 | fuse vision **yaw only** (bit 3). Never enable position bits. |
| `EKF2_MAG_TYPE` | None (5) | stop fusing the magnetometer |
| `SYS_HAS_MAG` | 0 | no magnetometer needed to arm |
| `EKF2_EV_NOISE_MD` | 0 | use the uncertainty we send... |
| `EKF2_EVA_NOISE` | 0.05 | ...but never trust it better than ~3° (rad) |
| `EKF2_EV_DELAY` | 80 to start | then use the value from `estimate_latency.py` (ms) |

Check it in the MAVLink console: `listener vehicle_visual_odometry` should update,
and `listener estimator_status_flags` should show `cs_ev_yaw: True`.

> `mavlink.firmware` in `config.yaml` must match what's on the Pixhawk. `scripts/fc_link_check.py` warns if it doesn't.

## 4. Calibrate

```bash
# 1. Dial + needle colour (needs a screen: monitor on the Jetson, or ssh -X)
python3 -m compass_vision.calibrate setup
#    Click the dial rim at the drone's NOSE, RIGHT, TAIL, LEFT (in that order),
#    then click the red north tip in the "rectified" window. [ / ] adjust colour, s saves.
#    Headless alternative: `calibrate snapshot`, copy snapshot.jpg to a laptop,
#    run `calibrate setup --image snapshot.jpg` there, copy calibration.yaml back.

# 2. Compass swing (plain SSH is fine). Outdoors, drone powered as in flight.
python3 -m compass_vision.calibrate swing
#    Point the nose at a known MAGNETIC heading (handheld compass a few metres away,
#    or a road/runway edge), type it, repeat. 8 headings (every 45°) is good,
#    1 heading = simple offset. References must be MAGNETIC with declination: auto
#    (with a fixed declination number you can use --true for true-heading references).
```

Results go to `calibration.yaml`, which overrides `config.yaml`. You can edit it by hand.

## 5. Run

```bash
python3 -m compass_vision --dry-run     # no FC: just check the reading is solid
python3 -m compass_vision --shadow      # talk to the FC, log its heading next to ours, send nothing
python3 -m compass_vision               # the real thing: send heading to the FC
```

The status line prints every 2 s:
`30 fps | valid 100% | sent 30 Hz | mag 115.9 true 123.4 (decl +7.5 gps) | FC ArduPilot yaw 122.9 | latency 6 ms`

- **Debug video:** open `http://<jetson-ip>:8080/` in a browser (raw view, rectified dial, mask, heading).
- **Logs:** `logs/compass_*.csv`, one row per frame (our heading, FC heading and attitude, reject reason, latency).
- **Latency:** in shadow mode, props off, twist the drone left/right for ~20 s, then
  `python3 scripts/estimate_latency.py logs/<file>.csv`. It prints the `EKF2_EV_DELAY` / `VISO_DELAY_MS` to use.
- **Start on boot:** edit the username in `scripts/compass-vision.service`, then
  `sudo cp scripts/compass-vision.service /etc/systemd/system/ && sudo systemctl enable --now compass-vision`.
  Watch it with `journalctl -u compass-vision -f`.

Follow the bring-up order in [docs/PROJECT_BRIEF.md](docs/PROJECT_BRIEF.md#bring-up-and-test-plan) before flying.

## 6. Testing without the drone

```bash
pip3 install --user pytest && python3 -m pytest tests     # synthetic dial images + MAVLink round trip
python3 -m compass_vision --dry-run --source some_video.mp4
```

**SITL** (checks that the FC accepts the messages): set the params above in the simulator.
- PX4: `make px4_sitl gz_x500`, then `mavlink.device: udpin:0.0.0.0:14540`.
- ArduPilot: `sim_vehicle.py -v ArduCopter`, set `SERIAL1_PROTOCOL 2`, then `mavlink.device: tcp:127.0.0.1:5762`.

Feed it a still image (`--source`) and the simulated vehicle's heading should snap to the image's heading.

## 7. Troubleshooting

| Symptom | Likely cause |
|---|---|
| `could not open camera source 'csi'` | pip OpenCV instead of JetPack's. Or try `sudo systemctl restart nvargus-daemon`. |
| `FC NOT CONNECTED` | TX/RX not crossed, GND missing, baud mismatch, FC port not set to MAVLink, wrong `/dev/ttyTHS*` |
| Status shows `no_needle_pixels` / `blob_*` rejects | Lighting changed or colour range too tight. Watch the debug video and re-run `calibrate setup`. |
| Heading turns the **wrong way** when you rotate the drone | Rim points clicked in the wrong order (must be nose, right, tail, left) |
| Heading off by ~180° | You sampled the south end's colour. Click the north tip. |
| Status shows `true ---` / `waiting_for_gps_declination`, nothing sent | No 3D GPS fix yet. Wait for lock, or set a fixed `declination_deg` for flying without GPS. |
| Heading off by a constant | Run the swing. With a fixed declination, check its sign (east = +). |
| FC heading jumps when the drone tilts or spins fast | Expected: those readings are skipped and the FC coasts on its gyros. Raise `max_tilt_deg` / `max_yaw_rate_dps` only if your compass handles it. |
| Drone circles in position hold | Heading error. Recheck declination, swing and delay. |

## Layout

```
compass_vision/   main.py (loop), needle.py (vision), heading.py (math + gates),
                  fc_link.py (MAVLink), declination.py (WMM from GPS), camera.py,
                  calibrate.py, debug_stream.py, config.py
scripts/          uart_loopback_test.py, fc_link_check.py, estimate_latency.py, compass-vision.service
tests/            synthetic compass images, heading math, MAVLink round trip, latency estimate
config.yaml       your settings     calibration.yaml   written by the calibrate tool
```
