# Project brief: getting drone heading from a camera reading a physical compass

*This is the original idea, rewritten as a spec. The approach is fixed. This is an
experiment, and the job is to make it work well enough to measure.*

## Goal

Replace the drone's digital magnetometer as the heading source with a **physical
needle compass** mounted on the airframe. A camera on a **Jetson** watches the
needle and works out the heading. The Jetson sends that heading to a **Pixhawk 6C
Mini** over **UART**, and the flight controller's EKF uses it for yaw in place of
the magnetometer.

## System

| Part | Role |
|---|---|
| Physical compass (liquid-damped, red north tip) | Mounted rigidly on the airframe, face visible to the camera |
| Camera (Jetson CSI such as IMX219, or USB) | On the **same rigid bracket** as the compass, so the two never move relative to each other |
| Second camera, looking straight down | Measures real rotation from the ground, to hold the heading steady and catch compass drift |
| Jetson (Nano / Xavier NX / Orin, any JetPack) | Runs `compass_vision`, a Python + OpenCV program. No GPU or ML needed. |
| Pixhawk 6C Mini running **ArduPilot** 4.1+ (PX4 v1.14+ also supported) | Fuses the heading. Gets position from GPS/baro as before. |
| 3 wires: TX, RX, GND | Jetson 40-pin UART to Pixhawk TELEM2, 3.3 V logic on both ends |

## How the heading gets into the Pixhawk (decided)

**Don't fake raw magnetometer data.** To fake it you would need the drone's tilt,
the local field strength and the field's dip angle, all to build a 3-axis field
vector. The FC would then turn that vector straight back into a heading, adding
error at both steps. Neither autopilot accepts raw magnetometer data over MAVLink
in normal flight either. You'd have to impersonate a specific sensor.

**Do send the heading itself**, through the "external yaw" input both autopilots
already have for vision systems. Only the yaw field is filled in:

| Firmware | MAVLink message | FC parameters that make it the yaw source |
|---|---|---|
| PX4 | `ODOMETRY` (#331), `LOCAL_NED` frame, position/velocity = NaN, quaternion = yaw only, yaw variance included | `EKF2_EV_CTRL = 8` (vision yaw only), `EKF2_MAG_TYPE = 5` (no mag) |
| ArduPilot | `VISION_POSITION_ESTIMATE` (#102), yaw only (position unused) | `VISO_TYPE = 1`, `EK3_SRC1_YAW = 6` (ExternalNav), compass off |

Plus a 1 Hz heartbeat out. The Jetson also reads the FC's `ATTITUDE` (tilt and yaw rate).

## Requirements

1. Capture at 30 fps or more. Always process the **newest** frame and never a queued, stale one.
2. Measure the needle angle relative to the drone's nose, and tell **north from south**
   by the needle's coloured north end. A plain line would be ambiguous by 180°.
3. Correct for the camera not being exactly overhead (oblique view, rotation, mirror)
   using a 4-point calibration on the dial rim.
4. Convert in this order: needle angle → magnetic heading → **deviation correction** →
   **+ declination** → true heading. The EKF works in true north. Compute the declination
   automatically from the Pixhawk's GPS position with the World Magnetic Model, once at the
   first 3D fix, and send no heading before that. A fixed value is allowed for flying without GPS.
5. **Don't send a reading when it can't be trusted:** needle not found or not
   needle-shaped, drone tilted past the limit, yawing too fast, or a one-frame jump.
   When nothing arrives, the FC coasts on its gyros. Never send a stale value.
6. Send about 30 Hz with a capture timestamp and an uncertainty.
7. Log every frame to CSV (our heading, the FC's heading, why a frame was rejected) to compare with the FC log after flights.
8. Run headless: start on boot (systemd) and serve live debug video over Wi-Fi.
9. Include tools for: dial/colour calibration, a compass swing (deviation table), a UART loopback test,
   an FC link check, and latency estimation.
10. Allow testing without hardware: synthetic-image unit tests, video-file input, SITL.
11. **Loiter must be rock solid: no slow rotation.** A second, ground-facing camera measures the
    drone's real rotation and acts as a feedback loop on the compass. It holds the heading outright in
    Loiter/PosHold/Brake, lets the compass correct it only slowly in other modes, and reports compass
    drift to the ground station. If the camera loses the ground, the heading falls back to the compass smoothly.

**Non-goals:** machine learning, custom firmware, position estimation, any FC change beyond parameters.

## Things the original idea didn't cover

| Issue | Why it matters | How it's handled |
|---|---|---|
| **Magnetic vs true north** | The compass points to magnetic north and the EKF wants true north. The difference is the declination, which can be 10–20° depending on location. | Computed on the Jetson from the Pixhawk's GPS fix (WMM2025), or a fixed value in config |
| **North vs south end** | A needle seen as a line reads the same at θ and θ+180° | Detect the coloured north end, and check that the blob is needle-shaped |
| **Camera angle** | An oblique view squashes the dial into an ellipse and bends angles | Homography from 4 clicked rim points |
| **Mount offset + airframe magnetism** | The physical compass is **also** pulled by steel screws, the battery, power wires and motor magnets. It's the same physics as the digital mag. | Compass swing builds a deviation table (what pilots do for aircraft compasses), plus a throttle interference test |
| **Needle physics** | A needle compass misreads when tilted, during acceleration and turns, and lags or overshoots while yawing | Readings skipped above `max_tilt_deg`, while turning, and until the needle stops swinging after a turn. That is detected by comparing the compass with the FC's gyro, so it adapts to your compass, and the log reports its settle time. Use a liquid-damped compass. |
| **Latency** | Camera + processing ≈ 50–120 ms. If the EKF thinks the reading is fresh, it fights the gyros. | Capture timestamps, `EKF2_EV_DELAY` / `VISO_DELAY_MS`, and `estimate_latency.py` to measure them |
| **Dropouts** | Glare, prop shadows, a loose cable | Send nothing, so the FC uses its gyros. ArduPilot won't arm without the data, and PX4 stops fusing after ~200 ms of silence. |
| **Startup order** | The Jetson boots in ~30–60 s and the Pixhawk in ~5 s | The FC fails pre-arm (no heading) until data flows. systemd restarts the program if it crashes. |
| **Camera practicalities** | Pi-style lenses are focused at infinity. Vibration blurs the image. Auto-exposure shifts colours. | Refocus the lens for the close range, mount on vibration damping together, cap exposure, add a small LED |
| **Slow compass creep in Loiter** | Loiter holds the EKF heading. If the compass creeps (needle settling, magnetic field from the power wires changing as current rises), the drone slowly yaws with it. | Ground camera holds the heading (see requirement 11). Drift is logged and reported. |
| **Trust before flying** | You don't want the first test of the heading to be in the air | Shadow mode logs our heading next to the FC's existing one while sending nothing |

## Bring-up and test plan

1. **Bench, no FC:** unit tests pass. `--dry-run` with the real compass shows a steady reading (±1°) with the drone still.
2. **Calibration:** `calibrate setup`, then `calibrate swing` at 8 headings. Residual error under ~3°.
   If the corrections vary by more than 20° around the circle, move the compass away from magnetic parts.
3. **Link:** UART loopback test, then `fc_link_check.py` sees heartbeats and attitude.
4. **Shadow mode, props off:** walk the drone around and rotate it. Compare `true_deg` with `fc_yaw_deg` in the CSV.
   Twist it back and forth for 20 s and run `estimate_latency.py` to get the delay parameter.
5. **Throttle interference, props OFF, drone secured:** run the motors through the throttle range
   while logging. If the heading moves by more than a few degrees, move the compass further from power wiring.
6. **Switch FC params** to vision yaw. Rotate by hand and confirm the FC heading follows.
   Cover the camera: the FC should keep its heading on gyros and report the loss, and ArduPilot should refuse to arm.
7. **Fly:** hover in a mode that doesn't need heading (Stabilize/Manual), then Position/Loiter.
   Circling or "toilet-bowling" in position hold means the heading is wrong (check declination, swing, delay). Keep a kill switch ready.
8. **Loiter hold test:** hover in Loiter for 3+ minutes. `out_deg` in the CSV should be flat and the drone
   shouldn't rotate. Plot `compass_drift_deg` against battery current from the FC log: if they climb
   together, the power wiring is affecting the compass (the camera hides it, but fix the cause).
9. **Analyse:** CSV + FC log. Look at EKF yaw innovations, rejected-frame reasons, and latency.
