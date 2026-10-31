"""MAVLink link to the Pixhawk over UART.

How the heading gets into the flight controller
------------------------------------------------
We do NOT fake raw magnetometer data. Instead we use the "external yaw" input that
both autopilots already have for vision systems, and fill in only the yaw:

  PX4       -> ODOMETRY (#331), frame LOCAL_NED, position/velocity = NaN,
               quaternion = pure yaw, yaw variance in the covariance.
               FC side: EKF2_EV_CTRL = 8 (fuse vision yaw only).
  ArduPilot -> VISION_POSITION_ESTIMATE (#102), position = 0 (unused),
               roll = pitch = 0, yaw = heading.
               FC side: VISO_TYPE = 1, EK3_SRC1_YAW = 6 (ExternalNav),
               position sources left on GPS.

Both expect yaw relative to TRUE north in radians, NED convention
(0 = north, +pi/2 = east).

The link also sends a 1 Hz heartbeat, reads the FC's ATTITUDE so the heading
estimator can skip readings while the drone is tilted or spinning fast, and (with
declination_deg: auto) reads GPS_RAW_INT so the Jetson can compute declination.
"""

import math
import os
import time

os.environ.setdefault("MAVLINK20", "1")  # ODOMETRY needs MAVLink 2; must be set before import
from pymavlink import mavutil  # noqa: E402

from .heading import Attitude  # noqa: E402

mavlink = mavutil.mavlink
NAN = float("nan")


class FcLink(object):
    def __init__(self, device, baud=921600, firmware="px4", source_system=1,
                 source_component=197, attitude_rate_hz=20.0, connection=None):
        firmware = str(firmware).lower()
        if firmware not in ("px4", "ardupilot"):
            raise ValueError("mavlink.firmware must be 'px4' or 'ardupilot', got %r" % firmware)
        self.firmware = firmware
        self.attitude_rate_hz = float(attitude_rate_hz)
        if connection is None:
            connection = mavutil.mavlink_connection(
                device, baud=int(baud), source_system=int(source_system),
                source_component=int(source_component), autoreconnect=True)
        self.conn = connection
        self.mav = connection.mav

        self.target_system = None
        self.target_component = None
        self.fc_autopilot = None
        self._last_fc_heartbeat = 0.0
        self._attitude = None
        self._gps = None
        self._last_heartbeat_sent = 0.0
        # Messages we ask the FC to stream (msg id -> Hz). Re-asked until they arrive,
        # so it works whatever the port's stream-rate params (SRx_*) are set to.
        self._wanted = {}
        if self.attitude_rate_hz > 0:
            self._wanted[mavlink.MAVLINK_MSG_ID_ATTITUDE] = self.attitude_rate_hz
        self._last_rx = {}
        self._last_request = {}
        self.sent_count = 0

    def want_gps(self, rate_hz=1.0):
        """Also stream GPS_RAW_INT (used to compute declination)."""
        self._wanted[mavlink.MAVLINK_MSG_ID_GPS_RAW_INT] = float(rate_hz)

    # ------------------------------------------------------------------ inbound
    def poll(self):
        """Drain whatever bytes are waiting. Non-blocking."""
        now = time.monotonic()
        while True:
            try:
                msg = self.conn.recv_match(blocking=False)
            except Exception:  # serial hiccup; autoreconnect handles the port
                break
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == "HEARTBEAT":
                if msg.autopilot == mavlink.MAV_AUTOPILOT_INVALID:
                    continue  # GCS, companion, gimbal...
                self.target_system = msg.get_srcSystem()
                self.target_component = msg.get_srcComponent()
                self.fc_autopilot = msg.autopilot
                self._last_fc_heartbeat = now
            elif mtype in ("ATTITUDE", "GPS_RAW_INT"):
                if self.target_system is not None and msg.get_srcSystem() != self.target_system:
                    continue
                self._last_rx[msg.get_msgId()] = now
                if mtype == "ATTITUDE":
                    self._attitude = Attitude(msg.roll, msg.pitch, msg.yaw, msg.yawspeed, now)
                elif msg.fix_type >= 3 and (msg.lat != 0 or msg.lon != 0):  # 3D fix or better
                    self._gps = (msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3, now)

    def fc_connected(self, timeout=3.0):
        return (time.monotonic() - self._last_fc_heartbeat) < timeout

    def attitude(self, max_age=0.5):
        a = self._attitude
        if a is None or (time.monotonic() - a.t) > max_age:
            return None
        return a

    def gps_fix(self, max_age=5.0):
        """(lat_deg, lon_deg, alt_m) from the latest 3D GPS fix, or None."""
        g = self._gps
        if g is None or (time.monotonic() - g[3]) > max_age:
            return None
        return g[:3]

    def autopilot_name(self):
        return {mavlink.MAV_AUTOPILOT_PX4: "PX4",
                mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA: "ArduPilot"}.get(self.fc_autopilot, str(self.fc_autopilot))

    # ----------------------------------------------------------------- outbound
    def tick(self):
        """Housekeeping: heartbeat at 1 Hz, ask for wanted messages until they arrive."""
        now = time.monotonic()
        if now - self._last_heartbeat_sent >= 1.0:
            self._last_heartbeat_sent = now
            self.mav.heartbeat_send(mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                    mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavlink.MAV_STATE_ACTIVE)
        if not self.fc_connected():
            return
        for msg_id, rate_hz in self._wanted.items():
            silent = now - self._last_rx.get(msg_id, 0.0) > max(2.0, 3.0 / rate_hz)
            if silent and now - self._last_request.get(msg_id, 0.0) >= 3.0:
                self._last_request[msg_id] = now
                self.mav.command_long_send(
                    self.target_system, self.target_component,
                    mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    msg_id, int(1e6 / rate_hz), 0, 0, 0, 0, 0)

    def send_heading(self, t_capture, yaw_rad, std_rad, quality=100):
        """Send one heading measurement.

        t_capture: time.monotonic() when the frame was captured
        yaw_rad:   heading relative to TRUE north, radians
        std_rad:   1-sigma uncertainty, radians (PX4 uses it; ArduPilot uses VISO_YAW_M_NSE)
        """
        time_usec = int(t_capture * 1e6)
        if self.firmware == "px4":
            q = [math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)]
            pose_cov = [0.0] * 21
            pose_cov[0] = pose_cov[6] = pose_cov[11] = 1e4  # position: not provided (NaN below)
            pose_cov[15] = pose_cov[18] = 1.0              # roll/pitch: not provided, not fused
            pose_cov[20] = float(std_rad) ** 2             # yaw variance
            vel_cov = [NAN] + [0.0] * 20                   # velocity: unknown
            self.mav.odometry_send(
                time_usec, mavlink.MAV_FRAME_LOCAL_NED, mavlink.MAV_FRAME_BODY_FRD,
                NAN, NAN, NAN, q, NAN, NAN, NAN, NAN, NAN, NAN,
                pose_cov, vel_cov, 0, mavlink.MAV_ESTIMATOR_TYPE_VISION,
                int(max(1, min(100, quality))))
        else:
            cov = [NAN] + [0.0] * 20  # "unknown" -> ArduPilot uses VISO_YAW_M_NSE
            self.mav.vision_position_estimate_send(time_usec, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                   float(yaw_rad), cov, 0)
        self.sent_count += 1

    def close(self):
        try:
            self.conn.close()
        except Exception:
            pass
