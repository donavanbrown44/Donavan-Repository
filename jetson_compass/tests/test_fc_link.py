"""Round-trip the MAVLink messages over a local UDP socket (stands in for the UART)."""

import math
import os
import socket
import sys
import time

os.environ["MAVLINK20"] = "1"
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest  # noqa: E402
from pymavlink import mavutil  # noqa: E402

from compass_vision.fc_link import FcLink  # noqa: E402

m = mavutil.mavlink


def free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def recv(conn, mtype, timeout=2.0):
    end = time.time() + timeout
    while time.time() < end:
        msg = conn.recv_match(type=mtype, blocking=True, timeout=0.1)
        if msg is not None:
            return msg
    return None


@pytest.fixture
def pair():
    port = free_port()
    fc = mavutil.mavlink_connection("udpin:127.0.0.1:%d" % port, source_system=1, source_component=1)
    yield port, fc
    fc.close()


def _fc_says_hello(fc, link, autopilot):
    link.tick()  # our heartbeat lets the udpin side learn our address
    assert recv(fc, "HEARTBEAT") is not None
    fc.mav.heartbeat_send(m.MAV_TYPE_QUADROTOR, autopilot, 0, 0, m.MAV_STATE_STANDBY)
    end = time.time() + 2
    while not link.fc_connected() and time.time() < end:
        link.poll()
        time.sleep(0.01)
    assert link.fc_connected()


def test_px4_odometry_carries_yaw_only(pair):
    port, fc = pair
    link = FcLink("udpout:127.0.0.1:%d" % port, firmware="px4")
    _fc_says_hello(fc, link, m.MAV_AUTOPILOT_PX4)
    assert link.autopilot_name() == "PX4"

    yaw = math.radians(-100.0)
    link.send_heading(123.456, yaw, math.radians(3.0), quality=80)
    msg = recv(fc, "ODOMETRY")
    assert msg is not None
    assert msg.time_usec == 123456000
    assert msg.frame_id == m.MAV_FRAME_LOCAL_NED and msg.child_frame_id == m.MAV_FRAME_BODY_FRD
    assert all(math.isnan(v) for v in (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz))
    w, x, y, z = msg.q
    got_yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    assert abs(got_yaw - yaw) < 1e-5
    assert abs(msg.pose_covariance[20] - math.radians(3.0) ** 2) < 1e-6
    assert msg.estimator_type == m.MAV_ESTIMATOR_TYPE_VISION and msg.quality == 80
    link.close()


def test_ardupilot_vision_position_estimate(pair):
    port, fc = pair
    link = FcLink("udpout:127.0.0.1:%d" % port, firmware="ardupilot")
    _fc_says_hello(fc, link, m.MAV_AUTOPILOT_ARDUPILOTMEGA)
    link.send_heading(5.0, math.radians(45.0), math.radians(3.0))
    msg = recv(fc, "VISION_POSITION_ESTIMATE")
    assert msg is not None
    assert abs(msg.yaw - math.radians(45.0)) < 1e-6
    assert msg.x == msg.y == msg.z == msg.roll == msg.pitch == 0.0
    assert math.isnan(msg.covariance[0])
    link.close()


def test_attitude_is_requested_and_read(pair):
    port, fc = pair
    link = FcLink("udpout:127.0.0.1:%d" % port, firmware="px4", attitude_rate_hz=20)
    _fc_says_hello(fc, link, m.MAV_AUTOPILOT_PX4)
    link.tick()
    cmd = recv(fc, "COMMAND_LONG")
    assert cmd is not None and cmd.command == m.MAV_CMD_SET_MESSAGE_INTERVAL
    assert int(cmd.param1) == m.MAVLINK_MSG_ID_ATTITUDE and int(cmd.param2) == 50000

    fc.mav.attitude_send(1000, 0.1, -0.2, 1.5, 0.0, 0.0, 0.3)
    end = time.time() + 2
    while link.attitude() is None and time.time() < end:
        link.poll()
        time.sleep(0.01)
    a = link.attitude()
    assert a is not None and abs(a.yaw - 1.5) < 1e-6 and abs(a.yawspeed - 0.3) < 1e-6
    link.close()


def test_bad_firmware_name():
    with pytest.raises(ValueError):
        FcLink("udpout:127.0.0.1:9", firmware="betaflight")
