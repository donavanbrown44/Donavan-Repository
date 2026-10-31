import datetime
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

pytest.importorskip("pygeomag")
from compass_vision.declination import decimal_year, declination_deg  # noqa: E402


@pytest.mark.parametrize("lat,lon,expected", [
    (47.61, -122.33, 14.9),   # Seattle
    (39.74, -104.99, 7.5),    # Denver
    (40.71, -74.00, -12.5),   # New York (west = negative)
    (-33.87, 151.21, 12.8),   # Sydney
])
def test_known_places(lat, lon, expected):
    assert abs(declination_deg(lat, lon, 0.0, year=2026.8) - expected) < 0.5


def test_bad_clock_is_clamped_not_fatal():
    # Jetson with no RTC battery / no NTP can boot thinking it's 1970 or far future.
    assert abs(declination_deg(39.74, -104.99, year=1970.0) - declination_deg(39.74, -104.99, year=2025.0)) < 1e-9
    declination_deg(39.74, -104.99, year=2040.0)


def test_decimal_year():
    mid = datetime.datetime(2026, 7, 2, 12, tzinfo=datetime.timezone.utc)
    assert abs(decimal_year(mid) - 2026.5) < 0.01
