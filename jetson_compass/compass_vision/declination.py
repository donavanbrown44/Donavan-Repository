"""Magnetic declination from a GPS position, using the World Magnetic Model.

Used when `heading.declination_deg: auto`. The Jetson reads the Pixhawk's GPS
position over MAVLink and computes declination once, at the first 3D fix, with
WMM2025 (NOAA/BGS model, valid 2025-2029) via the pure-Python `pygeomag` package.

Declination barely changes across a flying field (well under 0.1 deg per km at
mid latitudes) or over time (~0.1 deg/year), so computing it once is enough. A
Jetson clock that is off by a year or two changes the result by a fraction of a
degree; dates outside the model's life are clamped with a warning.
"""

import datetime
import logging

log = logging.getLogger(__name__)

MODEL_FILE = "wmm/WMM_2025.COF"   # bundled with pygeomag >= 1.1
MODEL_NAME = "WMM2025"
MODEL_START, MODEL_END = 2025.0, 2030.0


def require_model():
    """Fail early (at startup) if pygeomag is missing, rather than at first GPS fix."""
    try:
        from pygeomag import GeoMag
    except ImportError:
        raise ImportError("heading.declination_deg is 'auto', which needs pygeomag: "
                          "pip3 install --user pygeomag   (or set a number in config.yaml)")
    return GeoMag(coefficients_file=MODEL_FILE)


def decimal_year(now=None):
    now = now or datetime.datetime.now(datetime.timezone.utc)
    start = datetime.datetime(now.year, 1, 1, tzinfo=now.tzinfo)
    end = datetime.datetime(now.year + 1, 1, 1, tzinfo=now.tzinfo)
    return now.year + (now - start).total_seconds() / (end - start).total_seconds()


def declination_deg(lat, lon, alt_m=0.0, year=None):
    """Declination in degrees, EAST positive, for a lat/lon (deg) and altitude (m)."""
    y = decimal_year() if year is None else float(year)
    if not (MODEL_START <= y < MODEL_END):
        log.warning("date %.2f is outside %s's range %.0f-%.0f (Jetson clock wrong, or time to "
                    "update pygeomag); using the nearest valid date", y, MODEL_NAME, MODEL_START, MODEL_END)
        y = min(max(y, MODEL_START), MODEL_END - 0.01)
    return float(require_model().calculate(glat=lat, glon=lon, alt=alt_m / 1000.0, time=y).d)
