"""
PiCar direct-hardware control — runs ON the Raspberry Pi.

Uses the `picar` library directly instead of the Django HTTP server.
Motor stop is a local function call that cannot be lost to a network timeout.

Prerequisites (Pi-side):
  - `picar` installed: sudo python3 setup.py install  (in SunFounder_PiCar-V/)
  - mjpg-streamer running on port 8080 for snapshot()
  - Django server NOT running (both would compete for the I2C bus)
"""

import base64
import pathlib
import time

import picar
from picar import back_wheels, front_wheels
import requests

# ── Hardware config ───────────────────────────────────────────────────────────

def _find_db() -> str:
    candidates = [
        pathlib.Path.home() / "SunFounder_PiCar-V/remote_control/remote_control/driver/config",
        pathlib.Path("/home/pi/SunFounder_PiCar-V/remote_control/remote_control/driver/config"),
    ]
    for p in candidates:
        if p.exists():
            return str(p)
    raise FileNotFoundError(
        "PiCar config not found. Update _find_db() with the correct path."
    )

DB_FILE = _find_db()
SNAPSHOT_URL = "http://localhost:8080/?action=snapshot"

# ── Speed ─────────────────────────────────────────────────────────────────────
SPEED = 60  # 0–100

# ── Dead-reckoning calibration ────────────────────────────────────────────────
# Tune with calibrate.py on your floor surface, then paste the printed values here.
FORWARD_CM_PER_SEC = 25.0    # cm/s at SPEED=60 on a flat surface
TURN_DEGREES_PER_SEC = 50.0  # degrees/s with full steering lock at SPEED=60

# ── Hardware initialisation ───────────────────────────────────────────────────
picar.setup()
_fw = front_wheels.Front_Wheels(debug=False, db=DB_FILE)
_bw = back_wheels.Back_Wheels(debug=False, db=DB_FILE)
_fw.ready()
_bw.ready()

# ── Public API ────────────────────────────────────────────────────────────────

def stop() -> None:
    """Halt back wheels and straighten front wheels."""
    _bw.stop()
    _fw.turn_straight()


def move_forward(metres: float) -> None:
    """Drive forward by approximately `metres` metres."""
    duration = (metres * 100) / FORWARD_CM_PER_SEC
    _bw.speed = SPEED
    _bw.forward()
    time.sleep(duration)
    stop()


def move_backward(metres: float) -> None:
    """Reverse by approximately `metres` metres."""
    duration = (metres * 100) / FORWARD_CM_PER_SEC
    _bw.speed = SPEED
    _bw.backward()
    time.sleep(duration)
    stop()


def turn(degrees: float) -> None:
    """Turn by approximately `degrees` degrees.

    Positive = right, negative = left.
    Uses Ackermann steering: locks front wheels then drives forward briefly.
    """
    duration = abs(degrees) / TURN_DEGREES_PER_SEC
    if degrees > 0:
        _fw.turn_right()
    else:
        _fw.turn_left()
    _bw.speed = SPEED
    _bw.forward()
    time.sleep(duration)
    stop()


def snapshot() -> str:
    """Fetch a JPEG from mjpg-streamer and return it as a base64 string."""
    response = requests.get(SNAPSHOT_URL, timeout=5)
    response.raise_for_status()
    return base64.standard_b64encode(response.content).decode("utf-8")
