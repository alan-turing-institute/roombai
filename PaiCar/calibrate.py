"""
Interactive calibration for picar_control.py dead-reckoning constants.

Run once on the floor surface you'll use, then paste the printed values
into picar_control.py.

Usage:
    python calibrate.py
"""

import picar_control as car

REQUESTED_DISTANCE = 1.0   # metres
REQUESTED_TURN = 90.0      # degrees


def calibrate_forward():
    print("\n── Forward calibration ─────────────────────────────────")
    print(f"Place the car on the floor and mark its starting position.")
    input("Press Enter to drive forward (target: 1.0 m)…")

    car.move_forward(REQUESTED_DISTANCE)

    raw = input("Measure the distance the car actually travelled (metres): ")
    actual = float(raw)
    new_constant = car.FORWARD_CM_PER_SEC * (actual / REQUESTED_DISTANCE)
    print(f"\n  Current FORWARD_CM_PER_SEC : {car.FORWARD_CM_PER_SEC}")
    print(f"  Actual distance driven     : {actual} m")
    print(f"  → New FORWARD_CM_PER_SEC   : {new_constant:.2f}")
    return new_constant


def calibrate_turn():
    print("\n── Turn calibration ─────────────────────────────────────")
    print("Mark the car's heading direction (e.g. with tape on the floor).")
    input("Press Enter to turn right 90° …")

    car.turn(REQUESTED_TURN)

    raw = input("Measure the actual angle turned (degrees): ")
    actual = float(raw)
    new_constant = car.TURN_DEGREES_PER_SEC * (actual / REQUESTED_TURN)
    print(f"\n  Current TURN_DEGREES_PER_SEC : {car.TURN_DEGREES_PER_SEC}")
    print(f"  Actual angle turned           : {actual}°")
    print(f"  → New TURN_DEGREES_PER_SEC    : {new_constant:.2f}")
    return new_constant


def main():
    print("PaiCar calibration (direct hardware mode)")

    fwd = calibrate_forward()
    trn = calibrate_turn()

    print("\n── Paste these lines into picar_control.py ─────────────")
    print(f"FORWARD_CM_PER_SEC   = {fwd:.2f}")
    print(f"TURN_DEGREES_PER_SEC = {trn:.2f}")


if __name__ == "__main__":
    main()
