"""
Obstacle-avoiding forward drive loop.

Takes snapshots, uses OpenCV to estimate clearance in three zones, and drives
in the direction with the most space up to --max-distance metres per move.

Usage:
    python3 drive.py
    python3 drive.py --max-distance 3.0 --max-steps 50
"""

import argparse
import pathlib

import decision_logger
import move_planner as planner
import picar_control as car


def drive(
    max_distance: float = 5.0,
    max_steps: int = 100,
    log_dir: pathlib.Path = pathlib.Path("logs"),
) -> None:
    print(f"PaiCar drive  max_distance={max_distance}m  max_steps={max_steps}  log_dir={log_dir}\n")

    for step in range(1, max_steps + 1):
        print(f"[{step}/{max_steps}] snapshot…", end=" ", flush=True)
        image_b64 = car.snapshot()

        action = planner.plan_move(image_b64, max_distance=max_distance)
        img_path = decision_logger.log(image_b64, action, log_dir=log_dir)
        print(f"{action.direction} {action.distance_metres:.2f}m | {action.reasoning} | logged→{img_path.name}")

        if action.direction.startswith("reverse"):
            # "reverse" → straight, "reverse+left" → left, "reverse+right" → right
            steer = action.direction.split("+")[1] if "+" in action.direction else "straight"
            car.steer_backward(steer, action.distance_metres)
        else:
            car.steer_forward(action.direction, action.distance_metres)

    print("\nDrive complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PaiCar obstacle-avoiding drive")
    parser.add_argument(
        "--max-distance", type=float, default=5.0,
        help="Maximum travel distance per move in metres (default: 5.0)",
    )
    parser.add_argument(
        "--max-steps", type=int, default=100,
        help="Maximum number of moves before stopping (default: 100)",
    )
    parser.add_argument(
        "--log-dir", type=pathlib.Path, default=pathlib.Path("logs"),
        help="Directory for decision logs and snapshots (default: logs/)",
    )
    args = parser.parse_args()
    drive(max_distance=args.max_distance, max_steps=args.max_steps, log_dir=args.log_dir)
