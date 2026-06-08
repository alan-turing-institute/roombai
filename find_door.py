#!/usr/bin/env python3
"""
find_door.py — Navigate the Roomba to find and stop in a doorway.

Vision inference uses the local `claude` CLI (no API key required):
  echo "<prompt>" | claude --print --allowedTools Read --dangerously-skip-permissions

Strategy:
  1. SCAN  — spin in 45° steps, take a photo at each, ask Claude if the door
             is visible and where it is.
  2. APPROACH — drive toward the door in short bursts, steering to stay centred.
                Handle bumps by backing up and steering around.
  3. WAIT  — if the door is visible but closed, stop nearby and poll until it opens.
  4. ARRIVE — stop when Claude confirms we are in the doorway.

If no door is found in a full 360° scan, move to a new position and try again.
"""

import json
import socket
import subprocess
import sys
import time
from pathlib import Path

# ---- Config -----------------------------------------------------------------
PILOT_HOST = "127.0.0.1"
PILOT_PORT = 9999

PHOTO_PATH = "/tmp/roomba_view.jpg"
PHOTO_WIDTH = 1280
PHOTO_HEIGHT = 720

SCAN_STEPS = 8                   # 360/8 = 45° per step
SCAN_STEP_DEG = 360.0 / SCAN_STEPS
TURN_RATE_DEG_S = 60.0

DRIVE_SPEED_CM_S = 12.0
DRIVE_CHUNK_CM = 25.0
STEER_DEG = 20.0
BUMP_BACKUP_S = 0.8

WAIT_POLL_S = 3.0                # seconds between camera checks when waiting for door
MAX_SCAN_ROUNDS = 4
MAX_WAIT_POLLS = 20              # give up waiting after this many polls (~1 min)

CLAUDE_VISION_TIMEOUT = 30       # seconds for claude CLI call

# ---- Pilot communication ----------------------------------------------------

def send_cmd(cmd: str) -> str:
    try:
        with socket.create_connection((PILOT_HOST, PILOT_PORT), timeout=5) as s:
            s.sendall((cmd + "\n").encode())
            s.settimeout(5)
            reply = s.makefile().readline().strip()
            print(f"  pilot> {cmd!r}  →  {reply}")
            return reply
    except Exception as e:
        print(f"  pilot> {cmd!r}  →  ERROR: {e}")
        return f"ERR {e}"


def bumped() -> bool:
    resp = send_cmd("bumps")
    return "bumpL=1" in resp or "bumpR=1" in resp


def turn(deg: float):
    resp = send_cmd(f"turn {deg:.0f}")
    time.sleep(abs(deg) / TURN_RATE_DEG_S + 0.4)


def drive_forward(cm: float):
    secs = cm / DRIVE_SPEED_CM_S
    send_cmd(f"forward {DRIVE_SPEED_CM_S:.0f} {secs:.1f}")
    time.sleep(secs + 0.3)


def backup_and_turn(deg: float = 30.0):
    send_cmd(f"back {DRIVE_SPEED_CM_S:.0f} {BUMP_BACKUP_S:.1f}")
    time.sleep(BUMP_BACKUP_S + 0.4)
    turn(deg)

# ---- Camera -----------------------------------------------------------------

def capture() -> str:
    """Capture a still and return the file path."""
    subprocess.run(
        [
            "rpicam-still", "--nopreview",
            "--width", str(PHOTO_WIDTH), "--height", str(PHOTO_HEIGHT),
            "-o", PHOTO_PATH, "-t", "800",
        ],
        check=True,
        capture_output=True,
    )
    return PHOTO_PATH

# ---- Vision (claude CLI) ----------------------------------------------------

_VISION_PROMPT = """\
Read the image at {path} and reply with ONLY a JSON object — no markdown, no prose:

{{
  "door_visible": true or false,
  "door_open": true or false,
  "door_position": "left" | "center" | "right" | null,
  "in_doorway": true or false,
  "confidence": "high" | "medium" | "low",
  "notes": "one short sentence"
}}

Definitions (camera is mounted low on a robot vacuum, pointing forward):
- door_visible: an open or closed door / doorway is present in the image.
- door_open: the door is open and passable (not just visible but closed).
- door_position: which horizontal third of the frame the door opening is in.
- in_doorway: the robot is at the threshold — door frame visible on both sides.
"""


def analyze(image_path: str) -> dict:
    prompt = _VISION_PROMPT.format(path=image_path)
    try:
        result = subprocess.run(
            [
                "claude", "--print",
                "--allowedTools", "Read",
                "--dangerously-skip-permissions",
            ],
            input=prompt,
            capture_output=True,
            text=True,
            timeout=CLAUDE_VISION_TIMEOUT,
        )
        text = result.stdout.strip()
        if text.startswith("```"):
            text = text.split("\n", 1)[1].rsplit("```", 1)[0].strip()
        parsed = json.loads(text)
    except (subprocess.TimeoutExpired, json.JSONDecodeError, Exception) as e:
        print(f"  [vision] error: {e}")
        parsed = {
            "door_visible": False, "door_open": False,
            "door_position": None, "in_doorway": False,
            "confidence": "low", "notes": f"error: {e}",
        }
    print(f"  [vision] {parsed}")
    return parsed

# ---- Navigation phases ------------------------------------------------------

def scan_for_door() -> float | None:
    """
    Rotate 360° in SCAN_STEPS, taking a photo at each position.
    Returns the fine-tune heading offset (degrees) when a door is spotted,
    or None if no door was found after a full rotation.
    Also returns 0.0 if we are already in the doorway.
    """
    print("\n── SCAN ─────────────────────────────────────────")
    for i in range(SCAN_STEPS):
        print(f"  Step {i+1}/{SCAN_STEPS}  (facing ~{i * SCAN_STEP_DEG:.0f}°)")

        path = capture()
        v = analyze(path)

        if v.get("in_doorway"):
            print("  Already in the doorway!")
            return 0.0

        if v.get("door_visible"):
            pos = v.get("door_position") or "center"
            offset = {"left": -SCAN_STEP_DEG / 3,
                      "center": 0.0,
                      "right": +SCAN_STEP_DEG / 3}[pos]
            door_state = "open" if v.get("door_open") else "CLOSED"
            print(f"  Door found ({door_state}, {pos})  fine-tune offset: {offset:+.0f}°")
            return offset

        turn(SCAN_STEP_DEG)

    print("  No door found in full 360° scan.")
    return None


def wait_for_door_to_open() -> bool:
    """
    Assume we're already facing/near a closed door.
    Poll the camera until it opens.  Returns True when open, False on timeout.
    """
    print("\n── WAITING for door to open ─────────────────────")
    for poll in range(1, MAX_WAIT_POLLS + 1):
        print(f"  Poll {poll}/{MAX_WAIT_POLLS}")
        path = capture()
        v = analyze(path)
        if v.get("door_open") or v.get("in_doorway"):
            print("  Door is open!")
            return True
        time.sleep(WAIT_POLL_S)
    print("  Timed out waiting for door.")
    return False


def approach_door() -> str:
    """
    Drive toward the door.
    Returns:
      "arrived"  — stopped in doorway
      "lost"     — door disappeared, need rescan
      "closed"   — door reached but closed, need to wait
    """
    print("\n── APPROACH ─────────────────────────────────────")
    lost_count = 0

    while True:
        if bumped():
            print("  Bump! Backing up and steering around.")
            backup_and_turn(30.0)
            continue

        path = capture()
        v = analyze(path)

        if v.get("in_doorway"):
            print("  In doorway — stopping!")
            send_cmd("stop")
            return "arrived"

        if v.get("door_visible") and not v.get("door_open", True):
            print("  Door is closed — stopping to wait.")
            send_cmd("stop")
            return "closed"

        if not v.get("door_visible"):
            lost_count += 1
            print(f"  Door not visible ({lost_count}/3)")
            if lost_count >= 3:
                send_cmd("stop")
                return "lost"
            drive_forward(10.0)
            continue

        lost_count = 0
        pos = v.get("door_position") or "center"

        if pos == "left":
            print("  Steering left")
            turn(+STEER_DEG)
        elif pos == "right":
            print("  Steering right")
            turn(-STEER_DEG)

        drive_forward(DRIVE_CHUNK_CM)

# ---- Main -------------------------------------------------------------------

def main():
    print("═══ Roomba door finder ═══")
    status = send_cmd("sense")
    if status.startswith("ERR"):
        print("Cannot reach pilot daemon — is it running?")
        sys.exit(1)
    print(f"Robot status: {status}\n")

    for round_num in range(1, MAX_SCAN_ROUNDS + 1):
        print(f"\n═══ Round {round_num}/{MAX_SCAN_ROUNDS} ═══")

        offset = scan_for_door()

        if offset is None:
            print("No door found. Moving to a new position and trying again.")
            drive_forward(60.0)
            continue

        if offset == 0.0:
            # Already in doorway (flagged by scan)
            print("\n═══ Mission complete: already in doorway ═══")
            return

        # Fine-tune heading then approach
        if abs(offset) > 5:
            print(f"\nFine-tuning heading {offset:+.0f}°")
            turn(offset)

        outcome = approach_door()

        if outcome == "arrived":
            print("\n═══ Mission complete: stopped in doorway ═══")
            return

        if outcome == "closed":
            opened = wait_for_door_to_open()
            if opened:
                # Resume approach from standstill
                outcome2 = approach_door()
                if outcome2 == "arrived":
                    print("\n═══ Mission complete: stopped in doorway ═══")
                    return

        # Door lost or still couldn't get through — rescan from here
        print("\nRescanning from current position.")

    print("\nCould not find or reach the door after all attempts. Stopping.")
    send_cmd("stop")


if __name__ == "__main__":
    main()
