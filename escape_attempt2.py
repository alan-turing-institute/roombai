#!/usr/bin/env python3
"""
Escape Attempt 2 (v2) — Bumper-Based Perimeter Crawl
Strategy: Follow right wall, detect door via bumper gap (≥2 steps with no wall on right)

Key fixes from v1:
- Correct bumper output format: bumpL=1/0 not left=true/false
- Check bumpers at startup (may already be against wall)
- Proper wall reacquisition: after turn, sidle right until wall contact, then back off
- Larger probe angle for gap detection (turn 30°, move 20cm) to reliably detect wall
"""

import subprocess
import time
import os
import sys
import datetime

LOG_FILE = "/tmp/execution_log.txt"
SPEAK_QUEUE = "/tmp/speak_queue.txt"
PILOT_DIR = "/home/hackweek26/roombai/roomba_pilot"
PILOT_BIN = os.path.join(PILOT_DIR, "target/debug/pilot")
IMG_DIR = "/home/hackweek26/roombai/tmp"

os.makedirs(IMG_DIR, exist_ok=True)

# ── Primitives ──────────────────────────────────────────────────────────────

def log(msg):
    ts = datetime.datetime.now().strftime("%H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line, flush=True)
    with open(LOG_FILE, "a") as f:
        f.write(line + "\n")

def speak(msg):
    with open(SPEAK_QUEUE, "a") as f:
        f.write(msg + "\n")
    log(f"SPEAK: {msg}")

def pilot(cmd, timeout=15):
    """Run a pilot send command, return stdout string."""
    try:
        result = subprocess.run(
            [PILOT_BIN, "send", cmd],
            capture_output=True, text=True, timeout=timeout,
            cwd=PILOT_DIR
        )
        output = result.stdout.strip()
        stderr = result.stderr.strip()
        log(f"pilot({cmd!r}) → {output!r}" + (f" ERR:{stderr!r}" if stderr else ""))
        return output
    except subprocess.TimeoutExpired:
        log(f"pilot({cmd!r}) TIMEOUT")
        return ""
    except Exception as e:
        log(f"pilot({cmd!r}) EXCEPTION: {e}")
        return ""

def bumped():
    """Return dict with left, right booleans from bumps command.
    Output format: 'OK bumpL=1 bumpR=0 dropL=0 dropR=0 caster=0'
    """
    out = pilot("bumps")
    # Parse bumpL and bumpR fields (value is 1 or 0)
    left = "bumpL=1" in out
    right = "bumpR=1" in out
    return {"left": left, "right": right, "raw": out}

def capture(name):
    """Capture a still image."""
    path = os.path.join(IMG_DIR, name if name.endswith(".jpg") else name + ".jpg")
    try:
        result = subprocess.run(
            ["rpicam-still", "-o", path, "--width", "640", "--height", "480",
             "--nopreview", "-t", "1000"],
            capture_output=True, text=True, timeout=10
        )
        log(f"capture({name}) → {path} (rc={result.returncode})")
        return path
    except Exception as e:
        log(f"capture({name}) FAILED: {e}")
        return None

def abort(reason):
    log(f"ABORT: {reason}")
    speak(f"Abort: {reason}")
    pilot("stop")
    sys.exit(1)

# ── Phase 0a: Connectivity ───────────────────────────────────────────────────

def phase0a_connectivity():
    log("=== PHASE 0a: Connectivity Check ===")
    speak("Phase zero. Connectivity check.")

    ping_out = pilot("ping")
    if not ping_out:
        abort("Pilot daemon not responding")
    log(f"Ping OK: {ping_out}")

    safe_out = pilot("safe")
    log(f"Safe mode: {safe_out}")
    time.sleep(0.5)

    sense_out = pilot("sense")
    log(f"Sense: {sense_out}")
    speak("Connected. Battery OK.")
    return True

# ── Phase 0b: Camera Diagnostic ─────────────────────────────────────────────

def phase0b_camera():
    log("=== PHASE 0b: Camera Diagnostic ===")
    speak("Camera diagnostic.")

    path = capture("diag_cam.jpg")
    if path is None:
        log("Camera capture failed — continuing without camera")
        speak("Camera not available. Proceeding bumper-only.")
        return "unknown"

    try:
        import cv2
        img = cv2.imread(path)
        if img is None:
            log("Could not read image with OpenCV")
            return "unknown"
        h, w = img.shape[:2]
        top_brightness = float(img[:h//2].mean())
        bot_brightness = float(img[h//2:].mean())
        log(f"Camera: top_half={top_brightness:.1f} bottom_half={bot_brightness:.1f}")

        if top_brightness > bot_brightness + 20:
            orientation = "UPWARD/CEILING"
        elif bot_brightness > top_brightness + 20:
            orientation = "DOWNWARD/FLOOR"
        else:
            orientation = "LEVEL/FORWARD"

        log(f"Camera orientation: {orientation}")
        speak(f"Camera orientation: {orientation}")
        return orientation
    except ImportError:
        log("OpenCV not available — skipping image analysis")
        return "unknown"
    except Exception as e:
        log(f"Camera analysis error: {e}")
        return "unknown"

# ── Phase 0c: Bumper Diagnostic ──────────────────────────────────────────────

def phase0c_bumper_test():
    """Drive toward a wall until bumper triggers. Return distance in cm."""
    log("=== PHASE 0c: Bumper Diagnostic ===")
    speak("Bumper diagnostic. Driving toward wall.")

    # Check if already against wall at startup
    b_initial = bumped()
    if b_initial["left"] or b_initial["right"]:
        log("BUMPER TEST SUCCESS: already against wall at startup — backing up 20cm")
        speak("Wall contact at startup. Backing up.")
        pilot("move -20")
        time.sleep(1.0)
        return 0  # 0cm driven = already at wall

    # Drive forward in 10cm increments up to 3m
    for i in range(30):
        pilot("move 10")
        time.sleep(0.7)
        b = bumped()
        if b["left"] or b["right"]:
            dist_cm = (i + 1) * 10
            log(f"BUMPER TEST SUCCESS: wall contact at step {i+1}, ~{dist_cm}cm")
            speak(f"Bumper confirmed at {dist_cm} centimeters")
            pilot("move -20")
            time.sleep(1.0)
            return dist_cm

    # No bump in 3m — try 3 more 50cm moves
    log("No bump in 3m — trying 3x 50cm extensions")
    speak("No bump in three meters. Extending search.")
    for j in range(3):
        pilot("move 50")
        time.sleep(2.0)
        b = bumped()
        if b["left"] or b["right"]:
            dist_cm = 300 + (j + 1) * 50
            log(f"BUMPER TEST SUCCESS: wall at ~{dist_cm}cm")
            speak(f"Bumper confirmed at {dist_cm} centimeters")
            pilot("move -20")
            time.sleep(1.0)
            return dist_cm

    abort("No wall contact after 4.5m. Bumpers may be non-functional or room too large.")
    return -1  # never reached

# ── Wall reacquisition — sidle right until contact ───────────────────────────

def reacquire_right_wall():
    """Turn right 90°, drive until wall contact, back off 20cm, turn left 90°.
    This establishes the robot ~20cm from the right wall, facing along it.
    Returns True if wall found, False otherwise.
    """
    log("WALL REACQUISITION: sidling right to find wall")

    # Turn right 90° to face the right wall
    pilot("turn -90")
    time.sleep(1.5)

    # Drive toward the wall in 10cm steps (max 2m)
    for i in range(20):
        pilot("move 10")
        time.sleep(0.7)
        b = bumped()
        if b["left"] or b["right"]:
            dist = (i + 1) * 10
            log(f"Right wall found at {dist}cm during reacquisition")
            speak(f"Right wall found at {dist} centimeters")
            # Back off 20cm
            pilot("move -20")
            time.sleep(1.0)
            # Turn left 90° to face along wall
            pilot("turn 90")
            time.sleep(1.5)
            return True

    log("WARNING: Right wall not found in 2m during reacquisition — continuing")
    speak("Right wall not found. Continuing without wall contact.")
    # Turn back left to resume forward direction
    pilot("turn 90")
    time.sleep(1.5)
    return False

# ── Right wall probe ─────────────────────────────────────────────────────────

def probe_right_wall():
    """Probe for the right wall.
    Turn right 60°, drive 25cm, check bumps, reverse, turn back left 60°.
    Returns True if wall found (no gap), False if gap detected.
    """
    # Turn right 60°
    pilot("turn -60")
    time.sleep(0.8)
    # Drive 25cm toward where wall should be
    pilot("move 25")
    time.sleep(1.2)
    b = bumped()
    wall_found = b["left"] or b["right"]
    # Always back out
    pilot("move -25")
    time.sleep(1.2)
    # Turn back left 60°
    pilot("turn 60")
    time.sleep(0.8)
    return wall_found

# ── Phase 1: Establish Wall Reference ────────────────────────────────────────

def phase1_wall_reference():
    log("=== PHASE 1: Establish Wall Reference ===")
    speak("Wall acquired. Establishing right wall reference.")

    # Turn left 90° so initial wall (behind us after backing off) is on RIGHT
    pilot("turn 90")
    time.sleep(1.5)

    # Now sidle right to actually touch and confirm the wall, then back off properly
    reacquire_right_wall()

    speak("Wall on right side. Beginning perimeter crawl.")
    log("Wall reference established. Robot is ~20cm from right wall, facing along it.")

# ── Phase 2: Perimeter Crawl (Right-Wall Follow) ─────────────────────────────

def phase2_perimeter_crawl():
    log("=== PHASE 2: Perimeter Crawl (Right-Wall Follow) ===")
    speak("Perimeter crawl starting.")

    wall_turns = 0
    steps = 0
    gap_start = None
    gap_steps = 0
    consecutive_no_wall = 0  # For tracking if we've truly lost the wall vs. gap

    start_time = time.time()
    MAX_TIME = 20 * 60  # 20 minutes
    MAX_ITERATIONS = 600

    for iteration in range(MAX_ITERATIONS):
        # Time limit check
        elapsed = time.time() - start_time
        if elapsed > MAX_TIME:
            abort(f"Time limit exceeded: {elapsed:.0f}s elapsed")

        # Step 1: Move forward 30cm
        pilot("move 30")
        time.sleep(1.2)
        steps += 1

        # Step 2: Check forward bumps
        b = bumped()

        if b["left"] or b["right"]:
            # Hit forward wall/obstacle — it's a corner
            log(f"CORNER/FORWARD WALL at step {steps}, wall_turns={wall_turns}, bump: L={b['left']} R={b['right']}")
            speak(f"Corner found, turning left. Turn count: {wall_turns + 1}")
            pilot("move -20")
            time.sleep(1.0)
            pilot("turn 90")
            time.sleep(1.5)
            wall_turns += 1
            gap_start = None
            gap_steps = 0
            consecutive_no_wall = 0

            if wall_turns >= 8:
                abort("Full perimeter traversed without finding door (wall_turns >= 8)")

            # Reacquire right wall after corner turn
            log("Reacquiring right wall after corner turn")
            reacquire_right_wall()

        else:
            # No forward bump — probe right wall
            wall_on_right = probe_right_wall()

            if wall_on_right:
                # Wall still there — normal following
                log(f"Step {steps}: right wall confirmed")
                consecutive_no_wall = 0
                if gap_start is not None:
                    # Gap was a recess that ended — reset
                    log(f"Gap ended at step {steps} (was {gap_steps} steps) — too short for door or recess")
                    gap_start = None
                    gap_steps = 0
            else:
                # No wall on right — we're in a gap
                consecutive_no_wall += 1

                if gap_start is None:
                    gap_start = steps
                    gap_steps = 0
                gap_steps += 1

                log(f"GAP PROBE: no right wall at step {steps}, gap_steps={gap_steps}, gap_start={gap_start}")

                if gap_steps >= 2:
                    width_cm = gap_steps * 30
                    log(f"DOOR GAP CONFIRMED at step {gap_start}, width ~{width_cm}cm")
                    speak(f"Door gap confirmed! Width approximately {width_cm} centimeters")
                    return gap_start, gap_steps, wall_turns

        if iteration % 5 == 0:
            log(f"Status: steps={steps}, wall_turns={wall_turns}, gap_start={gap_start}, gap_steps={gap_steps}, elapsed={elapsed:.0f}s")

    abort("Max iterations (600) reached without finding door")
    return None, None, None

# ── Phase 3: Door Exit ────────────────────────────────────────────────────────

def phase3_door_exit(gap_start, gap_steps, wall_turns):
    log(f"=== PHASE 3: Door Exit (gap_start={gap_start}, gap_steps={gap_steps}) ===")
    speak("Executing door exit maneuver.")

    # Back up a bit to align with start of gap
    pilot("move -20")
    time.sleep(1.0)

    # Turn right 90° to face into the door gap
    pilot("turn -90")
    time.sleep(1.5)

    speak("Driving through door gap. Monitoring bumpers.")

    # Drive forward 200cm — monitor bumps during drive
    # Break into 4 segments of 50cm for bump monitoring
    total_moved = 0
    bumped_during_exit = False

    for seg in range(4):
        pilot("move 50")
        time.sleep(2.5)
        total_moved += 50
        b = bumped()
        log(f"Exit drive segment {seg+1}: moved {total_moved}cm total, bumps: L={b['left']} R={b['right']}")
        if b["left"] or b["right"]:
            log(f"BUMP during exit drive at segment {seg+1} after {total_moved}cm — false gap (recess/alcove)")
            speak(f"Bump after {total_moved} centimeters. False gap. Resuming perimeter crawl.")
            bumped_during_exit = True
            break

    if bumped_during_exit:
        pilot("stop")
        time.sleep(0.5)
        # Back out
        move_back = total_moved + 30
        pilot(f"move -{move_back}")
        time.sleep(move_back / 10 + 1.0)
        # Turn left 90° to face along wall again
        pilot("turn 90")
        time.sleep(1.5)
        log("Returned to wall after false gap exit")
        # Re-establish wall reference before returning
        reacquire_right_wall()
        return False

    # 200cm drive complete with no bumps
    pilot("stop")
    speak("Forward drive of 200 centimeters complete. Verifying open space ahead.")
    log(f"200cm exit drive complete. No bumps. wall_turns was {wall_turns}. Doing verification probe.")

    # VERIFICATION: Drive another 100cm — if this also has no bumps, we're genuinely in open space
    pilot("move 100")
    time.sleep(5.5)
    b_verify = bumped()
    log(f"Verification probe (100cm): bumps L={b_verify['left']} R={b_verify['right']}")

    if b_verify["left"] or b_verify["right"]:
        # Hit wall — this was a long recess, not a door exit
        log("VERIFICATION FAILED: hit wall after 300cm total — long recess, not a door exit")
        speak("Verification failed. Long recess detected. Not the door. Backing out.")
        pilot("stop")
        time.sleep(0.5)
        pilot("move -130")  # back out 300cm + 30cm margin
        time.sleep(7.0)
        pilot("turn 90")
        time.sleep(1.5)
        # Re-establish wall reference
        reacquire_right_wall()
        return False

    # Both 200cm and additional 100cm with no bumps — genuinely in open space
    pilot("stop")

    # Capture post-exit frame
    capture("post_exit.jpg")

    # Final sense check
    sense_out = pilot("sense")
    log(f"Post-exit sense: {sense_out}")

    log("EXIT_SUCCESS")
    speak("SUCCESS: Robot has exited the room!")
    return True

# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    log("=" * 60)
    log("ESCAPE ATTEMPT 2 (v2) — Bumper-Based Perimeter Crawl")
    log("=" * 60)
    speak("Escape attempt 2 version 2 starting. Bumper perimeter crawl strategy.")

    # Phase 0a: Connectivity
    phase0a_connectivity()

    # Phase 0b: Camera diagnostic (non-blocking)
    cam_orientation = phase0b_camera()

    # Phase 0c: Bumper test (CRITICAL)
    phase0c_bumper_test()

    # Phase 1: Establish wall reference
    phase1_wall_reference()

    # Phase 2+3 loop — retry up to 3 times if we get false gaps
    max_exit_attempts = 5
    for attempt in range(max_exit_attempts):
        log(f"Perimeter crawl attempt {attempt + 1}/{max_exit_attempts}")

        # Phase 2: Perimeter crawl — returns when door gap found
        gap_start, gap_steps, wall_turns = phase2_perimeter_crawl()

        if gap_start is None:
            abort("Phase 2 returned without gap — should have aborted internally")

        # Phase 3: Door exit
        success = phase3_door_exit(gap_start, gap_steps, wall_turns)

        if success:
            log("MISSION COMPLETE")
            speak("Mission complete. Robot has escaped the room.")
            return

        log(f"False gap attempt {attempt + 1} — re-establishing wall and resuming perimeter crawl")
        speak(f"False gap number {attempt + 1}. Re-establishing wall. Resuming perimeter crawl.")

    abort(f"Exhausted {max_exit_attempts} exit attempts — all were false gaps or alcoves")

if __name__ == "__main__":
    main()
