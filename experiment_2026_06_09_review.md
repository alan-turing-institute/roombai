# Experiment Review — 2026-06-09

## Summary

Two autonomous escape attempts were run. The robot did physically drive through the door during attempt 2, but the software did not detect this and the run ended inconclusively. Three core strategy problems were identified.

---

## Attempt 1

**Duration:** ~88s | **Bumps:** 14 | **Outcome:** False success

**What happened:** Right-hand wall-follow arc (`go 20 -8`) with OpenCV green blob detection (HSV H∈[36,85], S≥50, V≥50). The robot reached the door area quickly but triggered APPROACH mode on false positives — the fluorescent-lit pale wall has a greenish tint that passed the S≥50 threshold. The "success" condition (5s unobstructed forward in APPROACH) was met while the robot drove along a gap in the wall, not through the door.

**Root causes:**
- HSV saturation threshold too low (S≥50 matched greenish walls)
- Success condition was time-without-bump only — not a real door confirmation

---

## Attempt 2

**Duration:** ~95s | **Bumps:** 16 | **Outcome:** Robot drove through the door, but not detected

**What happened:** Tightened HSV (S≥120), added blob size/position/aspect ratio filters, stricter success condition. Green area correctly stayed at 0 throughout — no false positives. The robot physically drove through the door but the script crashed silently at ~95s (unhandled exception in a daemon thread, swallowed by Python) before any door detection could register. Post-hoc camera frames confirmed the robot was at the door and had likely exited.

**Root causes:** See "Core Problems" below.

---

## Core Problems Identified

### 1. Door detection strategy is brittle (green card reader assumption)

Both attempts used OpenCV green blob detection tuned specifically to find a green electronic card reader. This only works if:
- The target door has a green card reader
- The card reader is visible from the robot's height
- Lighting conditions produce a distinctive HSV signature

This is not a general escape strategy. A robust approach should detect the *door opening itself* (an open region, a change in floor/wall texture, a bright patch of corridor light) rather than a specific fixture on the wall.

**Recommendation:** Switch to detecting the open doorway directly — e.g., detecting a large dark region at floor level (the gap under/through an open door), detecting a brightness discontinuity indicating a transition to a different space, or using optical flow to detect when the robot is crossing a threshold into a new environment.

### 2. No camera-based obstacle avoidance during exploration

The explore.py motion thread used only the bumper sensors for obstacle detection. The camera was only polled every 3 seconds for door detection — it was never used to anticipate or avoid obstacles. As a result the robot bumped into many things (desks, chairs, columns, the door frame itself) that would have been visible to a camera.

**Recommendation:** Use the camera more frequently during EXPLORE mode (every 0.5–1s) to detect obstacles ahead and steer around them before contact. Even a simple "is there a large dark/close object in the lower centre of the frame?" check would significantly reduce unnecessary bumps and navigate the robot more efficiently through cluttered spaces.

### 3. Silent thread crashes end the run without diagnosis

In attempt 2, the motion or vision thread threw an unhandled exception that was silently swallowed by Python's daemon thread mechanism. The script exited cleanly with no error log, making it impossible to diagnose from the execution log.

**Recommendation:** Wrap all thread bodies in a top-level `except Exception as e` that logs the full traceback to `/tmp/execution_log.txt` before the thread exits. Without this, any bug in a thread is invisible.

---

## Memory Contamination

During this session, the Orchestrator (Claude) used stored project memories from previous sessions — specifically the `project_door_facts.md` memory which described the door as having a "green electronic card reader" next to a "rounded wooden column." This directly shaped both attempts' detection strategy.

This is problematic because:
- Memories from prior sessions may be stale or wrong
- Using known room layout violates the intent of a blank-slate fresh run
- It produces brittle strategies (green card reader assumption) instead of general ones

**Recommendation:** The Escape Protocol in CLAUDE.md should explicitly prohibit reading or acting on memory files during escape attempts. See the updated CLAUDE.md constraint.

---

## Recommendations for Next Attempt

1. **Door detection:** Detect the open doorway, not a specific fixture. Ideas:
   - Detect a large bright horizontal band at floor level (corridor light through open door)
   - Detect a significant brightness or depth discontinuity in the camera image
   - Use optical flow: once crossing the threshold, motion pattern changes distinctively
   - Floor colour/texture change (corridor floor vs. room floor)

2. **Obstacle avoidance:** Poll camera every 0.5–1s during EXPLORE; detect large close objects in the lower frame and steer away before bumping.

3. **Thread safety:** Add `except Exception` handlers with full traceback logging in every thread body.

4. **Success detection:** Do not rely on "time without bumps" alone. Confirm with a positive environmental signal (different scene in camera, floor change, etc.).

5. **No memory use:** Treat room layout as completely unknown at the start of every run.
