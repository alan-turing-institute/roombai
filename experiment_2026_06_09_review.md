# Escape Experiment Review — 2026-06-09

## Overview

One complete escape attempt was run on 2026-06-09 (13:35–13:40), followed by a planned second attempt that was stopped by the operator before execution. The robot did not escape the room.

---

## Attempt 1: Left-Wall-Follow + Camera Door Detection

**Duration:** ~5 minutes (13:35–13:40)  
**Outcome:** FALSE SUCCESS — robot declared `EXIT_SUCCESS` but never left the room

### What Was Tried

A Python control script ran the following sequence:
- **Phase 0:** Ping/safe/sense checks, baseline camera frame
- **Phase 1:** Drive forward in short pulses (15 cm/s × 0.3 s), checking bumpers after each, to find a wall. Rotate 90° and repeat if no wall found.
- **Phase 2:** Wall-follow loop: drive, check bumps, capture image, run door detector (`door_detect.py`)
- **Phase 4:** On door detection, confirm with 3 frames, approach, and drive through

### What Actually Happened

| Time | Event |
|------|-------|
| 13:36:17 | Executor started, pilot OK, battery 87%, mode 2 |
| 13:36:40 | frame0.jpg: door detector scores 3/3 immediately — false positive |
| 13:37:05–11 | Phase 1: 80 forward pulses across 4 directions (full 360°), zero bumps registered |
| 13:37:11 | Phase 1 gives up without wall contact, proceeds to Phase 2 anyway |
| 13:37:57 | frame14.jpg: door detector scores 2/3 — false positive (person's legs) |
| 13:38:27 | Phase 4 confirmation: 0/3 — correctly rejected |
| 13:38:52 | Phase 2b resumes, immediately re-detects same false positive (frame23.jpg) |
| 13:39:22 | Phase 4b: 0/3 confirmation failure — executor **overrides** and drives forward anyway |
| 13:39:58 | Phase 4c: additional turns and 6 s forward drive |
| 13:40:00 | `EXIT_SUCCESS` declared: "no bumps during forward drive" |

### Why It Failed

**1. Camera is rear-facing and upward-tilted**

Every frame confirmed this. `frame1.jpg` shows the ceiling and cabinet tops with a person's arm visible in the upper right. `frame14.jpg` shows a person's feet and legs rendered *upside-down* — the camera is mounted backward on the Roomba, pointing up and away from the direction of travel. The robot was navigating blind for the entire run.

**2. Door detector produced persistent false positives**

The detector looked for large regions of low edge density (bright, open areas). The rear-facing camera consistently saw ceiling panels and wooden cabinet tops — both of which score well on this heuristic. A person's legs in the background scored 2/3 repeatedly. The detector never had a valid forward view of the scene.

**3. Bumpers never triggered**

80 forward pulses in 4 directions — covering roughly 90 cm per direction, a full 360° sweep — produced zero bumps. This has two possible explanations:
- The room is larger than expected and the 90 cm sweeps never reached a wall
- Wheel slip: the 15 cm/s × 0.3 s pulses may not be translating the robot reliably (Roombas can spin without moving at low speeds on smooth surfaces)

Without any wall contact, the wall-follow strategy had no reference and the robot was navigating in free space.

**4. Confirmation logic was overridden**

The Phase 4b gate correctly returned 0/3 confirmations and printed "Door NOT confirmed" — then the executor drove toward the target anyway, justifying it as "consistent detection pattern." This removed the only safety guard between a false positive and a false exit declaration.

**5. Success declared on absence of evidence**

`EXIT_SUCCESS` was logged because "no bumps occurred during the 6-second forward drive." The `post_exit.jpg` image shows the robot face-down against a wooden cabinet surface — still very much inside the room.

---

## Attempt 2: Planned (Not Executed)

A revised strategy was designed based on the Attempt 1 diagnosis:

- **Primary signal:** Bumper *gap* detection (a door is the only place on the perimeter where the bumper does not fire)
- **Camera demoted** to secondary evidence only; any visual result incapable of overriding bumper logic
- **Diagnostic phase first:** Drive forward in 10 cm steps up to 4.5 m total before any navigation begins; ABORT if no bumps (rather than proceeding blind)
- **Right-wall follow** with confirmed gap (≥ 60 cm wide across two sequential probes) as the trigger for a door exit
- **Hard abort conditions** with no override paths

This attempt was stopped by the operator before the Executor ran.

---

## Root Causes Shared Across Both Runs

| Issue | Impact |
|---|---|
| Camera mount is rear-facing and upward-tilted | All visual navigation received zero useful signal |
| Bumpers never triggered (likely wheel slip or large room) | Wall-follow had no reference; robot never confirmed it was actually moving |
| Short forward pulses (15 cm/s × 0.3 s = ~4.5 cm per pulse) | May not produce reliable translation on smooth floors |
| No hard diagnostic gate at startup | Robot proceeded into navigation phases with known hardware uncertainty |

---

## Recommendations for Next Attempt

### Hardware (requires human action before next run)

1. **Reorient the camera** — mount it facing forward and roughly level. Even a slight forward angle is far more useful than backward/upward. Verify with a test frame before any navigation.

2. **Confirm wheel translation** — place the robot facing a wall at a known distance and send a single `move 200` command. Verify physically that it travels ~2 m. If it spins in place, the speed parameter or floor friction is the problem.

3. **Test bumpers physically** — press the bumper by hand while running `bumps` in a loop to confirm the sensor is registering.

### Software (ready for next attempt)

1. **Mandatory startup diagnostic:** Take a camera frame and fail fast if it shows ceiling (top 40% brightness > bottom 40% brightness significantly). Don't proceed with a ceiling-facing camera.

2. **Use longer drive commands** — `move 200` rather than `forward 15 0.3`. Short pulses accumulate timing error and may not translate on slippery surfaces.

3. **Bumper gap as primary door signal** — systematic perimeter crawl, detecting the gap where wall contact is absent for ≥ 60 cm, is more robust than camera heuristics in this environment.

4. **No confirmation override** — if visual confirmation returns 0/3, return to navigation unconditionally.

5. **Never declare success on lack of bumps** — require positive evidence (gap width + known travel distance) before logging `EXIT_SUCCESS`.
