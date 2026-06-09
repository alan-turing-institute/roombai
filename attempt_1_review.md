# Attempt 1 Review — 2026-06-09

## Strategy Tried

Left-wall-follow + camera door detection. Phases:
- Phase 0: Startup sanity check (ping, safe mode, baseline frame)
- Phase 1: Find wall (drive forward in pulses until bumper triggers)
- Phase 2: Wall-follow loop while detecting doors via camera scoring
- Phase 4: Door approach and exit on detection trigger

## Key Events (Timeline)

| Time | Event |
|------|-------|
| 13:35:21 | Attempt start |
| 13:36:17 | Executor started |
| 13:36:40 | Phase 0: frame0.jpg door_detect=3/3 — FALSE POSITIVE (camera shows wooden floor close-up) |
| 13:37:05 | Phase 1: Wall search starts — 80 forward pulses, 4x 90° left turns = 360° rotation |
| 13:37:11 | Phase 1 ends: NO wall found, zero bumps throughout. Robot gives up and proceeds anyway. |
| 13:37:25 | Phase 2: Wall-follow loop starts. frame1.jpg score=0 (correct). |
| 13:37:57 | Phase 2 step 13: frame14.jpg score=2 — DOOR DETECTED — FALSE POSITIVE (person's legs against wooden panels) |
| 13:38:27 | Phase 4 confirmation: 0/3 — correctly rejected false positive |
| 13:38:52 | Phase 2b resumes — immediately detects same false positive again (frame23.jpg score=2, person's legs again) |
| 13:39:22 | Phase 4b: 0/3 confirmation failure — but system OVERRIDES and drives forward anyway toward false positive |
| 13:39:58 | Phase 4c: Turns left 30° more, drives 4s + 2s forward |
| 13:40:00 | EXIT_SUCCESS declared (no bumps during drive = "passed through door") |

## Critic Verdict

**UNCERTAIN** — The exit success is not verified.

The Executor declared EXIT_SUCCESS based solely on "no bumps during forward drive." However:

1. Every "door detection" event (frames 14, 23, final_exit3, align1) was a FALSE POSITIVE — all showed a person's legs against wooden wall panels, not a door opening.
2. post_drive1.jpg and post_drive2.jpg show the same wooden wall panel environment as all previous frames — no change in environment that would indicate passing from inside to outside.
3. The confirmation system correctly failed twice (0/3 both times) but was overridden in Phase 4b/4c.
4. Wall contact was NEVER achieved (zero bumps in 100+ movements). The wall-follow strategy never had a wall to follow.

The robot may have driven in an open part of the room and happened not to bump into anything — which was then interpreted as a successful door exit.

## Root Cause Failures

### 1. Camera Pointing Upward
Every frame shows ceiling and upper wall panels. The camera is not directed forward and level. This made all visual-based navigation unreliable. The door detector scored high on ceiling-wall junctions and a person's legs.

### 2. Door Detector Produces False Positives
The scoring heuristic fires on vertical lines + light background patterns common throughout the room (wooden panels, person's legs, wall-ceiling junctions). It scored 2-3/3 on zero actual door openings.

### 3. Wall Never Contacted
Phase 1 drove 80 pulses in 4 directions (~90cm per direction) with zero bumps. The left-wall-follow strategy is completely blind without a wall reference. Possible causes: very large room, wheel slip causing robot to spin in place rather than translate, or bumper sensors not registering.

### 4. Confirmation Logic Overridden
Phase 4b explicitly detected 0/3 confirmation hits, printed "Door NOT confirmed" — then drove toward the target anyway "based on consistent detection." This bypasses the safety guard and is a logic bug.

## What the Next Attempt Should Do Differently

1. **Verify camera angle first**: Take a test frame at startup. If it shows ceiling (no floor or forward obstacles visible), stop and alert for human camera adjustment. Do not proceed with a ceiling-pointing camera.

2. **Fix door detection**: The current heuristic is fooled by anything with vertical contrast lines. Use a more specific signal:
   - Look for a **dark rectangular region** (open doorway = darkness/outside)
   - Or look for **bright light leaking under a door** at floor level
   - Require the detected "door" to persist across multiple frames taken at slightly different angles (parallax test)

3. **Never override confirmation failure**: If 0/3 confirmations, do NOT drive toward target under any circumstances. Return to navigation loop.

4. **Test bumpers before navigation**: At startup, deliberately drive slowly into a known obstacle (e.g., drive forward until bump or until 3m elapsed). If 3m with no bump, raise an explicit "bumper may be non-functional" alert. The strategy cannot work without reliable bump detection.

5. **Use longer wall search sweeps**: Phase 1 used ~90cm per direction. A typical room is 3-5m across. Use at least 300cm (60 pulses at the same rate) before giving up on a direction, or increase pulse speed/duration.

6. **Consider spin-scan approach instead of wall-follow**: Do a slow 360° rotation in place, capturing frames every 30°, scoring all 12 views for door candidates, then navigate toward the highest-scoring direction. This doesn't require wall contact.
