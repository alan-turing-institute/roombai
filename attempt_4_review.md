# Attempt 4 Review — Door-Finding Navigation

## What the robot did

1. Launched `explore.py` with concurrent movement and camera threads
2. Robot immediately bumped left from start position, turned ~132° CW to heading 228°
3. Continued bumping and turning, accumulating 5 bumps total, covering real ground
4. Supervisor spotted what looked like a rounded wooden column and Venetian blinds in frames 17/19
5. Switched to APPROACH mode, bearing 66°
6. Robot turned to face that bearing and drove forward
7. The robot lost SAFE mode mid-run (dropped to passive/mode=1) — movement silently stopped
8. Supervisor noticed and manually re-entered SAFE mode, restarted explore.py
9. APPROACH mode resumed — robot immediately started bumping chair legs at bearing 66°
10. Stuck in a bump loop: approach → bump → back → -30° → re-aim to 66° → bump again, repeat
11. User called stop

## What went well

- **Real movement**: 5+ bumps, robot covered significant ground — much better than attempts 1–3
- Concurrent movement + vision (explore.py) is the right architecture — don't wait for vision before moving
- Supervisor actively monitoring frames and updating strategy was useful
- Camera was showing useful environmental detail (chairs, curtain, wooden structures)

## What went wrong

### 1. Relied on stale assumptions from previous attempts
The supervisor (Claude) looked at frames and assumed it was seeing the "rounded wooden column" and "Venetian blinds" from previous attempt notes. This led to switching to APPROACH mode prematurely, before actually confirming the target door was in frame. Room layout from previous runs should NOT be trusted — treat every run as starting fresh.

### 2. APPROACH mode has no real obstacle avoidance
When the robot bumped, the APPROACH code only turned 30° and then immediately re-aimed at the same bearing. With chairs blocking the path, this creates an infinite bump loop. The robot needs to actually navigate around obstacles, not just twitch.

### 3. Robot lost SAFE mode silently
During the first run, the robot dropped from mode=2 to mode=1 (passive), and `explore.py` kept sending forward commands with no effect — nothing in the logs indicated failure. This caused several minutes of wasted time. A `ensure_safe()` function was added to recover, but it wasn't deployed in time to help this run.

### 4. Strategy was too locked-in
Once APPROACH mode was set with a fixed bearing, the robot kept hammering that bearing even as it became clear the path was blocked. The supervisor should have switched back to EXPLORE much sooner.

## User feedback

- **Do NOT use assumptions about room layout from previous runs** — findings from prior attempts should not be trusted. Start fresh each time.
- Aggressive exploration and bumping was good — keep this.
- **Use the camera more actively** to help navigate — don't just use it for occasional door-spotting.
- Don't be tied to `explore.py` — be more fluid about controlling strategy.
- **Come up with a strategy at the start of each run** and monitor against it.
- If things aren't going well, **stop and iterate** on the strategy rather than persisting with a failing approach.

## What to do in attempt 5

- Start with a **blank slate** — no assumed room layout, no pre-set bearing
- Define a clear strategy before launching (e.g. "wall-follow clockwise for 2 min, take Claude vision check every 30s, stop and reassess if >10 bumps without door sighting")
- Use camera frames actively to make real-time steering decisions, not just periodic confirmation
- Use vision to identify clear paths and openings, not just to confirm a pre-held belief about where the door is
- Keep EXPLORE as the default until the camera gives a genuine door sighting
- APPROACH should use shorter bursts and wider obstacle-avoidance turns (60–90° not 30°)
- Set a clear time/bump budget: "if X minutes pass without door sighting, reassess"
