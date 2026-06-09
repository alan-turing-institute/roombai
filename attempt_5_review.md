# Attempt 5 Review — Door-Finding Navigation

## What the robot did

1. Launched `explore.py` — robot immediately moved forward with zero bumps for ~24s (was not actually moving due to cliff-sensor OI mode drops)
2. Fixed by switching to FULL mode (mode 3) — disables cliff-sensor safety stops that were silently preventing movement
3. Robot began genuine exploration: 20+ real bumps over ~4 minutes, covering significant ground
4. 360° targeted scan revealed the open wooden door at step 6 (clear corridor visible)
5. Robot navigated to the wooden door and drove through — exit_2.jpg confirmed corridor floor visible, room carpet behind
6. Attempted to verify by turning 180° and driving further — but drove back into the main room instead
7. Verification loop became confused; attempt stopped by user

## What went well

- **FULL mode fix** was critical — solved the silent OI mode drop problem
- **Camera repositioning + rotation 180° fix** — corrected the upside-down image and alignment offset that was causing the robot to hit the speaker instead of the door gap
- **Targeted 360° scan** was highly effective — found the open door quickly when explore.py was paused
- **Camera at floor level** (new position) is much better for navigation than the old steep upward angle
- Robot did physically pass through the wooden door (confirmed by floor change in exit frames)

## What went wrong

### 1. Exploration was very inefficient — kept revisiting the same areas
The random bump-and-turn (120° ± 30°) in a bounded room causes the robot to circle back repeatedly. There is no mechanism to avoid previously-explored directions. The robot spent multiple minutes covering the same floor area rather than systematically sweeping the room.

### 2. Very long pauses between movements
Main causes:
- `ensure_safe()` runs every 5th move cycle and calls `send_cmd("sense")` — this TCP round-trip + robot serial query adds 1–2s of dead time per 5 moves
- The `bumped()` call after every forward burst adds another TCP round-trip
- `SUPERVISE_EVERY = 5.0s` spawns a `claude --print` subprocess every 5s — while this runs in a background thread, the spawning overhead may create intermittent delays
- Multiple concurrent Claude vision subprocesses (each taking 20–30s) can cause resource contention on the Pi

### 3. Verification drove back into the room
After exiting, turning 180° to verify then calling `forward` sent the robot back through the door. The verification procedure should have been: drive further into corridor first, THEN turn around to look back.

### 4. Wrong door
The wooden door that was navigated was likely not the target. Rescan step 4/5 revealed a **glass door in a glass wall** — which was closed and in a different part of the room. (Note: earlier attempts incorrectly described "rounded wooden columns" — the user confirmed there are no rounded columns; the door is simply set in a glass wall.)

## User feedback

- **Exploration was inefficient** — robot kept going back to the same places; random bump-and-turn does not give good room coverage
- **Very long undesirable pauses between movements** — the process needs to be sped up significantly
- Exploration must be made faster and more systematic

## What to do in attempt 6

### Fix the pauses
- Remove `ensure_safe()` from the mover loop — instead, just blindly re-send `full` at startup and after any mode drop detected implicitly (check mode only when a command fails, not on a timer)
- Or: run `ensure_safe()` every 20th iteration, not every 5th
- Reduce `bumped()` calls — query bumpers less often (only after each forward burst that might have touched something)
- Consider removing the per-move bumper check entirely and relying on the OI's built-in bump detection via the `sense` response

### Fix exploration efficiency  
- Replace random turn direction with **consistent right-turn wall-following**: after every bump, always turn right (CW) by ~90°. This guarantees systematic coverage of the room perimeter
- Or: increase `TURN_AFTER_BUMP` to always be exactly 90° (no randomness) and always the same direction — this is closer to a wall-following heuristic
- Increase `MOVE_SPEED` from 25 to 35 cm/s and `MOVE_BURST` from 2.5s to 3.0s so each burst covers more ground

### Speed up the loop
- The main loop overhead (bumps check, strategy read, safe check) should total < 100ms per cycle
- Target: move every ~3s with < 200ms of non-movement overhead between bursts
