# Attempt 1 Review

**Duration:** 88 seconds  
**Outcome:** FALSE SUCCESS — script declared success but robot did not escape

## What Happened

- Strategy: right-hand wall-follow arc (go 20 -8) + OpenCV green blob detection
- Robot bumped 14 times and switched to APPROACH mode 3 times
- At ~78s: large green blob (6052px area) detected centred → APPROACH mode
- 5 seconds of unobstructed forward motion triggered "SUCCESS" condition
- Script exited; robot returned to passive mode (mode=0), left bumper still touching wall

## Root Cause Failures

### 1. Vision false positives — HSV threshold too broad
The camera wall is a pale greenish-white surface (fluorescent lighting tints everything slightly green). HSV mask H∈[36,85], S≥50, V≥50 was triggering on the wall itself. The green blobs detected (3600–23448px) were the wall surface, not the card reader.

**Fix:** Tighten to S≥120 (much more saturated) and possibly H∈[40,80]. The actual card reader will be a vivid saturated green. Optionally also require area in a specific pixel range (not too large = not the wall).

### 2. Success condition unreliable
"10 consecutive 0.5s without bump in APPROACH mode" can be satisfied by driving along an open wall section — does not confirm the robot is through the door.

**Fix:** Success should require BOTH: (a) unobstructed forward for 5s AND (b) green blob no longer visible (robot has passed through door, card reader is behind it). Or: require the robot to be driving and the scene through the camera to show a corridor/different environment.

### 3. Mode 0 on exit
After explore.py exits, the robot falls to passive mode (mode=0). For the next attempt, ensure `safe` mode is restored before starting.

## Recommendations for Attempt 2

1. Tighten green HSV: H∈[40,80], S≥120, V≥80
2. Add maximum area cap on green blob: 300 < area < 8000 (the card reader is small, the wall is huge)
3. Change success condition: unobstructed forward for 5s AND green blob disappears from frame (robot passed the door) AND open space (no bumps for 10s from first approach step)
4. Consider adding a blob aspect ratio check — the card reader is roughly rectangular/square, the wall fill is not
5. The motion exploration itself worked well — robot covered ground in 88s with 14 bumps
