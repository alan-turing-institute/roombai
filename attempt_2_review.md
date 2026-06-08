# Attempt 2 Review — Door-Finding Navigation

## What the robot did

1. Started from a new position — found a good first-frame showing the whiteboard edge, a glass partition with a wooden door frame and handles, and teal bench seating
2. Drove ~120cm forward toward the glass partition
3. Discovered it was a window-with-bench, not a door
4. Backed up, hit the whiteboard behind it (robot was sandwiched between whiteboard and window wall)
5. Turned right 90° — saw the rounded wooden column and a glass door with green card reader immediately (this was the door from attempt 1 analysis!)
6. Started navigating around the teal bench toward it — then stopped

## What went well

- Found the rounded wooden column + card-reader door within ~4 moves from a good starting position
- Starting position was much better (not under a desk)
- The first image was highly informative

## What went wrong

### 1. Still way too slow (~20cm total effective movement)
The approach remained: capture photo → analyse → decide → move a tiny amount → repeat. Each cycle took 10–30 seconds. The robot was nowhere near the door when the attempt was cut short.

### 2. Claude as the main movement loop is fundamentally broken
Having the Claude chat session be the loop driver means every step requires:
- A Bash tool call (~1s)
- A Read tool call for the image (~2s)
- Claude reasoning (~5s)
- Another Bash tool call for the move (~1s)

This produces ~10s per move cycle at best. A room is ~10m across. At 20cm/move this is 50 moves = 8 minutes minimum, with no concurrency, no recovery from dead ends, and no ability to respond to fast-changing situations.

### 3. No concurrent movement and perception
The robot was stationary during every vision analysis. Real navigation requires the robot to be moving while perception happens in the background.

### User feedback
- Total movement across both attempts was ~20cm — completely inadequate
- The robot had plenty of room to move; hesitation was the problem not the environment
- The robot should drive aggressively and use its bump sensors to handle collisions
- Don't worry about hitting things — the bumpers handle it
- Need concurrent movement + perception, not a sequential photo-then-move loop

## What to do in attempt 3

- **Write a standalone exploration script** (`explore.py`) that runs the robot continuously and autonomously. Claude acts only as a high-level supervisor that reads its log and updates a strategy file.
- **Movement runs independently** of vision — the robot keeps moving while Claude analyses frames in a background thread
- **Aggressive movement**: 25+ cm/s, 2–3s bursts, immediate bump-and-turn recovery
- **Hailo AI HAT** for real-time YOLO inference on the camera stream — obstacle/person detection at ~10fps without blocking movement
- **Keyframes + log** written to disk continuously so Claude can catch up at any pace
- **Strategy file** (`/tmp/roomba_strategy.json`) that Claude can update at any time to redirect the robot
- The door is known: **glass door with green card reader, next to a rounded wooden column, adjacent to the fixed whiteboard wall**
