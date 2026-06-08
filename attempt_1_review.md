# Attempt 1 Review — Door-Finding Navigation

## What the robot did

1. Performed a 360° scan in 45° steps from starting position, capturing one photo per step
2. Ran parallel subagents to analyse all 8 photos for door presence
3. Identified step 3 (135°) as the most promising direction — pink curtains + wooden framing visible
4. Attempted to navigate toward that area through a series of turns and short forward bursts
5. Spent most of the run trying to escape a desk cluster without success
6. Eventually achieved a clearer view of the room but never reached the door

## Where the door actually is

The door is on the wall **adjacent to the fixed whiteboard**, set beside a distinctive **rounded wooden column** (door jamb). It is visible in the final images (`opposite2.jpg`, `corner_aim.jpg`) at roughly **135–165° from the robot's starting orientation**. The opening leads to a corridor; no door panel was visible blocking it.

## What went wrong

### 1. Started trapped under a desk
The robot's initial position was directly under or against a desk with a Dell monitor. Nearly every view in the first half of the run was dominated by desk undersides and ceiling. This made the first scan almost useless and wasted many moves just getting to open floor.

### 2. No heading tracking
Turns were issued (e.g. "turn 165", "turn 60", "turn -45", "turn 90", "turn -90"...) but never tracked cumulatively. After ~10 turns the robot had no reliable sense of which direction it was facing, making it impossible to navigate to the known 135° bearing of the door.

### 3. Camera angle too steep
The camera is mounted on the Roomba pointing upward at a steep angle. Most frames show ceiling, upper walls, and the backs of monitors rather than floor-level obstacles and openings. The scan images for steps 4–7 were almost entirely ceiling and unusable.

### 4. No committed strategy
The robot switched between: free exploration, turning toward suspected directions, backing up, and partial wall-following — without committing to any one approach long enough for it to work. Wall-following was identified as the right strategy but never fully executed.

### 5. Physical obstacles blocking the door
- A **mobile whiteboard on wheels** was parked directly in front of the door gap
- The **drawn curtain** concealed the door opening until the robot was nearly on top of it
- **Multiple people** standing near the door blocked both view and path

### 6. Vision was slow and sequential
Each Claude vision call (via `claude --print`) spawned a new process taking ~20–30 seconds. This made the navigation loop very slow. (This is now addressed by the Hailo HAT.)

## User feedback

- **Total movement was ~20cm** — completely inadequate for a room-scale task. The robot barely moved.
- **Way too slow** — each move was tiny and followed by long pauses for vision analysis. The navigation loop needs to be much faster.
- **Plenty of room to move** — the robot was not actually boxed in; it just failed to commit to movement. Hesitation was the problem, not the environment.
- **Use the bump sensors actively** — the Roomba's bumpers and proximity sensors mean it's fine to drive around freely. Hitting a wall or chair is not a problem; the robot can detect and recover. Drive aggressively, not cautiously.
- **Map by moving** — drive around the room perimeter, use bumps to detect walls, and build up a rough sense of the space rather than trying to solve it from a single spot.

## What to do differently in attempt 2

- **Escape the furniture cluster first** — drive backward/sideways until open floor is visible before starting any scan
- **Track heading precisely** — maintain a running heading counter from every `turn` command issued
- **Wall-follow systematically** — drive to the nearest wall, then follow it around the perimeter; the door will appear as a gap
- **Use Hailo for real-time obstacle detection** — enables continuous safe navigation without waiting for vision calls
- **Use Claude vision sparingly** — only for periodic semantic confirmation ("is this a door?"), not for every navigation step
- **Narrate via TTS** — speak key decisions aloud so observers can follow the robot's reasoning
- **The target**: rounded wooden column + opening to the right of the fixed whiteboard, at the wall adjacent to the whiteboard wall
