# RoombaI Escape — Design Spec

Status: draft v1 (2026-06-10)

## 1. Goal

Autonomously drive a Roomba 770 (Pi 5 + Camera Module v3, mounted the correct way up)
out of an unknown room through the target door, as fast as possible, without collisions.

Success criteria, in order:
1. The robot exits through the door.
2. No collisions with obstacles (chairs, walls, legs, bags) — light contact via the
   compliant bumper is a recoverable event, not a failure, but the system must not
   *rely* on bumping.
3. Time to exit is minimized.

Prior attempts failed because the control loop was an LLM making slow, coarse,
non-interruptible decisions. This design replaces that with a classical autonomous
robotics stack: continuous control, probabilistic perception, occupancy-grid mapping,
and frontier exploration.

## 2. Design principles

- **Interruptible control.** No fire-and-forget timed motion. The robot is driven by
  velocity setpoints that can change at any instant, behind a watchdog.
- **Probabilistic everything.** Perception outputs evidence with confidence, not
  binary labels. The map is log-odds. Door detection accumulates belief across
  viewpoints. Decisions are made on posteriors, not single observations.
- **Sensors are guilty until proven working.** The 770's OI support for encoder and
  light-bump packets is unverified. Every sensor is characterized on the robot before
  any module depends on it, and each has a designed fallback.
- **Offline-first development.** Robot time is scarce. Every module must be testable
  on the dev machine via unit tests, the simulator, or recorded-data replay. On-robot
  sessions are scheduled, scripted, and primarily for calibration/validation/recording.
- **Rust-first.** All runtime software is Rust. Python is allowed only in tightly
  bounded *offline* tools (camera calibration, dataset visualization).
- **`roomba_pilot/` is untouched.** New code lives in a new workspace. We may depend
  on `roomba_pilot` as a library for its pure OI protocol encoders/decoders; if that
  causes friction we copy the ~230-line protocol module instead of modifying it.

## 3. System architecture

### 3.1 Process model

A single Rust binary (`brain`) owning all runtime threads, communicating via channels:

```
 rpicam-vid ──pipe──> [vision thread] ──FreeSpace──┐
                                                   v
 serial ─────────────> [driver thread] ──SensorFrame/Pose──> [mapper thread] ──Grid──> [planner thread]
                ^                                                                          │
                └────────────────────── Twist setpoints ──────────────────────────────────┘
```

Rationale for in-process over a TCP daemon: lower latency, one shared clock, simpler
state. Safety against planner crashes is provided by the driver watchdog, not process
isolation. The existing `pilot` daemon remains available for ad-hoc manual control.

The driver is abstracted behind a trait so the simulator and replay harness slot in:

```rust
trait RobotIo {
    fn set_twist(&self, v_mm_s: f64, omega_rad_s: f64);   // takes effect immediately
    fn subscribe(&self) -> Receiver<SensorFrame>;          // ~20 Hz minimum
}
```

### 3.2 Workspace layout

```
escape/                  # new cargo workspace
  crates/
    core/                # shared types: Pose2, Twist, SensorFrame, FreeSpace, Grid; no I/O
    driver/              # serial + OI, control loop, watchdog, reflexes, odometry
    vision/              # YUV ingest, floor segmentation, IPM -> FreeSpace
    mapper/              # log-odds occupancy grid, frontier extraction, door belief layer
    planner/             # heading chooser, exploration policy, behavior state machine
    sim/                 # 2D differential-drive simulator implementing RobotIo
    replay/              # dataset recording format + replay implementing RobotIo / frame source
    brain/               # orchestrator binary (also: record mode, calibration-capture mode)
    characterize/        # S1 session runner (sensor probe, odometry cal, board stills)
  tools/
    calibrate/           # Python+OpenCV: intrinsics + extrinsics from checkerboard captures
    viz/                 # Python: plot grids, trajectories, overlay free-space on frames
```

### 3.3 Build & deploy

Develop and unit-test on the Mac (sim and replay run natively; `core`, `mapper`,
`planner`, `vision` are platform-independent). Deploy by building on the Pi 5
(`git pull && cargo build --release` — the Pi 5 is fast enough). Cross-compilation is
a fallback, not a requirement.

## 4. Module specifications

### 4.1 `driver` — continuous control and sensing

Owns the serial port. Two responsibilities: a fixed-rate control loop and a sensor
pump, sharing one thread (the OI is half-duplex-ish over one UART; serialize access).

Control loop (20 Hz):
- Latest `Twist` setpoint → wheel velocities via differential-drive kinematics
  (wheel span 235 mm) → `DRIVE_DIRECT`.
- Acceleration limiting (configurable, ~150 mm/s² default) for smooth motion and
  reduced wheel slip (protects odometry).
- **Watchdog:** no fresh setpoint within 300 ms → command zero velocity.
- **Reflexes (in-driver, below the planner):** bump or cliff event → immediate stop +
  latched `Reflex` event published upward; driver refuses forward motion until the
  planner acknowledges and commands a recovery (reverse/turn). Wheel-drop → full stop.

Sensor pump:
- Preferred: OI stream mode (opcode 148) with a chosen packet set; decode the
  16-Hz framed stream (framing/checksum decoding is new code — `roomba_pilot`
  doesn't have it).
- Fallback (if 148 unsupported on the 770): `QUERY_LIST` polling at 20 Hz interleaved
  with drive commands.
- Packet set: bumps/wheel-drops (7), cliff signals (28–31), encoders (43, 44),
  light bumps (46–51), battery (25), OI mode (35).
- Odometry: integrate encoder deltas (43/44, ~0.4446 mm/tick) → `Pose2` published
  with every `SensorFrame`. **Fallback** if 43/44 absent: packets 19/20 (integrated
  distance mm / angle deg, coarser and quantized) — same interface, degraded accuracy
  flag set so downstream can widen uncertainty.

### 4.2 `vision` — free space from monocular floor segmentation

Acquisition: spawn `rpicam-vid --codec yuv420 --width 640 --height 480 --framerate 15
--lens-position <fixed> -t 0 -o -`, read raw frames from the pipe (camera is mounted
the correct way up — no flips).
Process at 640×480 or downsample to 320×240; budget ≥10 Hz on one Pi 5 core.

Pipeline per frame:
1. Convert ROI to HSV (integer math, no deps needed).
2. **Floor appearance model bank:** k histogram models (H×S, coarse bins) for distinct
   floor materials seen so far. The trapezoidal patch directly ahead of the bumper
   seeds/updates the active model *unless* a recent reflex or light-bump contradiction
   marks it suspect. The target room's floor is known: light-blue carpet with
   occasional dark-blue tiles and grey hatch panels — so the bank starts with ≥3
   classes pre-seeded from S1 footage, and a dark tile must classify as floor, not
   obstacle.
3. Per-pixel floor likelihood from the model bank → per-column scan upward from the
   bottom to the first sustained non-floor run → boundary row per column.
4. Project boundary pixels through the ground-plane homography →
   `FreeSpace { rays: Vec<(bearing, free_dist, confidence)> }` in the robot frame.
   Confidence derives from likelihood margin and distance (far = less confident).

Calibration (offline, once, **no props**): the homography is recovered from the
robot's own ego-motion in the S1 video, not a checkerboard. During the straight
legs, floor features flow through the image in a pattern that fixes the camera's
height-and-tilt geometry; the spins cross-check rotation. The result is expressed
in **odometry/ground units, not metres** — only self-consistency between camera and
motion is needed. A rough absolute scale comes free from the nominal wheel spec
(72 mm wheels, 508.8 ticks/rev, ≈0.4446 mm/tick), which is enough to know the
robot's own footprint for clearance. Output is a TOML consumed by `vision`. Video↔
log time alignment uses the S1 sync pulse (motion-onset cross-correlation).

Online self-correction: light-bump and bumper events are labeled ground truth. A
contradiction (vision said free, IR says obstacle at 5 cm) penalizes the active floor
model and triggers re-seeding — this is the floor-material robustness mechanism.

Door/target detection runs in the same thread at reduced rate (~2 Hz, full-width
frames): vertical edge pairs at plausible door widths + appearance prior from the
known photo of the target door → `DoorObservation { bearing, width, score }`.
Single-frame output is deliberately noisy; belief lives in the mapper.

### 4.3 `mapper` — occupancy grid and door belief

- Log-odds occupancy grid, 5 cm cells, robot-start-anchored frame.
- Inverse sensor models:
  - `FreeSpace` ray: cells along ray get free evidence scaled by confidence; the
    boundary cell gets weak occupied evidence (it might be floor-material change).
  - Light bump: strong occupied evidence in a small arc at the sensor's bearing.
  - Bumper press: very strong occupied evidence at the contact arc.
  - Cliff: mark cell as a hard no-go (separate layer, never decays).
- Pose source: driver odometry, taken as ground truth in v1 (no scan matching).
  Rotation drift is the known weakness; mitigated by slow spins and acceleration
  limits. If session data shows the map shearing within one room, add rotation-only
  visual odometry (image registration during spins) as v2 — designed-for but not built.
- **Frontier extraction:** boundary cells between known-free and unknown, clustered;
  output ranked frontier list.
- **Door belief layer:** `DoorObservation`s are ray-cast into the grid and associated
  with wall segments; each candidate door accumulates a Beta-style belief across
  viewpoints. An open door also manifests as free space punching through a wall line —
  both signals feed the same candidate.

### 4.4 `planner` — behavior and local navigation

Behavior state machine:

```
CALIBRATE_SPIN -> EXPLORE <-> RECOVER -> APPROACH_DOOR -> TRAVERSE -> DONE
```

- `CALIBRATE_SPIN`: slow 360° on start — seeds the map, the floor model, and door
  candidates before moving.
- `EXPLORE`: frontier-based. Goal = best frontier by (size / distance) with a strong
  bonus for high-belief door candidates. Re-plan continuously.
- `RECOVER`: reflex latched → reverse ~10 cm, turn away from contact bearing, mark
  map, resume.
- `APPROACH_DOOR`: door belief above threshold → drive a centered approach, verify
  (belief keeps rising as we approach; else demote and resume EXPLORE).
- `TRAVERSE`: align perpendicular, drive through, declare success when odometry shows
  passage beyond the wall line and vision shows open space.

Local navigation (runs every cycle regardless of state): VFH-style heading chooser —
candidate headings scored by free distance (from `FreeSpace` + grid), clearance, and
goal progress; forward speed scales with forward clearance (max ~250 mm/s, creep
~80 mm/s near obstacles). Output: one `Twist` per cycle to the driver.

### 4.5 `sim` — offline control/planning testbed

2D kinematic differential-drive robot in a polygon world (walls, furniture, a door
gap). Implements `RobotIo`: synthesizes encoder ticks (with slip noise), bumper and
light-bump returns from world geometry, and optionally synthetic `FreeSpace` (bypassing
vision) so mapper/planner run unchanged. Deterministic with seeded noise. Headless for
tests + a debug renderer (grid/trajectory dump via `tools/viz`).

### 4.6 `replay` — datasets

Recording mode in `brain`: write timestamped JSONL sensor log + raw YUV (or H.264)
video side by side. Replay implements the frame source and `SensorFrame` stream so
`vision` and `mapper` are developed offline against real-world data.

## 5. Testing strategy

| Layer | How it's tested off-robot |
|---|---|
| OI encoding/decoding | unit tests (existing pattern in `roomba_pilot`) |
| driver control loop | unit tests against a mock serial; watchdog/reflex timing tests |
| vision | replay of recorded sessions; golden free-space outputs on key frames |
| mapper | sim (synthetic FreeSpace) + replay; map-vs-ground-truth in sim |
| planner | sim end-to-end: time-to-exit + collision count across seeded worlds |
| brain integration | sim end-to-end smoke test in CI |

On-robot time is reserved for: calibration, sensor characterization, dataset
recording, and validation of already-sim-proven behavior.

## 6. On-robot session plan

**S1 — characterization & first dataset (~5 min robot time, fully scripted):**
Run by the `characterize` binary (`escape/crates/characterize`), narrated over TTS,
**no keyboard input and no props** (no checkerboard, ruler, or tape). See
`escape/RUNBOOK_S1.md` for the full runbook. Phases:
A — stationary OI probe (packets 43/44? 46–51? stream 148?) + battery;
sync — forward/back jerks to align the video clock to the encoder log offline;
B — straight runs (slow + fast + creep to bumper contact): pixel→ground flow
calibration and the light-bump curve anchored at contact;
C — spins (slow CCW, slow CW, fast CCW): encoder rotation cross-checked against
visually-observed rotation (the rotation-drift question), VO test footage;
D — out-and-back over the dark tiles (cliff reflectivity + floor appearance video).
Video records continuously from the sync pulse through D; all sensors logged at
20 Hz to JSONL. The calibration target is the robot's own ego-motion, not a board,
so calibration happens offline from this data. Validated end-to-end off-robot via
`--dry-run` against a mock OI robot.

**S2 — driver validation:** smooth setpoint driving, watchdog, reflexes.
**S3 — vision-in-the-loop:** drive-toward-free-space without map.
**S4 — full stack:** exploration + door approach in the real room.

## 7. Milestones

- **M0** Workspace scaffold, `core` types, sim skeleton, CI (`cargo test` green on Mac).
- **M1** Session S1 done → sensor truth table written down; calibration TOML produced;
  dataset #1 recorded.
- **M2** `driver` complete and validated (S2).
- **M3** `vision` produces good free space on dataset #1 (offline, eyeballed via viz +
  golden tests).
- **M4** Smooth obstacle-free wandering on the robot (S3) — vision + driver + local
  nav, no map.
- **M5** Mapping + frontier exploration solid in sim; then on robot.
- **M6** Door detection + APPROACH/TRAVERSE; full escape runs (S4). Tune for time.

## 8. Risks and open questions

| Risk | Mitigation |
|---|---|
| 770 lacks encoder packets 43/44 | fallback to 19/20 (coarser); widen pose uncertainty |
| 770 lacks light bumps / stream mode | bumper-only reflexes + slower max speed; QUERY_LIST polling |
| Rotation odometry drift shears map | slow spins, accel limiting; v2: rotation-only visual odometry |
| 66° FOV blind zone near bumper | map memory + light bumps cover near field; measure in S1 |
| Floor variation (dark-blue tiles, grey hatches in the light-blue carpet) read as obstacles | pre-seeded model bank from S1 footage + reflex-labeled online updates; boundary cells get only weak occupied evidence |
| Autofocus hunting | pin `--lens-position` after S1 focus check |
| People moving through the scene | grid evidence decays toward unknown slowly; reflexes handle the rest |

## 9. Decisions log

- 2026-06-10 — Rust for all runtime code; Python only for offline calibrate/viz tools.
- 2026-06-10 — New `escape/` workspace; `roomba_pilot/` untouched (may be depended on).
- 2026-06-10 — Single-process `brain` with `RobotIo` trait, not a TCP daemon split.
- 2026-06-10 — v1 mapping uses odometry poses (no loop closure / scan matching);
  rotation-only VO is the designed escape hatch if drift proves fatal (confirmed:
  keep rotation VO on the table).
- 2026-06-10 — Camera remounted the correct way up; no image flipping anywhere.
- 2026-06-10 — S1 is capped at ~5 minutes of robot time → fully scripted
  `characterize` runner with TTS narration, validated off-robot via `--dry-run`.
- 2026-06-10 — No metric/world-frame calibration. Camera↔motion is self-calibrated
  from ego-motion in the S1 video; everything lives in odometry/ground units, with
  approximate absolute scale taken free from the nominal wheel spec. No checkerboard,
  ruler, or tape — none are available. Video↔log sync via a motion-onset pulse.
- 2026-06-10 — Room floor known a priori: light-blue carpet, occasional dark-blue
  tiles, grey hatch panels → floor model bank pre-seeded with ≥3 classes from S1.
