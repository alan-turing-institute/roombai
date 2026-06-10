# S1 Runbook — 5-minute characterization session

One scripted run of the `characterize` binary. The robot narrates each phase
over the speaker; it needs **no keyboard input and no props** (no checkerboard,
ruler, or tape). Total robot time ≈ 3.5 min with buffer.

## The idea

We don't measure anything in the real world. The robot's own motion is the
calibration signal: it drives known-in-encoder-units moves while recording
video, and offline we recover (a) the pixel→ground relationship and (b) whether
wheel odometry is trustworthy, purely from how the floor and scene shift in the
video relative to the encoder trace. Everything stays in self-consistent
encoder/ground units; absolute metres are never needed (a rough metric scale
comes free from the wheel spec, 72 mm / 508.8 ticks-per-rev).

## What the session buys us

| Phase | Robot does | We learn |
|---|---|---|
| A (~25 s) | stationary sensor probe | which OI packets the 770 answers (encoders 43/44? light bumps 46–51? stream 148?), battery state |
| sync (~6 s) | two forward/back jerks | a sharp double-lurch to align the video clock to the encoder log offline (cross-correlation) |
| B (~55 s) | slow 0.9 m fwd, fast leg, creep to wall contact, reverse home | pixel→ground flow calibration (slow = low blur), odometry at speed, light-bump signal-vs-distance curve (anchored at bumper contact), bumper works, reverse symmetry |
| C (~70 s) | spins: CCW 45°/s, CW 45°/s, CCW 150°/s | encoder rotation vs visually-observed rotation (the rotation-drift question), VO test footage, directional asymmetry |
| D (~50 s) | 1.2 m out over dark tiles, 180° arc, 1.2 m back | cliff-signal reflectivity over carpet/dark-tile/hatch, floor appearance video, arc-drive check |

Video (720p H.264) records continuously from the sync pulse through phase D.
Every sensor sample is logged at 20 Hz to `session.jsonl` with timestamps
relative to runner start; the video-start offset is logged as an event and
refined offline via the sync pulse.

## Pre-session checklist

Space (no taping, no marks):
- [ ] Put the robot on the floor with a **wall or large flat surface ~1.5–2 m
      directly ahead** (for the approach + light-bump curve + the arc's
      clearance).
- [ ] Make sure **~1 m of width is clear to the robot's left** for the phase D
      arc, and that the area ahead crosses some **dark-blue tiles / grey hatch
      panels** if possible (floor-variation data).
- [ ] Roughly note the robot's start spot and heading so you could repeat the
      run (a phone photo of the scene is enough).
- [ ] *Optional:* prop a phone in a corner filming the whole area — an
      independent cross-check on the spin angles. Not required.

Software (one command does it all):
- [ ] Pilot daemon NOT running (it owns the serial port; `pilot shutdown` if so).
- [ ] Speaker volume up (the launcher starts the TTS daemon for you).
- [ ] Run:  `cd escape && ./run_s1.sh`
      (defaults to `--port /dev/ttyUSB0 --out /tmp/s1`; it builds release, starts
      the TTS daemon if needed, prints preflight checks, then waits for Enter.)
      Resume after an abort, e.g.:  `./run_s1.sh /dev/ttyUSB0 /tmp/s1 --from C`

## During the run (the human's script)

1. Power on the Roomba on its start spot, facing the wall. Run `./run_s1.sh`
   and press Enter at the preflight prompt. **Stand clear of the lane and the
   area to the left.**
2. That's it — every phase is automatic and narrated. Just keep people, feet,
   and bags out of the driving area.
3. **Ctrl-C at any time stops the wheels within one tick (50 ms).** If the
   runner died mid-session, restart with `--from C` (etc.) to skip completed
   phases — phase A re-runs automatically (it's cheap and stationary).

## Immediately after

- [ ] Jot a couple of lines in `/tmp/s1/notes_template.md` (which floor regions
      were crossed; anything unexpected). No measurements to record.
- [ ] Copy everything off the Pi: `scp -r pi:/tmp/s1 .` (session.jsonl,
      video.h264, summary.json, notes) + the phone video if you took one.
- [ ] Sanity-glance `summary.json`: the `caps` block is the 770 sensor truth
      table that unblocks the driver crate design.

## Failure modes & responses

| Symptom | Response |
|---|---|
| "Failed to enter safe mode" at start | Roomba asleep/charging — power-cycle it, restart runner |
| Encoders/light bumps reported missing | Fine — that's the answer; session continues on fallbacks (packets 19/20, bumper-only) |
| Camera warning at video start | Session continues; sensor data still collected, but the vision calibration data is lost — fix the camera and rerun |
| Robot stalls against wall without bump | Creep leg times out after 20 s and continues — note it |
| Anything alarming | Ctrl-C, then restart with `--from <next phase>` |
