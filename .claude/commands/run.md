Launch a RoombaI door-finding run on the Raspberry Pi.

Arguments: $ARGUMENTS
Supported flags (passed straight through to run.sh):
  --greet   Enable human greeting mode (80 % "Hello human" / 20 % Yoda phrase)

## Steps

### 1. Resolve Pi SSH address

Read `.pi_host` from the project root. It contains a single line: the SSH
target for the Pi, e.g. `hackweek26@raspberrypi.local` or `pi@192.168.1.42`.

If `.pi_host` does not exist or is empty, ask the user:
> "What is the SSH address for the Raspberry Pi? (e.g. hackweek26@192.168.1.x)"
Then write the answer to `.pi_host` (one line, no trailing newline).

### 2. Push latest code to the Pi

Run:
```
ssh <pi_host> "cd /home/hackweek26/roombai && git pull --ff-only"
```

If the pull fails (merge conflict, diverged history, etc.) report the error
and stop — do not proceed with a stale codebase.

### 3. Ensure run.sh is executable on the Pi

```
ssh <pi_host> "chmod +x /home/hackweek26/roombai/run.sh"
```

### 4. Execute run.sh on the Pi

```
ssh -t <pi_host> "bash /home/hackweek26/roombai/run.sh $ARGUMENTS"
```

The `-t` flag allocates a pseudo-TTY so Ctrl-C propagates correctly.

`run.sh` performs in order:
  a. **Model installer** — runs `install_models.sh`, which installs
     `hailo-model-zoo` and `ultralytics` pip packages if missing, then
     downloads any absent HEF files (yolo_seg, midas, fast_scnn, deeplab)
     via `hailomz download`. Skips anything already present so repeated
     runs are fast. Failures are warnings, not fatal.
  b. **Vision model pre-flight** — calls `model_setup.ensure_models()`.
     For each of the five Hailo models (yolo_det, yolo_seg, midas,
     fast_scnn, deeplab) it: checks the HEF file exists and is valid,
     downloads it via the Hailo model-zoo CLI if missing, and falls back
     to ONNX → HEF compilation if download fails.
     The run aborts here if the required `yolo_det` model cannot be resolved.
  b. **Pilot daemon** — starts `roomba_pilot/target/debug/pilot serve /dev/ttyUSB0`
     if not already running. Builds the binary first if the executable is absent.
  c. **TTS daemon** — starts the espeak-ng speaker background process.
  d. **explore.py** — runs the main exploration loop, which also calls
     `ensure_models()` internally (instant because models are already resolved).

### 5. Stream the run log

After run.sh exits or is interrupted, offer to show the last 50 lines of
`/tmp/roomba_log.txt`:

```
ssh <pi_host> "tail -50 /tmp/roomba_log.txt"
```

Also offer to fetch the saved map:
```
ssh <pi_host> "ls -lh /tmp/roomba_map_*.png 2>/dev/null | tail -5"
```

### Error handling

- If ssh fails to connect: remind the user to check that the Pi is on the
  same network and that `.pi_host` contains the correct address.
- If the pilot daemon fails to start: tell the user to verify the Roomba is
  powered on, the USB cable is connected, and `/dev/ttyUSB0` exists on the Pi.
- If a required model cannot be downloaded: show the manual download
  instructions printed by `ensure_models()` (they include the exact
  `hailomz download` command and fallback HEF placement path).
