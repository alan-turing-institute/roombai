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

### 2. Commit and push any local changes on this machine

Before touching the Pi, make sure GitHub has the latest code to pull from.

Check for uncommitted changes:
```
git status --short
git diff --stat
```

If there are staged or unstaged changes, commit them:
```
git add -A
git commit -m "Pre-run commit before deploying to Pi"
```

Then push to GitHub regardless (ensures Pi can fast-forward):
```
git push
```

If the push is rejected (remote has commits not yet pulled locally), pull
and rebase first:
```
git pull --rebase && git push
```

### 3. Sync the Pi to the current branch

Find out which branch is currently checked out on this machine:
```
git rev-parse --abbrev-ref HEAD
```

SSH to the Pi and, **before switching branches**, commit any uncommitted
changes there so nothing is lost:
```
sshpass -p aipi ssh -o StrictHostKeyChecking=no <pi_host> \
  "source ~/yolo_new/bin/activate \
   && cd /home/hackweek26/roombai \
   && git diff --quiet && git diff --cached --quiet \
   || git add -A && git commit -m 'Pi pre-switch commit $(date +%Y-%m-%d_%H:%M)'"
```

Then switch to the correct branch and pull:
```
sshpass -p aipi ssh -o StrictHostKeyChecking=no <pi_host> \
  "source ~/yolo_new/bin/activate \
   && cd /home/hackweek26/roombai \
   && git fetch origin \
   && git checkout <local_branch> \
   && git pull --ff-only origin <local_branch>"
```

If the pull fails for any reason (diverged history, merge conflict, etc.)
report the full error and stop — do not proceed with a mismatched codebase.

### 4. Clear /tmp/ on the Pi

Stop any leftover processes from previous runs, then remove roomba-owned
/tmp/ files (system files are left untouched):

```
sshpass -p aipi ssh -o StrictHostKeyChecking=no <pi_host> \
  "pkill -f explore.py 2>/dev/null; pkill -f speak_queue 2>/dev/null; sleep 1; \
   rm -rf /tmp/roomba_* /tmp/roomba_frames /tmp/roomba_maps \
          /tmp/pilot.log /tmp/speak_* /tmp/sync_stop"
```

### 5. Ensure scripts are executable on the Pi

```
sshpass -p aipi ssh -o StrictHostKeyChecking=no <pi_host> \
  "chmod +x /home/hackweek26/roombai/run.sh /home/hackweek26/roombai/install_models.sh"
```

### 6. Create the local run directory and start the live sync

Determine the run timestamp and create the directory:
```
TIMESTAMP=$(date +%Y-%m-%d_%H-%M)
RUN_DIR="runs/$TIMESTAMP"
mkdir -p "$RUN_DIR/frames"
```

Start `sync_run.sh` in the background — it will download Pi data every 60 s,
re-downloading logs and only fetching new frames (skipping already-downloaded ones).
The password for the Pi is `aipi`.
```
bash sync_run.sh "$RUN_DIR" <pi_host> aipi &
SYNC_PID=$!
echo "[run] live sync started (PID $SYNC_PID) → $RUN_DIR"
```

### 7. Execute run.sh on the Pi (background so sync keeps running)

Run the SSH session **in the background** so the sync loop continues in parallel.
Always activate the venv first:
```
sshpass -p aipi ssh -o StrictHostKeyChecking=no -tt <pi_host> \
    "source ~/yolo_new/bin/activate && bash /home/hackweek26/roombai/run.sh $ARGUMENTS" &
SSH_PID=$!
echo "[run] SSH session started (PID $SSH_PID)"
```

Then tail the local log so progress is visible:
```
tail -f "$RUN_DIR/roomba_log.txt" &
TAIL_PID=$!
```

Wait for the SSH session to finish (you will be notified when the background
job completes). Do NOT poll — just wait for the notification.

`run.sh` performs in order:
  a. **Model installer** — runs `install_models.sh`: installs `python3-numba`
     via apt, installs `hailo-model-zoo` from `~/hailo-model-zoo`, installs
     `ultralytics`, then downloads any absent HEFs (yolo_seg, midas, fast_scnn,
     deeplab). Skips anything already present. Failures are warnings, not fatal.
  b. **Vision model pre-flight** — validates all HEFs via `ensure_models()`.
     Aborts if the required `yolo_det` model is missing.
  c. **Pilot daemon** — starts `pilot serve /dev/ttyUSB0` if not running.
  d. **TTS daemon** — starts the espeak-ng speaker background process.
  e. **explore.py** — runs the main exploration loop.

### 8. After the run ends: stop sync and finalise

When the SSH session background job completes:

Stop the log tail:
```
kill $TAIL_PID 2>/dev/null || true
```

Signal `sync_run.sh` to stop (it will do one final sync pass before exiting):
```
touch "$RUN_DIR/.sync_stop"
```

Wait a few seconds for the final sync to complete, then print a summary:
```
sleep 10
echo "Run data saved to $RUN_DIR"
ls -lh "$RUN_DIR/"
echo "Frames downloaded: $(ls $RUN_DIR/frames/ | wc -l)"
echo "Log lines: $(wc -l < $RUN_DIR/roomba_log.txt)"
```

The `runs/` folder is in .gitignore so none of this is committed to git.

### Error handling

- If ssh fails to connect: remind the user to check that the Pi is on the
  same network and that `.pi_host` contains the correct address.
- If the pilot daemon fails to start: tell the user to verify the Roomba is
  powered on, the USB cable is connected, and `/dev/ttyUSB0` exists on the Pi.
- If a required model cannot be downloaded: show the manual download
  instructions printed by `ensure_models()` (they include the exact
  `hailomz download` command and fallback HEF placement path).
