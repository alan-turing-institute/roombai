# PaiCar — Autonomous Drive

Obstacle-avoiding drive loop for the SunFounder PiCar-V, using OpenCV texture analysis to estimate clearance and steer around obstacles.

## Running on the PiCar

### Prerequisites

- Battery pack switched on (the Pi runs on USB power but the motors need the battery)
- `picar` library installed: `cd ~/SunFounder_PiCar-V && sudo python3 setup.py install`
- Python dependencies installed: `pip3 install -r requirements.txt`

### Step 1 — Start the camera stream

Run this in its own terminal (or background it with `&`). It must be running before `drive.py`.

```bash
LD_LIBRARY_PATH=/usr/local/lib/ sudo mjpg_streamer \
  -i 'input_uvc.so -d /dev/video0 -r 640x480 -f 15' \
  -o 'output_http.so -p 8080 -w /usr/local/share/mjpg-streamer/www'
```

Verify it is working: `curl -s http://localhost:8080/?action=snapshot > /dev/null && echo OK`

### Step 2 — Pull the latest changes

```bash
ssh hw@100.88.2.9
cd ~/path/to/roombai/PaiCar
git pull
```

### Step 3 — Run the drive loop

```bash
python3 drive.py
```

Optional flags:

| Flag | Default | Description |
|---|---|---|
| `--max-distance` | `5.0` | Maximum travel distance per move (metres) |
| `--max-steps` | `100` | Number of moves before stopping |
| `--log-dir` | `logs/` | Directory for decision logs and snapshots |

Example:

```bash
python3 drive.py --max-distance 3.0 --max-steps 20 --log-dir ~/logs
```

### What happens on startup

`drive.py` imports `picar_control`, which immediately calls `picar.setup()` and initialises the I²C bus. The wheels will briefly move to their ready position. The loop then runs: snapshot → plan → log → move, repeating until `--max-steps` is reached or you interrupt with `Ctrl-C`.

### Decision logs

Each move is logged to `--log-dir` (default `logs/`):

- `logs/<timestamp>.jpg` — copy of the snapshot the decision was based on
- `logs/decisions.jsonl` — one JSON line per move with direction, distance, clearances, and reasoning

### Troubleshooting

**Car does not move**: check the battery pack is on and that mjpg-streamer is responding (`curl http://localhost:8080/?action=snapshot`).

**I²C permission error**: add your user to the `i2c` group: `sudo usermod -aG i2c hw`, then log out and back in.

**mjpg-streamer plugin not found**: confirm the `.so` files are in `/usr/local/lib/mjpg-streamer/` and that `LD_LIBRARY_PATH` is set as shown above.
