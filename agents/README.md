# RoombaI RL Agents

Reinforcement learning agents that train a virtual Roomba to navigate from its starting position in the **Enigma 2.0** room to the kitchen door (**Door 13**) in the RoombaI simulator.

---

## Overview

The agent observes local sensor data (lidar, bumpers, heading) and long-range guidance (distance to target door, door open/closed state) to learn a navigation policy via **Proximal Policy Optimisation (PPO)**. Each training episode re-randomises obstacle positions and resets doors, so the policy must generalise rather than memorise a fixed path. Doors open and close on a 5-second timer, requiring the agent to time its approach.

---

## Prerequisites

- Python 3.10+
- [`uv`](https://docs.astral.sh/uv/) for environment management
- The RoombaI simulator built and available at `../simulator/`

Create the virtual environment and install dependencies:

```bash
cd agents
uv venv .venv
uv pip install -r requirements.txt
```

All scripts below use `uv run`, which automatically picks up `.venv` in the current directory. Alternatively activate the venv manually (`source .venv/bin/activate`) and call `python` directly.

---

## File Layout

```
agents/
├── requirements.txt    # Python dependencies
├── roomba_env.py       # Custom Gymnasium environment
├── train.py            # PPO training script
├── run_agent.py        # Run a trained agent with TTS narration
├── manual_control.py   # Keyboard-driven manual control TUI
├── record_demo.py      # Record successful manual runs as imitation-learning demos
├── pretrain.py         # Behavioural cloning pretraining from recorded demos
├── demos/              # Saved demo files (demo_*.npz) — gitignored
├── checkpoints/        # Auto-saved checkpoints (created during training)
├── models/
│   ├── best/           # Best model saved by EvalCallback
│   └── pretrained.zip  # BC-pretrained model (input to --resume)
└── logs/               # TensorBoard logs + training_progress.csv
```

---

## Running the Simulator

The simulator must be running before any Python script is started.

**Training** (fast, no humans):
```bash
cd ..
cargo run --release -p simulator -- 20
```
The `20` sets the speed multiplier to 20× real-time. At this speed a 5-second door cycle happens every 250 ms wall-clock, and each training episode completes in a few seconds.

**Demo / evaluation** (real-time, no humans):
```bash
cargo run --release -p simulator -- 1
```

**With humans** (optional, for testing robustness):
```bash
cargo run --release -p simulator -- 1 --humans
```

The simulator listens on `127.0.0.1:9999`.

---

## Training

Start the simulator at 20× speed first:

```bash
cargo run --release -p simulator -- 20
```

### Option A — cold start (pure RL from scratch)

```bash
cd agents
uv run train.py
```

### Option B — warm start (recommended)

Record a few successful manual runs first, pretrain via behavioural cloning, then let PPO fine-tune. See [Imitation Learning Pretraining](#imitation-learning-pretraining) for the full workflow.

```bash
cd agents
uv run train.py --resume models/pretrained.zip
```

### Options

| Flag | Default | Description |
|------|---------|-------------|
| `--timesteps N` | 1 000 000 | Total environment steps |
| `--resume PATH` | — | Start from a checkpoint or pretrained `.zip` |
| `--resume-best` | — | Shorthand for `--resume models/best/best_model.zip` |

**Resuming from the best checkpoint:** `uv run train.py --resume-best --timesteps 500000` continues training for 500 000 *additional* steps from `models/best/best_model.zip`, regardless of how many steps it was originally trained for. The timestep counter is preserved (not reset).

### Output

Training progress is written to:
- `logs/training_progress.csv` — per-episode steps, reward, success flag
- `logs/` — TensorBoard event files (`tensorboard --logdir logs/`)

Checkpoints are saved every 50 000 steps to `checkpoints/`. The best model (by mean eval reward) is saved to `models/best/best_model.zip`.

**Expected convergence**: cold-start PPO typically reaches reliable door-finding around 300 000–500 000 steps. A BC warm start should reach the same point noticeably faster. Watch `ep_rew_mean` in TensorBoard.

---

## Running a Trained Agent

First, start the speak daemon so TTS narration works:

```bash
nohup bash -c 'tail -n 0 -f /tmp/speak_queue.txt | while IFS= read -r line; do espeak-ng -s 145 -- "$line" 2>/dev/null; done' > /tmp/speak_daemon.log 2>&1 &
```

Then run the agent:

```bash
cd agents
uv run run_agent.py models/best/best_model.zip
uv run run_agent.py models/best/best_model.zip --speed 5   # 5× faster replay
```

The agent narrates every command to the speak queue before executing it, including the current door state and distance to the exit.

---

## Manual Control

`manual_control.py` is a curses-based TUI for driving the Roomba by keyboard. Use it to explore the simulator environment manually before or between RL training runs — for example, to verify that a path to the kitchen door is navigable and that door timing feels right.

It connects to `127.0.0.1:9999`, the same TCP port used by both the simulator and the real-robot `roomba_pilot` daemon, so it works with either.

**Start the simulator first** (any speed):
```bash
cargo run --release -p simulator -- 1
```

**Launch manual control**:
```bash
cd agents
uv run manual_control.py
# or against a different host/port:
uv run manual_control.py --host 127.0.0.1 --port 9999
```

`uv run` automatically uses the `.venv` in the current directory. Alternatively, activate the venv first: `source .venv/bin/activate`, then call `python` directly.

**Key bindings**:

| Key | Action |
|-----|--------|
| W / ↑ | Forward 20 cm |
| S / ↓ | Backward 10 cm |
| A / ← | Turn left 30° |
| D / → | Turn right 30° |
| Q | Turn left 90° |
| E | Turn right 90° |
| Space | Stop |
| ESC / X | Quit |

The display shows lidar distances in all 8 directions (cm), bumper state, distance to the target door (mm), and whether the door is open or closed. It refreshes automatically every second even without keypresses.

No additional packages are needed — `curses` is Python stdlib.

---

## Imitation Learning Pretraining

Cold-start PPO exploration is slow because Door 13 is far from the Enigma start. Recording a few successful manual runs and pretraining the policy via **behavioural cloning (BC)** gives it a warm start; PPO then fine-tunes toward a generalisable policy that adapts to randomised obstacles and door timing.

The simulator does **not** need to be running during the pretraining step.

### 1 — Record demos with `record_demo.py`

`record_demo.py` is a recording TUI that drives the robot using exactly the RL action set (so there is no remapping when the demos are consumed by the trainer). Start the simulator at real-time speed first:

```bash
cargo run --release -p simulator -- 1
```

Then in `agents/`:

```bash
uv run record_demo.py
# optional flags:
uv run record_demo.py --host 127.0.0.1 --port 9999 --demos-dir demos
```

**Key bindings** (match `roomba_env.py` action indices exactly):

| Key | RL action | Command |
|-----|-----------|---------|
| W / ↑ | 0 | `move 30` — forward 30 cm |
| S / ↓ | 1 | `move -15` — backward 15 cm |
| A / ← | 2 | `turn 45` — left 45° |
| D / → | 3 | `turn -45` — right 45° |
| Q | 4 | `turn 90` — left 90° |
| Space | — | `stop` (not recorded) |
| ESC / X | — | Quit and save |

On a successful escape the demo is saved automatically to `demos/demo_<timestamp>.npz`. If you quit without escaping you are prompted whether to save the partial run (partial demos are still useful for learning early navigation).

Each `.npz` file contains two arrays:
- `obs` — shape `(N, 13)` float32 — the observation before each action
- `actions` — shape `(N,)` int64 — the corresponding action index

### 2 — Pretrain with `pretrain.py`

```bash
uv run pretrain.py
# optional flags:
uv run pretrain.py --demos-dir demos/ --epochs 50 --batch-size 64 --lr 1e-3 --out models/pretrained
```

Loads every `demo_*.npz` in `--demos-dir`, trains the PPO policy network with cross-entropy loss for the specified number of epochs, and saves `models/pretrained.zip`.

Expect loss to decrease steadily over epochs. If it plateaus early, record more diverse demos or increase `--epochs`.

### 3 — Resume RL from the pretrained policy

```bash
uv run train.py --resume models/pretrained.zip --timesteps 1000000
```

The `--resume` flag loads the BC-pretrained weights into the PPO model before RL begins. The policy already knows a reasonable path to Door 13; PPO then improves generalisation across varied obstacle layouts and door timing.

---

## Observation and Action Reference

### Observation space — `Box(13,)` float32

| Index | Source | Range | Description |
|-------|--------|-------|-------------|
| 0–7 | `lidar` (F, FL, L, BL, B, BR, R, FR) | [0, 1] | Lidar in cm ÷ 500 |
| 8 | `target dist` | [0, 1] | Distance to kitchen door (Door 13) in mm ÷ 60 000 |
| 9 | `target door` | {0, 1} | 1 = door open |
| 10 | heading | [−1, 1] | sin(heading) |
| 11 | heading | [−1, 1] | cos(heading) |
| 12 | bumpers | {0, 1} | 1 = either bumper active |

No absolute x, y position is given; the agent must navigate by relative sensors alone, which helps it generalise to varied obstacle layouts.

### Action space — `Discrete(5)`

| Index | Command | Description |
|-------|---------|-------------|
| 0 | `move 30` | Forward 30 cm |
| 1 | `move -15` | Backward 15 cm |
| 2 | `turn 45` | Rotate left 45° |
| 3 | `turn -45` | Rotate right 45° |
| 4 | `turn 90` | Rotate left 90° |

### Reward function

| Event | Reward |
|-------|--------|
| Distance improvement | +2 × Δdist / 10 000 mm per step |
| Time penalty | −0.002 per step |
| Bump | −0.05 |
| At closed door (dist < 1500 mm) | −0.01 |
| Escape (crossed Door 13 by ≥ 1 m while open) | +10.0 |
| Timeout (1000 steps) | 0 (episode ends) |

---

## Hyperparameter Notes

The PPO configuration in `train.py` is a reasonable starting point:

- `ent_coef=0.01` — maintains exploration; set to 0 only if the policy has clearly converged
- `n_steps=2048` — increase to 4096 if training is noisy on long episodes
- `net_arch=[128, 128]` — two hidden layers of 128 units; increase if the task is not converging
- `learning_rate=3e-4` — standard; reduce to 1e-4 if training is unstable

To resume training from a checkpoint:
```bash
python train.py --resume checkpoints/rl_model_500000_steps.zip --timesteps 500000
```

---

## Extending the System

- **Add position to observations**: include normalised (x, y) for curriculum learning — start with position visible, then gradually remove it.
- **Continuous actions**: switch to SAC with a `Box` action space (forward velocity, turn rate) for smoother motion.
- **Multi-room curriculum**: train first with only Enigma obstacles, then progressively expose the full floor plan.
- **Human avoidance**: run the simulator with `--humans` and extend the observation with the nearest human bearing and distance.


## PPO - what the agent learns

PPO trains a **two-layer MLP** (128 × 128 hidden units) that maps the 13-float observation to a probability distribution over the 5 discrete actions. At each step it samples an action, receives a reward, and updates the network weights to increase the likelihood of high-reward trajectories. Over roughly 300 000–500 000 steps the policy converges from random exploration to a reliable Door-13-seeking strategy.

### What it optimises for

| Signal | Reward |
|--------|--------|
| Getting closer to Door 13 | +2 × Δdist / 10 000 mm per step |
| Escaping through Door 13 | +10 (sparse goal) |
| Time penalty | −0.002 per step (incentivises speed) |
| Bumper hit | −0.05 |
| Waiting at a closed door (dist < 1 500 mm) | −0.01 |

The policy receives no absolute position — it must learn to navigate using only relative sensor signals. In practice this forces it to develop compass-like heading use (obs[10–11]) and distance-seeking behaviour (obs[8]).

### Sim-to-real translation

| Observation | Simulator source | Real Roomba equivalent | Status |
|---|---|---|---|
| Lidar [0–7] — 8 rays (cm) | Raycast engine | None — no lidar on the robot | **Gap (zeroed)** |
| Target distance [8] | Game state | Dead reckoning from known start + encoder odometry | **Solved** |
| Door open [9] | Game state | Hardcoded to open (1.0) — door assumed passable | **Approximated** |
| sin/cos heading [10–11] | Simulator pose | Dead reckoning (accumulated turn commands) | **Solved** |
| Bumpers [12] | `bumps` command | `bumps` command (identical) | **Direct** |

`run_agent_real.py` implements the solved/approximated rows: it tracks (x, y, heading) by accumulating every `move`/`turn` command from a known start position (Enigma room spawn, facing right), then computes distance to Door 13 analytically. Lidar rays are zeroed.

**Remaining gap — lidar [0–7]:** The policy trained with 8 continuous-distance rays but receives all zeros on the real robot. In practice the bumper signal (obs[12]) partially compensates — the policy has learned to react to bumper hits — but it will not anticipate walls the way it does in simulation. The iRobot Create 2 has 7 light-bump IR sensors (`LIGHTBUMP_*`, OI packet IDs 45–51) that return binary proximity rather than continuous distance. Substituting these (obs[0–6] = LIGHTBUMP booleans) and fine-tuning for a small number of real episodes would close this gap.

