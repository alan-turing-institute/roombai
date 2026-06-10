"""
Behavioural cloning (BC) pretraining from recorded demos.

Loads all demo_*.npz files, trains the SB3 PPO policy network with
cross-entropy loss, and saves a SB3-compatible .zip that train.py
can resume from.

The simulator does NOT need to be running for pretraining.

Usage:
    cd agents
    uv run pretrain.py
    uv run pretrain.py --demos-dir demos/ --epochs 50 --out models/pretrained
"""

import argparse
import glob
import os
from pathlib import Path

import numpy as np
import torch
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO


class _DummyEnv(gym.Env):
    """Minimal env used only to satisfy PPO's constructor; never stepped."""
    def __init__(self):
        obs_low  = np.zeros(13, dtype=np.float32)
        obs_high = np.ones(13,  dtype=np.float32)
        obs_low[10:12] = -1.0   # sin/cos heading range
        self.observation_space = spaces.Box(low=obs_low, high=obs_high, dtype=np.float32)
        self.action_space      = spaces.Discrete(5)

    def reset(self, **kwargs):
        return np.zeros(13, dtype=np.float32), {}

    def step(self, action):
        return np.zeros(13, dtype=np.float32), 0.0, False, False, {}


def load_demos(demos_dir: str) -> tuple:
    files = sorted(glob.glob(os.path.join(demos_dir, "demo_*.npz")))
    if not files:
        raise FileNotFoundError(
            f"No demo_*.npz files found in {demos_dir!r}.\n"
            "Record some demos first:  uv run record_demo.py"
        )
    obs_list, act_list = [], []
    for f in files:
        d = np.load(f)
        obs_list.append(d["obs"])
        act_list.append(d["actions"])
        print(f"  {f}: {len(d['actions'])} steps")
    obs  = np.concatenate(obs_list).astype(np.float32)
    acts = np.concatenate(act_list).astype(np.int64)
    return obs, acts


def pretrain(demos_dir: str, n_epochs: int, batch_size: int, lr: float, out: str):
    print("── Loading demos ──────────────────────────────")
    obs_np, acts_np = load_demos(demos_dir)
    n_total = len(obs_np)
    print(f"Total: {n_total} steps across {demos_dir}")

    obs_t  = torch.tensor(obs_np)
    acts_t = torch.tensor(acts_np)

    print("\n── Building model ─────────────────────────────")
    model = PPO(
        "MlpPolicy",
        _DummyEnv(),
        policy_kwargs={"net_arch": [128, 128]},
        verbose=0,
    )

    print("\n── Behavioural cloning ────────────────────────")
    print(f"epochs={n_epochs}  batch={batch_size}  lr={lr}  samples={n_total}")
    optimizer = torch.optim.Adam(model.policy.parameters(), lr=lr)

    for epoch in range(1, n_epochs + 1):
        perm   = torch.randperm(n_total)
        losses = []
        for start in range(0, n_total, batch_size):
            idx   = perm[start:start + batch_size]
            obs_b = obs_t[idx]
            act_b = acts_t[idx]
            dist  = model.policy.get_distribution(obs_b)
            loss  = -dist.log_prob(act_b).mean()
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            losses.append(loss.item())
        avg = sum(losses) / len(losses)
        print(f"  epoch {epoch:3d}/{n_epochs}  loss={avg:.4f}")

    # Strip .zip so SB3 doesn't double-append the extension
    save_path = out[:-4] if out.endswith(".zip") else out
    Path(save_path).parent.mkdir(parents=True, exist_ok=True)
    model.save(save_path)
    zip_path = save_path + ".zip"
    print(f"\nSaved → {zip_path}")
    print(f"Resume RL:  uv run train.py --resume {zip_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Behavioural cloning pretraining from recorded demos."
    )
    parser.add_argument("--demos-dir", default="demos",
                        help="Directory containing demo_*.npz files (default: demos/)")
    parser.add_argument("--epochs",     type=int,   default=50,
                        help="Number of BC training epochs (default: 50)")
    parser.add_argument("--batch-size", type=int,   default=64,
                        help="Mini-batch size (default: 64)")
    parser.add_argument("--lr",         type=float, default=1e-3,
                        help="Learning rate (default: 1e-3)")
    parser.add_argument("--out",        default="models/pretrained",
                        help="Output path without .zip extension (default: models/pretrained)")
    args = parser.parse_args()

    pretrain(args.demos_dir, args.epochs, args.batch_size, args.lr, args.out)


if __name__ == "__main__":
    main()
