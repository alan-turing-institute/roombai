"""
Train a PPO agent to navigate the Roomba from the Enigma room to Door 17.

Usage:
    # Start the simulator first (no humans, 20x speed):
    #   cargo run --release -p simulator -- 20
    python train.py
    python train.py --timesteps 500000
    python train.py --resume checkpoints/rl_model_500000_steps.zip
"""

import argparse
import csv
from datetime import datetime
import glob
import os
from typing import Any

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    BaseCallback,
    CheckpointCallback,
    EvalCallback,
)
from stable_baselines3.common.vec_env import DummyVecEnv
import torch

from roomba_env import RoombaEnv


class EpisodeLogger(BaseCallback):
    """Logs per-episode success/failure/steps to logs/training_progress.csv."""

    def __init__(self, log_path: str = "logs/training_progress.csv"):
        super().__init__()
        os.makedirs("logs", exist_ok=True)
        self._log_path = log_path
        self._file = open(log_path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(["episode", "steps", "reward", "success"])
        self._ep = 0
        self._ep_steps = 0
        self._ep_reward = 0.0

    def _on_step(self) -> bool:
        self._ep_steps += 1
        self._ep_reward += float(self.locals["rewards"][0])
        if self.locals["dones"][0]:
            info = self.locals["infos"][0]
            success = int(self.locals["rewards"][0] > 5.0)  # reward >5 = escape bonus
            self._writer.writerow([self._ep, self._ep_steps, round(self._ep_reward, 3), success])
            self._file.flush()
            self._ep += 1
            self._ep_steps = 0
            self._ep_reward = 0.0
        return True

    def _on_training_end(self) -> None:
        self._file.close()


class SuccessRecorderAndBCCallback(BaseCallback):
    """
    Records trajectory observations and actions for successful episodes,
    saves them to demos/ directory, and periodically runs behavioral cloning
    on the policy using both manual and dynamic demonstrations.
    Also decays PPO's entropy coefficient over time.
    """

    def __init__(
        self,
        demos_dir: str = "demos",
        bc_freq: int = 50_000,
        bc_epochs: int = 10,
        bc_batch_size: int = 64,
        bc_lr: float = 5e-4,
        ent_coef_decay: float = 0.9,
        verbose: int = 0,
    ):
        super().__init__(verbose)
        self.demos_dir = demos_dir
        self.bc_freq = bc_freq
        self.bc_epochs = bc_epochs
        self.bc_batch_size = bc_batch_size
        self.bc_lr = bc_lr
        self.ent_coef_decay = ent_coef_decay

        self.current_episode_obs = []
        self.current_episode_actions = []
        self._last_obs_stored = None

    def _on_training_start(self) -> None:
        self._last_obs_stored = self.model._last_obs.copy()

    def _on_step(self) -> bool:
        # Record the observation before the action and the action chosen
        obs = self._last_obs_stored[0]
        action = self.locals["actions"][0]
        self.current_episode_obs.append(obs)
        self.current_episode_actions.append(action)

        # Update for the next step (the environment step updates self.model._last_obs)
        self._last_obs_stored = self.model._last_obs.copy()

        if self.locals["dones"][0]:
            # Evaluate success. Escape is defined as a large success reward (> 5.0) or escaped info flag.
            reward = float(self.locals["rewards"][0])
            info = self.locals["infos"][0]
            escaped = reward > 5.0 or (isinstance(info, dict) and (info.get("is_success") or info.get("escaped")))
            
            if escaped and len(self.current_episode_actions) > 0:
                ts = datetime.now().strftime("%Y%m%dT%H%M%S")
                filename = f"demo_auto_success_{ts}_step{self.num_timesteps}.npz"
                path = os.path.join(self.demos_dir, filename)
                
                obs_arr = np.stack(self.current_episode_obs).astype(np.float32)
                act_arr = np.array(self.current_episode_actions, dtype=np.int64)
                np.savez(path, obs=obs_arr, actions=act_arr)
                print(f"\n[Callback] Success captured! Saved {len(act_arr)} steps -> {path}")

            self.current_episode_obs = []
            self.current_episode_actions = []

        # Periodically trigger BC update
        if self.num_timesteps > 0 and self.num_timesteps % self.bc_freq == 0:
            self.run_bc_update()

        return True

    def run_bc_update(self) -> None:
        print(f"\n─── Starting Behavioral Cloning Update at step {self.num_timesteps} ───")
        
        # Load all demo_*.npz files from demos_dir (manual and automatic successes)
        files = sorted(glob.glob(os.path.join(self.demos_dir, "demo_*.npz")))
        if not files:
            print("No demo files found for BC update. Skipping.")
            return
            
        obs_list, act_list = [], []
        for f in files:
            try:
                d = np.load(f)
                obs_list.append(d["obs"])
                act_list.append(d["actions"])
            except Exception as e:
                print(f"Error loading {f}: {e}")
                
        if not obs_list:
            print("No valid demos loaded. Skipping.")
            return
            
        obs_np = np.concatenate(obs_list).astype(np.float32)
        acts_np = np.concatenate(act_list).astype(np.int64)
        n_total = len(obs_np)
        
        print(f"Loaded {n_total} transitions from {len(files)} demo files.")
        
        # Convert to PyTorch tensors and send to model device
        device = self.model.device
        obs_t = torch.tensor(obs_np).to(device)
        acts_t = torch.tensor(acts_np).to(device)
        
        # Create optimization parameters targeting policy parameters
        optimizer = torch.optim.Adam(self.model.policy.parameters(), lr=self.bc_lr)
        
        # Put policy in training mode
        self.model.policy.train()
        
        # Run BC epochs
        for epoch in range(1, self.bc_epochs + 1):
            perm = torch.randperm(n_total)
            losses = []
            for start in range(0, n_total, self.bc_batch_size):
                idx = perm[start : start + self.bc_batch_size]
                obs_b = obs_t[idx]
                act_b = acts_t[idx]
                
                # Cross-entropy loss on demonstration actions
                dist = self.model.policy.get_distribution(obs_b)
                loss = -dist.log_prob(act_b).mean()
                
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                   
                losses.append(loss.item())
                
            avg_loss = sum(losses) / len(losses)
            print(f"  BC Epoch {epoch}/{self.bc_epochs} - Loss: {avg_loss:.4f}")
            
        # Decay entropy coefficient to reduce exploration
        if hasattr(self.model, "ent_coef") and isinstance(self.model.ent_coef, float):
            old_ent_coef = self.model.ent_coef
            self.model.ent_coef = max(0.0, self.model.ent_coef * self.ent_coef_decay)
            print(f"Decayed PPO ent_coef from {old_ent_coef:.6f} to {self.model.ent_coef:.6f}")
            
        print("─── Behavioral Cloning Update Complete ───\n")


def make_env():
    return RoombaEnv()


def train(
    total_timesteps: int = 1_000_000,
    resume_from: str | None = None,
    bc_freq: int = 50_000,
    bc_epochs: int = 10,
    bc_lr: float = 5e-4,
):
    os.makedirs("checkpoints", exist_ok=True)
    os.makedirs("models/best", exist_ok=True)
    os.makedirs("logs", exist_ok=True)

    train_env = DummyVecEnv([make_env])
    eval_env  = DummyVecEnv([make_env])

    checkpoint_cb = CheckpointCallback(
        save_freq=50_000,
        save_path="./checkpoints/",
        name_prefix="rl_model",
    )
    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path="./models/best/",
        log_path="./logs/",
        eval_freq=25_000,
        n_eval_episodes=5,
        deterministic=True,
        render=False,
    )
    logger_cb = EpisodeLogger()
    bc_cb = SuccessRecorderAndBCCallback(
        demos_dir="demos",
        bc_freq=bc_freq,
        bc_epochs=bc_epochs,
        bc_lr=bc_lr,
    )

    if resume_from:
        print(f"Resuming from {resume_from}")
        # When resuming, override hyperparameters to rely more on pretraining / successes:
        # lower learning rate (1e-4) and a smaller entropy coefficient (0.002) to reduce exploration.
        custom_objects = {
            "learning_rate": 1e-4,
            "ent_coef": 0.002,
        }
        model = PPO.load(resume_from, env=train_env, custom_objects=custom_objects)
    else:
        model = PPO(
            "MlpPolicy",
            train_env,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.01,
            policy_kwargs={"net_arch": [128, 128]},
            tensorboard_log="./logs/",
            verbose=1,
        )

    model.learn(
        total_timesteps=total_timesteps,
        callback=[checkpoint_cb, eval_cb, logger_cb, bc_cb],
        reset_num_timesteps=resume_from is None,
    )
    model.save("models/roomba_nav_final")
    print("Training complete. Model saved to models/roomba_nav_final.zip")

    train_env.close()
    eval_env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=1_000_000)
    parser.add_argument("--resume", type=str, default=None,
                        help="Path to a checkpoint .zip to resume from")
    parser.add_argument("--resume-best", action="store_true",
                        help="Resume from models/best/best_model.zip")
    parser.add_argument("--bc-freq", type=int, default=50_000,
                        help="Timestep interval to perform BC updates (default: 50,000)")
    parser.add_argument("--bc-epochs", type=int, default=10,
                        help="Number of BC training epochs (default: 10)")
    parser.add_argument("--bc-lr", type=float, default=5e-4,
                        help="Learning rate for BC updates (default: 5e-4)")
    args = parser.parse_args()

    resume_from = args.resume
    if args.resume_best:
        best_path = "models/best/best_model.zip"
        if not os.path.exists(best_path):
            parser.error(f"--resume-best: {best_path} not found")
        resume_from = best_path

    train(
        total_timesteps=args.timesteps,
        resume_from=resume_from,
        bc_freq=args.bc_freq,
        bc_epochs=args.bc_epochs,
        bc_lr=args.bc_lr,
    )
