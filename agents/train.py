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


def make_env():
    return RoombaEnv()


def train(total_timesteps: int = 1_000_000, resume_from: str | None = None):
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

    if resume_from:
        print(f"Resuming from {resume_from}")
        model = PPO.load(resume_from, env=train_env)
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
        callback=[checkpoint_cb, eval_cb, logger_cb],
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
    args = parser.parse_args()

    resume_from = args.resume
    if args.resume_best:
        best_path = "models/best/best_model.zip"
        if not os.path.exists(best_path):
            parser.error(f"--resume-best: {best_path} not found")
        resume_from = best_path

    train(total_timesteps=args.timesteps, resume_from=resume_from)
