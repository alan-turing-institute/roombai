"""
Gymnasium environment wrapping the RoombaI simulator TCP interface.

The simulator must be running before instantiating this env:
    cargo run --release -p simulator -- 20
"""

import re
import socket
import time

import gymnasium as gym
import numpy as np
from gymnasium import spaces

# Simulator TCP address
HOST = "127.0.0.1"
PORT = 9999

# Episode parameters
MAX_STEPS = 1000
TRAINING_SPEED = 20

# Coordinate constants (mm)
START_X = 1370.0 * 36.0   # 49320 mm
START_Y = 825.0 * 36.0    # 29700 mm
MAX_DIST = 60_000.0        # normalisation denominator for target distance
MAX_LIDAR_CM = 500.0       # normalisation denominator for lidar (500 cm = 5 m)

# Action definitions: (command_template, narration)
ACTIONS = [
    ("move 30",  "Moving forward 30 centimetres"),
    ("move -15", "Moving backward 15 centimetres"),
    ("turn 45",  "Turning left 45 degrees"),
    ("turn -45", "Turning right 45 degrees"),
    ("turn 90",  "Turning left 90 degrees"),
]


class RoombaEnv(gym.Env):
    """
    Observation (13 floats):
      [0-7]  lidar F,FL,L,BL,B,BR,R,FR  normalised to [0,1] (max 500 cm)
      [8]    distance to target door     normalised to [0,1] (max 60 000 mm)
      [9]    target door open            0 or 1
      [10]   sin(heading)               [-1, 1]
      [11]   cos(heading)               [-1, 1]
      [12]   any bumper active          0 or 1

    Action (Discrete 5):
      0  move 30   — forward 30 cm
      1  move -15  — backward 15 cm
      2  turn 45   — left 45°
      3  turn -45  — right 45°
      4  turn 90   — left 90°
    """

    metadata = {"render_modes": []}

    def __init__(self, host: str = HOST, port: int = PORT):
        super().__init__()
        self._host = host
        self._port = port
        self._sock: socket.socket | None = None
        self._step_count = 0
        self._prev_dist = MAX_DIST

        obs_low = np.zeros(13, dtype=np.float32)
        obs_high = np.ones(13, dtype=np.float32)
        obs_low[10:12] = -1.0   # sin/cos heading
        self.observation_space = spaces.Box(low=obs_low, high=obs_high, dtype=np.float32)
        self.action_space = spaces.Discrete(len(ACTIONS))

    # ------------------------------------------------------------------
    # TCP helpers
    # ------------------------------------------------------------------

    def _connect(self) -> None:
        for attempt in range(10):
            try:
                self._sock = socket.create_connection((self._host, self._port), timeout=5.0)
                self._sock.settimeout(10.0)
                return
            except OSError:
                time.sleep(0.5 * (attempt + 1))
        raise ConnectionRefusedError(
            f"Could not connect to simulator at {self._host}:{self._port}. "
            "Is it running? (cargo run --release -p simulator)"
        )

    def _cmd(self, command: str) -> str:
        if self._sock is None:
            self._connect()
        self._sock.sendall((command + "\n").encode())
        data = b""
        while b"\n" not in data:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise ConnectionResetError("Simulator closed connection")
            data += chunk
        return data.split(b"\n")[0].decode().strip()

    # ------------------------------------------------------------------
    # Observation parsing
    # ------------------------------------------------------------------

    def _parse_lidar(self, resp: str) -> np.ndarray:
        # "OK f=X fl=X l=X bl=X b=X br=X r=X fr=X"  (values in cm)
        values = re.findall(r"[\w]+=(\d+\.?\d*)", resp)
        arr = np.array(values, dtype=np.float32) / MAX_LIDAR_CM
        return np.clip(arr, 0.0, 1.0)

    def _parse_target(self, resp: str) -> tuple[float, int]:
        # "OK dist=NNNN door=open|closed"
        m_dist = re.search(r"dist=(\d+\.?\d*)", resp)
        m_door = re.search(r"door=(\w+)", resp)
        dist = float(m_dist.group(1)) if m_dist else MAX_DIST
        door_open = 1 if (m_door and m_door.group(1) == "open") else 0
        return dist, door_open

    def _parse_bumps(self, resp: str) -> int:
        # "OK bumpL=0 bumpR=0 ..."
        vals = re.findall(r"bump[LR]=(\d)", resp)
        return int(any(v == "1" for v in vals))

    def _parse_pos(self, resp: str) -> tuple[float, float, float]:
        # "OK x=NNNN y=NNNN heading=N.NNNN"
        mx = re.search(r"x=(-?\d+\.?\d*)", resp)
        my = re.search(r"y=(-?\d+\.?\d*)", resp)
        mh = re.search(r"heading=(-?\d+\.?\d*)", resp)
        x = float(mx.group(1)) if mx else START_X
        y = float(my.group(1)) if my else START_Y
        h = float(mh.group(1)) if mh else -np.pi / 2
        return x, y, h

    def _observe(self) -> tuple[np.ndarray, float, int, int]:
        lidar_resp  = self._cmd("lidar")
        target_resp = self._cmd("target")
        bumps_resp  = self._cmd("bumps")
        pos_resp    = self._cmd("pos")
        esc_resp    = self._cmd("escaped")

        lidar = self._parse_lidar(lidar_resp)
        dist, door_open = self._parse_target(target_resp)
        bump = self._parse_bumps(bumps_resp)
        _, _, heading = self._parse_pos(pos_resp)
        escaped = int(esc_resp.split()[-1]) if esc_resp.startswith("OK") else 0

        obs = np.empty(13, dtype=np.float32)
        obs[0:8] = lidar
        obs[8]   = min(dist / MAX_DIST, 1.0)
        obs[9]   = float(door_open)
        obs[10]  = float(np.sin(heading))
        obs[11]  = float(np.cos(heading))
        obs[12]  = float(bump)

        return obs, dist, door_open, escaped

    # ------------------------------------------------------------------
    # Gym API
    # ------------------------------------------------------------------

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if self._sock is None:
            self._connect()
        self._cmd("reset")
        time.sleep(0.15)  # wait for game loop to process the reset
        self._cmd(f"speed {TRAINING_SPEED}")
        self._step_count = 0
        obs, dist, _, _ = self._observe()
        self._prev_dist = dist
        return obs, {}

    def step(self, action: int):
        cmd, _ = ACTIONS[action]
        self._cmd(cmd)
        # Brief sleep so the command has time to execute before we sample sensors
        time.sleep(0.05)

        obs, dist, door_open, escaped = self._observe()
        self._step_count += 1

        # Reward
        reward = 0.0
        dist_improvement = (self._prev_dist - dist) / 10_000.0
        reward += dist_improvement * 2.0
        reward -= 0.002  # time penalty
        if obs[12] > 0.5:
            reward -= 0.05  # bump penalty
        if dist < 1500.0 and door_open == 0:
            reward -= 0.01  # penalise waiting at a closed door
        if escaped:
            reward += 10.0

        self._prev_dist = dist
        terminated = bool(escaped)
        truncated = self._step_count >= MAX_STEPS

        return obs, reward, terminated, truncated, {}

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
