"""
Demo recorder for RoombaI imitation learning.

Key bindings match the RL action set in roomba_env.py exactly, so
recordings feed directly into pretrain.py without any remapping.

On exit: auto-saves if escaped; otherwise prompts to save partial demo.

Usage:
    cd agents
    uv run record_demo.py
    uv run record_demo.py --host 127.0.0.1 --port 9999 --demos-dir demos
"""

import argparse
import curses
import math
import re
import socket
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np

HOST = "127.0.0.1"
PORT = 9999
LOG_LINES = 8
REFRESH_MS = 500

MAX_LIDAR_CM = 500.0
MAX_DIST_MM  = 60_000.0

# Must match roomba_env.py ACTIONS list exactly (index = action index sent to PPO)
RL_ACTIONS = [
    (0, "move 30",  "Fwd 30 cm"),
    (1, "move -15", "Back 15 cm"),
    (2, "turn 45",  "Left 45°"),
    (3, "turn -45", "Right 45°"),
    (4, "turn 90",  "Left 90°"),
]

KEY_MAP = {
    ord("w"): 0, ord("W"): 0, curses.KEY_UP:    0,
    ord("s"): 1, ord("S"): 1, curses.KEY_DOWN:  1,
    ord("a"): 2, ord("A"): 2, curses.KEY_LEFT:  2,
    ord("d"): 3, ord("D"): 3, curses.KEY_RIGHT: 3,
    ord("q"): 4, ord("Q"): 4,
}
STOP_KEYS = {ord(" ")}
QUIT_KEYS = {27, ord("x"), ord("X")}


# ---------------------------------------------------------------------------
# TCP helpers
# ---------------------------------------------------------------------------

def connect(host: str, port: int) -> socket.socket:
    for _ in range(20):
        try:
            s = socket.create_connection((host, port), timeout=3.0)
            s.settimeout(5.0)
            return s
        except OSError:
            time.sleep(0.5)
    raise ConnectionRefusedError(
        f"Cannot connect to {host}:{port}. Is the simulator running?"
    )


def cmd(sock: socket.socket, command: str) -> str:
    sock.sendall((command + "\n").encode())
    data = b""
    while b"\n" not in data:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionResetError("Connection closed by simulator")
        data += chunk
    return data.split(b"\n")[0].decode().strip()


# ---------------------------------------------------------------------------
# Observation assembly — mirrors RoombaEnv._observe() from roomba_env.py
# ---------------------------------------------------------------------------

def fetch_obs(sock: socket.socket):
    """
    Return (obs_13, sensors_dict) where obs_13 is the same 13-float vector
    produced by RoombaEnv._observe(). Returns (None, {}) on error.
    """
    try:
        lidar_r  = cmd(sock, "lidar")
        target_r = cmd(sock, "target")
        bumps_r  = cmd(sock, "bumps")
        pos_r    = cmd(sock, "pos")
        esc_r    = cmd(sock, "escaped")
    except Exception:
        return None, {}

    # lidar: "OK f=X fl=X l=X bl=X b=X br=X r=X fr=X"
    vals  = re.findall(r"[\w]+=(\d+\.?\d*)", lidar_r)
    lidar = np.array(vals, dtype=np.float32) / MAX_LIDAR_CM
    lidar = np.clip(lidar, 0.0, 1.0)
    if len(lidar) < 8:
        lidar = np.zeros(8, dtype=np.float32)

    # target: "OK dist=X door=open|closed"
    m_dist = re.search(r"dist=(\d+\.?\d*)", target_r)
    m_door = re.search(r"door=(\w+)", target_r)
    dist   = float(m_dist.group(1)) if m_dist else MAX_DIST_MM
    door_s = m_door.group(1) if m_door else "unknown"
    door_open = 1 if door_s == "open" else 0

    # bumps: "OK bumpL=0 bumpR=0 ..."
    bvals = re.findall(r"bump[LR]=(\d)", bumps_r)
    bump  = int(any(v == "1" for v in bvals))

    # pos: "OK x=X y=X heading=X"
    mh = re.search(r"heading=(-?\d+\.?\d*)", pos_r)
    heading = float(mh.group(1)) if mh else 0.0

    # escaped: "OK 0|1"
    escaped = int(esc_r.split()[-1]) if esc_r.startswith("OK") else 0

    obs = np.empty(13, dtype=np.float32)
    obs[0:8] = lidar
    obs[8]   = min(dist / MAX_DIST_MM, 1.0)
    obs[9]   = float(door_open)
    obs[10]  = float(math.sin(heading))
    obs[11]  = float(math.cos(heading))
    obs[12]  = float(bump)

    sensors = {
        "lidar":   lidar,
        "dist":    int(dist),
        "door":    door_s.upper(),
        "escaped": escaped,
    }
    return obs, sensors


# ---------------------------------------------------------------------------
# curses UI
# ---------------------------------------------------------------------------

def _fmt_lidar(lidar):
    dirs = ["F", "FL", "L", "BL", "B", "BR", "R", "FR"]
    return "  ".join(f"{d}={lidar[i] * MAX_LIDAR_CM:5.0f}" for i, d in enumerate(dirs))


def draw(stdscr, sensors: dict, log: deque, status: str, step_count: int):
    stdscr.erase()
    h, w = stdscr.getmaxyx()

    title = " RoombaI Demo Recorder  [● REC] "
    stdscr.addstr(0, 0, title.center(w), curses.A_REVERSE)

    row = 2
    stdscr.addstr(row, 2, "Keys (RL action set):", curses.A_BOLD)
    row += 1
    bindings = [
        ("W / ↑",  "0  move 30",  "Fwd 30 cm"),
        ("S / ↓",  "1  move -15", "Back 15 cm"),
        ("A / ←",  "2  turn 45",  "Left 45°"),
        ("D / →",  "3  turn -45", "Right 45°"),
        ("Q",      "4  turn 90",  "Left 90°"),
        ("Space",  "—  stop",     "Stop (not recorded)"),
        ("ESC / X","—",           "Quit & save"),
    ]
    col_w = max(w // 3, 30)
    for i, (k, act, desc) in enumerate(bindings):
        c = (i % 3) * col_w
        r = row + i // 3
        if r < h - LOG_LINES - 4:
            stdscr.addstr(r, c + 2, f"{k:<10}{act:<16}{desc}"[:col_w - 3])
    row += (len(bindings) + 2) // 3 + 1

    sep = "─" * (w - 2)
    if row < h:
        stdscr.addstr(row, 0, "├" + sep + "┤")
    row += 1

    lidar   = sensors.get("lidar", np.zeros(8))
    dist    = sensors.get("dist")
    door    = sensors.get("door", "?")
    escaped = sensors.get("escaped", 0)

    if row < h:
        stdscr.addstr(row, 2, f"Lidar(cm): {_fmt_lidar(lidar)}"[:w - 3])
    row += 1

    dist_str = f"{dist} mm" if dist is not None else "unknown"
    esc_str  = "★ ESCAPED!" if escaped else f"steps={step_count}"
    door_attr = curses.A_BOLD if door == "OPEN" else curses.A_NORMAL
    if row < h:
        stdscr.addstr(row, 2, f"Target: {dist_str}  Door: ")
        stdscr.addstr(door, door_attr)
        stdscr.addstr(f"    {esc_str}")
    row += 1

    if row < h:
        stdscr.addstr(row, 0, "├" + sep + "┤")
    row += 1
    for entry in list(log)[-(h - row - 1):]:
        if row < h - 1:
            stdscr.addstr(row, 2, entry[:w - 3])
            row += 1

    try:
        stdscr.addstr(h - 1, 0, status.ljust(w)[:w - 1], curses.A_DIM)
    except curses.error:
        pass
    stdscr.refresh()


def run_tui(stdscr, host: str, port: int):
    curses.curs_set(0)
    stdscr.timeout(REFRESH_MS)

    status = f"Connecting to {host}:{port}…"
    draw(stdscr, {}, deque(), status, 0)

    sock = connect(host, port)
    status = f"Connected to {host}:{port}   ESC/X to quit"
    log: deque = deque(maxlen=LOG_LINES * 2)
    demo = []  # list of (obs_13_float32, action_idx)

    _, sensors = fetch_obs(sock)
    draw(stdscr, sensors, log, status, 0)

    while True:
        key = stdscr.getch()

        if key in QUIT_KEYS:
            break

        if key == curses.ERR:
            _, sensors = fetch_obs(sock)
            if sensors.get("escaped"):
                draw(stdscr, sensors, log, status, len(demo))
                time.sleep(0.4)
                break
            draw(stdscr, sensors, log, status, len(demo))
            continue

        if key in STOP_KEYS:
            try:
                resp = cmd(sock, "stop")
            except Exception as e:
                resp = f"ERR {e}"
            log.append(f"> stop        {resp}")
            _, sensors = fetch_obs(sock)
            draw(stdscr, sensors, log, status, len(demo))
            continue

        action_idx = KEY_MAP.get(key)
        if action_idx is None:
            continue

        # Record obs BEFORE sending the action
        obs_before, _ = fetch_obs(sock)

        _, cmd_str, _ = RL_ACTIONS[action_idx]
        try:
            resp = cmd(sock, cmd_str)
        except Exception as e:
            resp = f"ERR {e}"

        if obs_before is not None:
            demo.append((obs_before, action_idx))

        _, sensors = fetch_obs(sock)
        log.append(f"[{len(demo):4d}] a={action_idx} {cmd_str:<12} {resp}")
        draw(stdscr, sensors, log, status, len(demo))

        if sensors.get("escaped"):
            time.sleep(0.3)
            break

    sock.close()
    escaped = bool(sensors.get("escaped", 0)) if sensors else False
    return demo, escaped


# ---------------------------------------------------------------------------
# Save and main
# ---------------------------------------------------------------------------

def save_demo(demo: list, demos_dir: str) -> tuple:
    Path(demos_dir).mkdir(parents=True, exist_ok=True)
    ts   = datetime.now().strftime("%Y%m%dT%H%M%S")
    path = Path(demos_dir) / f"demo_{ts}.npz"
    obs_arr = np.stack([o for o, _ in demo]).astype(np.float32)
    act_arr = np.array([a for _, a in demo], dtype=np.int64)
    np.savez(path, obs=obs_arr, actions=act_arr)
    return path, len(demo)


def main():
    parser = argparse.ArgumentParser(
        description="Record manual demos for imitation-learning pretraining."
    )
    parser.add_argument("--host",      default=HOST)
    parser.add_argument("--port",      type=int, default=PORT)
    parser.add_argument("--demos-dir", default="demos",
                        help="Directory to write demo_*.npz files (default: demos/)")
    args = parser.parse_args()

    demo, escaped = curses.wrapper(run_tui, args.host, args.port)

    if not demo:
        print("No steps recorded — nothing to save.")
        return

    if escaped:
        path, n = save_demo(demo, args.demos_dir)
        print(f"Escaped! Saved {n} steps → {path}")
    else:
        answer = input(
            f"Run ended without escape ({len(demo)} steps recorded). "
            "Save partial demo? [y/N] "
        ).strip().lower()
        if answer == "y":
            path, n = save_demo(demo, args.demos_dir)
            print(f"Saved {n} steps → {path}")
        else:
            print("Demo discarded.")


if __name__ == "__main__":
    main()
