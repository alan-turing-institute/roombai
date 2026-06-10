"""
Keyboard-driven manual control for the RoombaI simulator (or real robot).

Connects to the TCP pilot daemon on 127.0.0.1:9999 — the same port used by
the simulator and the real-robot roomba_pilot daemon.

Usage:
    # Start the simulator first:
    #   cargo run --release -p simulator -- 1
    cd agents
    .venv/bin/python manual_control.py
    .venv/bin/python manual_control.py --host 127.0.0.1 --port 9999
"""

import argparse
import curses
import socket
import time
from collections import deque

HOST = "127.0.0.1"
PORT = 9999
LOG_LINES = 8         # number of command log lines to keep
REFRESH_MS = 1000     # sensor auto-refresh interval in ms

BINDINGS = [
    ("W / ↑",  "move 20",   "Forward 20 cm"),
    ("S / ↓",  "move -10",  "Backward 10 cm"),
    ("A / ←",  "turn 30",   "Left 30°"),
    ("D / →",  "turn -30",  "Right 30°"),
    ("Q",      "turn 90",   "Left 90°"),
    ("E",      "turn -90",  "Right 90°"),
    ("Space",  "stop",      "Stop"),
    ("ESC/X",  None,        "Quit"),
]

KEY_MAP = {
    ord("w"): "move 20",   ord("W"): "move 20",
    ord("s"): "move -10",  ord("S"): "move -10",
    ord("a"): "turn 30",   ord("A"): "turn 30",
    ord("d"): "turn -30",  ord("D"): "turn -30",
    ord("q"): "turn 90",   ord("Q"): "turn 90",
    ord("e"): "turn -90",  ord("E"): "turn -90",
    ord(" "): "stop",
    curses.KEY_UP:    "move 20",
    curses.KEY_DOWN:  "move -10",
    curses.KEY_LEFT:  "turn 30",
    curses.KEY_RIGHT: "turn -30",
}

QUIT_KEYS = {27, ord("x"), ord("X")}  # ESC or X


# ---------------------------------------------------------------------------
# TCP helpers
# ---------------------------------------------------------------------------

def connect(host: str, port: int) -> socket.socket:
    for attempt in range(20):
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
# Sensor helpers
# ---------------------------------------------------------------------------

def fetch_sensors(sock: socket.socket) -> dict:
    try:
        lidar_resp  = cmd(sock, "lidar")
        target_resp = cmd(sock, "target")
        bumps_resp  = cmd(sock, "bumps")
    except Exception:
        return {}

    # lidar: "OK f=X fl=X l=X bl=X b=X br=X r=X fr=X"  (cm)
    lidar = {}
    for part in lidar_resp.split():
        if "=" in part:
            k, v = part.split("=", 1)
            try:
                lidar[k] = float(v)
            except ValueError:
                pass

    # target: "OK dist=X door=open|closed"
    dist, door = None, None
    for part in target_resp.split():
        if part.startswith("dist="):
            try:
                dist = int(float(part[5:]))
            except ValueError:
                pass
        elif part.startswith("door="):
            door = part[5:].upper()

    # bumps: "OK bumpL=0 bumpR=0 ..."
    bl, br = 0, 0
    for part in bumps_resp.split():
        if part.startswith("bumpL="):
            bl = int(part[6:])
        elif part.startswith("bumpR="):
            br = int(part[6:])

    return {"lidar": lidar, "dist": dist, "door": door, "bumpL": bl, "bumpR": br}


def fmt_cm(v):
    if v is None:
        return " -- "
    return f"{v:5.1f}" if v < 999.9 else "999+"


# ---------------------------------------------------------------------------
# curses UI
# ---------------------------------------------------------------------------

def draw(stdscr, sensors: dict, log: deque, status: str):
    stdscr.erase()
    h, w = stdscr.getmaxyx()

    # --- title bar ---
    title = " RoombaI Manual Control "
    stdscr.addstr(0, 0, title.center(w), curses.A_REVERSE)

    # --- key bindings ---
    row = 2
    stdscr.addstr(row, 2, "Keys:", curses.A_BOLD)
    row += 1
    col_w = max(w // 3, 28)
    for i, (key, _, desc) in enumerate(BINDINGS):
        c = (i % 3) * col_w
        r = row + i // 3
        if r < h - LOG_LINES - 4:
            stdscr.addstr(r, c + 2, f"{key:<10} {desc}"[:col_w - 3])
    row += (len(BINDINGS) + 2) // 3 + 1

    # --- sensor panel ---
    sep = "─" * (w - 2)
    if row < h:
        stdscr.addstr(row, 0, "├" + sep + "┤")
    row += 1

    lidar = sensors.get("lidar", {})
    dirs = ["f", "fl", "l", "bl", "b", "br", "r", "fr"]
    labels = ["F", "FL", "L", "BL", "B", "BR", "R", "FR"]
    lidar_str = "  ".join(f"{l}={fmt_cm(lidar.get(d))}" for d, l in zip(dirs, labels))
    if row < h:
        stdscr.addstr(row, 2, f"Lidar(cm): {lidar_str}"[: w - 3])
    row += 1

    dist = sensors.get("dist")
    door = sensors.get("door", "?")
    bl = sensors.get("bumpL", 0)
    br = sensors.get("bumpR", 0)
    bump_str = f"BumpL={'■' if bl else '□'}  BumpR={'■' if br else '□'}"
    dist_str = f"{dist} mm" if dist is not None else "unknown"
    door_attr = curses.A_BOLD if door == "OPEN" else curses.A_NORMAL
    if row < h:
        stdscr.addstr(row, 2, f"{bump_str}    Target: {dist_str}  Door: ")
        stdscr.addstr(door or "?", door_attr)
    row += 1

    # --- log panel ---
    if row < h:
        stdscr.addstr(row, 0, "├" + sep + "┤")
    row += 1
    for entry in list(log)[-(h - row - 1):]:
        if row < h - 1:
            stdscr.addstr(row, 2, entry[: w - 3])
            row += 1

    # --- status bar ---
    # Curses raises an error when writing to the very last cell (h-1, w-1),
    # so truncate to w-1 characters to avoid it.
    try:
        stdscr.addstr(h - 1, 0, status.ljust(w)[: w - 1], curses.A_DIM)
    except curses.error:
        pass
    stdscr.refresh()


def run_tui(stdscr, host: str, port: int):
    curses.curs_set(0)
    stdscr.timeout(REFRESH_MS)  # non-blocking getch; fires refresh every 1 s

    status = f"Connecting to {host}:{port}…"
    draw(stdscr, {}, deque(), status)

    sock = connect(host, port)
    status = f"Connected to {host}:{port}   ESC/X to quit"
    log: deque = deque(maxlen=LOG_LINES * 2)
    sensors = fetch_sensors(sock)

    draw(stdscr, sensors, log, status)

    while True:
        key = stdscr.getch()

        if key in QUIT_KEYS:
            break

        if key == curses.ERR:
            # timeout — just refresh sensors
            sensors = fetch_sensors(sock)
            draw(stdscr, sensors, log, status)
            continue

        command = KEY_MAP.get(key)
        if command is None:
            continue

        try:
            response = cmd(sock, command)
        except Exception as e:
            response = f"ERR {e}"

        log.append(f"> {command:<12}  {response}")
        sensors = fetch_sensors(sock)
        draw(stdscr, sensors, log, status)

    sock.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()
    curses.wrapper(run_tui, args.host, args.port)


if __name__ == "__main__":
    main()
