# roomba_pilot

A small Rust driver for an iRobot Roomba/Create over the serial Open Interface.
The protocol is ported (and TDD-tested) from the reference Python `create.py`.

The `pilot` binary is both a **daemon** that holds the serial port open and a
**client** that sends it commands.

## Build & test

```sh
cargo build
cargo test        # 29 unit tests, no hardware needed
```

## Run

Start the daemon (opens the port, enters SAFE mode, listens on `127.0.0.1:9999`):

```sh
./target/debug/pilot serve [SERIAL_PATH] [TCP_PORT]
# defaults: /dev/tty.usbserial-BG03LB1L  9999
```

Then send commands from another shell:

```sh
./target/debug/pilot send "sense"
./target/debug/pilot send "forward 20 2"
```

A watchdog auto-stops the robot if a motion command isn't refreshed, so it can't
run away.

## Commands

| Command | Action |
|---|---|
| `forward <cm/s> <s>` / `back <cm/s> <s>` | drive straight for a time, then stop |
| `move <cm>` | drive a signed distance, then stop |
| `spin <deg/s> <s>` | rotate in place for a time, then stop |
| `turn <deg>` | rotate a signed angle (+ = CCW), then stop |
| `go <cm/s> <deg/s>` / `drive <l_mm/s> <r_mm/s>` | raw continuous motion (3s safety window) |
| `stop` | halt the wheels |
| `motors <side> <main> <vac>` | brushes/vacuum (-1/0/1; vac 0/1) |
| `dock` | seek the charging dock |
| `led <color> <intensity> <play> <advance>` | set LEDs (color 0=green..255=red) |
| `sense` | battery + mode + bumper panel |
| `bumps` | bumper / wheel-drop state |
| `safe` / `full` | change OI mode |
| `ping` | health check |
| `shutdown` | stop, return to passive, exit daemon |

## Layout

- `src/protocol.rs` — pure command encoding / sensor-id constants
- `src/sensors.rs` — sensor response decoding
- `src/robot.rs` — command layer over a byte transport (mock-tested)
- `src/bin/pilot.rs` — daemon + client
