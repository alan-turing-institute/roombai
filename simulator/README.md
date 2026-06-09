# Roomba 2D Turing Office Simulator

Welcome to the **Roomba 2D Turing Office Simulator**! This simulator models a cleaning Roomba, moving obstacles, human traffic, and responsive doors within a faithful reconstruction of the 1st floor office layout of the Alan Turing Institute. 

Below is a high-level explanation of how the simulator and its components work, which will help you understand its design and coordinate systems.

---

## 📐 Bounding & Coordinate Systems

The simulator maintains three coordinate systems to bridge the physical layout, mathematical calculations, and screen rendering:

| Coordinate System | Units | Range / Size | Description |
| :--- | :--- | :--- | :--- |
| **PDF Map** | PDF points | $X \in [0, 1600]$<br>$Y \in [0, 1000]$ | Origin $(0, 0)$ is at the bottom-left. Dimensions are extracted from `turing_floor_map.pdf`. |
| **Physical World (Math)** | Millimeters (mm) | $X \in [0, 57600]$<br>$Y \in [0, 36000]$ | $1 \text{ PDF unit} = 36 \text{ mm}$. All physics calculations, velocities, and ray-casting are done at this scale for high fidelity. |
| **Visual Screen** | Pixels | $1200 \times 850$ | Fits the map horizontally with margins. The vertical axis is inverted (screen Y goes down as physical/math Y goes up). |

---

## 🧩 Key Components

### 1. Static Walls
*   **Definition:** Over 1800 line segments defined in [map_data.rs](file:///Users/echalstrey/projects/roombai/simulator/src/map_data.rs). Bounded strictly inside $X \in [125.2, 1603.54]$ and $Y \in [444.87, 866.63]$ (PDF coordinates).
*   **Behavior:** Act as impenetrable barriers for the Roomba, obstacles, humans, and Lidar rays.

### 2. Dynamic Doors
*   **Behavior:** Doors are reactive to human traffic. They open automatically when a human comes within $1200\text{ mm}$ ($1.2\text{m}$) of the door segment.
*   **Closing Probability:** Once all humans leave the door's immediate area, the door closes immediately with a $50\%$ probability.
*   **Visuals:**
    *   🔴 **Solid Red Line:** Closed door (solid barrier).
    *   🟢 **Dashed Green Line:** Open door (navigable).
    *   🟡 **Pulsing Gold Glow:** The target exit door.
*   **Modifications:** Doors 19 and 29 have been removed and replaced with walls, as requested.

### 3. Obstacles (Objects)
*   **Count & Scale:** Exactly 18 circular obstacles representing office furniture (chairs, tables). Bounded to a radius between $180\text{ mm}$ and $250\text{ mm}$.
*   **Distribution:** Bounded strictly within room boundaries:
    *   $9$ obstacles are placed inside the **Enigma** room.
    *   $9$ obstacles are placed in other bookable meeting/office rooms.
*   **Placement Safety:** Generated randomly avoiding room starting zones, door openings, walls, and overlapping other objects.

### 4. Humans
*   **Count & Scale:** Exactly 20 humans represented by colored circles of radius $250\text{ mm}$ moving at $120 - 180\text{ mm/s}$.
*   **Markov Chain Distribution:** Bounded to dynamically maintain an average of 50% population in Enigma:
    *   $10$ humans spawn initially in Enigma.
    *   $10$ humans spawn initially in non-Enigma rooms.
    *   When picking a target, humans transition between Enigma and non-Enigma targets with a 40% probability, ensuring realistic inter-room traffic.
*   **Smooth Waypoint Pathfinding:** Humans do not walk in straight lines through walls. They route themselves using waypoint nodes situated at room door exits, connecting rooms to the central horizontal corridor ($Y \approx 730$).
*   **Anti-Stuck Logic:** If a human gets blocked by a closed door, the Roomba, or another human (velocity drops below threshold), they will immediately pick a new destination and replan a path.

### 5. Roomba (The Robot)
*   **Dimensions:** $340\text{ mm}$ diameter ($170\text{ mm}$ radius). Starts in the middle of Enigma.
*   **Collision Resolution:** Employs sliding multi-pass collision solver against walls, closed doors, obstacles, and humans.
*   **Watchdog Safety:** Continuous motion commands (`go`, `drive`) stop automatically after 3 seconds of silence.
*   **Sensors:**
    *   **Bumper Sensors:** Triggered on left, right, or both depending on collision impact angle.
    *   **Lidar:** Simulates 8 rays spaced $45^\circ$ apart, returning obstacle/wall distances in cm.
*   **Seen Grid (Fog of War):** Highlights areas of the map visited or scanned by the Roomba. It maps a grid of $160 \times 75$ cells. Visited grid blocks glow cyan.

---

## 🔌 API & TCP Protocol

The simulator runs a TCP listener on `127.0.0.1:9999` to accept control protocol commands:

```
[Client / Control Script] ----(TCP port 9999)----> [Simulator / Rust Daemon]
```

### Supported Commands

| Command | Arguments | Return Format | Description |
| :--- | :--- | :--- | :--- |
| `ping` | None | `OK pong` | Health check. |
| `safe` / `full` | None | `OK safe` / `OK full` | Mode transitions. |
| `stop` | None | `OK stop` | Halts Roomba. |
| `speed` | `<multiplier>` | `OK speed <val>` | Sets simulation speed multiplier (e.g. `10.0`). |
| `move` | `<cm>` | `OK move <cm> (~<sec>s)` | Drives Roomba forward/backward by a distance. |
| `turn` | `<deg>` | `OK turn <deg> (~<sec>s)` | Rotates Roomba in place (+ = CCW, - = CW). |
| `go` | `<cm/s> <deg/s>` | `OK go <cm/s> <deg/s>` | Raw differential speed + rotation (3s timeout). |
| `drive` | `<left_mm_s> <right_mm_s>` | `OK drive L<l> R<r>` | Direct wheel speed control (3s timeout). |
| `sense` | None | `OK ... bumpL=<0\|1> bumpR=<0\|1>` | Status telemetry + bump sensors. |
| `bumps` | None | `OK bumpL=<0\|1> bumpR=<0\|1>` | Raw bumper state. |
| `lidar` | None | `OK f=<cm> fl=<cm> l=<cm> ...` | Telemetry from 8 Lidar rays. |
| `route` | None | `OK route <x1>,<y1> <x2>,<y2> ...`| List of all coordinates Roomba visited (in cm). |
| `target` | None | `OK dist=<mm> door=<open\|closed>` | Proximity check to the target gold door. |
| `shutdown` | None | `OK shutdown` | Shuts down simulator process. |
