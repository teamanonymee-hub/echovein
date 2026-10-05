# EchoVein: giving a collapsed mine its own memory

**TSYP14 Technical Challenge, "The Living Map: Spatial Memory for Emergency Robots", Phase 1.**
Environment: **Mines / Tunnels**. Events: **methane gas** and **trapped victim**, plus the **collapsed floor** as a safety event.

- Technical report (PDF): [`docs/EchoVein_Technical_Report.pdf`](docs/EchoVein_Technical_Report.pdf)
- Simulation demo video: REPLACE-ME (video link)

![EchoVein system architecture](docs/diagrams/architecture.png)

## Overview

A methane explosion collapses part of a gallery and a miner is trapped. Underground there is no GPS, and rock blocks
radio beyond line of sight. A robot exploring alone loses everything it learned if it fails.

1. The **Writer** drops a reference pod **P0** at the entrance (origin of its private frame) and explores the unknown
   tunnel with its own **360° lidar map**. It detects methane and the victim and drops small radio **memory pods**.
2. The pods form a **beacon mesh**: every event travels pod to pod to the **Outside Network Area** at the entrance.
3. The Outside Network Area **receives** it, **translates** it to GPS, **carries** it over LoRa to a distant
   **command post**, and later **briefs** the chosen robot on the docking pad.
4. The command post sees a **live map that grows while the robot explores**. The commander decides **who** goes
   (GO / HOLD); the robot decides **how**.
5. The **Executor** plans its own route (victim first, hazards avoided, pod by pod as checkpoints), delivers a rescue
   kit, and, if the Writer has failed, **takes over its role** and explores only what nobody has seen yet.

No robot ever talks to the command post directly: everything passes through the Outside Network Area.

## How the challenge requirements map to the code

| Requirement | Where |
|---|---|
| Writer explores a GPS-denied space autonomously | `sim/robots.py`: `Writer.explore` (lidar occupancy grid, depth-first, shortest-path exit from dead ends, cliff reflex, battery return) |
| At least 2 event types | debounced MQ-4 gas model and AMG8833 thermal model (3-frame confirm), `sim/world.py`, `sim/robots.py` |
| Deploy small radio beacons where relevant | `Writer.drop_pod`, `_keep_mesh` (relay pods), `_maybe_waypoint` |
| Beacon message: what, where to go, when | `sim/pod_message.py`: 18 bytes, CRC-16, aging; `firmware/pod/` (BLE advertising) |
| Outside Network Area, all traffic through it | `sim/network.py` `OutsideNetworkArea` (4 roles); ROS topic radio domains |
| Robot coordinates to real-world GPS | `sim/frame.py` (WGS84) + loop closure in `translate()` |
| Transfer to a distant command post | LoRa model: time on air, duty cycle, loss, ACK, retries; live map tiles `sim/maptiles.py` |
| Brief the Executor before entry | `brief()` on the pad: task + corrected pod table + map |
| Executor navigates with inherited beacons | `sim/planner.py` (on board), `Executor.run_mission`: RSSI peak homing, pose reset, NFC fallback |
| Live map at the command post | `ros2_ws/.../live_server.py` (http://localhost:8080), `command_post.html`, right panel of the videos |

## Key design choices

- **Data path: beacon to beacon.** Pods relay every event hop by hop (ESP-NOW, line of sight within 6 m). Before
  every move the Writer checks that its next position still reaches the mesh; if not, it drops a **relay pod** first.
  Robots carrying their log out on return are the backup.
- **Only useful pods:** events, relays that keep the mesh connected, P0, and junctions with no pod within 3 m.
  Breadcrumbs every 8 m only when there is no mesh (8 pods for the nominal mine).
- **Explore everything, leave dead ends fast:** the Writer enters every passage and chamber (gas, a missing floor
  and a person are only confirmed up close). At a dead end it drives the shortest known path to the nearest
  unexplored opening instead of reversing cell by cell (258 moves instead of 290 in the nominal mine).
- **Live map:** every 5 moves the robot sends only the 2 m x 2 m tiles of its lidar map that changed (6 bytes per
  tile, LoRa packet `0x17`). The whole takeover mine costs 9 LoRa packets, 396 bytes, under 3 s of airtime.
- **Aging lowers trust, never erases a hazard:** `c = exp(-age / tau)`, tau = 10 min (gas), 30 min (victim),
  permanent (waypoint, relay, drop-off). An EXPIRED gas pod means "unknown", not "safe".
- **Command post decides WHO, never HOW:** the downlink is 4 bytes (robot, task, target pod).

## Quick start (plain Python, no ROS needed)

```bash
pip install -r requirements.txt
python run_demo.py                         # nominal mission, writes out/nominal/demo.mp4
python run_demo.py --scenario pod_dead     # one pod battery dead: NFC backup
python run_demo.py --scenario lora_loss    # 45 % LoRa packet loss: store-and-forward
python run_demo.py --scenario writer_lost  # Writer lost: Executor recovers the pods
python run_demo.py --scenario takeover     # P0 reference, live mesh, Writer fault, Executor takes over
python run_demo.py --all --no-video        # every scenario, logs only (seconds)
python -m unittest discover -s tests -v    # 12 unit tests
```

Each run writes `out/<scenario>/`: `demo.mp4`, `final_frame.png`, `mission_log.txt`, `pods.csv` (decoded pods with GPS
and raw bytes), `summary.json` and `command_post.html`.

## ROS 2 and Gazebo

Works on **Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress** and on **Ubuntu 26.04 + ROS 2 Lyrical + Gazebo Jetty**;
the launch file detects the distribution. Full install commands: [`docs/INSTALL.md`](docs/INSTALL.md).

```bash
cd ros2_ws
colcon build --packages-select echovein_ros
source install/setup.bash
ros2 launch echovein_ros echovein.launch.py scenario:=nominal speed:=10 rviz:=true   # ROS 2 nodes + RViz
ros2 launch echovein_ros gazebo.launch.py                                            # 3D takeover mission
```

Then open **http://localhost:8080** for the live command post (`commander:=auto` approves missions alone).

The nodes match the real components: writer, executor, pod field (beacon mesh), gateway (Outside Network Area),
command post, visualisation. Topics are split into radio domains so the challenge rules hold:

| Domain | Topics | Who |
|---|---|---|
| Inside the tunnel | `/tunnel/writer/*`, `/tunnel/executor/*`, `/tunnel/pods/*` | robots, pods |
| Docking pad (only when docked) | `/pad/upload`, `/pad/brief` | robots, gateway |
| LoRa (outside) | `/outside/lora_up`, `/outside/lora_down`, `/outside/live_map` | gateway, command post |
| Simulator only | `/sim/*`, `/viz/markers` | sensor models, RViz |

The command post subscribes to `/outside/*` only and decodes the real LoRa packet bytes. RViz shows the Writer's
lidar occupancy grid (`/writer/map`), the point clouds, the pods and the mesh links. Without ROS installed:
`python tests/run_ros_nodes_without_ros.py` and `python tests/run_gazebo_mode_without_gazebo.py` (both print PASS).

**Takeover story (Gazebo):** the Writer drops P0, detects methane (P2, P4) with relays P3 and P5, explores the refuge
chamber and finds the miner lying on the floor (P6), then breaks down in the loading bay. The command post sends the
Executor 20 s later; it follows the pods, delivers the kit, finds the Writer, downloads its black box, switches to
Writer mode, drops relay P8 and drop-off P9 at the collapsed floor, refreshes the gas pod P4, and docks.

## Results

| Scenario | Result |
|---|---|
| nominal | Writer: 258 moves (129 m), 8 pods (2 gas, 1 victim, 1 drop-off, 4 waypoints); pod error 0.19 m raw, 0.07 m after loop closure; victim estimate 0.22 m from truth; Executor: 34 m safe route P1 > P8 > P7 > P6 > P5 avoiding the gas zone, pose error about 0.1 m after each pod; 15.2 min |
| pod_dead | P7 read through its NFC sticker, pose error 0.16 m > 0.08 m |
| lora_loss | 45 % loss: 24 of 36 transmissions lost, all data delivered |
| writer_lost | Gateway timeout, Executor recovers the Writer's 5 pods |
| takeover | Fault at 02:10, Executor sent at 02:30, kit delivered at 03:38, map completed, 104 Executor moves, 6.7 min |

## Repository map

| Path | What |
|---|---|
| `sim/` | tested simulation core: world and sensors, robots, pod message, mesh, map tiles, frame translation, Outside Network Area, command post, planner |
| `run_demo.py` | runs the full chain and writes all outputs |
| `ros2_ws/src/echovein_ros/` | ROS 2 package: nodes, launch files, Gazebo worlds, RViz configs (core copied by `tools/sync_core.sh`) |
| `tests/` | unit tests and ROS / Gazebo smoke tests |
| `docs/` | technical report, architecture diagram, bill of materials (`bom_phase2.csv`), install guide |
| `firmware/` | pin map and pod firmware starting point for Phase 2 |
| `out/` | created when you run the demo (videos, logs, pod tables, command-post page) |

## Phase 2 hardware (summary)

Two identical tracked robots (360° 2D lidar, Raspberry Pi 4 for mapping, ESP32 for motors and sensors, MQ-4,
AMG8833, VL53L0X ToF, HC-SR04, IMU, encoders, pod magazine), 14 memory pods (ESP32-C3, LiPo, reed switch, NFC
sticker) and the Outside Network Area (ESP32, NEO-6M GPS, SX1276 LoRa). About 2,800-4,140 TND, itemised in
`docs/bom_phase2.csv`. Plan: 8 weeks to 01/12/2026 (report, section 11).

## License

MIT.
# echovein
