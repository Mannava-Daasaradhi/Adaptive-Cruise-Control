# ROS 2 Simulation — Architecture & Operations

Distributed real-time counterpart of the offline simulator. Runs on **ROS 2
Jazzy** under **WSL2 Ubuntu 24.04** (installed 2026-07-05, see D-012).
Sources live in this repo (`ros2_ws/src/`), builds happen on WSL ext4.

## 1. Node graph (one process per vehicle — genuinely distributed)

```
                    (radar surrogate: direct state topics, no impairment)
 leader ──/platoon/v0/state──► vehicle_1 ──/platoon/v1/state──► vehicle_2 ─ ... ─► vehicle_N
    │                             ▲   │                            ▲
    └─/platoon/v0/beacon─┐        │   └─/platoon/v1/beacon─┐       │
                         ▼        │                        ▼       │
                     ┌───────── v2v_channel ────────────────────────┐
                     │  per link: delay θ (min-heap + 500 Hz flush) │
                     │  Bernoulli loss • multiplicative noise w(t)  │
                     └─/platoon/v1/v2v ─┘        └─/platoon/v2/v2v ─┘
 recorder ◄── all /platoon/v*/state ──► results CSV
 viz_node / gz_bridge_node ◄── all /platoon/v*/state ──► rviz2 markers / Gazebo poses
```

* `cacc_platoon_msgs` (ament_cmake): `VehicleState` (index, t, p, v, a, u, e),
  `V2VBeacon` (sender, t, accel, u_cmd) — the channel scales `accel`/`u_cmd`
  by the Ma-2025 16-bit noise factor.
* `cacc_platoon` (ament_python): `leader_node` (sine-burst / brake / constant
  profiles), `vehicle_node` (RK4 at 100 Hz on its own state only; controller
  = `acc | cacc | cthp`), `channel_node`, `recorder_node`.
* **Single source of truth:** vehicle nodes import `cacc.vehicle` and
  `cacc.controllers` from this repo (synced copy on PYTHONPATH) — the ROS
  nodes run *literally the same control-law code* as the Python simulator.

## 2. Setup (already done on this machine)

```bash
# inside WSL Ubuntu 24.04 (done 2026-07-05):
#   ros-jazzy-desktop + python3-colcon-common-extensions + numpy/yaml via apt
#   (classic keyring method; ~2.6 GB in /opt/ros/jazzy)
```

## 3. Build & run (from Windows PowerShell)

```powershell
# sync repo sources -> ~/cacc_ws (ext4) and colcon build  (~15 s)
wsl -d Ubuntu -- bash -c 'tr -d "\r" < "/mnt/c/.../scripts/ros2_sync_build.sh" > /tmp/s.sh && bash /tmp/s.sh'

# timed demo: N followers, duration, controller, params YAML
wsl -d Ubuntu -- bash -c 'tr -d "\r" < "/mnt/c/.../scripts/ros2_run_demo.sh" > /tmp/r.sh && bash /tmp/r.sh 6 120 cthp ma2025_case_a.yaml'
```

(`...` = `Users/daasa/OneDrive/Desktop/sem 5/06-Control-System/PROJECT/06_Cooperative-Adaptive-Cruise-Control`.)
The demo SIGINTs the launch after the duration so the recorder closes the
CSV cleanly, then copies it to `results/ros2/<stamp>_<controller>_n<N>.csv`.
Inside WSL you can equally `ros2 launch cacc_platoon platoon.launch.py n:=12
controller:=cthp params:=<config> outfile:=/tmp/run.csv`.

## 4. Analysis / cross-validation (Windows, conda env `cacc`)

```powershell
python scripts/plot_ros2_run.py results/ros2/<stamp>_cthp_n6.csv
```

Produces speeds / spacing-error / L2-amplification figures + metrics JSON.
Analysis conventions: spacing errors are detrended by their pre-maneuver
mean (`--baseline-t`, default 8 s), the tail every vehicle did not
co-observe is trimmed (SIGINT kills nodes ~1 s apart), and the verdict is
three-way — *attenuating* (all ratios ≤ 1.02), *neutral (noise
realization)* (scatter around 1 with mean ≤ 1), or *AMPLIFYING*. The crisp
string-stability verdict is the noiseless config
(`ma2025_case_a_noiseless.yaml`); see `07_…_Results.md` §5 for the
validated numbers.

## 5. Modeling differences vs the monolithic core (expected O(dt) effects)

| aspect | Python core | ROS 2 backend |
|---|---|---|
| integration | one global RK4, all vehicles coupled per stage | per-vehicle RK4, neighbor inputs ZOH at 100 Hz |
| time | simulated clock | wall-clock-measured dt per tick, substepped (D-014) |
| V2V delay | exact delay-line lookup | heap-scheduled republication (2 ms resolution) |
| noise | w(t) held per 10 ms interval, seeded | w drawn per beacon, seeded per link |
| radar | same-stage state of predecessor | last state msg extrapolated by its age |

Consequently ROS runs agree with the core qualitatively and in the L2
amplification ratios to a few percent — differences shrink with higher
`sim_rate_hz` / `beacon_rate_hz`. Startup handling (D-013): first messages
can be 10–100 ms stale (DDS discovery), a multi-metre error at v0 = 20 m/s
that would otherwise decay on the slow ~70 s closed-loop mode; therefore the
radar surrogate extrapolates each state msg by its wall-clock age, and every
follower holds itself pinned at exact spacing for its first 2 s before
releasing (leader is at constant v0 until t = 10 s, so the hold is benign).
Timing robustness (D-014): WSL2 freezes all node processes for up to ~1.4 s
every ~32.5 s; nodes therefore integrate over the *measured* elapsed wall
time (substepped RK4) and the radar extrapolation bridges the same interval,
so stalls cause no frame mismatch.

**Validated agreement (2026-07-06, case A, 6 followers, 200 s):** noiseless —
ROS per-hop L2 ratios [0.976, 0.994, 0.996, 0.994, 0.994] vs the core's
uniform 0.9973; with 16-bit ρ=5 noise the per-hop ratios scatter around 1
because each link injects a fresh noise realization (the core, run with the
noise held 40 ms like the 25 Hz beacons, scatters seeds over [0.948, 1.028],
bracketing the ROS observations).

## 6. Config

`ros2_ws/src/cacc_platoon/config/ma2025_case_a.yaml` mirrors
`scenarios/ma2025_sine.yaml` (base-paper case A). Copy-and-edit for other
cases (h = 0.65 unstable case, delay/loss studies: `delay_s`, `loss_prob`,
`noise_rho`). `noise_rho: 0` disables channel noise.

## 7. Car visualization (D-015, D-020)

Three views of the **one** physics (the CACC vehicle nodes stay the sole
authority): rviz2 markers, a Gazebo 3-D world, and an offline MP4 renderer.

**Live 3D (rviz2, WSLg):** `viz_node` publishes car-body CUBE markers
(colored green→red by |spacing error|, text labels with speed/error) plus a
TF `map → platoon` at the leader; `config/platoon.rviz` uses `platoon` as
the Fixed Frame so the camera rides with the platoon while world-fixed lane
dashes slide past. Run inside WSL (close rviz to stop):

```bash
bash scripts/ros2_view_demo.sh 6 cthp ma2025_case_a.yaml
```

(or add `viz:=true` to any `ros2 launch cacc_platoon platoon.launch.py` and
open `rviz2 -d ~/cacc_ws/src/cacc_platoon/config/platoon.rviz`.)

**Live 3D (Gazebo / gz-sim Harmonic, D-020):** a genuine 3-D world
(`worlds/highway.sdf`) with colored `cacc_car` models on a dashed highway.
The cars are **kinematic** — visual-only, gravity off — and `gz_bridge_node`
teleports each to its live CACC position (platoon frame) every tick via the
`set_pose` service, so Gazebo *renders* the paper-validated dynamics without
ever simulating them. Verified end-to-end on gz-sim Harmonic 8.14.0 (D-020).
Requires a one-time install **not** pulled by `ros-jazzy-desktop`:

```bash
bash scripts/setup_gazebo_wsl.sh          # adds OSRF repo + installs the stack
# then (inside WSL, after ros2_sync_build.sh):
bash scripts/ros2_gazebo_demo.sh 6 cthp ma2025_case_a.yaml rviz
```

`gazebo.launch.py` starts gz-sim, spawns N+1 cars, includes
`platoon.launch.py` (identical nodes/physics) and the bridge; `rviz:=true`
opens the rviz markers alongside (both 3-D views at once), `headless:=true`
runs the server only. The runner pre-checks the gz stack and prints the exact
`apt install` line if missing; Gazebo is optional — the core sim, rviz2 and
the MP4 renderer work without it.

**MP4/GIF renderer (Windows, conda env `cacc`):** top-down highway
animation with error-colored cars, tracking camera and speed/error traces,
from any recorder CSV *or* scenario YAML (no ROS needed):

```powershell
python scripts/render_platoon_video.py results/ros2/<stamp>_cthp_n6.csv
python scripts/render_platoon_video.py scenarios/ma2025_sine.yaml
```

Output: `results/videos/<stem>.mp4` (ffmpeg; GIF fallback), default 10×
time-lapse; `--frames N` dumps still PNGs instead.

## 8. Future polish (optional)

* `ros2 bag record` for replays; per-vehicle heterogeneous params.
