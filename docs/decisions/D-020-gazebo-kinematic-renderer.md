# D-020 — Gazebo (gz-sim) as a kinematic renderer, not a physics engine

**Date:** 2026-07-09 · **Status:** accepted, **verified end-to-end (headless)**
· **Type:** architecture / demo

## Context
[D-015] chose rviz2 + MP4 and **rejected Gazebo**, for two reasons: the
multi-GB install, and — decisively — that a Gazebo *physics* world would
replace our paper-validated τ-lag vehicle model with engine/tire dynamics,
breaking the reproduce-the-paper contract ([D-007], [D-004]).

The user has since asked to add a Gazebo + rviz view "to mimic as much as
possible" a companion ROS 2 project, **while keeping everything else
constant**. The first rejection reason (install size) is now an accepted,
explicit cost. The second — the contract-breaking one — is dissolved by *how*
Gazebo is used here.

## Decision — Gazebo renders, it does not simulate
The cars are **kinematic display bodies**, not physics actors:

- each `cacc_car` model is visual-only (no `<collision>`) with
  `<gravity>false</gravity>`, so nothing falls, contacts or is integrated by
  gz-sim's dynamics;
- `gz_bridge_node` subscribes to every `/platoon/v{i}/state` and writes the
  car pose each tick via the world's `set_pose` service (`gz.msgs.Pose`).

So the **authoritative** longitudinal state still comes only from the CACC
vehicle nodes, which import `cacc.vehicle` / `cacc.controllers` — the same
single source of truth the offline core, the ROS backend and the browser sim
all run ([D-003], [D-011]). Gazebo is a third *view* of one physics, exactly
like rviz2 is; it can never alter the validated dynamics, so [D-007] holds.

Cars are placed in the **platoon (leader) frame**
(`x_i = (p_i − p_leader) − L/2`), matching viz_node's Fixed Frame choice
([D-015]): the leader stays near the origin and what the eye tracks is the
inter-vehicle **spacing breathing** under the manoeuvre — the
string-stability phenomenon itself — under a fixed chase camera.

## What was added (`ros2_ws/src/cacc_platoon`)
- `worlds/highway.sdf` — road, shoulders, dashed lane, sun + fill light,
  isometric chase camera; systems: Physics, **UserCommands** (for
  `set_pose`), SceneBroadcaster.
- `models/cacc_car/` + `cacc_platoon/gz_assets.py` — self-contained car SDF
  (chassis, cabin, four wheels, head-lights); leader white, followers a
  viridis gradient shared with plotstyle / viz_node.
- `cacc_platoon/gz_bridge_node.py` — the ROS→gz pose driver (degrades to an
  idle-with-instructions node if the gz Python bindings are absent).
- `launch/gazebo.launch.py` — gz-sim + spawn N+1 cars + include
  `platoon.launch.py` (leader/channel/recorder/vehicles) + the bridge;
  `rviz:=true` also opens the rviz markers (both 3-D views at once),
  `headless:=true` runs gz-sim server-only for CI / no-display checks.
- `scripts/setup_gazebo_wsl.sh` — one-time installer: adds the OSRF apt repo
  and installs `ros-jazzy-ros-gz`, `gz-harmonic` and the Python bindings.
- `scripts/ros2_gazebo_demo.sh` — one-shot WSL runner with a dependency
  pre-check that prints the exact `apt install` line if the gz stack is
  missing.

## Verification (2026-07-09, gz-sim Harmonic 8.14.0, WSL)
Ran `gazebo.launch.py n:=3 headless:=true` (server-only, added `headless:=`
for exactly this — no display needed): gz-sim serves the world, all 4 cars
spawn, the bridge connects and `set_pose` acks with no warnings, and the gz
poses track the live CACC state — leader pinned at x = −2.0, followers
breathing (vehicle_1 −28.4→−32.5, vehicle_2 −54.7→−62.9 m over 6 s as the
leader accelerates and the gaps open). Two bugs found and fixed by running it:

1. **Spawn rejected** — `ros_gz_sim create -string` needs a full
   `<sdf>…</sdf>` document; a bare `<model>` errors as *"not an SDFormat
   string"*. Added `gz_assets.car_sdf_doc()` (wraps `car_sdf`) for spawning.
2. **Bridge `request()` signature** — gz-transport13's Python is
   `request(service, request, request_type, response_type, timeout)` →
   `(bool, response)`, not the 4-arg form first used.

## Cost / prerequisite
Gazebo is **optional at runtime** — the core sim, rviz2 and the MP4 renderer
still work without it. It needs a one-time install in WSL (not pulled by
`ros-jazzy-desktop`):

```bash
sudo apt install ros-jazzy-ros-gz gz-harmonic \
                 python3-gz-transport13 python3-gz-msgs10
```

## Consequences
- A genuine 3-D Gazebo view of the platoon, consistent with the paper's
  physics — the earlier objection is structurally avoided, not waived.
- `set_pose` at ~50 Hz teleports the bodies; because gravity is off and there
  are no contacts, poses hold exactly between writes (no drift, no jitter
  from the solver).
- Colours/labels are fixed per vehicle in Gazebo; spacing-error *heat* stays
  an rviz2 / browser-sim feature (recolouring gz materials per tick is not
  worth the cost).

## See also
[D-004] · [D-007] · [D-011] · [D-015] · [D-016] · `08_ROS2_Architecture.md` §7
