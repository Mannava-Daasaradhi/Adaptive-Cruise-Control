# R-09 — ROS 2 nodes reference (`ros2_ws/src/cacc_platoon`)

One rclpy node type per role; control laws imported from `cacc.*`
(synced to `~/cacc_ws/pysrc`, on PYTHONPATH). Architecture rationale:
D-011; real-time correctness: D-013/D-014.

## `leader_node`

Prescribed a0(t): `sine | brake | constant` profiles (params mirror the
offline `make_leader_profile`). Midpoint integration over the **measured
wall dt** (clamped [1 µs, 5 s]); publishes `VehicleState` + `V2VBeacon`
(beacon decimated to `beacon_rate_hz`). Stamp = the `now` used for the
step — the state is current at its stamp (the invariant everything else
relies on). **Workstream A adds:** `bursts` profile + `probe_amplitude`.

## `vehicle_node` (the heart)

Per tick (sampled-data ordering, D-014):
1. dt = measured wall elapsed (clamped ≤ 5 s), **substepped RK4** at
   ≤ 10 ms substeps (single-step stability limit ≈ 1.4 s for 1/τ = 2);
   integrates own [p, v, a] (+ controller state) with the *previous*
   input (ZOH actuation);
2. fresh measurement: radar surrogate = last predecessor `VehicleState`
   **extrapolated by its stamp age** (clip = 5 s, equal to the dt clamp —
   mismatched horizons re-create metre-scale spikes, see D-014 forensics);
   anchor-hold pins exact spacing for the first 2 s (D-013);
3. compute next input via `make_controller(...)`; publish stamped state;
   beacon at `beacon_rate_hz` (payload per `ff_signal`).
**Workstream A adds:** estimator/adapter/scheduler wiring (R-05 diagram),
`headway`/`rho_hat` in the published state, predictor on the beacon path.

## `channel_node`

All N directed links: per-beacon Bernoulli loss, per-beacon 16-bit
multiplicative noise (RNG `[seed, link, 0xCACC]`), delay via min-heap +
500 Hz flush (2 ms resolution). `noise_rho: 0` disables noise.
**Workstream A adds:** flat `rho_schedule` param + ρ-independent U′ draws.

## `recorder_node`

Subscribes all states → CSV (`index,t,position,velocity,acceleration,
u_cmd,spacing_error`); closes cleanly on SIGINT. **Workstream A adds:**
`headway`, `rho_hat` columns.

## `viz_node` (D-015)

20 Hz MarkerArray on `/platoon/markers`: car CUBEs colored by |e|
(green→red, saturating at 1 m), text labels (speed, e), road slab +
world-fixed lane dashes; TF `map → platoon` at the leader so the shipped
`config/platoon.rviz` camera rides along (fixed frame `platoon`) while
dashes slide past — the motion cue.

## Cross-cutting invariants (do not regress)

1. Every published state's `stamp` equals the wall time at which that
   state is current.
2. All nodes guard `destroy_node()` in a try/except (SIGINT teardown
   race produced the historical exit-code −2 noise).
3. dt clamp == radar-age clip (5 s), both nodes, always.
4. No equations in nodes — import from `cacc.*`.
5. Launch: `platoon.launch.py n:= controller:= params:= outfile:= viz:=`
   (OpaqueFunction spawns leader + N vehicles + channel + recorder
   [+ viz]); configs in `config/*.yaml` (`/**: ros__parameters:` form).
