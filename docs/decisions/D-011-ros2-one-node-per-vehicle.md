# D-011 — ROS 2 architecture: one node per vehicle

**Date:** 2026-07-05 · **Status:** accepted · **Type:** architecture

## Context
A ROS 2 "simulation" could be a single node running the monolithic
simulator and publishing results — technically ROS, but it would
demonstrate nothing about distributed control. The point of the ROS backend
([D-002]) is to show the *system* the theory describes: independent
vehicles, coupled only through sensing and an imperfect radio channel.

## Options considered
1. **One process per vehicle** — CHOSEN. Genuinely distributed: each
   follower integrates only its own dynamics and knows the world only
   through messages.
2. One node simulating all vehicles — no distribution, no asynchrony,
   nothing the offline core doesn't already do.
3. Gazebo/physics-engine vehicles — heavyweight and would *replace* the
   validated τ-lag model with engine physics (revisited and again rejected
   for visualization in D-015).

## Decision — node graph
- `leader_node` — prescribed a0(t) profiles (sine burst per the paper's
  eq. 46; brake; constant), midpoint integration, publishes state + beacon.
- `vehicle_node` × N — per-vehicle RK4 at `sim_rate_hz` (100 Hz) importing
  `cacc.vehicle` / `cacc.controllers` (single source of truth, [D-002]);
  controller ∈ {acc, cacc, cthp} via `make_controller`.
- `channel_node` — models all N directed links: per-beacon Bernoulli loss,
  per-beacon multiplicative 16-bit noise ([D-004]), transport delay via a
  min-heap of due-times flushed by a 500 Hz timer (2 ms resolution).
- `recorder_node` — subscribes to all states, writes the run CSV, closes
  it cleanly on SIGINT.
- Custom messages (`cacc_platoon_msgs`, ament_cmake): `VehicleState`
  (stamp, index, t, position, velocity, acceleration, u_cmd,
  spacing_error), `V2VBeacon` (stamp, sender, t, accel, u_cmd).

## Sensing split (deliberate)
- **Radar surrogate** = direct subscription to the predecessor's state
  topic, *no channel in between* — radar is onboard and doesn't suffer
  radio impairments. (Freshness handling evolved in [D-013]/[D-014].)
- **V2V feedforward** = beacons routed through `channel_node`, which is
  where delay/loss/noise live. The beacon payload (`accel` vs `u_cmd`) is
  selected by the controller's `ff_signal` ([D-003]).

## Known modeling differences vs the monolithic core
| aspect | offline core | ROS backend |
|---|---|---|
| integration | one global RK4, coupled per stage | per-vehicle RK4, ZOH coupling |
| time | simulated clock | measured wall dt ([D-014]) |
| delay | exact delay-line lookup | heap + 2 ms flush |
| noise | 10 ms hold, one stream | per-beacon draw, per-link stream |
| radar | same-stage exact state | stamped msg + age extrapolation |

Agreement is O(dt) by design and was *measured*, not assumed — see
`07_Base_Paper_Reproduction_Results.md` §5.

## Consequences
- Launch file (`launch/platoon.launch.py`, OpaqueFunction) spawns
  leader + N vehicles + channel + recorder from one `n:=` argument.
- The distributed asynchrony surfaced three real-time bugs that a
  monolithic sim can never have — documented as [D-013] and [D-014]; this
  is a feature of the exercise, not an accident.
