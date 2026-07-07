# D-014 — ROS 2 nodes integrate over measured wall time (substepped RK4)

**Date:** 2026-07-06 · **Status:** accepted · **Type:** real-time correctness

## Problem
Even after [D-013], a **noiseless** 200 s run showed per-hop L2 ratios
scattered 0.79–1.28 where the offline core gives a uniform 0.9973 — and a
noiseless distributed run has no randomness to blame.

## Diagnosis (from the recorded CSVs, three iterations)
1. Per-tick jump analysis found error spikes at t ≈ 56.5, 89.0, 121.5,
   154.0, 186.5 s — **periodic, every 32.5 s, identical across all six
   vehicles**: WSL2/Windows housekeeping freezes *all* node processes.
2. Tick accounting (rows vs wall duration) showed ticks are **dropped, not
   caught up** (~5.8 s missing over a 205 s run), and consecutive-t gaps
   revealed the stalls last **up to ~1.4 s**.
3. Mechanism of corruption: with tick-counted local time, a stalled
   vehicle's own state *freezes* during the stall while the radar
   extrapolation ([D-013]) tracks wall time — the two sides of the gap
   measurement diverge by v·(stall), so e spikes for one tick and then
   rings down the platoon on the ~70 s slow mode, dominating every L2 norm.
   Observed spike sizes matched the formula exactly in three regimes:
   ≈ +0.9 m (tick time + 50 ms age clip), −15…−18 m (wall dt clamped 1 s,
   age clip 0.25 s), +8.6 m (dt clamp 1.0 s vs 1.4 s true stall).

## Decision
1. **Wall-clock timebase:** leader and vehicle nodes measure
   `dt = now − last_now` per timer callback (clamped to [1 µs, 5 s]) and
   integrate over *that*, so a stall advances the state by the true elapsed
   time and every node's state always corresponds to wall-now. A stall then
   causes **no frame mismatch at all** — the whole platoon simply takes one
   long, consistent step.
2. **Substepped RK4:** a single RK4 step is only stable up to
   h ≈ 2.78·τ/1 ≈ 1.4 s for the actuator pole 1/τ = 2 s⁻¹, so long
   intervals are integrated as ⌈dt/dt_nom⌉ internal RK4 substeps of ≤10 ms.
3. **Matched horizons:** the radar-age clip equals the dt clamp (5 s).
   Both sides of the gap measurement must bridge the *same* stall — any
   mismatch spikes e by v·(mismatch) for one tick (this is precisely what
   the −15 m and +8.6 m intermediate regimes were). Extrapolation error
   over 5 s stays ≈ jerk·age²/2 < 1 m for the paper's maneuver.
4. **Sampled-data ordering:** the vehicle step was reordered into the
   standard loop — (1) integrate with the input computed at the previous
   sample (ZOH actuation), (2) measure fresh, (3) compute the next input —
   so own state, the radar extrapolation and the published stamp all refer
   to the same instant `now`.

## Verification (case A, 6 followers, 200 s)
- Noiseless run `20260706-190642`: per-hop ratios
  **[0.976, 0.994, 0.996, 0.994, 0.994]** vs offline 0.9973 — < 2 %/hop;
  |e| ∈ [−1.31, +0.55] m matching the offline response character; maneuver
  RMS decreases monotonically down the string.
- Noisy run `20260706-191139`: ratios [1.009, 0.979, 0.982, 0.989, 1.035],
  mean 0.999 — inside the offline seed-to-seed envelope [0.948, 1.028]
  once the offline noise is held per 40 ms like the ROS beacons
  (10-seed sweeps; see [D-004] on hold time and
  `07_Base_Paper_Reproduction_Results.md` §5).
- Analysis-side: the run tail no vehicle co-observed is trimmed (SIGINT
  kills nodes ~1 s apart) and the verdict is three-way (attenuating /
  neutral-noise-realization / AMPLIFYING) because per-hop ratios of a
  *noise-injected* run are not the transfer-function gain ([D-010]).

## Consequences
- The ROS backend is now robust to arbitrary host scheduling: correctness
  degrades to "one long consistent step" instead of metre-scale artifacts.
- This is the difference between a simulation that *runs* under ROS and one
  that is *trustworthy* under ROS — worth a paragraph in the report.
