# V-03 — Cross-backend validation protocol (monolithic ↔ ROS 2)

The claim "distributed execution does not corrupt the physics" is
*measured*, run by run. Reference results: `07_…_Results.md` §5.

## Why the backends can differ at all (legitimate O(dt) effects)
| aspect | monolithic core | ROS 2 backend |
|---|---|---|
| coupling | all vehicles per RK4 stage | per-vehicle RK4, neighbor ZOH @ 100 Hz |
| time | simulated clock | measured wall dt, substepped (D-014) |
| delay | exact delay-line | heap + 2 ms flush |
| noise | 10 ms hold, per link stream | per-beacon draw (≈ 40 ms hold) |
| radar | same-stage exact state | stamped msg + age extrapolation |

## Protocol (run after ANY node change)
1. Rebuild (RB-03); run the **noiseless** fixed-h case-A demo
   (200 s, 6 followers) — deterministic propagation, no realization
   scatter.
2. `plot_ros2_run.py` on the CSV: per-hop L2 ratios must be within
   **±2 % of the offline 0.9973** and monotone-ish; |e| envelope must
   match the offline character (±1.3 m for the eq.-46 sine).
3. Run the noisy case; ratios must fall inside the seed envelope **at
   the matching noise hold** ([0.948, 1.028] at 40 ms — using the 10 ms
   envelope here is the classic apples-to-oranges mistake, T-05).
4. Diagnostics if out of band: per-tick jump forensics on e (RB-04);
   check dt-clamp == age-clip; check stamps = state currency.

## Reference numbers (accepted 2026-07-06)
- Noiseless ROS: [0.976, 0.994, 0.996, 0.994, 0.994] vs core 0.9973 ✓
- Noisy ROS: [1.009, 0.979, 0.982, 0.989, 1.035], mean 0.999 — inside
  the 40 ms-hold envelope ✓
- Startup: quiescent-phase measured-e mean ±2 mm (post D-013).

## History (why this protocol exists — the short version)
Three successive real-time defects each corrupted the ratios before the
protocol converged: stale-first-message anchoring (+2 m bias), tick-time
vs wall-time frame mixing at periodic WSL freezes (±1 m spikes every
32.5 s), clamp/clip mismatches (−15 m / +8.6 m one-tick spikes). All
diagnosed from recorded data by per-tick jump analysis; all fixed in the
node design (D-013/D-014). The protocol is the guard against their
regression — treat out-of-band ratios as THAT class of bug first.

## Extension for Workstream A (QoS pipeline in ROS)
Add: ρ̂ column medians within 30 % of truth per phase; h(t) column
matching the offline run's h(t) within the transition-timing skew
(zone times are wall-relative in ROS); in-force ‖H̃‖∞ ≤ 1 at probes.
Acceptance bands already written into `10_Handoff_Plan.md` §3 A6.
