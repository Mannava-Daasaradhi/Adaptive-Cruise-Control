# V-04 — Known limitations register (the honesty ledger)

Every claim boundary in one place. Reviewers and future contributors
read this first; nothing here is hidden in prose elsewhere.

## Modeling
1. **Longitudinal only** — no lateral dynamics, lane changes, or cut-ins.
   Cut-in robustness is the biggest unmodeled disturbance class for
   real platoons.
2. **Linear plant + saturation** — τ-lag model; no engine maps, gear
   dynamics, grade, or aero coupling. Saturation handled empirically
   (min-gap checks), not by theory.
3. **Homogeneous platoon** — one (τ, L) for all; heterogeneity is
   plumbing-ready but unvalidated (R-06 extension recipe).
4. **Channel model** — Ma-2025 multiplicative n-bit noise + Bernoulli
   loss + constant delay. No fading correlation, no congestion coupling
   between links (each link's noise is independent), no interference
   between beacons.
5. **v0 = 20 m/s chosen** (paper leaves it free); error dynamics are
   speed-independent but gap transitions and capacity numbers scale
   with v.

## Theory
6. **Quasi-static adaptation argument** is engineering-grade (rate
   limits + margins + live spot-checks), not a dwell-time theorem
   (T-08 sketches the formalization route).
7. **L2 string stability** is the only stability notion used; L∞
   (worst-case gap excursion) claims would need new machinery.
8. **Per-hop decoupling** relies on one-vehicle look-ahead; any
   multi-look-ahead extension invalidates the heterogeneous-parameter
   argument as stated.

## Estimation / adaptation
9. **Excitation dependence** — ρ̂ needs communicated acceleration
   content; the probe dither is a design *choice* with a comfort cost
   (±0.08 m/s²).
10. **Recovery latency** ≈ max_age (20 s) by construction — capacity is
    conceded for that long after a zone clears.
11. **Estimation uncertainty is expensive below ρ ≈ 3** (steep h_req);
    κ = 1.15–1.2 costs ~0.5 s of headway in a ρ=3 zone (E-03).
12. **Simultaneous transitions** compound gap waves down the string
    (unstaggered; T-08).

## Platform
13. **WSL2 timing** — the ~1.4 s/32.5 s freezes are absorbed, but a
    hard-real-time claim would need a real RT target (the node design
    transfers: wall-dt + stamp extrapolation are platform-agnostic).
14. **Same-host clock** — θ̂ from stamps assumes synchronized clocks;
    multi-host deployments need PTP/NTP characterization.
15. **ROS QoS pipeline** not yet integrated (Workstream A) — offline
    core is the validated reference.

## Non-claims (things we deliberately do NOT assert)
- No safety certification of real vehicles is implied — the platform
  certifies *controller designs against modeled imperfections*.
- No fuel/energy model beyond the RMS-acceleration proxy.
- The 25 % capacity figure is an equilibrium-headway calculation, not a
  traffic-flow simulation with merges.
