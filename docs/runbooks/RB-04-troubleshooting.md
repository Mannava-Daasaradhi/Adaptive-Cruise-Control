# RB-04 — Troubleshooting playbook (symptom → diagnosis → fix)

Compiled from every real incident in the project history. Check here
before debugging from scratch.

## Offline simulation

| symptom | diagnosis | fix |
|---|---|---|
| adaptive h drops absurdly low / targets look wrong | plant τ left at the 0.1 default while the adapter models 0.5 | set `vehicle.tau: 0.5` in the scenario (R-11 warning) |
| `ValueError: not string-stable even at h=10` from a table build | that ρ is infeasible at those gains — a *finding*, not a bug (T-07) | tables cap at 10.0; adapters saturate at h_max |
| NaN in adapter targets | interpolating a table with inf cells (pre-cap bug) | fixed by the min(tab, 10.0) cap; if NaN returns, check new table builders |
| per-hop L2 > 1 on a "stable" noisy run | realization scatter (±2–5 % depending on noise hold) | lead with ‖H̃‖∞; compare against the seed envelope (T-05) |
| ratios polluted in adaptive runs | windows include commanded h-transition waves | evaluate per-phase after settle; hinf-in-force is the verdict (T-08) |
| results differ between "identical" runs | config not actually identical (mutable path) or different noise hold | configs are frozen — diff `dataclasses.asdict`; check `noise_rate` |

## Estimation / adaptation

| symptom | diagnosis | fix |
|---|---|---|
| ρ̂ stuck after channel improves | samples too old but count-window kept them | `max_age_s` expiry exists — check it wasn't disabled |
| ρ̂ = None forever | no excitation (platoon cruising) | `probe_amplitude` in the leader spec (T-05) |
| h(t) wiggles on a plateau | ρ̂ jitter through steep h_req | known (roadmap: smoothed ρ̂); harmless within margin |
| h pegged at h_max unexpectedly | ρ̂/κ landed below the steep knee (ρ ≲ 2.5) with fixed gains | enable `adapt_gains` (D-017) |

## ROS 2 backend

| symptom | diagnosis | fix |
|---|---|---|
| metre-scale one-tick e spikes, periodic | OS stall + mismatched dt-clamp/age-clip | both must be 5 s and wall-dt substepped (D-014) |
| +2 m constant error from t=0 | anchored to a stale first message | 2 s anchor-hold + extrapolation must be present (D-013) |
| −20 m ramp at the very end of CSV | leader died first under SIGINT; followers kept measuring | analysis trims the non-co-observed tail; ignore |
| `exit code -2` on every node | SIGINT teardown of a timed run | expected; not an error |
| build fails only in WSL | CRLF or /tmp wiped or `set -u` | RB-01 gotcha table |
| nothing publishes | forgot to source `install/setup.bash` + PYTHONPATH to pysrc | use the provided run scripts, they do it |

## Certification

| symptom | diagnosis | fix |
|---|---|---|
| a suite FAILs after a "harmless" change | that's the harness doing its job | read the failing check's measured value; bisect the change |
| all suites fail with import errors | public API renamed without updating `__init__` exports | R-01 API list is the contract |

## When something new and weird appears

1. Reproduce with a fixed seed and the smallest config (n=3, short t).
2. Look at *measured* signals per hop (e, then ė, then u) — every past
   bug was visible as a per-tick jump or a slow ramp in e.
3. Per-tick jump forensics: `np.where(np.abs(np.diff(e)) > thresh)` and
   match times against known periodicities (32.5 s WSL freezes!).
4. Write the diagnosis into a D-record before fixing — the forensic
   trail (D-013/D-014) has been the most reused documentation in the
   project.
