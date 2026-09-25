# P-05 — User guide: evaluate your controller with `cacc`

> **Summary.** Install the package, wrap your longitudinal controller in a
> small plugin class, describe the conditions in a test-plan YAML, run
> `cacc evaluate`. You get a PASS / FAIL / ERROR verdict per case, the worst
> seed for every failing check, a measured string-stability curve, and
> reports for humans (`summary.md`) and CI (`junit.xml`, exit code). Design
> rationale: D-025. Market context: P-04.

**Reading map.** First run → §1. Plugging in your controller → §2. Writing a
plan → §3. Metrics → §4. String-stability sweep and its limits → §5.
Reports, CI and reproduction → §6–§7. FAQ → §8.

---

## 1. Install and first run (2 minutes)

```bash
pip install -e .                                   # Python >= 3.11; adds the `cacc` command
cacc evaluate examples/plans/smoke.yaml -j 2       # built-in CTHP design: PASS
cacc evaluate examples/plans/idm_acc.yaml -j 2     # plugin IDM ACC: FAIL (string-unstable)
```

Reports land in `results/evaluations/<plan>/<timestamp>/` (override with
`-o DIR`). Exit code: `0` PASS, `1` FAIL, `2` ERROR or invalid plan.

Other commands:

```bash
cacc sweep scenarios/leader_brake.yaml -c acc     # measured |Γ(jω)| per hop
cacc run   scenarios/leader_brake.yaml -c cacc --seed 3   # one run, all metrics
cacc metrics                                       # the metrics criteria can use
```

## 2. Plug in your controller

A controller is any class the simulator can drive like the built-in laws.
Reference it as `path/to/file.py:ClassName` (no packaging needed) or
`package.module:ClassName`. It is constructed as
`ClassName(params, **options)`, where `params` is the scenario's
`ControllerParams` (spacing policy `h`, `r` and standard gains) and
`options` comes from the plan.

```python
import numpy as np

class MyACC:
    n_states = 0          # internal states the simulator integrates (0 = static)
    uses_v2v = False      # True -> u_ff carries the predecessor's V2V broadcast
    ff_signal = "a"       # what the V2V link carries: "a" realized / "u" commanded accel
    raw_inputs = True     # optional: also receive gap, v, a, t as keywords

    def __init__(self, params, k_gap=0.2, k_speed=0.7, T=1.4, s0=2.0):
        self.k_gap, self.k_speed, self.T, self.s0 = k_gap, k_speed, T, s0

    def equilibrium_gap(self, v):          # optional: start in your equilibrium
        return self.s0 + self.T * v

    def output(self, xc, e, e_dot, u_ff, dv=0.0, *, gap, v, a, t):
        # commanded acceleration [m/s^2]; the simulator saturates it to the
        # vehicle limits and applies the actuator lag tau
        return self.k_gap * (gap - self.s0 - self.T * v) + self.k_speed * dv

    def deriv(self, xc, e, e_dot, u_ff, dv=0.0, **raw):
        return np.empty(0)                 # d(xc)/dt, length n_states
```

Signals always passed: `xc` (your internal state), `e` / `e_dot` (spacing
error and its rate w.r.t. the scenario policy `r + h·v`), `u_ff` (V2V value
exactly as the impaired link delivers it — delay, loss, noise, attacks
applied; 0 when `uses_v2v` is False) and `dv` (predecessor speed minus
yours). The protocol is `cacc.plugins.LongitudinalController`; violations
are reported precisely at load time. Full example:
`examples/controllers/idm_acc.py`.

## 3. Write a test plan

```yaml
name: my-release-gate
description: What this gate protects.
base: ../../scenarios/ma2025_sine.yaml       # or  scenario: {inline scenario mapping}
under_test:
  controller: ../controllers/my_acc.py:MyACC # plan-relative path
  options: {T: 1.4}
seeds: 5                                     # 1..5, or a list [3, 7, 11]
overrides:                                   # dot paths into the scenario, all cases
  platoon.n_followers: 6
  sim.t_final: 120.0
cases:                                       # named variants (default: one "base")
  - name: sine
  - name: hard-brake
    set:
      leader: {profile: brake, t_start: 10.0, duration: 3.0, decel: -5.0}
matrix:                                      # cartesian product, applied to every case
  network.delay: [0.0, 0.1, 0.2]
  vehicle.tau: [0.3, 0.5]
criteria:
  - "min_gap >= 2.0"
  - "min_ttc >= 2.0"
  - {metric: peak_decel, op: "<=", value: 6.0, name: comfort}
string_stability:                            # optional black-box sweep per case
  max_gain: 1.0
  sweep: {n_followers: 3}                    # any cacc.stringstab.SweepConfig field
```

- **Dot paths** address the scenario schema (R-11): `platoon`, `vehicle`,
  `controller`, `network`, `adaptation`, `attack`, `trust`, `leader`, `sim`.
  A misspelled block (`netwrk.delay`) or field fails at load time, before any
  simulation runs.
- **Case names** combine the case and matrix values, e.g.
  `hard-brake[delay=0.1,tau=0.5]`.
- **Relative paths** (`base`, a `file.py` controller) resolve against the
  plan's directory, so a plan and its controller move together.
- Runs execute in parallel with `-j N` (one process per run).

## 4. Metrics

Every metric is the worst value over all followers and the whole run; across
seeds the plan keeps the worst again (lowest for `>=`, highest for `<=`) and
records which seed produced it.

| metric | unit | meaning | typical rule |
|---|---|---|---|
| `min_gap` | m | smallest bumper-to-bumper gap (≤ 0 = collision) | `>=` |
| `min_time_gap` | s | smallest gap ÷ own speed (while moving > 1 m/s) | `>=` |
| `min_ttc` | s | smallest time-to-collision while closing in (> 0.1 m/s); `never` if no closing | `>=` |
| `peak_spacing_error` | m | largest \|spacing error\| vs the scenario policy | `<=` |
| `l2_amplification_max` | – | largest vehicle-to-vehicle L2 error growth | `<=` |
| `peak_decel` / `peak_accel` | m/s² | hardest realized braking / acceleration | `<=` |
| `peak_jerk` | m/s³ | largest \|da/dt\| (comfort) | `<=` |
| `rms_accel` | m/s² | largest per-follower RMS acceleration (comfort/energy proxy) | `<=` |

## 5. The string-stability sweep

String stability means a disturbance does not grow down the platoon:
per-hop gain |Γ(jω)| ≤ 1 at every frequency. For a controller without a
closed-form transfer function, `cacc` **measures** it:

1. the leader drives a small odd-harmonic multisine (16 tones, 0.031–3.0 rad/s,
   0.01 m/s speed per tone, Schroeder phases);
2. after one settling period (200 s) each vehicle's speed is fitted per tone
   by least squares, period by period;
3. per-hop gain = speed phasor of vehicle *i* ÷ vehicle *i−1*, skipping the
   leader hop (not homogeneous for every controller family).

It reproduces the analytic |Γ| of the built-in ACC, CACC and CTHP laws to
< 1e-4 (`tests/test_stringstab.py`), and IDM's linearization to < 0.02.

**Read the result with its limits:**

- The peak over tones is a *lower bound* on ‖Γ‖∞; a resonance narrower
  than the tone spacing, or below 0.031 rad/s, can be missed.
- It is *small-signal* string stability around the cruise speed (`platoon.v0`);
  a nonlinear controller is measured at that operating point only. (The IDM
  example passes the large 20 → 30 → 20 m/s sine maneuver in the time domain
  yet measures |Γ| = 1.038 at cruise — both statements are true.)
- Under a **stochastic channel** (noise, loss, radar noise) the sweep
  automatically measures 4 periods and reports a standard error. A verdict
  within 2 standard errors of `max_gain` is labelled **marginal**: the
  measurement cannot resolve it. It also measures the *average* channel, not
  the worst case — for built-in laws use the analytic robust certificate
  (`cacc.certificate`, D-023) for worst-case channel claims.

## 6. Reports

| file | for | content |
|---|---|---|
| `summary.md` | humans, PRs | verdict, case × check table, worst seed, marginal notes |
| `junit.xml` | CI dashboards | one testcase per (case, check); failures carry worst value + seed |
| `report.json` | tools, audits | everything: per-seed values, sweep gain curves ± SE, controller + options, seeds, tool version, git commit + dirty flag, timestamp |
| `cases/<case>.yaml` | reproduction | the fully resolved scenario of each case |

## 7. CI and reproducing a failure

`.github/workflows/ci.yml` is the template: the `controller-gate` job runs a
plan, appends `summary.md` to the GitHub job summary, uploads the evidence,
and fails the build on a FAIL verdict. It also runs a known-bad controller
and requires it to be rejected — a canary that the gate still has teeth.

To reproduce a failing row, take the case file and the worst seed from the
report:

```bash
cacc run results/evaluations/<plan>/<stamp>/cases/<case>.yaml \
    -c <controller> --option key=value --seed <worst_seed>
```

## 8. FAQ

**Why is `l2_amplification_max` not in the CTHP example gate?** Under
multiplicative channel noise each link's noise realization scatters single-hop
L2 ratios by ±6 % (noiseless: 0.997 on every hop, `scenarios/ma2025_sine.yaml`
with a hard brake), so the time-domain ratio would judge the noise, not the
controller. String-stability claims come from the frequency domain (D-010);
use `string_stability`. The ratio is still useful for deterministic channels
and radar-only controllers.

**Can I evaluate a Simulink or C controller?** Not directly yet: wrap it in
a Python plugin (e.g. via a shared library or a generated Python model). FMU
import is roadmap step 3 in P-04.

**How long does a plan take?** A 120 s, 6-follower run is ~1–4 s; a sweep
~4 s (deterministic channel) or ~10 s (stochastic). `examples/plans/cthp_case_a.yaml`
(18 runs + 6 sweeps) takes ~45 s on 4 workers.

## Revision history
- 2026-09-25 — created with D-025 (v0.4.0).
