# D-025 — Product core: bring-your-own-controller, test plans, black-box string stability

**Date:** 2026-09-25 · **Status:** accepted, **validated** · **Type:**
product / architecture · **Version:** 0.4.0 · **Depends on:** D-002, D-003,
D-010, D-018, D-023 · **Market context:** P-04 · **User guide:** P-05

## Context

The owner asked which industry to build for and how to turn the repository
from a study into a product (or part of one). P-04 answers the first
question: validation tooling for longitudinal ADAS/V2X controllers, ACC/CACC
first, convoy platooning second. That customer brings **their own
controller** and **their own acceptance criteria**, often cannot share a
transfer function, and wants **CI-gateable evidence**. Every entry point in
the repository assumed the opposite (built-in controllers, criteria in Python,
analytic transfer functions, figures as output; P-04 §1).

## Decision

Build the product core as four composable pieces on top of the unchanged
physics:

1. **Plugin controllers** (`src/cacc/plugins.py`). `make_controller` accepts
   `'package.module:Name'` or `'path/file.py:Name'`; the object is validated
   against the `LongitudinalController` protocol (the same duck-typed
   interface the built-ins already had since D-003). Two opt-in extensions
   serve nonlinear laws: `raw_inputs = True` (receive `gap, v, a, t`) and
   `equilibrium_gap(v)` (start the platoon in the law's own equilibrium).
2. **Black-box string-stability sweep** (`src/cacc/stringstab.py`). Odd
   multisine leader excitation with equal speed amplitude per tone and
   Schroeder phases; per-period least-squares tone phasors with a linear
   trend; per-hop transfer = speed phasor ratio, leader hop skipped; standard
   error from the period-to-period spread; automatic 4 periods for
   stochastic channels.
3. **Test plans** (`src/cacc/evaluate.py`, `src/cacc/criteria.py`). A YAML
   plan = base scenario + dot-path overrides + named cases × matrix + seeds +
   declarative criteria (`"min_ttc >= 2.0"`) + optional sweep. Scenario I/O
   moved to `src/cacc/scenario.py` (`scenario_from_dict`) so plans reuse the
   full scenario schema instead of a second one; unknown blocks now fail
   loudly.
4. **Evidence and CLI** (`src/cacc/reporting.py`, `src/cacc/cli.py`).
   `report.json`, `junit.xml`, `summary.md`, resolved `cases/*.yaml`, git
   provenance; `cacc evaluate | sweep | run | metrics` with exit codes 0/1/2
   and a process pool; GitHub Actions workflow with a must-pass gate and a
   must-reject canary.

## Options considered

### A. Keep scripts, add more scenarios and a nicer report — rejected
Cheapest. Leaves the core blocker intact: a customer cannot evaluate their
controller without editing our code. It improves the study, not the product.

### B. Plugin + plan + black-box sweep in the Python core — **chosen**
Delivers the customer workflow end-to-end with no new dependency and without
touching the validated physics (the built-in path is bit-for-bit unchanged:
`tests/test_plugins.py::test_file_plugin_reproduces_builtin_bit_for_bit`).
Cost: the plugin boundary is Python.

### C. FMU (FMI) import first — deferred to P-04 roadmap step 3
What production teams ultimately need (controllers live in Simulink/C). But
it adds a dependency (`fmpy`), a co-simulation stepping contract and a test
FMU to maintain, and it is useless without the plan/verdict layer — which B
builds. B first, then FMU as one more plugin kind.

### D. Hosted web service — rejected for now
Premature: no customer has asked for hosting, controllers are often
confidential (on-prem is a feature), and CI integration is simpler with a CLI.

### Sweep design sub-decisions (each measured)
| choice | alternative | evidence |
|---|---|---|
| **odd** harmonics | consecutive 1, 2, 3, … | consecutive tones put every adjacent-pair difference product on harmonic 1: the IDM plugin read 1.63 at the lowest tone vs 1.00 from its linearization |
| **equal speed** amplitude per tone | equal acceleration | equal acceleration gives the slow tones ~100× the speed of the fast ones; their distortion swamped the tail hops (0.12 abs error at 3 rad/s) |
| **linear** trend in the fit | cubic trend | cubic trend is nearly collinear with harmonic 1 over one period; linear kept all built-ins < 1e-4 and IDM < 0.02 |
| base period **200 s** | 100 s | 100 s has no tone below 0.063 rad/s and missed CTHP's low-frequency peak (true ‖H̃‖∞ = 1.00227 at h = 0.6 s; 200 s measures 1.0022) |
| **4 periods + SE** under stochastic channels | 1 period | at ρ = 5, θ = 0.2 s one period scattered 1.0002–1.0020 over seeds vs nominal 0.9997; 4 periods cut the error ~3× and the SE makes the residual visible (*marginal*) |
| leader hop skipped | include it | the leader realizes its profile exactly; for Ploeg CACC (broadcasts commanded accel) that hop is not Γ |

## Evidence

- Sweep vs analytic Γ (deterministic channel, max abs error over tones and
  hops): Ploeg ACC 1.3e-9, CACC θ = 0.1 s 5.0e-5, CTHP case A < 5e-4 (slow
  ~70 s pole); IDM vs its linearization < 0.02. Pinned in
  `tests/test_stringstab.py`.
- Plugins: file and module references reproduce the built-in CTHP bit for
  bit; contract violations are named precisely (`tests/test_plugins.py`).
- Plans: expansion, worst-seed tracking, strict-JSON reports, JUnit counts,
  ERROR verdicts for crashing controllers, CLI exit codes
  (`tests/test_evaluate.py`). Full suite: 135 tests.
- End-to-end: `examples/plans/smoke.yaml` PASS in ~20 s on 2 workers;
  `examples/plans/idm_acc.yaml` FAIL (sweep 1.038 at 0.094 rad/s while every
  time-domain maneuver check passes); `examples/plans/cthp_case_a.yaml` — safe
  in all 6 cases, string-stable at θ = 0 and 0.1 s, *marginal* FAIL at
  θ = 0.2 s (1.0002 ± 0.0003; the worst-case robust bound there is 1.018).

## Consequences

- The repository now has a user who is not its author. New research features
  should land as plan-addressable scenario fields and criteria, not as new
  one-off scripts.
- `cacc.platoon.load_scenario` still works (lazy re-export) but the canonical
  home is `cacc.scenario`.
- The time-domain L2 growth ratio is kept as a metric but is not recommended
  for string-stability verdicts under channel noise (±6 % per-hop scatter at
  ρ = 5; D-010). Example gates use the sweep instead.
- Version is single-sourced from `cacc.__version__` (was 0.1.0 in
  `pyproject.toml` vs 0.3.0 in the package).

## Limits (carried into P-05 §5)

Sweep = lower bound on ‖Γ‖∞ between tones; small-signal around `v0`;
average (not worst-case) channel; ~1e-3 precision under ρ = 5 noise.
Plugins are Python-only until FMU import. Physics is longitudinal-only (V-04).
