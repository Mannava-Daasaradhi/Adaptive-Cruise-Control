# R-01 — `cacc` package overview and dependency map

**Location:** `src/cacc` (v0.4.0) · Python 3.13, NumPy + PyYAML only in
the core (matplotlib only in scripts). 55-test pytest suite.

## Module map and allowed dependencies

```
 vehicle.py      ── no deps ──────────────► the plant (T-01)
 controllers.py  ── no deps ──────────────► control laws (T-02)
 network.py      ── no deps ──────────────► V2V link: delay/loss/noise/
                                            schedule/predictor (T-05/06)
 analysis.py     ── network (gammas) ─────► frequency-domain verdicts (T-03/04)
 estimation.py   ── analysis ─────────────► ρ̂ / adapter / gain scheduler
 platoon.py      ── all of the above ─────► monolithic RK4 sim + scenarios
 metrics.py      ── platoon (SimResult) ──► scalar metrics
 logging_config  ── stdlib ───────────────► setup_logging()
 __init__.py     ── re-exports the public API

 product layer (D-025, v0.4.0) — imports only from the list above:
 plugins.py      ── no deps ──────────────► LongitudinalController protocol,
                                            'module:Name' / 'file.py:Name' loader
                                            (controllers.make_controller calls it)
 scenario.py     ── platoon ──────────────► YAML/dict -> Scenario (moved here)
 stringstab.py   ── platoon ──────────────► black-box multisine |Γ(jω)| sweep
 criteria.py     ── metrics, platoon ─────► run metrics + acceptance rules
 evaluate.py     ── scenario, criteria,
                    stringstab ───────────► test plans -> report dict
 reporting.py    ── stdlib + yaml ────────► report.json / junit.xml / summary.md
 cli.py          ── all product modules ──► the `cacc` command
```

Rule: arrows only point downward in this list — e.g. `network` must never
import `platoon`. The ROS backend (`ros2_ws/`) imports `cacc.vehicle`,
`cacc.controllers`, `cacc.estimation`, `cacc.network` (constants) — never
the reverse.

## Public API (`from cacc import …`)

Plant/control: `VehicleParams`, `ControllerParams`, `ACC`, `CACC`, `CTHP`,
`make_controller`.
Channel: `V2VLink`, `MA2025_GAMMAS`.
Analysis: `gamma`, `gamma_magnitude`, `hinf_norm`, `is_string_stable`,
`min_stable_headway`, `cthp_h_lb`, `cthp_optimal`, `cthp_gains_feasible`,
`expected_w`.
Adaptation: `AdaptConfig`, `ChannelEstimator`, `HeadwayAdapter`,
`GainScheduler`.
Simulation: `PlatoonConfig`, `PlatoonSim`, `SimResult`, `Scenario`,
`load_scenario`, `scenario_from_dict`, `make_leader_profile`.
Product (D-025): `LongitudinalController` (plugin protocol); the modules
`cacc.evaluate`, `cacc.stringstab`, `cacc.criteria`, `cacc.reporting` and
the `cacc` CLI (`cacc.cli:main`) — see P-05.
Infra: `setup_logging`.

## Design principles (enforced by review)

1. **Single source of truth** — every equation exists once; the ROS nodes
   import it (D-002).
2. **Determinism** — all randomness flows from named seeded streams;
   RK4-stage evaluations must see consistent values (interval caches, not
   per-call draws) (D-004).
3. **Theory next to simulation** — the same operator that runs in the sim
   is evaluated on the imaginary axis (predictor P(s) is the exemplar).
4. **Frozen dataclass configs** — `PlatoonConfig`/`AdaptConfig`/params are
   frozen; variation via `dataclasses.replace` keeps runs hashable and
   comparisons honest.
5. **Files < 500 lines**, validated inputs at boundaries, logging via the
   `cacc.*` logger tree.

## Versioning

v0.1 Ploeg-family core → v0.2 Ma-2025 CTHP + reproduction → v0.3
QoS-adaptive (estimation/prediction/adaptation + gain re-tuning). Version
lives in `__init__.__version__`; bump on public-API change.
