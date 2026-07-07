# D-003 — CTHP integrates into the existing controller framework

**Date:** 2026-07-05 · **Status:** accepted · **Type:** code design

## Context
`src/cacc/controllers.py` had two controllers behind a common interface:
`ACC` (u = kp·e + kd·ė) and Ploeg-2014 `CACC` (first-order filter
h·ξ̇ + ξ = kp·e + kd·ė + u_ff, u = ξ, one internal state). The base paper's
law (their eq. (6), our sign convention):

    u_i = kp·e_i + kv·(v_{i−1} − v_i) + ka·w(t)·a_{i−1}

needs a *relative-speed* term (not the full ė = Δv − h·a_i term) and feeds
forward the predecessor's **realized acceleration**, whereas Ploeg-CACC
feeds forward the predecessor's **commanded input** u.

## Options considered
1. **Extend the shared interface** — CHOSEN.
2. A separate CTHP simulator/code path — would fork the RK4 loop, the link
   handling and the metrics; ACC-vs-CACC-vs-CTHP comparisons would no
   longer run through one pipeline.

## Decision
- Add class `CTHP` (`n_states = 0`, stateless static law).
- Interface additions (backwards-compatible):
  - `output(xc, e, e_dot, u_ff, dv)` and `deriv(...)` gained a trailing
    `dv: float = 0.0` (relative speed measured by radar).
  - Class attribute `ff_signal`: `"u"` for Ploeg-CACC (commanded input),
    `"a"` for CTHP (realized acceleration). The platoon simulator and the
    ROS beacons select which quantity to transmit based on this attribute.
  - `ControllerParams` gained `kv` (=0.63 default) and `ka` (=0.5 default)
    next to the existing `kp, kd, h, r`.
  - `make_controller('cthp', params)` factory case.
- CTHP output: `p.kp * e + p.kv * dv + p.ka * u_ff` — the noise factor w(t)
  multiplies `u_ff` **inside the channel**, not here ([D-004]).

## Why
- One simulator, one metrics module, one plotting path for all three
  controller families — the course demo's ACC-vs-CACC contrast needs them
  interchangeable behind `make_controller`.
- The existing Ploeg-family tests (28 at the time) keep guarding the old
  behavior unmodified; CTHP added 13 new tests (`tests/test_cthp.py`),
  bringing the suite to 41.

## Consequences
- The ROS `vehicle_node` also constructs controllers via `make_controller`
  and selects the beacon payload via `ff_signal` — the exact same class
  runs in both backends ([D-002]).
- Any future controller (e.g., bidirectional platooning) follows the same
  recipe: subclass, declare `n_states`/`uses_v2v`/`ff_signal`, register in
  the factory.
