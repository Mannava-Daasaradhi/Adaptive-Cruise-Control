# R-02 — `vehicle.py` and `controllers.py` reference

## `cacc.vehicle`

- `N_STATES = 3` — per-vehicle state [p, v, a].
- `VehicleParams(tau=0.1, length=4.0, u_min=-8.0, u_max=3.0)` — frozen
  dataclass; `clamp(u)` saturates the command. **τ default is the
  Ploeg-era 0.1 s — Ma-2025 scenarios must set τ = 0.5 s explicitly**
  (forgetting this once produced a confusing adapter transient; see
  D-016 development notes).
- `vehicle_deriv(state, u, params) -> ẋ` — the only place the plant ODE
  exists (T-01). Called per RK4 stage by both backends.

## `cacc.controllers`

`ControllerParams(kp, kd, kv, ka, h, r)` — frozen; defaults mix families
(kp=0.2/kd=0.7/h=0.7 are Ploeg demo values; kv=0.63/ka=0.5 are Ma case-A;
**kp must be set per family** — 0.2-class for Ploeg, 0.009-class for Ma).

Uniform controller interface (all three classes):

| attr / method | meaning |
|---|---|
| `n_states` | internal controller states appended to the vehicle block |
| `uses_v2v` | whether a feedforward link is consumed |
| `ff_signal` | `"u"` = predecessor's commanded input (Ploeg), `"a"` = realized acceleration (Ma) — the platoon/beacons transmit accordingly |
| `output(xc, e, ė, u_ff, dv)` | commanded acceleration before saturation |
| `deriv(xc, e, ė, u_ff, dv)` | ẋc (empty array when stateless) |

Implementations:
- `ACC` — u = kp·e + kd·ė; no V2V; stateless.
- `CACC` (Ploeg 2014 eq. 10) — h·ξ̇ + ξ = kp·e + kd·ė + u_ff; u = ξ;
  one state. The 1/(hs+1) filter is what buys small-h string stability.
- `CTHP` (Ma 2025 eq. 6) — u = kp·e + kv·dv + ka·u_ff; stateless. The
  noise factor w(t) is applied by the *link*, not here (D-004), matching
  u = ka·w·a_{i−1} − …

`make_controller(kind, params)` — factory; kinds `'acc' | 'cacc' | 'cthp'`.

## Contracts and invariants

1. Controllers are **pure** given (params, inputs) — no hidden state
   outside `xc`. This is what makes the D-017 gain re-tuning safe: a
   stateless CTHP instance can be swapped per estimator tick
   (`make_controller("cthp", replace(params, kp=…, kv=…))`) without any
   handover logic.
2. Signature discipline: `dv` was appended with a default when CTHP
   arrived (D-003) so the Ploeg family was untouched; any future input
   must follow the same append-with-default pattern.
3. Adding a controller: subclass with the four members, register in the
   factory, extend `analysis.gamma` with its Γ(s), and add pinned tests —
   the D-003 recipe.

## Extension points

- Per-vehicle heterogeneous params: construct different
  `ControllerParams` per follower (the platoon already holds one
  controller instance per follower; only config plumbing is missing).
- Bidirectional/leader-following topologies would need `_deriv` changes
  in `platoon.py` and new Γ derivations — a research task, not a refactor.
