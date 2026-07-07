# D-004 — Channel noise lives in the V2V link, not the controller

**Date:** 2026-07-05 · **Status:** accepted · **Type:** code design

## Context
The base paper models V2V quantization/fading as a *multiplicative* noise
factor on the communicated acceleration (their eqs. (4)–(5)):

    w(t) = (1 − 1/ρ) + (1/ρ) · Σ_{j=1..n} z_j / 2^j ,  z_j ~ Bernoulli(γ_j)

so w ∈ [1 − 1/ρ, 1 + 1/ρ), n = 16 bits, and the received feedforward is
w(t)·a_{i−1}. The paper's Section IV uses 16 specific γ values.

## Options considered
1. **Apply w inside `V2VLink.receive`** — CHOSEN. The controller sees an
   already-corrupted feedforward, exactly like a real receiver.
2. Apply w inside the CTHP controller — couples a channel property into a
   control law and would have to be replicated for every controller and
   again in the ROS channel node.

## Decision
- `src/cacc/network.py`: `V2VLink(..., noise_rho=None, noise_gammas=None,
  noise_rate=100.0)`; `receive()` returns `noise_at(t) * raw`.
  `noise_gammas=None` selects `MA2025_GAMMAS` (the paper's 16 values,
  committed as a module constant). `noise_rho=None` (offline) /
  `noise_rho: 0` (ROS YAML) disables noise; `passthrough` mode additionally
  requires noise disabled.
- The controller keeps only the deterministic gain `ka` — matching the
  paper's u_i = ka·w(t)·a_{i−1} − … factorization.
- In the ROS backend the same physics lives in `channel_node.py`: one draw
  of w per forwarded beacon, per directed link.

## Determinism design (the subtle part)
The offline simulator integrates with RK4; a naive `rng.random()` inside
`receive()` would be called once per RK4 *stage*, making stage evaluations
inconsistent (the same t sees different w) and making run results depend on
integrator internals. Therefore:
- w(t) is **piecewise-constant**: held for 1/`noise_rate` (default 10 ms)
  intervals; `noise_at(t)` indexes a lazily-grown cache keyed by the
  interval number, so every evaluation at the same t returns the same w.
- The noise RNG is a **separate seeded stream**
  (`np.random.default_rng([seed, 0xCACC])`), so enabling/disabling noise
  does not shift the packet-loss Bernoulli draws — loss studies stay
  comparable across noise settings.
- ROS channel RNGs are seeded per link: `[seed, link_index, 0xCACC]`.

## Consequences
- Reruns with the same seed reproduce **exactly** (pinned by
  `test_noise_deterministic_per_seed`).
- The noise *hold time* differs between backends (10 ms offline vs one draw
  per 40 ms beacon in ROS). This is a modeling difference with measurable
  consequences for L2-ratio scatter — quantified in [D-014]'s cross-check
  and `07_…_Results.md` §5 (offline with 40 ms hold scatters seeds over
  [0.948, 1.028] vs [0.981, 1.007] at 10 ms).
