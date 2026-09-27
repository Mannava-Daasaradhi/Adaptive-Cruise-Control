# cacc — string-stability & V2X-robustness test engine for longitudinal vehicle controllers

> Does your ACC / CACC / platooning controller amplify traffic waves? Does it
> stay safe when V2V messages arrive late, get lost, are noisy, or are
> spoofed? Point `cacc` at your controller and a test plan; get a verdict per
> operating condition with reproducible evidence your CI can gate on.

**Who it is for:** ADAS validation and longitudinal-controls engineers, V2X
stack vendors who must show controller-level impact of latency/loss, and
convoy/platooning teams. Why this market, and what comes next:
[`docs/product/P-04`](docs/product/P-04-industry-and-product-strategy.md).

## Quick start

```bash
pip install -e .                                  # Python >= 3.11, installs the `cacc` command
cacc evaluate examples/plans/smoke.yaml -j 2      # built-in CTHP design       -> PASS
cacc evaluate examples/plans/idm_acc.yaml -j 2    # commercial-style ACC plugin -> FAIL (string-unstable)
cacc sweep scenarios/leader_brake.yaml -c acc     # measured |Γ(jω)| per hop
cacc calibrate examples/data/synthetic_openacc_platoon.csv   # digital twin of each car's ACC from a drive log
cacc audit examples/data/synthetic_openacc_platoon.csv       # the customer deliverable: HTML report + release gates
cacc init my-acc                                  # scaffold your own controller project + CI gate
```

## What you get

| capability | where |
|---|---|
| **Bring your own controller** — any Python class as `file.py:Class` or `pkg.mod:Class`; nonlinear laws get raw gap/speed | `src/cacc/plugins.py` |
| **Test plans** — base scenario × named cases × condition matrix × seeds, criteria like `"min_ttc >= 2.0"` | `src/cacc/evaluate.py`, `examples/plans/` |
| **Black-box string stability** — multisine sweep measures per-hop \|Γ(jω)\| without a transfer function; matches theory to 1e-4 | `src/cacc/stringstab.py` |
| **Impairment-faithful V2V** — delay, loss, Ma-2025 16-bit multiplicative noise, time-varying/spatial channel quality, spoofing attacks + physics-consistency gate | `src/cacc/network.py`, `trust.py` |
| **Evidence** — `report.json` (per seed, provenance), `junit.xml`, `summary.md`, resolved `cases/*.yaml`; exit 0/1/2 | `src/cacc/reporting.py` |
| **Digital twin from drive logs** — reads JRC OpenACC or generic CSV logs, fits each car's ACC, reports the time-gap margin to string stability ± uncertainty, a model-free cross-check and a V2V what-if | `src/cacc/twin.py`, `fielddata.py` |
| **Audit report** — one command from a drive log to a self-contained HTML report: verdict per car, recommended time gap, five-car brake test, V2V feedforward × latency grid with a latency budget, and a release-gate plan per car | `src/cacc/audit.py` |
| **CI template** — must-pass gate + must-reject canary; `cacc init` scaffolds the same for your repo | `.github/workflows/ci.yml`, `src/cacc/scaffold.py` |

User guide: [`docs/product/P-05`](docs/product/P-05-user-guide-evaluate-your-controller.md) ·
design records: [`D-025`](docs/decisions/D-025-product-core-bring-your-own-controller.md),
[`D-026`](docs/decisions/D-026-digital-twin-from-drive-logs.md),
[`D-027`](docs/decisions/D-027-string-stability-audit-deliverable.md) ·
go-to-market: [`docs/company/`](docs/company/GTM-01-go-to-market-plan.md).

## Why trust the verdicts

- The physics core reproduces **Ma, Pagilla & Darbha, IEEE T-ITS 26(1) 2025** to its printed
  digits (h_lb = 0.9375, ka* = 0.3183, h* = 0.8727 — pinned unit tests), cross-validated across
  three backends: Python, distributed ROS 2, and Simulink (1e-4 agreement, D-024).
- The black-box sweep is tested against the analytic Γ of every built-in controller family.
- Limits are printed, not hidden: the sweep is a lower bound on ‖Γ‖∞ between tones, small-signal
  around cruise speed, and under a noisy channel reports a standard error and flags *marginal*
  verdicts ([`V-04`](docs/validation/V-04-limitations.md) is the full limitations register).

---

## The research platform underneath

This started as a control-systems project on **Cooperative Adaptive Cruise Control**: when cars
follow each other, small speed changes by the leader get **amplified** down the line ("phantom
traffic jams" and crashes) — **string instability**. CACC shares intent over **V2V wireless** so
the platoon stays tight and stable; the hard part is staying stable despite wireless noise,
delay and loss.

**Deliverables:** `01_Proposal.md` · `02_IEEE_Base_Paper.md` · `03_OATD_Thesis.md` ·
`04_Industry_Resume_Solutions.md` · `05_Decision_Log.md` · `06_Simulation_Design.md` ·
`07_Base_Paper_Reproduction_Results.md` · `08_ROS2_Architecture.md` · `09_QoS_Adaptive_CACC.md` ·
`10_Handoff_Plan.md`.

**Research code:** base-paper reproduction (`scripts/reproduce_base_paper.py`); QoS-aware CACC —
online channel estimation, timestamp feedforward prediction, string-stability-preserving headway
adaptation (D-016, `scripts/qos_adaptive_study.py`); flagship extensions — predictive QoS map,
physics-consistency V2V gate, chance-constrained certificate (`docs/flagship/`); Monte-Carlo
certification harness (`scripts/certify.py`, D-018). **Car simulation:** Gazebo and rviz2 live 3D
(`scripts/ros2_gazebo_demo.sh`, `scripts/ros2_view_demo.sh`), MP4 renderer
(`scripts/render_platoon_video.py`), interactive browser sim (`demo/cacc_live_sim.html`).
**Decisions:** `05_Decision_Log.md` → `docs/decisions/D-###`. **Docs index:** `docs/INDEX.md`.

**Headline research result:** a leader speed perturbation is attenuated (not amplified) down a
5–10 vehicle platoon under CACC, enabling smaller gaps (higher road capacity) and lower fuel use
vs ACC — robust to V2V delay.

**Tests:** `python -m pytest -q`.
