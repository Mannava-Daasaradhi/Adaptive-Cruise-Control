# Master Plan & Handoff — QoS-Aware CACC Virtual Validation Platform

**Purpose of this document:** the complete, self-contained execution plan
for finishing this project. It was written so that any capable engineer or
model (the user will continue with Claude Opus) can pick up the work and
deliver it *exactly on this plan* without re-deriving context. Read
`05_Decision_Log.md` → `docs/decisions/` for why everything is the way it
is; read this file for **what to do next, in what order, and what "done"
means**.

---

## 0. Product vision (why the bar is "really really good")

This is not just a course simulation. The ambition is **product-grade
virtual validation that replaces hardware testbeds** for cooperative
driving controllers. Concretely, the platform must be able to *certify* a
CACC design without physical vehicles, which requires all five of:

1. **Theory in the loop** — every run carries a frequency-domain verdict
   (‖H̃‖∞ at the configuration in force), not just time traces. Built.
2. **Deterministic reproducibility** — seeded, stage-consistent noise;
   identical reruns; pinned-number unit tests (52). Built.
3. **Faithful impairments** — the V2V channel models noise (Ma-2025
   16-bit), latency, packet loss, and *time-varying quality* (rho
   schedules). Built.
4. **Distributed real-time execution** — one process per vehicle on
   ROS 2, robust to OS scheduling (wall-clock substepped integration,
   D-013/D-014), cross-validated against the monolithic core to < 2 % per
   hop. Built for the base controller; **pending for the QoS pipeline
   (Workstream A)**.
5. **A closed adaptation loop** — the novel contribution: online channel
   estimation → feedforward prediction → headway adaptation
   (`09_QoS_Adaptive_CACC.md`, D-016). Built and validated offline.

The "replace hardware" pitch, in one sentence for the paper/slides: *a
distributed, impairment-faithful, theory-instrumented simulation stack
reproduces a published T-ITS design to its printed digits, quantifies its
deployment gaps (fixed gains, latency, unknown channel), and validates the
fix — before any vehicle exists.*

## 1. Current state (verified 2026-07-07, all tests green)

| asset | state |
|---|---|
| `src/cacc` core (vehicle, CTHP/CACC/ACC, V2V link, RK4 platoon, analysis) | **done**, 55/55 pytest |
| Base-paper reproduction (Ma 2025 printed numbers exact) | **done** — `07_…_Results.md`, `results/base_paper/20260705-213624` |
| Delay/loss extension (delay is the binding constraint) | **done** — D-008 |
| ROS 2 distributed backend (Jazzy/WSL2, per-vehicle nodes) | **done & cross-validated** for fixed-h CTHP — `08_…`, D-011…D-014 |
| rviz2 live 3D + MP4 renderer | **done** — D-015, `results/videos/` |
| QoS-adaptive pipeline (estimator, predictor, adapter) **offline** | **done** — `09_…`, D-016, `results/qos_adaptive/20260707-144337` |
| Online gain re-tuning (breaks the ρ* wall) | **done** — D-017, fig4, 3 new tests |
| Monte-Carlo certification harness + HTML report | **done** — D-018, first full run CERTIFIED (`results/certification/20260707-144611`) |
| Detailed documentation corpus (`docs/` tree: theory/reference/runbooks/experiments/validation/product) | **done** — `docs/INDEX.md`, 50+ documents |
| QoS-adaptive pipeline **in ROS 2** | **not built** — Workstream A below (now includes `gains` columns + scheduler port) |
| IEEE-style paper | **not written** — Workstream B (contribution list now includes D-017/D-018) |
| Slides / demo script | **not written** — Workstream C |

Repo conventions (do not break): serial docs `01…10`; every non-trivial
decision gets `docs/decisions/D-###-slug.md` + a row in `05_Decision_Log.md`;
files < 500 lines; tests after every change (`python -m pytest tests -q`);
never commit unless asked.

## 2. Invariants — things that must stay true after any change

1. `python -m pytest tests -q` → **all pass** (55 now; grow, never
   shrink), and `python scripts/certify.py --quick` stays CERTIFIED.
2. `python scripts/reproduce_base_paper.py --quick` still reproduces the
   paper's printed numbers (0.9375 / 0.3183 / 0.8727) in `theory_checks`.
3. The ROS backend imports `cacc.*` from `pysrc` — the control law exists
   in exactly one place. Never duplicate equations into a node.
4. Noiseless ROS cross-validation stays within a few % per hop of the
   offline core (re-run the §5 check of `07_…_Results.md` after touching
   any node).
5. Windows/WSL gotchas (D-012): LF-only scripts via `tr -d "\r"`; drivers
   in a space-free path; `set -eo pipefail` (never `-u`); builds on ext4
   `~/cacc_ws`; `/tmp` is volatile.

## 3. Workstream A — QoS pipeline in ROS 2 (the remaining build)

Goal: the interference-zone experiment of `09_…` §4 running distributed,
with live rviz. Estimated 4–6 focused hours. Steps, in order:

**A1. Messages.** Add `float64 headway`, `float64 rho_hat`,
`float64 kp_live`, `float64 kv_live` to
`cacc_platoon_msgs/msg/VehicleState.msg`. Rebuild. Update
`recorder_node.py` header/row and `scripts/plot_ros2_run.py::load_run`
(genfromtxt reads by name, so extra columns are backward-compatible).

**A2. Channel schedule.** `channel_node.py`: parameter
`rho_schedule: [t0, rho0, t1, rho1, …]` (flat list — ROS params dislike
nested); replicate `V2VLink.rho_at` + the ρ-independent bit-cache pattern
(draw U′ per beacon, form w with the scheduled ρ at send time). Zone times
are wall-clock-relative to node start (document the ~2 s skew as noise).

**A3. Vehicle node.** Import `ChannelEstimator`, `HeadwayAdapter`,
`AdaptConfig` from `cacc.estimation` (synced via pysrc — no new code).
Parameters mirroring the YAML `adaptation:` block. Wiring:
- keep a ring buffer of (t_msg, acceleration) from the predecessor state
  subscription (the radar history);
- on each V2V beacon: θ̂ = median of (now − beacon.stamp) over last ~50;
  pair beacon accel with radar accel at (beacon.stamp) via the ring buffer
  → `est.add_sample(t, y, a_ref)`;
- in `step()` at `est_rate`: `self.h = adapter.update(dt_est, est.rho_hat(t))`
  and use `self.h` instead of `self.cprm.h` in e/e_dot/anchor;
- predictor: replace the beacon value by
  `y + θ̂ · slope(two 0.2 s averaged blocks of received beacon values)`
  when `predictor:=true` (port of `V2VLink.receive_predicted`; keep the
  same T = 0.4 s baseline so theory matches).
- publish `headway=self.h`, `rho_hat=est.rho_hat() or nan`.
Build the adapter's table **once at node start** (log it); when
`adapt_gains:=true`, also port the `GainScheduler` wiring exactly as
`platoon.py` does (swap the stateless CTHP instance per estimator tick —
D-017; publish `kp_live`/`kv_live`). The anchor-hold and wall-dt logic
(D-013/D-014) must be preserved. Reference for every wiring detail:
`docs/reference/R-05` (diagram) and `R-09`.

**A4. Config + launch.** `config/qos_adaptive.yaml` mirroring
`scenarios/qos_adaptive.yaml` (bursts profile: leader_node already knows
`sine`; add `bursts` + `probe_amplitude` there — port `_with_probe`).
Launch arg `adapt:=true`.

**A5. Viz.** `viz_node.py`: append `h=…` to the label; color the label
amber while `rho_hat < 5` (zone indicator). Optional: second text row.

**A6. Acceptance (do not skip).** 200→240 s run, 6–8 followers:
- ρ̂ per vehicle reaches [8, 13] in good phases and [2.5, 4] in the zone
  (compare `rho_hat` CSV columns against the offline run);
- h(t) rises into [1.5, 2.0] in-zone and returns to [0.9, 1.05];
- no |e| beyond the commanded transition waves (≈ v·Δh) ± 20 %;
- noiseless fixed-h regression run still matches §5 of `07_…_Results.md`;
- `python scripts/plot_ros2_run.py` works on the new CSV (extra columns).
Then render the MP4 (`scripts/render_platoon_video.py`) of the ROS zone
run for the demo.

## 4. Workstream B — the IEEE-style paper

Target: 6–8 pages, T-ITS letter/conference format. **Every number already
exists** — the writing job is assembly. Skeleton (section → source):

1. *Introduction & related work* — `01_Proposal.md`, `02_IEEE_Base_Paper.md`
   (verified DOIs), product framing from §0 above.
2. *Modeling & base design* — `06_Simulation_Design.md` §§1–5; the sign
   map D-005; Ma-2025 law + Theorem III.2 (cite, don't re-derive).
3. *Reproduction* — `07_…_Results.md` §§1–3 tables (exact printed-number
   agreement is the credibility anchor).
4. *Deployment gaps* — D-008 (delay binds, loss doesn't), fig5 of the
   reproduction + the fixed-vs-retuned table of `09_…` §3 (ρ* wall).
5. *QoS-aware pipeline* — `09_…` §2 architecture + §3 theory (predictor
   flatline table is the headline), §4 zone experiment, §5 noise×delay
   interaction. Figures: qos fig1/fig2/fig3 (regenerate at final params).
6. *Distributed validation* — `08_…` §5 modeling-differences table +
   validated agreement numbers; D-013/D-014 as the "sim-to-real timing
   robustness" story (this is the hardware-replacement argument).
7. *Limitations & future work* — `09_…` §7 verbatim (gain re-tuning,
   staggering, ρ̂ smoothing), plus Workstream A status if unfinished.
Claims discipline: string-stability claims come from ‖H̃‖∞ tables ONLY;
time-domain figures illustrate (D-010). State v0 = 20 m/s is our choice
(D-006). Cite the arXiv preprint until campus IEEE access.

## 5. Workstream C — slides & live demo (order of the demo)

1. Hook: phantom jams / string instability (one animation frame).
2. Base paper + exact reproduction table (trust).
3. `results/videos/` MP4: case-A platoon absorbing the maneuver.
4. Deployment gaps: delay budget figure + fixed-gain wall figure.
5. QoS pipeline diagram (`09_…` §2 ASCII → redraw).
6. Zone experiment fig2 + (if A is done) the live rviz zone run
   (`bash scripts/ros2_view_demo.sh` with the qos config).
7. Predictor flatline table — "latency removed from the design problem".
8. Product slide: the five §0 capabilities as the hardware-replacement
   checklist.

## 6. Command cheat-sheet (Windows PowerShell unless noted)

    conda run -n cacc python -m pytest tests -q            # gate
    conda run -n cacc python scripts/reproduce_base_paper.py --quick
    conda run -n cacc python scripts/qos_adaptive_study.py  # figs + metrics
    conda run -n cacc python scripts/plot_ros2_run.py results/ros2/<f>.csv
    conda run -n cacc python scripts/render_platoon_video.py <csv|yaml>
    # WSL (build + timed run + live view) — see 08_… §3; ALWAYS via an
    # LF driver script in a space-free path (D-012):
    wsl -d Ubuntu -- bash /mnt/c/<no-spaces>/driver.sh

## 7. Risk register (what will bite, and the planned response)

| risk | response |
|---|---|
| ROS estimator pairing noisier than offline (real stamps, jitter) | widen κ to 1.3 in the ROS config; acceptance band already loose |
| WSL stalls corrupt an adaptive run | D-014 machinery already handles it; if new artifacts appear, check dt-clamp vs age-clip equality first |
| h transitions excite big waves in demos | show fig2 with the axis note ("commanded fallback"); stagger option in `09_…` §7 |
| Per-hop L2 misread as instability by reviewers | always lead with ‖H̃‖∞ tables; cite D-010 |
| Paper scope creep (gain re-tuning temptation) | it's explicitly future work; the ρ* wall is a *finding*, not a bug |

---
*Written by Claude Fable 5 on 2026-07-07 as the planning handoff; the
decision history (D-001…D-016) is the authoritative "why" record.*
