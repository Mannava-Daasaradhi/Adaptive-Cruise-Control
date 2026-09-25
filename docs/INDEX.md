# Documentation index

**Using the product (evaluate your own controller):**
`product/P-05-user-guide-evaluate-your-controller.md` → why this product
and for which industry: `product/P-04-industry-and-product-strategy.md` →
design records: `decisions/D-025-product-core-bring-your-own-controller.md`,
`decisions/D-026-digital-twin-from-drive-logs.md`.

**Running the company (go-to-market):** `company/GTM-01-go-to-market-plan.md`
(ICP, wedge offer, pricing hypotheses, 90-day plan, kill criteria) →
`company/GTM-02-customer-discovery-kit.md` (hypotheses, targets, outreach,
interview script, call log) → `company/GTM-03-demo-script.md` (5-minute demo).

**Start here (the research platform):** `MASTER_SYSTEM_EXPLAINER.md` (plain-English tour of all
10 systems, how they talk to each other, and the base-paper deltas) →
`../10_Handoff_Plan.md` (the execution plan) →
`../09_QoS_Adaptive_CACC.md` (the novel contribution) →
`../05_Decision_Log.md` (why everything is the way it is).

## Numbered deliverables (repo root)
01 Proposal · 02 IEEE base paper & references · 03 OATD thesis notes ·
04 Industry/resume solutions · 05 Decision log (index → `decisions/`) ·
06 Simulation design (model→code map) · 07 Base-paper reproduction
results · 08 ROS 2 architecture & ops · 09 QoS-adaptive CACC (novel
extension) · 10 Master plan & handoff.

## Decisions (`decisions/`, ADR-style, D-001…D-018)
Scope & stack (D-001, D-002) · code design (D-003…D-005) · provenance &
methodology (D-006…D-008) · theory findings (D-009, D-010) · ROS
architecture & environment (D-011, D-012) · real-time correctness
(D-013, D-014) · visualization (D-015) · **QoS-adaptive pipeline
(D-016)** · **gain re-tuning (D-017)** · **certification harness
(D-018)** · Gazebo renderer (D-020) · flagship A/B/C (D-021…D-023) ·
Simulink backend (D-024) · **product core: plugins, test plans, black-box
string stability (D-025)** · **digital twin from drive logs (D-026)**.

## Theory (`theory/`)
| doc | contents |
|---|---|
| T-01 | vehicle model, actuator lag, why τ dominates |
| T-02 | spacing policy, error signals, sign map, time-varying h |
| T-03 | L2 string stability, transfer functions, numerics, sufficient-vs-necessary |
| T-04 | Ma Theorem III.2 closed forms + pinned numbers + intuitions |
| T-05 | noise model, ρ estimability, estimator dynamics, θ̂ |
| T-06 | delay, the noise×delay interaction, the predictor operator |
| T-07 | fixed-gain requirement, the wall, ρ*, the re-tuning fix |
| T-08 | quasi-static adaptation argument + honesty section |
| T-09 | system classification (what kind of system) + control-block diagrams (base + flagship) + signal-flow/comm architecture ledger |
| T-10 | QoS-adaptive & flagship formulas in full detail: ρ̂ estimation, delay predictor, fixed-gain wall, quasi-static argument, Flagship A/B/C derivations |

## Diagrams (`diagrams/`)
Open these directly in a browser. `architecture-signal-flow.html` — the
two-runtime signal-flow/communication schematic (per-tick call order,
ROS 2 topic chain, full ledger). `control-block-and-clusters.html` — the
rendered base control-block diagram (reference/summing-junction/
controller/plant/feedback/feedforward) plus all 25 systems grouped into
their 8 clusters. Companions to T-09.

## Reference (`reference/`)
R-01 package overview/API/principles · R-02 vehicle+controllers ·
R-03 network/V2VLink · R-04 analysis · R-05 estimation (AdaptConfig/
estimator/adapter/scheduler) · R-06 platoon/SimResult/scenarios ·
R-07 metrics+logging · R-08 scripts · R-09 ROS nodes · R-10 messages/
topics/QoS · R-11 scenario YAML schema.

## Runbooks (`runbooks/`)
RB-01 environment setup (+ the WSL gotcha table) · RB-02 offline daily
loop · RB-03 ROS build/run/view/analyze · RB-04 troubleshooting playbook
(symptom → diagnosis → fix, from real incidents).

## Experiments (`experiments/`)
E-01 reproduction protocol · E-02 delay/loss study · E-03 QoS zone ·
E-04 delay-predictor 2×2 · E-05 deep-zone gain re-tuning ·
E-06 certification suite.

## Validation (`validation/`)
V-01 V&V strategy (five layers) · V-02 test inventory (55 mapped) ·
V-03 cross-backend protocol + reference numbers · V-04 limitations
register (the honesty ledger).

## Product (`product/`)
P-01 vision · P-02 the hardware-replacement argument · P-03 roadmap ·
**P-04 industry & product strategy** (which market, what product, what
next) · **P-05 user guide** (evaluate your controller with `cacc`).

## Provenance rule
Every number quoted anywhere in this tree traces to a
`results/**/metrics.json` (runs referenced by timestamp) or to a pinned
unit test. If you find one that doesn't, that's a bug — file it in the
decision log.
