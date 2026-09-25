# P-04 — Which industry, what product, and the path from study to product

> **Summary.** The strongest commercial fit for this codebase is **validation
> tooling for longitudinal driver-assistance controllers** — an evaluation
> engine that tells an ADAS/V2X team whether *their* ACC or CACC controller is
> string-stable and safe under realistic communication impairments, with
> evidence they can gate CI on. The beachhead is passenger-car **ACC/CACC
> development and validation** (huge installed base, a documented unsolved
> string-stability problem, two ISO test standards, a dated V2X transition).
> The high-value second market is **truck/defense convoy platooning**, where
> the V2V-spoofing gate (Flagship B) and degraded-comms robustness matter most.
> **Rail virtual coupling** is the same mathematics and worth watching. The
> product is a *component* that plugs into the simulators these teams already
> own — not a replacement simulator. v0.4.0 (D-025) builds that component's
> core: bring-your-own-controller plugins, test plans, a black-box
> string-stability sweep, CI-ready reports.

**Reading map.** Deciding direction → §2–§3. Explaining the product in one
minute → §4. What changed in code → §5. What to build next → §6. What could
go wrong → §7.

---

## 1. Why the repository reads like a study today

The engineering underneath is strong — a physics core that reproduces Ma,
Pagilla & Darbha (IEEE T-ITS 2025) to its printed digits, three
cross-validated backends (Python, ROS 2, Simulink), and three research
extensions (predictive QoS spacing, spoof gate, chance-constrained
certificate). What makes it a *study* is not quality; it is **who the user
is**. Every entry point answered "is *our* controller good?":

| symptom (before v0.4.0) | why it blocks a product |
|---|---|
| Scripts evaluate only the built-in CTHP/CACC laws, with constants hard-coded (`scripts/certify.py`: `KA, KP, KV, TAU0 = 0.5, 0.009, 0.63, 0.5`; probe times 70/165/235 s) | A customer's controller cannot be plugged in |
| Acceptance criteria are Python code inside the script | A customer cannot express their own test plan |
| String stability is judged from a closed-form transfer function (`cacc.analysis`) | Impossible for a customer's black-box or nonlinear controller |
| No CLI, no exit codes, cwd-relative paths (`load_scenario("scenarios/…")`) | Cannot run in CI or from another project |
| Outputs are figures for a paper | Engineers need machine-readable verdicts and reproducible evidence |
| `certify.py --quick` takes ~6 min on one core | Iteration speed of a tool, not a batch study |
| Positioning: "replaces hardware testbeds" | Buyers will not accept a replacement claim before field correlation; it invites the wrong comparison |

## 2. Where this technology is used (market scan, September 2026)

| segment | who | status 2025–26 | the pain this repo addresses | fit |
|---|---|---|---|---|
| **Passenger-car ACC** | OEM ADAS teams; Tier-1s (Bosch, Continental, ZF, Aptiv, Denso, Mobileye) | Standard equipment. The JRC OpenACC campaigns tested 20+ commercial ACC models and found **all string-unstable** in every perturbation event; calibrated-model studies agree [1][2][3]. Performance is standardized in ISO 15622. | No accepted black-box string-stability test in the development loop; traffic-impact scrutiny of ACC | **high** |
| **CACC / V2X features** | same, plus V2X stack vendors | **ISO 20035:2019** defines CACC performance requirements and test procedures (reconfirmed 2024; Amendment 1 in development) [4]. US: FCC final C-V2X rules (Nov 2024); DSRC licenses end Dec 2026; OEM integration expected from MY2026–27 [5]. China: 20-city vehicle-road-cloud pilot running to 2026 [5]. | Controller-level evidence against latency / loss / noise budgets — exactly the impairments this repo models | **very high** |
| **Truck platooning / leader-follower** | Kratos, truck OEM platooning teams, AV trucking | Kratos leader-follower deployed in 11+ US states, I-70 Ohio–Indiana corridor with DriveOhio/INDOT (Apr 2025), forestry hauling in Quebec (Jan 2025) [6]. Driver-assistive platooning (human-driven lead truck) holds ~95 % of the market per market-report vendors [7]. | String stability with degraded comms; V2V message integrity | **very high, few buyers** |
| **Defense autonomous logistics** | US Army ATV-S (Carnegie Robotics, Forterra) | Down-selected 2025; single solution targeted FY2026; budget uncertainty [8]. | Convoys under jamming/spoofing — Flagship B's bounded-injection gate | high value, slow and access-restricted procurement |
| **Rail virtual coupling** | EU-Rail FP2-R2DATO, DLR, rail suppliers | First real-world train-to-train VC test (DLR, NS Amersfoort, Apr 2025): stable comms to 250 m, relative localization ±20–80 cm [9]. | String stability of virtually coupled trains under T2T comms loss | medium-high; research stage; needs a train-dynamics/braking model |

Market-size figures for platooning (e.g. ~USD 170 M in 2025, ~20 % CAGR)
come from market-report vendors [7]; treat them as directional only.

## 3. Recommendation

### 3.1 Primary: ADAS / V2X validation tooling (ACC first, CACC next)
Position the project as a **string-stability and V2X-robustness test engine
for longitudinal controllers**. Why this and not the others:

1. **Volume with a documented, unsolved problem.** Every car with ACC has a
   longitudinal controller, and the best public field evidence says they
   amplify traffic waves [1][2]. A tool that measures that — in the
   developer's loop, before a car exists — has an obvious user.
2. **Standards to anchor to.** ISO 15622 (ACC) and ISO 20035 (CACC) define
   performance requirements and test procedures. Ready-made test plans that
   mirror their procedures are a sellable artifact.
3. **A dated forcing function.** The US C-V2X transition (2026) and China's
   pilot programs put V2V-coupled controllers on real roads; latency and loss
   budgets become engineering questions with deadlines.
4. **The assets map one-to-one.** Frequency-domain verdicts, an
   impairment-faithful channel (noise, delay, loss, time-varying quality),
   deterministic reproducibility, and a spoofing defense are exactly what a
   CACC validation engineer lacks.
5. **Component, not competitor.** Validation teams already own CarMaker,
   VTD, dSPACE or CARLA. A focused evaluation engine that plugs into them
   (plugins now; FMU import next, §6) is "part of the product" — easier to
   adopt, and it avoids a head-to-head with incumbent simulators.

**Beachhead users:** ADAS validation and longitudinal-controls engineers at
Tier-1s and OEMs; V2X stack vendors who must show controller-level impact of
their latency/loss; test labs.

### 3.2 Second market: convoy and truck platooning
Fewer buyers, higher stakes, and the best fit for the security story: lead
with Flagship B (injection bounded at `ka·r0/2` for any spoof size, D-022)
plus robustness to degraded channels. Companies to study: Kratos,
Forterra, Carnegie Robotics, truck OEM platooning groups, AV-trucking
companies running convoy pilots.

### 3.3 Watch: rail virtual coupling
Same mathematics (predecessor-following string stability under
communication loss), serious funding, certification-heavy culture that values
evidence. Entry needs a train model (traction/braking curves, gradient) and
EN 50126/50128/50129 vocabulary; best reached through EU research projects.

### 3.4 Not recommended
- **A standalone "simulator that replaces testbeds."** Competes with mature
  tools, and the replacement claim needs field correlation this repo does not
  yet have (P-02 draws that boundary honestly — keep it).
- **Traffic-flow simulation (SUMO-class).** Different product, different
  buyer (P-03 already rules it out).

### 3.5 If the near-term goal is a job, not a company
The same positioning is a strong portfolio story for ADAS validation,
longitudinal controls, V2X systems and SOTIF roles: *"I built a validation
engine that measures string stability of black-box ACC controllers; it
reproduces a T-ITS paper to its printed digits and flags the instability the
JRC found in commercial ACC."* Demo it with `examples/plans/idm_acc.yaml`
(fails, correctly) next to `examples/plans/smoke.yaml` (passes).

## 4. The product in one minute

> *An ADAS engineer points `cacc` at their longitudinal controller and a
> test plan. In minutes they get a verdict per operating condition —
> safety margins, comfort, and measured string stability — with the worst
> seed for every failure, and reports their CI can gate on.*

```
  their controller ──► plugin (file.py:Class, D-025) ──┐
  their test plan  ──► scenario × matrix × seeds ──────┤
                                                       ▼
                 ┌─────────────── cacc evaluate ───────────────┐
                 │  parallel runs (RK4 platoon, V2V impairments)│
                 │  criteria: min gap, TTC, time gap, decel,    │
                 │            jerk, RMS accel, L2 growth        │
                 │  black-box sweep: measured |Γ(jω)| per hop   │
                 └──────────────────────────┬───────────────────┘
                                            ▼
   report.json (evidence) · junit.xml (CI) · summary.md (PR) · cases/*.yaml
   exit code 0 PASS / 1 FAIL / 2 ERROR
```

What makes it more than a simulator is the **verdict discipline** already in
the repo (P-01): theory-anchored checks, worst case over seeds, reproducible
(case, seed) pairs, and honest limits printed next to the verdict (for
example, a sweep result within two standard errors of the limit is labelled
*marginal*, not silently passed or failed).

## 5. What v0.4.0 changes (D-025)

| before | after |
|---|---|
| Only built-in controllers | Any controller as a plugin: `pkg.mod:Class` or `path/file.py:Class`, validated against a documented protocol (`cacc.plugins`); raw gap/speed inputs and an equilibrium hook for nonlinear laws |
| String stability only from a known transfer function | **Black-box sweep** (`cacc.stringstab`): odd multisine, per-period least squares, per-hop \|Γ(jω)\|; matches the analytic Γ of all three built-in families to < 1e-4; standard error under stochastic channels |
| Criteria hard-coded in scripts | Declarative **test plans** (`cacc.evaluate`): base scenario, overrides, named cases × matrix, seeds, criteria like `"min_ttc >= 2.0"` |
| Figures | **Evidence**: `report.json`, `junit.xml`, `summary.md`, resolved `cases/*.yaml`, git commit + tool version |
| Scripts run from the repo root | Installable **`cacc` CLI** (`evaluate`, `sweep`, `run`, `metrics`), exit codes, parallel workers |
| No CI | GitHub Actions: tests on 3.11/3.13 + a controller gate that must PASS the smoke plan and must REJECT the known-bad IDM ACC |

The research scripts, the certification harness (D-018) and the ROS 2 /
Simulink backends are untouched; they remain the reference the product's
numbers are anchored to.

## 6. Roadmap (next steps, in priority order)

> **Status 2026-09-25 (D-026, v0.5.0).** Step 1: the discovery kit is ready
> (`docs/company/GTM-02`) — the calls themselves are the founder's next
> action. Step 2: the pipeline is **built and validated on synthetic ground
> truth** (`cacc calibrate`, `scripts/openacc_study.py`); running it on the
> real OpenACC files is blocked only by data access in the build
> environment. Onboarding (`cacc init`) shipped.

| # | step | why | acceptance |
|---|---|---|---|
| 1 | **Customer discovery** — 8–10 conversations with ADAS validation, V2X and platooning engineers | Confirms (or kills) §3 before building §6.3–6.4 | Written notes; the top-3 pains ranked; a decision on ACC-first vs platooning-first |
| 2 | **Real-data credibility: OpenACC ingestion** — estimate per-hop string stability from logged speeds of real ACC platoons | Turns "simulation says" into "matches real cars"; the sweep's estimator already works on speed signals | Reproduce the published string-instability finding for ≥ 3 OpenACC vehicle models; document agreement bands |
| 3 | **FMU import** (FMI 2.0/3.0 co-simulation, e.g. via `fmpy`) | Production controllers live in Simulink/C; FMU is how they leave it | A Simulink-exported CACC FMU passes/fails the same plans as its Python twin within 1e-3 |
| 4 | **Standards-shaped plan templates** (ISO 15622, ISO 20035 procedures) | Maps verdicts to what validation teams are audited against | Templates reviewed against the purchased standards (do not copy standard text into the repo) |
| 5 | **Speed** — batch/vectorized RK4 across seeds or numba | Monte-Carlo plans should take seconds | 10× on `examples/plans/cthp_case_a.yaml`; `certify.py --quick` < 60 s |
| 6 | **Scenario breadth** — heterogeneous platoons, cut-ins (lateral hook), multi-predecessor topologies | Real traffic is not homogeneous one-look-ahead | Per-hop verdicts on mixed platoons; a cut-in maneuver family |
| 7 | **Security evidence** — package the trust gate + attack scenarios as an ISO/SAE 21434-style evidence set | Differentiator for platooning/defense | Attack-matrix plan with bounded-injection verdicts |

**Business model options** (decide after step 1): open-core (permissive core;
paid FMU adapter, standards templates, hosted report history), or
consulting-led (use the engine to deliver validation studies, productize what
repeats). For a solo developer, consulting-led is lower risk.

## 7. Risks and honest limits

| risk | mitigation |
|---|---|
| Incumbent toolchains add equivalent string-stability checks | Be the plug-in engine they integrate; the moat is verdict discipline + channel fidelity + security, not the integrator |
| No field correlation yet | Roadmap step 2 before any "replaces testing" language (P-02 boundary stands) |
| CACC adoption slower than forecast | ACC-first positioning works without any V2X on the road |
| Sweep limits: small-signal around the cruise speed; lower bound on ‖Γ‖∞ between tones; under channel noise precision ~1e-3 (reported as a standard error, and verdicts inside 2 SE flagged *marginal*) | Documented in `cacc.stringstab`, D-025, P-05; built-in families still get the analytic robust certificate |
| Longitudinal-only physics (V-04) | Roadmap step 6; state it in every report's scope |

## Sources
1. Makridis et al., *OpenACC. An open database of car-following experiments to study the properties of commercial ACC systems*, Transportation Research Part C (2021). https://www.sciencedirect.com/science/article/pii/S0968090X21000772
2. Gunter et al., *Are commercially implemented adaptive cruise control systems string stable?* (arXiv 1905.02108; IEEE T-ITS 2021). https://arxiv.org/pdf/1905.02108
3. Makridis et al., *Requiem on the positive effects of commercial adaptive cruise control on motorway traffic…*, Transportation Research Part C (2021). https://www.sciencedirect.com/science/article/pii/S0968090X21003144
4. ISO 20035:2019, *Intelligent transport systems — Cooperative adaptive cruise control systems (CACC) — Performance requirements and test procedures*; ISO 20035:2019/CD Amd 1. https://www.iso.org/standard/66879.html · https://www.iso.org/standard/90881.html
5. Venable LLP, *FCC adopts final rules on C-V2X in 5.9 GHz* (Nov 2024); Federal Register 2024-28980; 5GAA *C-V2X in action*. https://www.venable.com/insights/publications/2024/11/fcc-adopts-final-rules-on-c-v2x-in-5-9-ghz-for · https://www.federalregister.gov/documents/2024/12/13/2024-28980/use-of-the-5850-5925-ghz-band · https://5gaa.org/c-v2x-in-action/
6. Kratos Defense newsroom: I-70 Ohio/Indiana expansion (Apr 2025); Quebec forestry demonstration (Jan 2025). https://www.kratosdefense.com/newsroom/kratos-expands-deployment-of-automated-truck-platooning-technology-in-ohio-and-indiana-in-partnership-with-driveohio-indot-and-ease-logistics · https://www.kratosdefense.com/newsroom/kratos-unmanned-systems-demonstrates-leader-follower-platooning-technology-in-quebecs-forestry-sector-in-a-timber-hauling-operation
7. GMI, *Truck Platooning Market Report 2026–2035* (market-report vendor; directional). https://www.gminsights.com/industry-analysis/truck-platooning-market
8. Breaking Defense, *Army picks Carnegie, Forterra for autonomous logistics truck prototyping* (Feb 2025); FW-MAG, *US Army awards prototype contracts for Autonomous Transport Vehicles but future uncertain*. https://breakingdefense.com/2025/02/army-picks-carnegie-forterra-for-autonomous-logistics-truck-prototyping/ · https://www.fw-mag.com/shownews/433/us-army-awards-prototype-contracts-for-autonomous-transport-vehicles-but-future-uncertain
9. DLR, *DLR tests virtual coupling system in the real world* (2025); Europe's Rail FP2-R2DATO. https://www.dlr.de/en/latest/news/2025/dlr-tests-virtual-coupling-system-in-the-real-world · https://rail-research.europa.eu/latest-news/how-eu-rail-fp2-r2dato-project-advances-digitalisation-and-automation/

## Revision history
- 2026-09-25 — created with D-025 (v0.4.0): market scan, recommendation, product definition, roadmap.
