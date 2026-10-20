# cacc — string-stability & V2X-robustness test engine for longitudinal vehicle controllers

> Does your ACC / CACC / platooning controller amplify traffic waves? Does it
> stay safe when V2V messages arrive late, get lost, are noisy, or are
> spoofed? Point `cacc` at your controller and a test plan; get a verdict per
> operating condition with reproducible evidence your CI can gate on.

**Who it is for:** ADAS validation and longitudinal-controls engineers, V2X
stack vendors who must show controller-level impact of latency/loss, and
convoy/platooning teams. Why this market, and what comes next:
[`docs/product/P-04`](docs/product/P-04-industry-and-product-strategy.md).

## Real cars: production ACC amplifies braking

`cacc field-study --fetch microsim` downloads 358 two-car runs of a
commercial ACC car following a braking lead car (FAU microSIM-ACC "ICE"
experiments) and measures, without any model, whether the ACC car braked
harder than the car it followed:

| ACC gap setting | runs | braked harder than the car ahead | median amplification | worst |
|---|---|---|---|---|
| long | 119 | 34 % | 0.97× | 1.45× |
| medium | 119 | 60 % | 1.05× | 1.31× |
| short | 120 | 78 % | 1.13× | 1.47× |
| **all** | **358** | **57 %** | **1.03×** | **1.47×** |

The shorter the gap the driver picks, the more often the car amplifies the
disturbance: string instability, measured on a real production car. (The
dataset's spacing is reconstructed from integrated speeds, so its absolute
gaps are approximate; the amplification uses speeds only.)

## Key result: the engine caught a flaw in its own spoof defence

![Minimum gap in an honest emergency brake vs radar lag](docs/figures/radar_lag_min_gap.png)

A V2V spoof gate that checks messages against the car's radar keeps a
+5 m/s² phantom-throttle attack from causing a crash (8.2 m minimum gap vs
−30.6 m without it). But radar acceleration is *late*. In an honest −7 m/s²
emergency brake at a 0.8 s headway, the original gate distrusts the early
V2V brake message: a 0.5 s radar lag shrinks the 1.23 m ungated margin to
1 cm, and a 0.8 s lag turns it into a collision (−1.27 m). Comparing the
radar with the V2V value received one radar-lag earlier keeps every case
within 0.04 m of the ungated platoon. Reproduce: `python scripts/radar_lag_study.py` (about 2 minutes);
details in [`D-028`](docs/decisions/D-028-realism-fixes-from-the-critique.md).

## Quick start

```bash
pip install -e .                                  # Python >= 3.11, installs the `cacc` command
cacc evaluate examples/plans/smoke.yaml -j 2      # built-in CTHP design       -> PASS
cacc evaluate examples/plans/idm_acc.yaml -j 2    # commercial-style ACC plugin -> FAIL (string-unstable)
cacc stress -c cthp --preset full -j 6            # EV platoon: 6 maneuvers x 24 conditions grid
cacc stress -c cacc --preset crash-db --aeb       # NHTSA + Euro NCAP crash scenarios, with emergency braking
cacc standards -c cacc --aeb -o results/std       # ISO 15622 / 20035, R157, Euro NCAP checklist
cacc energy -c cthp cacc acc -m stop-and-go       # range lost, regen and comfort per controller
cacc history save results/s/report.json --label v2 && cacc history diff   # regressions between versions
cacc hil --udp 192.168.1.20:5005 --realtime       # hardware in the loop (Raspberry Pi / Arduino / CAN)
cacc lab --preset cacc -o my.yaml && cacc lab my.yaml   # design a controller from transfer functions
cacc tune my.yaml -o tuned.yaml                   # auto-tune it: shortest headway that passes
cacc stress -c tuned.yaml -o results/s            # ...stress-test the design
cacc explain results/s/report.json                # why each failed cell failed + verified fixes
cacc envelope -c tuned.yaml --maneuver hard-brake # where exactly it breaks, per stress axis
cacc field-study --fetch microsim                 # how real production ACC cars behave
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
| **Stress campaigns** — one controller against a maneuver × condition grid (emergency stop, stop-and-go, traffic wave… × latency, loss, spoofing, wet/snow road, derated battery, slow brakes); CACC Studio's *Stress test* page shows it as a heat map, replays any cell as telemetry and diffs against a baseline run | `src/cacc/campaign.py`, `src/cacc/gui/page_stress.py` |
| **3-D replay** — every stress cell replays as a 3-D highway scene (drone / chase / side cameras): brake lights, V2V links (red when the trust gate distrusts them), cars cutting in from the next lane, a collision flash; the trace plots follow the replay | `src/cacc/gui/view3d.py` |
| **Why it failed** — for a failed cell: who crashed, when, at what speed; braking needed vs what grip and motor allowed; reaction delay; spoofed / lost / missing V2V; analytic \|Γ\| with the cell's delay; then candidate fixes re-simulated on the same seed (`cacc explain`) | `src/cacc/explain.py` |
| **Breaking point & safety envelope** — drives each axis (latency, loss, grip, motor power, dead time, spoofing, cut-in speed, lead braking) from benign to harsh and bisects to where the controller first fails; or maps two axes against each other with the breaking boundary (`cacc envelope`) | `src/cacc/envelope.py` |
| **Auto-tuner** — searches the numbers in a Lab design: analytic screen (stable poles, \|Γ\| ≤ 1 at every V2V delay), then the key stress cells; returns the shortest headway that passes, same block structure (`cacc tune`, Lab *Auto-tune*) | `src/cacc/autotune.py` |
| **Emergency braking (AEB)** — a radar-only safety layer under any controller: compares the braking the situation needs with what grip and brakes can still give, latches until the threat is gone; never touches a calm run (`--aeb`, Studio *+ AEB*); the explainer offers it as a fix | `src/cacc/aeb.py` |
| **Sensor & powertrain faults** — frozen, biased or noisy radar, V2V blackout, brake fade, motor derate, a dragging brake, per car and time window; seven new grid conditions | `src/cacc/faults.py` |
| **Crash-database scenarios** — NHTSA pre-crash typology (lead vehicle stopped / decelerating / slower, lane change; the stopped car revealed by a lane change) and Euro NCAP AEB Car-to-Car (CCRs, CCRm, CCRb); `--preset crash-db` | `src/cacc/crashlib.py` |
| **Standards checklist** — pre-assessment against ISO 15622 comfort limits, ISO 20035 string stability / V2V loss, UN R155 spoofing, UNECE R157 and Euro NCAP scenarios, ISO 26262-style fault reaction; each row with the run that proves it (`cacc standards`, Studio *Standards* page). Requirements are paraphrased: not a certification | `src/cacc/standards.py` |
| **Energy & comfort score** — kWh/100 km and range per car, range lost against the lead car, regen vs friction braking, RMS jerk and time beyond the ISO 15622 comfort limits; `range_loss_pct` / `rms_jerk` / `harsh_share` criteria (`cacc energy`, Studio tab) | `src/cacc/energy.py` |
| **Version history** — save each run as a version, diff any two cell by cell (regression / fixed / worse / better), exit 1 on a regression for CI (`cacc history`, Studio *Versions* tab) | `src/cacc/history.py` |
| **Hardware in the loop** — the simulated platoon drives a controller on a Raspberry Pi (UDP), an Arduino (serial) or a CAN ECU, sampled with zero-order hold, optionally paced to real time; round-trip times, late and missed replies, and the drift from the same law in software (`cacc hil`, `cacc hil-device`; stdlib-only device file + Arduino sketch) | `src/cacc/adapters/hil.py`, `hil_device.py`, `examples/hil/` |
| **Mixed traffic** — human drivers (IDM + CAH, reaction time; nobody behind them gets V2V) and cars cutting in mid-maneuver; gap / TTC metrics see the real car in front | `src/cacc/traffic.py` |
| **Field study of real cars** — model-free amplification, undershoot and gaps for every follower of a folder of logs (OpenACC, microSIM-ACC, generic) plus the twin's verdict | `src/cacc/fieldstudy.py` |
| **Controller Lab** — build a controller from transfer functions in s (one per measured signal: spacing error, its rate, relative speed, V2V feedforward); live \|Γ(jω)\|, closed-loop poles and the equations being solved, plus a maneuver replay that splits the command into each block's contribution and shows where grip / motor power clipped it; export to MATLAB, send to the stress grid | `src/cacc/tfctrl.py`, `src/cacc/gui/page_lab.py` |
| **Integrations** — controllers exported as FMI 2.0 FMUs (Simulink, Modelica, CarMaker) run in every plan and campaign; CARLA co-simulation logs runs for `cacc audit`; `cacc ros2-params` drives `ros2_ws` with any controller | `src/cacc/adapters/` |
| **GitHub Action** — `uses: Mannava-Daasaradhi/Adaptive-Cruise-Control@main` runs a plan or stress grid on every PR, comments the grid on the PR, writes the job summary and a status badge | `action.yml`, `src/cacc/ci.py` |
| **EV powertrain** — speed-dependent motor-power and tyre-grip limits, battery energy with regen and drafting, `energy_kwh_100km` / `regen_share` criteria | `src/cacc/ev.py` |
| **Black-box string stability** — multisine sweep measures per-hop \|Γ(jω)\| without a transfer function; matches theory to 1e-4 | `src/cacc/stringstab.py` |
| **Impairment-faithful V2V** — delay with jitter, independent or bursty (Gilbert–Elliott) loss with packet inter-reception stats, Ma-2025 value noise, time-varying/spatial channel quality, spoofing attacks + a physics-consistency gate checked against a modelled (late, noisy) radar | `src/cacc/network.py`, `trust.py`, `radar.py` |
| **Evidence** — `report.json` (per seed, provenance), `junit.xml`, `summary.md`, resolved `cases/*.yaml`; exit 0/1/2 | `src/cacc/reporting.py` |
| **Digital twin from drive logs** — reads JRC OpenACC or generic CSV logs, fits each car's ACC with and without actuation dead time, reports the time-gap margin ± uncertainty, a confidence level with warnings, a model-free cross-check and a V2V what-if | `src/cacc/twin.py`, `twin_checks.py`, `fielddata.py` |
| **Audit report** — one command from a drive log to a self-contained HTML report: verdict per car, recommended time gap, five-car brake test, V2V feedforward × latency grid with a latency budget, and a release-gate plan per car | `src/cacc/audit.py` |
| **CI template** — must-pass gate + must-reject canary; `cacc init` scaffolds the same for your repo | `.github/workflows/ci.yml`, `src/cacc/scaffold.py` |

User guide: [`docs/product/P-05`](docs/product/P-05-user-guide-evaluate-your-controller.md) ·
design records: [`D-025`](docs/decisions/D-025-product-core-bring-your-own-controller.md),
[`D-026`](docs/decisions/D-026-digital-twin-from-drive-logs.md),
[`D-027`](docs/decisions/D-027-string-stability-audit-deliverable.md) ·
go-to-market: [`docs/company/`](docs/company/GTM-01-go-to-market-plan.md).

## Install

**Windows:** `python scripts/build_exe.py` builds `CACC-Studio-<version>-setup.exe`
(per-user install, Start menu, no Python needed; needs Inno Setup) plus a
portable zip.

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

**Tests:** `python -m pytest -q` (276 tests). **Licence:** Apache-2.0.
**What changed after the 2026-10-08 review, and why:** [`D-028`](docs/decisions/D-028-realism-fixes-from-the-critique.md).
