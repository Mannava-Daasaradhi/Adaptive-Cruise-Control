# GTM-03 — The 5-minute demo

> One story in four steps: *a maneuver test passes a bad car → the engine
> catches it → one drive log is enough → V2V shows the fix.* Every command
> below runs from a fresh checkout (`pip install -e .`) in under a minute on
> a laptop.

## Step 1 — "Your maneuver tests would pass this car" (60 s)

```bash
cacc evaluate examples/plans/idm_acc.yaml -j 4
```

Point at the table: every time-domain check (min gap, TTC, L2 growth)
**passes**, while the frequency sweep reports **peak |Γ| = 1.038 → FAIL**.
Say: *"This is an IDM-style ACC with a commercial-like tuning. Every
maneuver looks fine; it still amplifies slow traffic waves by ~4 % per car.
That's the JRC OpenACC finding in miniature."*

## Step 2 — "One drive log is enough" (90 s)

```bash
cacc audit examples/data/synthetic_openacc_platoon.csv -o audit/   # ~12 s
```

Open `audit/report.html`. Point at the summary: *"2 of 3 cars amplify
disturbances; in a line of five, SedanA's tuning ends in a collision under
a 3 m/s² brake."* Then SedanA's page: *"runs at 1.2 s, needs ≥ 2.95 s,
margin −1.76 s with uncertainty; the model-free estimate from the raw data
agrees (1.29 vs the twin's 1.31); at the recommended 3.0 s the same brake
leaves 48 m."* Say: *"This is the file format of the JRC OpenACC database —
the same command runs on real production cars, and this report is what the
audit delivers."*

## Step 3 — "What would V2V change?" (60 s)

Same report, the V2V grid: SUV-B keeps its 1.5 s gap string-stable with
feedforward ka = 0.5 up to **824 ms** of latency; SedanA needs ka = 0.8 and
then only tolerates **47 ms**; stronger feedforward is not always better
(the EV needs *more* gap at ka = 0.8). Say: *"This is the controller-level
evidence V2X programs are missing: the latency budget, per car."*

## Step 4 — "And it gates your CI" (60 s)

```bash
cacc init my-acc && cd my-acc && cacc evaluate plans/release_gate.yaml -j 4
```

Show `results/gate/summary.md`, `junit.xml`, and the GitHub workflow the
scaffold created: *"Every controller change gets a verdict and the
evidence is archived — worst seed, exact scenario, git commit."*

## Close (30 s)

*"We'd like to run this on two of your vehicles as a 2–4 week audit — you
send logs, you get this report for your cars — twins, margins, the V2V
latency budget — and you keep the release-gate test plans."* Then ask interview questions 10–11 from GTM-02.

## Backup answers

- **"Your model is linear."** The twin is the small-signal model used in the
  field-data literature; its verdict is checked against a model-free
  estimate from the same data, and validated on a nonlinear (IDM) platoon
  (`tests/test_twin.py`).
- **"Why trust the simulator?"** It reproduces a 2025 IEEE T-ITS paper to
  its printed digits and three independent backends agree (Python, ROS 2,
  Simulink).
- **"We use CarMaker / dSPACE."** Keep them; this is the evaluation layer
  beside them. FMU import is on the roadmap for exactly that.
