"""``cacc init``: scaffold a controller-evaluation project (D-026).

Creates a working starting point — a controller plugin template, a release
gate test plan, a CI workflow and a README — so a new user's first
``cacc evaluate`` is green within minutes and the only remaining step is to
replace the template law with their own.
"""

from __future__ import annotations

from pathlib import Path

CONTROLLER = '''"""My longitudinal controller (replace the law in ``output``).

Contract: cacc.plugins.LongitudinalController — see the cacc user guide
(docs/product/P-05). The simulator saturates the returned command to the
vehicle's limits and applies its actuator lag.
"""

import numpy as np


class MyController:
    n_states = 0        # internal states integrated by the simulator
    uses_v2v = False    # True: u_ff carries the predecessor's V2V message
    ff_signal = "a"     # V2V payload: "a" realized / "u" commanded accel
    raw_inputs = True   # receive gap, v, a, t as keyword arguments

    def __init__(self, params, k_gap=0.08, k_speed=0.55, time_gap=2.2,
                 standstill=5.0):
        self.k_gap, self.k_speed = k_gap, k_speed
        self.time_gap, self.standstill = time_gap, standstill

    def equilibrium_gap(self, v):
        return self.standstill + self.time_gap * v

    def output(self, xc, e, e_dot, u_ff, dv=0.0, *, gap, v, a, t):
        # TODO: replace with your control law (commanded accel, m/s^2)
        gap_error = gap - self.equilibrium_gap(v)
        return self.k_gap * gap_error + self.k_speed * dv

    def deriv(self, xc, e, e_dot, u_ff, dv=0.0, **raw):
        return np.empty(0)
'''

PLAN = '''# Release gate for {name}: run with  cacc evaluate plans/release_gate.yaml -j 4
name: {name}-release-gate
description: >
  Safety, comfort and string stability of {name} across actuator lag and
  leader maneuvers. Edit the matrix and thresholds to match your spec.
scenario:
  platoon: {{n_followers: 5, v0: 25.0}}
  vehicle: {{tau: 0.5, length: 4.5, u_min: -8.0, u_max: 2.5}}
  controller: {{h: 2.2, r: 5.0}}   # spacing policy used for e / e_dot only
  network: {{delay: 0.0}}
  leader: {{profile: sine, t_start: 10.0, amplitude: 1.0, freq_hz: 0.05,
           duration: 40.0}}
  sim: {{dt: 0.01, t_final: 90.0}}
under_test:
  controller: ../controllers/my_controller.py:MyController
  options: {{}}
seeds: 2
cases:
  - name: sine
  - name: hard-brake
    set:
      leader: {{profile: brake, t_start: 10.0, duration: 3.0, decel: -5.0}}
matrix:
  vehicle.tau: [0.3, 0.5, 0.7]
criteria:
  - "min_gap >= 2.0"
  - "min_ttc >= 2.0"
  - "peak_decel <= 6.0"
string_stability:
  max_gain: 1.0
'''

WORKFLOW = '''name: controller-gate
on: [push, pull_request]
jobs:
  gate:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with: {python-version: "3.12"}
      - run: pip install "cacc @ git+https://github.com/Mannava-Daasaradhi/Adaptive-Cruise-Control"
      - run: cacc evaluate plans/release_gate.yaml -j 2 -o results/gate
      - if: always()
        run: cat results/gate/summary.md >> "$GITHUB_STEP_SUMMARY"
      - uses: actions/upload-artifact@v4
        if: always()
        with: {name: controller-evidence, path: results/gate/}
'''

README = '''# {name} — controller evaluation

```bash
cacc evaluate plans/release_gate.yaml -j 4     # verdict + reports in results/
cacc sweep <scenario.yaml> -c controllers/my_controller.py:MyController
cacc calibrate <drive_log.csv> -o twins/       # digital twin of a recorded ACC
```

1. Put your control law in `controllers/my_controller.py` (keep the
   attributes; `output` returns the commanded acceleration).
2. Tune `plans/release_gate.yaml` to your spec: maneuvers (`cases`),
   conditions (`matrix`), seeds, thresholds (`criteria`).
3. Commit — `.github/workflows/controller-gate.yml` gates every push and
   attaches the evidence (report.json, junit.xml, summary.md).
'''


def scaffold(directory: str | Path, name: str | None = None,
             force: bool = False) -> list[Path]:
    """Write the project skeleton into ``directory``; return created files."""
    root = Path(directory)
    name = name or root.resolve().name or "my-controller"
    files = {
        root / "controllers" / "my_controller.py": CONTROLLER,
        root / "plans" / "release_gate.yaml": PLAN.format(name=name),
        root / ".github" / "workflows" / "controller-gate.yml": WORKFLOW,
        root / "README.md": README.format(name=name),
    }
    clash = [p for p in files if p.exists()]
    if clash and not force:
        raise FileExistsError("refusing to overwrite " + ", ".join(
            str(p) for p in clash) + " (use --force)")
    for path, text in files.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    return list(files)
