# Examples — evaluate a controller with `cacc`

| file | what it shows |
|---|---|
| `plans/smoke.yaml` | CI release gate for the built-in CTHP design (case A) — **PASS** |
| `plans/cthp_case_a.yaml` | Latency × maneuver matrix on the same design — safe everywhere, *marginal* string-stability FAIL at 200 ms |
| `plans/idm_acc.yaml` | Bring-your-own-controller: an IDM-style commercial ACC as a plugin file — **FAIL** (string-unstable, as field studies found for real ACCs) |
| `controllers/idm_acc.py` | The plugin: a nonlinear, radar-only law using `raw_inputs` and `equilibrium_gap` |
| `data/synthetic_openacc_platoon.csv` | A drive log in the JRC OpenACC layout (seeded, synthetic: three ACC tunings behind a human leader, one driver takeover) — `cacc calibrate` it; regenerate with `scripts/make_synthetic_openacc.py` |

```bash
pip install -e .
cacc evaluate examples/plans/smoke.yaml -j 2
cacc evaluate examples/plans/idm_acc.yaml -j 2
cacc calibrate examples/data/synthetic_openacc_platoon.csv -o twins/
```

Guide: `docs/product/P-05-user-guide-evaluate-your-controller.md`.
