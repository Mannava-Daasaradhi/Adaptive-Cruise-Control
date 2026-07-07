# RB-02 — Offline workflows (the daily loop)

All commands: repo root, conda env `cacc` (prefix with
`conda run -n cacc` or activate once).

## The gate (run after EVERY code change)

    python -m pytest tests -q                       # 55 passed, ~25 s

If anything touched analysis/controllers/network:

    python scripts/reproduce_base_paper.py --quick  # theory_checks intact

## Running experiments

    python scripts/run_scenario.py scenarios/ma2025_sine.yaml
    python scripts/qos_adaptive_study.py            # figs 1-4, ~4 min
    python scripts/qos_adaptive_study.py --quick    # smoke, ~1 min
    python scripts/certify.py --seeds 5             # the certification
    python scripts/certify.py --quick --seeds 2     # CI-grade smoke

Outputs: `results/<family>/<timestamp>/…` — timestamped, never
overwritten; every quotable number lands in a `metrics.json`.

## Interactive exploration pattern

The API is designed for notebooks/scratch scripts:

    import dataclasses, numpy as np
    from cacc import PlatoonSim, load_scenario
    sc = load_scenario("scenarios/qos_adaptive.yaml")
    cfg = dataclasses.replace(sc.config, n_followers=4, seed=7)
    res = PlatoonSim(cfg, "cthp", sc.leader).run()
    # res.t, res.err, res.h, res.rho_hat, res.gains ...

Vary configs ONLY via `dataclasses.replace` (frozen configs are the
provenance record of a run).

## Rendering a video from an offline run

    python scripts/render_platoon_video.py scenarios/qos_adaptive.yaml
    # -> results/videos/qos_adaptive.mp4 (10x time-lapse)
    python scripts/render_platoon_video.py <input> --frames 4   # quick check

## Adding an experiment (checklist)

1. Scenario YAML (R-11) or a `dataclasses.replace` variant in a study
   script — prefer YAML if it will be reused.
2. Metrics into `metrics.json`; acceptance thresholds → a suite in
   `certify.py` if it certifies anything (D-018 pattern).
3. Figures follow the house style (constrained sizes, labeled ‖H̃‖∞
   where relevant, axis units).
4. A doc in `docs/experiments/E-XX-…` recording design + expected
   numbers + interpretation caveats.
5. Tests pinning any new closed-form/algorithmic behavior.

## Performance expectations (calibrate surprise)

8-follower 240 s adaptive run ≈ 5 s; the qos study ≈ 4 min; full
certification (5 seeds × 4 suites) ≈ 4 min; reproduce_base_paper ≈ 2 min.
An order-of-magnitude regression means an accidental per-stage
allocation or a per-follower table rebuild — both happened once.
