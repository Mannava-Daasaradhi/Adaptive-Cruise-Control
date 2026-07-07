# R-08 — `scripts/` reference: every entry point and its outputs

All run from the repo root in the `cacc` conda env unless marked WSL.

## Analysis / studies

| script | purpose | outputs |
|---|---|---|
| `reproduce_base_paper.py [--quick]` | full Ma-2025 reproduction + D-008 delay/loss extension | `results/base_paper/<stamp>/fig1…fig5 + metrics.json` (incl. `theory_checks`) |
| `qos_adaptive_study.py [--quick]` | D-016/D-017 evidence: theory curves, zone experiment, delay 2×2, gain re-tuning | `results/qos_adaptive/<stamp>/fig1…fig4 + metrics.json` |
| `certify.py [--seeds N] [--quick]` | D-018 certification suite | `results/certification/<stamp>/report.html + metrics.json` |
| `run_scenario.py <yaml>` | single scenario run + standard figures | `results/<name>/<stamp>/` |
| `sweep_delay.py` | Ploeg-family delay sweep (legacy v0.1 study) | `results/delay_sweep/…` |

## ROS-run analysis (Windows side)

| script | purpose |
|---|---|
| `plot_ros2_run.py <csv> [--baseline-t 8]` | cross-validation figures + metrics for a recorder CSV; detrends startup, trims the non-co-observed tail, three-way verdict (attenuating / neutral-noise / AMPLIFYING) |
| `render_platoon_video.py <csv\|yaml> [--speedup 10] [--fps 30] [--frames N]` | top-down highway MP4 (ffmpeg; GIF fallback) from a ROS CSV *or* an offline scenario; `--frames` dumps check PNGs |

## WSL drivers (bash; see RB-03 for invocation pattern)

| script | purpose |
|---|---|
| `ros2_sync_build.sh` | rsync repo → `~/cacc_ws` (sources + `pysrc/cacc`), CR-strip, colcon build |
| `ros2_run_demo.sh [N] [DUR] [CTRL] [YAML]` | timed SIGINT launch; copies CSV to `results/ros2/<stamp>_<ctrl>_n<N>.csv` |
| `ros2_view_demo.sh [N] [CTRL] [YAML]` | live run + rviz2 (WSLg); close rviz to stop |

## Conventions every new script must follow

1. `main()` + argparse with `description=__doc__`; module docstring shows
   a usage line.
2. `setup_logging()` first; log the output directory at the end.
3. Outputs under `results/<family>/<timestamp>/`; never overwrite;
   `metrics.json` for anything a table will quote.
4. matplotlib `Agg` backend before pyplot import (headless Windows).
5. `--quick` mode for anything slower than ~30 s (CI and smoke use it).
6. Numbers quoted in docs must be traceable to a `metrics.json` path.
