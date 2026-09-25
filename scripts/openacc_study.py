"""State of commercial ACC string stability — batch twin study (D-026).

Calibrates a linear ACC twin for every ACC-engaged follower in every CSV of
the JRC OpenACC database (or any folder of logs in a supported format) and
aggregates the verdicts per vehicle make/model.

Get the data (CC BY 4.0, attribute "European Commission, Joint Research
Centre (JRC): OpenACC"): https://data.jrc.ec.europa.eu/dataset/9702c950-c80f-4d2f-982f-44d06ea0009f
Unzip it anywhere, then:

    python scripts/openacc_study.py path/to/OpenACC -j 4

Outputs in results/openacc_study/<stamp>/:
    hops.csv         one row per (file, follower): twin parameters, margin,
                     ||Gamma||, model-free cross-check, fit quality
    by_vehicle.csv   per make/model: logs, total minutes, share unstable,
                     median time-gap margin and ||Gamma||
    summary.md       the table above, ready to publish
"""

from __future__ import annotations

import argparse
import csv
import logging
import statistics
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime
from pathlib import Path

from cacc.fielddata import read_log
from cacc.twin import calibrate_log

log = logging.getLogger("cacc.scripts.openacc_study")

HOP_FIELDS = ["file", "campaign", "follower", "leader", "duration_s",
              "v_mean_mps", "k_s", "k_v", "T", "s0", "tau", "min_T",
              "margin_s", "margin_se_s", "hinf", "peak_omega", "verdict",
              "emp_peak_gain", "emp_coherence", "gap_rmse_m",
              "speed_rmse_mps", "v2v_hinf", "v2v_min_T"]


def study_file(path: str, root: str, min_duration: float) -> tuple[list, list]:
    """Worker: calibrate one file -> (rows, errors)."""
    try:
        lg = read_log(path)
        reports, skipped = calibrate_log(lg, min_duration)
    except Exception as exc:  # noqa: BLE001 — one bad file must not stop the study
        msg = str(exc)
        return [], [msg if path in msg else f"{path}: {msg}"]
    rel = Path(path).relative_to(root)
    rows = []
    for r in reports:
        d = r.to_dict()
        rows.append({
            "file": str(rel), "campaign": rel.parts[0] if len(rel.parts) > 1 else "",
            "follower": r.follower, "leader": r.leader,
            "duration_s": d["duration_s"], "v_mean_mps": d["v_mean_mps"],
            **d["params"], "min_T": d["min_string_stable_time_gap_s"],
            "margin_s": d["time_gap_margin_s"],
            "margin_se_s": d["time_gap_margin_se_s"], "hinf": d["hinf"],
            "peak_omega": d["peak_omega_rad_s"], "verdict": r.verdict,
            "emp_peak_gain": d["empirical"].get("peak_gain"),
            "emp_coherence": d["empirical"].get("coherence"),
            "gap_rmse_m": d["fit"]["gap_rmse_m"],
            "speed_rmse_mps": d["fit"]["speed_rmse_mps"],
            "v2v_hinf": d["what_if_v2v"]["hinf"],
            "v2v_min_T": d["what_if_v2v"]["min_string_stable_time_gap_s"],
        })
    return rows, [f"{path}: {m}" for m in skipped]


def aggregate(rows: list[dict]) -> list[dict]:
    by: dict[str, list[dict]] = {}
    for r in rows:
        by.setdefault(r["follower"], []).append(r)
    out = []
    for name, rs in sorted(by.items()):
        margins = [r["margin_s"] for r in rs if r["margin_s"] is not None]
        out.append({
            "vehicle": name, "hops": len(rs),
            "minutes": round(sum(r["duration_s"] for r in rs) / 60, 1),
            "unstable_share": round(sum(r["verdict"] == "string-unstable"
                                        for r in rs) / len(rs), 3),
            "median_T": round(statistics.median(r["T"] for r in rs), 3),
            "median_margin_s": (round(statistics.median(margins), 3)
                                if margins else None),
            "median_hinf": round(statistics.median(r["hinf"] for r in rs), 4),
            "median_v2v_hinf": round(statistics.median(r["v2v_hinf"]
                                                       for r in rs), 4),
        })
    return out


def write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def summary_md(agg: list[dict], n_files: int, n_hops: int,
               errors: list[str]) -> str:
    lines = ["# Commercial ACC string stability — twin study", "",
             f"{n_files} log files · {n_hops} calibrated follower segments · "
             "linear lag ACC twins (cacc.twin, D-026).", "",
             "| vehicle | hops | minutes | unstable share | median T [s] | "
             "median margin [s] | median ‖Γ‖∞ | with V2V (ka 0.5, 100 ms) |",
             "|---|---|---|---|---|---|---|---|"]
    for a in agg:
        lines.append(f"| {a['vehicle']} | {a['hops']} | {a['minutes']} | "
                     f"{a['unstable_share']:.0%} | {a['median_T']} | "
                     f"{a['median_margin_s']} | {a['median_hinf']} | "
                     f"{a['median_v2v_hinf']} |")
    lines += ["", "Margin = time gap in use minus the smallest string-stable "
              "time gap at the fitted gains (negative = string-unstable).",
              "Data: European Commission, Joint Research Centre (JRC), "
              "OpenACC (CC BY 4.0)."]
    if errors:
        lines += ["", f"<details><summary>{len(errors)} skipped items"
                  "</summary>", ""] + [f"- {e}" for e in errors] + [
                  "", "</details>"]
    return "\n".join(lines) + "\n"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("data", help="folder with OpenACC (or generic) CSV logs")
    ap.add_argument("-j", "--jobs", type=int, default=1)
    ap.add_argument("--min-duration", type=float, default=60.0)
    ap.add_argument("-o", "--out", default=None)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    root = Path(args.data).resolve()
    files = sorted(str(p) for p in root.rglob("*.csv"))
    if not files:
        raise SystemExit(f"no CSV files under {root}")
    log.info("%d files under %s", len(files), root)
    rows, errors = [], []
    if args.jobs > 1:
        with ProcessPoolExecutor(args.jobs) as pool:
            results = pool.map(study_file, files, [str(root)] * len(files),
                               [args.min_duration] * len(files))
            for rs, es in results:
                rows += rs
                errors += es
    else:
        for f in files:
            rs, es = study_file(f, str(root), args.min_duration)
            rows += rs
            errors += es
    out = Path(args.out) if args.out else (
        Path("results") / "openacc_study" / datetime.now().strftime("%Y%m%d-%H%M%S"))
    out.mkdir(parents=True, exist_ok=True)
    write_csv(out / "hops.csv", rows, HOP_FIELDS)
    agg = aggregate(rows)
    if agg:
        write_csv(out / "by_vehicle.csv", agg, list(agg[0]))
    (out / "summary.md").write_text(summary_md(agg, len(files), len(rows),
                                               errors), encoding="utf-8")
    log.info("%d follower segments calibrated, %d skipped -> %s", len(rows),
             len(errors), out)


if __name__ == "__main__":
    main()
