"""Field study: how real production ACC cars behave, from a folder of logs.

For every follower of every log this measures, **model-free**, what a
string-stability claim is ultimately about:

* ``decel_amplification`` — the follower's hardest braking divided by the
  hardest braking of the car it follows (> 1: the car amplified the
  disturbance it was given, the signature of string instability);
* ``speed_undershoot`` — how far below the leader's lowest speed the follower
  dipped [m/s] (an overreaction);
* ``min_gap`` / ``min_time_gap`` — how close it got.

Speeds are smoothed over ``smooth_s`` before differentiating (production
OBD speeds are quantized to 1 km/h). The fitted digital twin
(:mod:`cacc.twin`) is reported next to it with its confidence; on short
single-maneuver runs the model-free numbers are the headline, because a
twin fitted to one braking event is poorly identified.

``fetch_microsim`` downloads the FAU microSIM-ACC "ICE" two-car runs
(github.com/microSIM-ACC/ICE) so a study runs on real cars with one command:
``cacc field-study --fetch microsim``.
"""

from __future__ import annotations

import csv
import io
import statistics
import urllib.request
import zipfile
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from cacc.fielddata import PlatoonLog, read_log

MICROSIM_URL = ("https://github.com/microSIM-ACC/ICE/raw/main/"
                "2-Vehicle%20ACC%20Car%20Following%20Experiments%20"
                "(CCF,%20Same%20Desired%20Speed).zip")
MIN_LEAD_DECEL = 0.5  # [m/s^2] runs where the leader barely braked are skipped
MOVING = 5.0  # [m/s] speed above which the time gap is measured

FIELDS = ["file", "group", "maneuver", "follower", "duration_s", "v_start_mps",
          "lead_peak_decel", "peak_decel", "decel_amplification",
          "speed_undershoot", "min_gap", "min_time_gap", "twin_verdict",
          "twin_T", "twin_hinf", "twin_confidence"]


def _smooth(x: np.ndarray, n: int) -> np.ndarray:
    if n <= 1:
        return x
    k = np.ones(n) / n
    pad = np.pad(x, (n // 2, n - 1 - n // 2), mode="edge")
    return np.convolve(pad, k, mode="valid")


def model_free(log: PlatoonLog, i: int, smooth_s: float = 1.0) -> dict | None:
    """Model-free response numbers of follower ``i`` (1-based), or None when
    the leader did not brake enough to test anything."""
    ok = (np.isfinite(log.speed[:, i - 1]) & np.isfinite(log.speed[:, i])
          & np.isfinite(log.gap[:, i - 1]))
    if ok.sum() < 20:
        return None
    t = log.t[ok]
    n = max(1, int(round(smooth_s / log.dt)))
    vl = _smooth(log.speed[ok, i - 1], n)
    vf = _smooth(log.speed[ok, i], n)
    gap = log.gap[ok, i - 1]
    al, af = np.gradient(vl, t), np.gradient(vf, t)
    lead_dec = float(max(0.0, -al.min()))
    if lead_dec < MIN_LEAD_DECEL:
        return None
    dec = float(max(0.0, -af.min()))
    moving = vf > MOVING
    return {"duration_s": round(float(t[-1] - t[0]), 1),
            "v_start_mps": round(float(vf[0]), 2),
            "lead_peak_decel": round(lead_dec, 3), "peak_decel": round(dec, 3),
            "decel_amplification": round(dec / lead_dec, 3),
            "speed_undershoot": round(float(max(0.0, vl.min() - vf.min())), 3),
            "min_gap": round(float(gap.min()), 2),
            "min_time_gap": (round(float((gap[moving] / vf[moving]).min()), 3)
                             if moving.any() else None)}


def study_file(path: str, root: str, twin: bool = True) -> tuple[list, list]:
    """Worker: one log -> (rows, problems)."""
    try:
        log = read_log(path)
    except Exception as exc:  # noqa: BLE001 — one bad file must not stop a study
        return [], [f"{path}: {exc}"]
    rel = Path(path).resolve().relative_to(Path(root).resolve())
    rows, problems = [], []
    for i in range(1, log.n_vehicles):
        mf = model_free(log, i)
        if mf is None:
            problems.append(f"{rel}: follower {i}: leader did not brake")
            continue
        row = {"file": str(rel), "group": log.names[i],
               "maneuver": (f"{log.meta['from_mph']}→{log.meta['to_mph']} mph"
                            if "from_mph" in log.meta else ""),
               "follower": i, **mf, "twin_verdict": "", "twin_T": None,
               "twin_hinf": None, "twin_confidence": ""}
        if twin:
            from cacc.twin import calibrate_hop

            try:
                d = calibrate_hop(log, i, min_duration=min(20.0, mf["duration_s"]))
                dd = d.to_dict()
                row.update(twin_verdict=d.verdict, twin_T=dd["params"]["T"],
                           twin_hinf=dd["hinf"],
                           twin_confidence=dd.get("confidence", ""))
            except ValueError as exc:
                problems.append(f"{rel}: twin: {exc}")
        rows.append(row)
    return rows, problems


def study(folder, jobs: int = 1, twin: bool = True, progress=None
          ) -> tuple[list[dict], list[str]]:
    """Every CSV under ``folder`` -> (rows, problems)."""
    root = Path(folder)
    files = sorted(str(p) for p in root.rglob("*.csv"))
    if not files:
        raise ValueError(f"no CSV logs under {root}")
    rows, problems = [], []
    tick = progress or (lambda done, total: None)
    if jobs > 1:
        with ProcessPoolExecutor(jobs) as pool:
            for k, (rs, ps) in enumerate(pool.map(
                    study_file, files, [str(root)] * len(files),
                    [twin] * len(files), chunksize=4)):
                rows += rs
                problems += ps
                tick(k + 1, len(files))
    else:
        for k, f in enumerate(files):
            rs, ps = study_file(f, str(root), twin)
            rows += rs
            problems += ps
            tick(k + 1, len(files))
    return rows, problems


def aggregate(rows: list[dict]) -> list[dict]:
    """Per group (vehicle or ACC setting): share of runs that amplified."""
    by: dict[str, list[dict]] = {}
    for r in rows:
        by.setdefault(r["group"], []).append(r)
    by["all runs"] = list(rows)
    out = []
    for name, rs in by.items():
        amp = [r["decel_amplification"] for r in rs]
        tg = [r["min_time_gap"] for r in rs if r["min_time_gap"] is not None]
        out.append({
            "group": name, "runs": len(rs),
            "amplified_share": round(sum(a > 1.0 for a in amp) / len(rs), 3),
            "median_amplification": round(statistics.median(amp), 3),
            "worst_amplification": round(max(amp), 3),
            "median_undershoot_mps": round(statistics.median(
                r["speed_undershoot"] for r in rs), 2),
            "min_gap_m": round(min(r["min_gap"] for r in rs), 2),
            "min_time_gap_s": round(min(tg), 2) if tg else None,
            "twin_unstable_share": round(sum(r["twin_verdict"] == "string-unstable"
                                             for r in rs) / len(rs), 3)})
    return sorted(out, key=lambda a: (a["group"] == "all runs", a["group"]))


def summary_md(agg: list[dict], source: str, n_problems: int) -> str:
    lines = [f"# Field study: {source}", "",
             "| group | runs | braked harder than the car ahead | median "
             "amplification | worst | median speed undershoot [m/s] | min gap "
             "[m] | min time gap [s] | twin says unstable |",
             "|---|---|---|---|---|---|---|---|---|"]
    for a in agg:
        lines.append(
            f"| {a['group']} | {a['runs']} | {a['amplified_share']:.0%} | "
            f"{a['median_amplification']} | {a['worst_amplification']} | "
            f"{a['median_undershoot_mps']} | {a['min_gap_m']} | "
            f"{a['min_time_gap_s']} | {a['twin_unstable_share']:.0%} |")
    lines += ["", "Amplification = follower's peak deceleration / leader's "
              "peak deceleration, from speeds smoothed over 1 s (> 1 = the "
              "disturbance grew). The twin column is the fitted linear ACC "
              "model's verdict; on single-maneuver runs its parameters are "
              "poorly identified, so read it as secondary.",
              "Gaps are only as good as the log's spacing signal: where a "
              "dataset reconstructs spacing by integrating speeds (microSIM-"
              "ACC), absolute gaps drift; amplification uses speeds only.",
              f"{n_problems} follower segments skipped (leader did not brake, "
              "or unreadable)."]
    return "\n".join(lines) + "\n"


def write(rows, agg, problems, out, source: str) -> Path:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "runs.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    with open(out / "by_group.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(agg[0]))
        w.writeheader()
        w.writerows(agg)
    (out / "summary.md").write_text(summary_md(agg, source, len(problems)),
                                    encoding="utf-8")
    return out / "summary.md"


def fetch_microsim(dest, url: str = MICROSIM_URL, opener=urllib.request.urlopen
                   ) -> Path:
    """Download and unzip the microSIM-ACC two-car runs into ``dest``; keeps
    only the files with a bumper-gap column (the ones :func:`read_log` reads)."""
    dest = Path(dest)
    dest.mkdir(parents=True, exist_ok=True)
    with opener(url, timeout=300) as r:
        data = r.read()
    kept = 0
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        for name in z.namelist():
            if not name.lower().endswith(".csv"):
                continue
            raw = z.read(name)
            head = raw.split(b"\n", 1)[0].decode("utf-8", "replace").split(",")
            if "spacing" in head and "speed.1" in head:
                (dest / Path(name).name).write_bytes(raw)
                kept += 1
    if not kept:
        raise ValueError(f"{url}: no microSIM-ACC runs with a spacing column")
    return dest


def headline(agg: list[dict]) -> str:
    a = next(x for x in agg if x["group"] == "all runs")
    return (f"{a['runs']} real runs: the ACC car braked harder than the car "
            f"ahead in {a['amplified_share']:.0%} of them (median "
            f"{a['median_amplification']:.2f}x, worst "
            f"{a['worst_amplification']:.2f}x)")
