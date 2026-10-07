"""String-Stability Audit: one drive log in, a customer-ready report out (D-027).

``cacc audit LOG -o DIR`` wraps the digital-twin calibration (D-026) into the
deliverable of the paid pilot (GTM-01 §3):

- ``report.html`` — self-contained and printable: verdict per car, fitted
  parameters with uncertainty, fit quality, |Γ(jω)| with the model-free
  cross-check, and a V2V what-if grid (feedforward gain × latency → smallest
  string-stable time gap, and the latency budget at the car's own time gap),
  plus what is needed to reproduce every number;
- ``gate_<i>_<car>.yaml`` — a release-gate test plan per car (as-calibrated
  and at the recommended time gap), run with ``cacc evaluate``;
- ``twin_<i>_<car>.yaml`` and ``calibration.json`` — as ``cacc calibrate -o``.

Everything here is presentation and packaging: every number comes from
:mod:`cacc.twin`, whose accuracy is established in ``tests/test_twin.py``.
This module computes the numbers and writes the files; the HTML itself is
rendered by :mod:`cacc.audit_report`.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

import cacc
from cacc.fielddata import PlatoonLog, read_log
from cacc.twin import (TwinReport, _segment_data, calibrate_log,
                       simulate_follower)

#: V2V feedforward gains shown in the what-if grid (ka = 1 is never strictly
#: string-stable: |Γ| -> ka at high frequency)
KA_GRID = (0.3, 0.5, 0.8)
#: V2V latencies shown in the what-if grid [s]
LATENCY_GRID = (0.0, 0.05, 0.1, 0.2, 0.3)
#: search ceiling for the latency budget [s]
BUDGET_MAX = 1.0

# ----------------------------------------------------------------- numbers
def v2v_grid(model, kas=KA_GRID, latencies=LATENCY_GRID) -> list[dict]:
    """Smallest string-stable time gap for each (ka, latency) pair.

    ``min_time_gap_s`` is ``None`` when no time gap up to 10 s is stable.
    """
    rows = []
    for ka in kas:
        for th in latencies:
            t_min = model.min_stable_time_gap(ka, th)
            rows.append({"ka": ka, "delay_s": th,
                         "min_time_gap_s": None if t_min is None
                         else round(t_min, 3)})
    return rows


def latency_budget(model, ka: float, time_gap: float | None = None,
                   ceiling: float = BUDGET_MAX, scan: float = 0.01,
                   tol: float = 0.001) -> float | None:
    """Largest V2V latency [s] up to which feedforward ``ka`` keeps the car
    string-stable at ``time_gap`` (default: the gap it uses) — every
    latency from 0 to the budget is stable.

    ``None`` if not even a zero-latency link suffices; ``ceiling`` if the
    whole searched range does. Stability need not be monotonic in latency
    (the delayed feedforward's phase rotates), so the first failure is found
    by a ``scan``-step sweep and then refined by bisection; a stable window
    beyond the first failure is deliberately not counted. Rounded down to
    1 ms so the reported budget is never past the boundary.
    """
    T = model.T if time_gap is None else time_gap

    def ok(theta: float) -> bool:
        t_min = model.min_stable_time_gap(ka, theta)
        return t_min is not None and t_min <= T

    if not ok(0.0):
        return None
    lo = 0.0
    for hi in np.append(np.arange(scan, ceiling, scan), ceiling):
        if not ok(float(hi)):
            break
        lo = float(hi)
    else:
        return ceiling
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return math.floor(lo * 1000) / 1000


def recommended_time_gap(r: TwinReport, step: float = 0.1) -> float | None:
    """Smallest time gap, rounded up to ``step``, that clears the stability
    boundary by two standard errors; ``None`` if no gap up to 10 s is stable.

    When the alternative model structure fits about as well (see
    :mod:`cacc.twin_checks`) the larger of the two boundaries is used: the
    data cannot rule it out, so the recommendation must survive it.
    """
    if r.min_time_gap is None:
        return None
    t_min = r.min_time_gap
    alt = r.alternative or {}
    if alt.get("comparable"):
        a_tmin = alt.get("min_string_stable_time_gap_s")
        if a_tmin is None:
            return None
        t_min = max(t_min, a_tmin)
    se = r.margin_se if math.isfinite(r.margin_se) else 0.0
    return round(math.ceil((t_min + 2 * se) / step - 1e-9) * step, 3)


def road_test(model, v0: float, time_gap: float | None = None) -> dict:
    """What the verdict means on the road: a line of five cars tuned like
    this one, lead car braking at 3 m/s² for 3 s from ``v0``.

    Runs the twin's own scenario (as in its release gate) with the CTHP law;
    ``time_gap`` overrides the time gap in use.
    """
    from cacc.platoon import PlatoonSim
    from cacc.scenario import scenario_from_dict

    raw = model.scenario(v0, "road-test")
    if time_gap is not None:
        raw["controller"]["h"] = round(float(time_gap), 4)
    sc = scenario_from_dict(raw)
    res = PlatoonSim(sc.config, "cthp", sc.leader).run()
    gaps = np.diff(-res.pos, axis=1) - sc.config.vehicle.length  # (S, n)
    per_car = gaps.min(axis=0)
    k = int(np.argmin(per_car))
    lead = raw["leader"]
    return {"time_gap_s": raw["controller"]["h"], "cars": raw["platoon"][
                "n_followers"],
            "brake_mps2": -lead["decel"], "brake_s": lead["duration"],
            "v0_mps": raw["platoon"]["v0"],
            "min_gap_m": round(float(per_car[k]), 2), "min_gap_car": k + 1,
            "collision": bool(per_car[k] <= 0.0),
            "peak_decel_mps2": round(float(max(0.0, -res.acc[:, 1:].min())),
                                     2)}


@dataclass
class VehicleAudit:
    """Everything the report shows for one follower."""

    twin: TwinReport
    grid: list[dict]
    budgets: dict  # ka -> latency budget [s] at the time gap in use
    recommended_T: float | None
    trace: dict  # longest engaged segment: measured vs twin
    road: dict = field(default_factory=dict)  # 'as_calibrated' / 'recommended'

    @property
    def best_v2v(self) -> tuple[float, float] | None:
        """(ka, budget) with the largest latency budget, if any ka works."""
        ok = [(ka, b) for ka, b in self.budgets.items() if b is not None]
        return max(ok, key=lambda kb: kb[1]) if ok else None

    def to_dict(self) -> dict:
        best = self.best_v2v
        return {**self.twin.to_dict(),
                "recommended_time_gap_s": self.recommended_T,
                "v2v_grid": self.grid,
                "latency_budget_s": {str(k): v for k, v in self.budgets.items()},
                "road_test": self.road,
                "best_v2v": None if best is None else
                {"ka": best[0], "latency_budget_s": best[1]}}


@dataclass
class AuditResult:
    source: str
    log: PlatoonLog
    vehicles: list[VehicleAudit]
    skipped: list[str]
    meta: dict = field(default_factory=dict)

    @property
    def exit_code(self) -> int:
        """0 all stable/marginal, 1 any string-unstable, 2 nothing calibrated."""
        if not self.vehicles:
            return 2
        return 1 if any(v.twin.verdict == "string-unstable"
                        for v in self.vehicles) else 0


def _trace(log: PlatoonLog, r: TwinReport, min_duration: float) -> dict:
    segs = log.hop_segments(r.hop, min_duration)
    seg = max(segs, key=lambda s: s.stop - s.start)
    t, v_l, v_f, gap, x0 = _segment_data(log, r.hop, seg)
    g_sim, v_sim = simulate_follower(r.model, t, v_l, x0)
    return {"t": t, "gap": gap, "gap_twin": g_sim, "speed": v_f,
            "speed_twin": v_sim, "speed_lead": v_l}


def sha256(path: str | Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def audit_log(path: str | Path, min_duration: float = 30.0,
              v2v_ka: float = 0.5, v2v_delay: float = 0.1) -> AuditResult:
    """Calibrate every follower of the log at ``path`` and add the what-ifs."""
    log = read_log(path)
    twins, skipped = calibrate_log(log, min_duration, v2v_ka=v2v_ka,
                                   v2v_delay=v2v_delay)
    vehicles = []
    for r in twins:
        rec = recommended_time_gap(r)
        road = {"as_calibrated": road_test(r.model, r.v_mean)}
        if rec is not None and rec > r.model.T:
            road["recommended"] = road_test(r.model, r.v_mean, rec)
        vehicles.append(VehicleAudit(
            twin=r, grid=v2v_grid(r.model),
            budgets={ka: latency_budget(r.model, ka) for ka in KA_GRID},
            recommended_T=rec, trace=_trace(log, r, min_duration), road=road))
    meta = {"tool": "cacc", "version": cacc.__version__,
            "created_utc": datetime.now(timezone.utc).isoformat(
                timespec="seconds"),
            "log_file": Path(path).name, "log_sha256": sha256(path),
            "min_duration_s": min_duration,
            "what_if": {"ka": v2v_ka, "delay_s": v2v_delay}}
    return AuditResult(str(path), log, vehicles, skipped, meta)


# ------------------------------------------------------------------- files
def _car_file_stem(r: TwinReport) -> str:
    return f"{r.hop}_" + "".join(c if c.isalnum() else "_" for c in r.follower)


def write_twins(reports: list[TwinReport], out: Path) -> list[Path]:
    """One CTHP scenario per twin (``cacc sweep twin.yaml -c cthp``)."""
    out.mkdir(parents=True, exist_ok=True)
    paths = []
    for r in reports:
        desc = (f"Linear ACC twin of {r.follower} calibrated from "
                f"{Path(r.source).name} (verdict: {r.verdict}).")
        p = out / f"twin_{_car_file_stem(r)}.yaml"
        p.write_text(yaml.safe_dump(r.model.scenario(
            r.v_mean, f"twin-{_car_file_stem(r)}", desc), sort_keys=False),
            encoding="utf-8")
        paths.append(p)
    return paths


def release_gate(v: VehicleAudit, twin_file: str) -> dict:
    """A test plan the customer keeps: the car as calibrated and at the
    recommended time gap, under a hard brake and the multisine sweep."""
    r = v.twin
    cases = [{"name": "as-calibrated"}]
    if v.recommended_T is not None and v.recommended_T > r.model.T:
        cases.append({"name": "recommended-time-gap",
                      "set": {"controller.h": v.recommended_T}})
    return {
        "name": f"release-gate-{_car_file_stem(r)}",
        "description": (
            f"String-stability gate for {r.follower}, generated by cacc audit "
            f"from {Path(r.source).name}. The twin runs at T = "
            f"{r.model.T:.2f} s (verdict: {r.verdict}). Re-run `cacc audit` "
            "on a log of a new tuning to regenerate the twin, then this "
            "gate."),
        "base": twin_file,
        "under_test": {"controller": "cthp"},
        "seeds": 1,
        "cases": cases,
        "criteria": ["min_gap >= 2.0"],
        "string_stability": {"max_gain": 1.0, "sweep": {"n_followers": 3}},
    }


def write_audit(res: AuditResult, out: str | Path) -> dict[str, Path]:
    """Write report.html, calibration.json, twins and gates into ``out``."""
    out = Path(out)
    twins = write_twins([v.twin for v in res.vehicles], out)
    paths = {"report": out / "report.html",
             "calibration": out / "calibration.json"}
    for v, twin_path in zip(res.vehicles, twins, strict=True):
        gate = out / f"gate_{_car_file_stem(v.twin)}.yaml"
        gate.write_text(yaml.safe_dump(release_gate(v, twin_path.name),
                                       sort_keys=False), encoding="utf-8")
        key = v.twin.follower
        if f"gate:{key}" in paths:  # two cars with the same name in one log
            key = f"{v.twin.hop}:{key}"
        paths[f"gate:{key}"] = gate
        paths[f"twin:{key}"] = twin_path
    paths["calibration"].write_text(json.dumps(
        {"meta": res.meta, "source": res.source,
         "followers": [v.to_dict() for v in res.vehicles],
         "skipped": res.skipped}, indent=2, default=float), encoding="utf-8")
    paths["report"].write_text(render_html(res), encoding="utf-8")
    return paths


def render_html(res: AuditResult) -> str:
    """The self-contained HTML report (rendered by :mod:`cacc.audit_report`)."""
    from cacc.audit_report import render_html as _render

    return _render(res)

