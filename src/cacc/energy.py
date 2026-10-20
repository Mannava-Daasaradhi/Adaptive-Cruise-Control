"""Energy and comfort score of a run: what a controller costs in range.

Safety is pass/fail; energy and comfort are what a driver notices every day.
:func:`score` reduces a run to per-car figures, the lead car included as
the reference (it drives the profile alone, no slipstream):

* **consumption** kWh/100 km and the **range** a full battery gives at it;
  it includes what it costs to get back to the starting speed, so a run
  that ends slower than it began is not credited with free energy;
* **range lost** against the lead car (steady cruise at the start speed
  when the "lead car" is a stopped obstacle), in km and % (negative = the
  follower goes further, because it drafts in the slipstream and does not
  over-brake);
* **regen**: braking energy sent back to the battery, and the energy burnt
  in the friction brakes (braking harder or more often than needed costs
  range twice: the brake heat and the re-acceleration);
* **comfort**: RMS and peak jerk, and the share of the run spent above the
  ISO 15622 ACC comfort limits (|jerk| > 2.5 m/s^3, braking beyond
  3.5 m/s^2, acceleration beyond 2 m/s^2), whatever the controller.

:func:`compare` runs one campaign cell for several controllers and ranks
them; ``cacc energy`` prints that table.
"""

from __future__ import annotations

import numpy as np

from cacc.ev import EVParams, _trapz, battery_power, wheel_power

#: ISO 15622 ACC comfort limits used for the "harsh" shares
JERK_LIMIT, DECEL_LIMIT, ACCEL_LIMIT = 2.5, 3.5, 2.0


def _jerk(res) -> np.ndarray:
    dt = float(res.t[1] - res.t[0])
    j = np.diff(res.acc, axis=0) / dt
    return np.vstack([j[:1], j])


def cruise_per100(ev: EVParams, v: float) -> float:
    """kWh/100 km of a solo car cruising steadily at ``v``."""
    v = max(v, 1.0)
    p = float(ev.resistance(v)) * v / ev.eta_drive + ev.aux_kw * 1e3
    return 100.0 * p / v * 1e3 / 3.6e6


def score(res) -> dict:
    """Per-car energy and comfort figures; index 0 is the lead car."""
    ev = res.config.vehicle.ev or EVParams()
    t, v, a = res.t, res.vel, res.acc
    gaps = -np.diff(res.pos, axis=1) - res.config.vehicle.length
    draft = np.column_stack([np.ones(t.size), ev.draft_scale(gaps)])
    p_batt = battery_power(ev, v, a, draft)
    kwh = _trapz(p_batt, t, axis=0) / 3.6e6
    # a car that ends slower than it started spent stored kinetic energy:
    # charge what it costs to get that speed back, so a stop is not "free"
    kwh += ev.mass * (v[0] ** 2 - v[-1] ** 2) / 2.0 / ev.eta_drive / 3.6e6
    km = _trapz(np.maximum(v, 0.0), t, axis=0) / 1e3
    p_wheel = wheel_power(ev, v, a, draft)
    braking = np.maximum(-p_wheel, 0.0)
    regen = np.minimum(braking, ev.regen_kw * 1e3)
    with np.errstate(invalid="ignore", divide="ignore"):
        per100 = np.where(km > 1e-3, 100.0 * kwh / km, np.nan)
        rng = np.where(per100 > 0, 100.0 * ev.battery_kwh / per100, np.inf)
        ref = per100[0] if np.isfinite(per100[0]) and per100[0] > 0 else cruise_per100(
            ev, float(res.config.v0))  # a stopped lead car: steady cruise instead
        lost_pct = 100.0 * (per100 / ref - 1.0)
        lost_km = 100.0 * ev.battery_kwh / ref - rng
    jerk = _jerk(res)
    return {
        "kwh_per_100km": per100,
        "range_km": rng,
        "range_lost_km": lost_km,
        "range_lost_pct": lost_pct,
        "regen_kwh": _trapz(regen, t, axis=0) * ev.eta_regen / 3.6e6,
        "friction_kwh": _trapz(braking - regen, t, axis=0) / 3.6e6,
        "rms_jerk": np.sqrt(np.mean(jerk**2, axis=0)),
        "peak_jerk": np.abs(jerk).max(axis=0),
        "harsh_share": np.mean((np.abs(jerk) > JERK_LIMIT) | (a < -DECEL_LIMIT)
                               | (a > ACCEL_LIMIT), axis=0),
    }


def range_loss_pct(res) -> float:
    """Worst follower's extra consumption over the lead car [%]."""
    return float(np.nanmax(score(res)["range_lost_pct"][1:]))


def rms_jerk(res) -> float:
    return float(score(res)["rms_jerk"][1:].max())


def harsh_share(res) -> float:
    return float(score(res)["harsh_share"][1:].max())


def summary(res) -> dict:
    """Platoon totals of :func:`score` (followers only)."""
    s = score(res)
    f = slice(1, None)
    return {"kwh_per_100km": float(np.nanmean(s["kwh_per_100km"][f])),
            "range_km": float(np.nanmean(s["range_km"][f])),
            "range_lost_pct": float(np.nanmax(s["range_lost_pct"][f])),
            "regen_kwh": float(s["regen_kwh"][f].sum()),
            "friction_kwh": float(s["friction_kwh"][f].sum()),
            "rms_jerk": float(s["rms_jerk"][f].max()),
            "harsh_share": float(s["harsh_share"][f].max())}


def compare(controllers, maneuver: str = "stop-and-go", condition: str = "nominal",
            aeb: bool = False, seed: int = 1) -> list[dict]:
    """:func:`summary` of one campaign cell for each controller, least
    energy first."""
    import yaml

    from cacc import campaign as cp

    base = yaml.safe_load(cp.BASE.read_text(encoding="utf-8"))
    rows = []
    for ctrl in controllers:
        tree = cp.compose(base, maneuver, condition, ctrl)
        if aeb:
            tree["aeb"] = {"enabled": True}
        res = cp.replay({"scenario": tree, "name": maneuver}, ctrl, seed)
        rows.append({"controller": ctrl, **summary(res)})
    return sorted(rows, key=lambda r: r["kwh_per_100km"])


def format_table(rows: list[dict]) -> str:
    head = (f"{'controller':<12}{'kWh/100km':>10}{'range km':>10}{'vs lead':>9}"
            f"{'regen kWh':>10}{'brakes kWh':>11}{'RMS jerk':>9}{'harsh':>7}")
    lines = [head, "-" * len(head)]
    for r in rows:
        lines.append(f"{r['controller']:<12}{r['kwh_per_100km']:>10.2f}"
                     f"{r['range_km']:>10.0f}{r['range_lost_pct']:>+8.1f}%"
                     f"{r['regen_kwh']:>10.3f}{r['friction_kwh']:>11.3f}"
                     f"{r['rms_jerk']:>9.2f}{r['harsh_share']:>6.0%}")
    return "\n".join(lines)
